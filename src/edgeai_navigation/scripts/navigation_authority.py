"""One cooperative owner of Nav2; recovery stays inside the accepted action."""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.task import Future
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from frontier_exploration_ros2.srv import ControlExploration

from authority_policy import Authority, AuthorityConfig, RecoveryEvidence


class NavigationAuthority(Node):
    def __init__(self, directory, mode='AUTO', tree=''):
        super().__init__('navigation_authority')
        self.config = AuthorityConfig()
        self.policy = Authority(mode=mode)
        self.group = ReentrantCallbackGroup()
        self.backend = ActionClient(self, NavigateToPose,
                                    self.config.backend_action, callback_group=self.group)
        self.manual = ActionClient(self, NavigateToPose, self.config.manual_action, callback_group=self.group)
        self.explorer = self.create_client(ControlExploration, self.config.explorer_control,
                                           callback_group=self.group)
        self.tree = tree
        self.current = None
        self.pending = None
        self.admissions = {}
        self.pose = None
        self.pose_time = 0.0
        self.odom_frame = None
        self.following = False
        self.recovery = None
        self.recoveries = {}
        self.seen = {}
        self.distance = None
        self.last_clock = self.get_clock().now().nanoseconds
        self.clock_advanced = time.monotonic()
        # Frontier explorer is launched with autostart=true. AUTO therefore
        # starts in the confirmed desired state and needs no startup RPC.
        # A service call is reserved for an actual pause/resume transition.
        self.explorer_sync = mode == 'AUTO'
        self.explorer_inflight = None
        self.explorer_request_at = 0.0
        self.explorer_request_seq = 0
        self.explorer_error = ''
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.audit = (self.directory/'authority-events.jsonl').open('a', buffering=1)
        self.publisher = self.create_publisher(String, '/navigation_authority/state',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.servers = [ActionServer(self, NavigateToPose, endpoint,
            execute_callback=self.execute, callback_group=self.group,
            handle_accepted_callback=self.upstream_accepted,
            goal_callback=lambda request, owner=owner: self.admit(request, owner),
            cancel_callback=self.cancel) for endpoint, owner in
            [('/exploration/navigate_to_pose', 'AUTO'), ('/navigate_to_pose', 'MANUAL')]]
        self.create_service(SetBool, '/navigation_authority/set_manual', self.set_manual,
                            callback_group=self.group)
        self.create_service(Trigger, '/navigation_authority/stop', self.stop,
                            callback_group=self.group)
        self.create_service(ControlExploration, '/control_exploration', self.control,
                            callback_group=self.group)
        self.create_subscription(PoseStamped, '/goal_pose', self.manual_pose, 1)
        self.create_subscription(Odometry, '/odom', self.odometry, qos_profile_sensor_data)
        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for kind in ('backup', 'drive_on_heading', 'spin', 'wait', 'follow_path', 'navigate_to_pose'):
            self.create_subscription(GoalStatusArray, '/navigation_backend/'+kind+'/_action/status',
                lambda msg, kind=kind: self.action_status(kind, msg), status_qos)
        self.create_timer(.25, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME),
                          callback_group=self.group)
        self.event('AUTHORITY_READY', '统一导航仲裁已启动', mode=mode)

    def event(self, event_type, message, **details):
        record = dict(event=event_type, message=message, wall_monotonic=time.monotonic(),
                      sim_time=self.get_clock().now().nanoseconds/1e9,
                      task=self.policy.generation, owner=self.policy.owner,
                      mode=self.policy.mode, phase=self.policy.phase)
        record.update(details)
        self.audit.write(json.dumps(record, ensure_ascii=False)+'\n')
        self.get_logger().info(f'{event_type}: {message}; task={self.policy.generation}; '
                               f'owner={self.policy.owner}; phase={self.policy.phase}; {details}')

    def admit(self, request, owner):
        if request.behavior_tree and request.behavior_tree != self.tree:
            self.event('GOAL_REJECTED', '拒绝客户端替换受管理的恢复行为树', requester=owner)
            return GoalResponse.REJECT
        if not self.backend.server_is_ready():
            return GoalResponse.REJECT
        replacing = (self.current is not None and self.pending is None
                     and self.policy.owner == owner and self.policy.mode == owner
                     and self.policy.phase in ('SENDING', 'ACTIVE', 'CANCELING')
                     and not self.policy.fault)
        token = None if replacing else self.policy.reserve(owner)
        if token is None and not replacing:
            self.event('GOAL_REJECTED', '控制权或交接状态不允许派发', requester=owner)
            return GoalResponse.REJECT
        context = dict(token=token, owner=owner, upstream=None, backend=None, cancel_sent=False,
                       created=time.monotonic(), feedback=time.monotonic(),
                       cancel_at=None, mode_changed=None, resolution=Future(), ready=Future(),
                       terminal_observed=None)
        self.admissions[id(request)] = context
        if replacing:
            self.pending = context
            self.policy.request_cancel(self.current['token'])
            self.event('GOAL_QUEUED', '替代目标已接受；等待旧任务终态后交接', requester=owner)
            self.send_cancel()
        else:
            self.activate(context, token)
        return GoalResponse.ACCEPT

    def activate(self, context, token):
        context['token'] = token
        context['created'] = context['feedback'] = time.monotonic()
        self.current = context
        self.distance, self.recovery, self.following = None, None, False
        self.recoveries.clear()
        context['ready'].set_result(True)
        self.event('GOAL_RESERVED', '任务已占用唯一导航执行槽', requester=context['owner'])

    def upstream_accepted(self, handle):
        context = self.admissions[id(handle.request)]
        context['upstream'] = handle
        handle.execute()

    async def execute(self, handle):
        context = self.admissions.pop(id(handle.request))
        allowed = await context['ready']
        if not allowed:
            if handle.is_cancel_requested:
                handle.canceled()
            else:
                handle.abort()
            return NavigateToPose.Result(error_msg='Queued goal canceled before dispatch')
        token = context['token']
        goal = NavigateToPose.Goal(pose=handle.request.pose, behavior_tree=self.tree)
        try:
            backend = await self.backend.send_goal_async(goal,
                feedback_callback=lambda msg: self.feedback(token, msg.feedback))
            if not backend.accepted:
                self.event('BACKEND_REJECTED', 'Nav2 拒绝任务，释放执行槽')
                handle.abort()
                self.finish(context)
                return NavigateToPose.Result(error_code=900, error_msg='Nav2 rejected goal')
            context['backend'] = backend
            self.policy.accepted(token)
            self.event('BACKEND_ACCEPTED', 'Nav2 已接单；恢复期间保留同一任务',
                       goal_uuid=bytes(backend.goal_id.uuid).hex())
            self.send_cancel()
            backend_result = backend.get_result_async()
            backend_result.add_done_callback(
                lambda done, context=context: self.backend_result_received(context, done))
            wrapped = await context['resolution']
        except Exception as error:
            # A transport failure cannot prove that the robot's task ended.
            self.policy.uncertain(token, str(error))
            self.event('STATE_UNKNOWN', '通信结果不明确，保持执行槽并等待终态核实', error=str(error))
            wrapped = await context['resolution']
        status, result = wrapped.status, wrapped.result
        if status == GoalStatus.STATUS_ABORTED and result.error_code == 205 and self.policy.owner == 'AUTO':
            # A planner failure is reported, but does not stop frontier scheduling.
            # The explorer decides whether to retry, suppress, or preempt the goal.
            self.event('START_OCCUPIED_REPORTED', '起点被判占据；保持自动探索，由探索器处理目标策略')
        if self.recovery and self.recovery.note_navigation(
                self.distance, self.following, self.fresh_pose(), status == GoalStatus.STATUS_SUCCEEDED):
            self.event('NAVIGATION_RESTORED', '恢复动作后原任务已完成', evidence=asdict(self.recovery))
        if status == GoalStatus.STATUS_SUCCEEDED:
            handle.succeed()
        elif status == GoalStatus.STATUS_CANCELED and handle.is_cancel_requested:
            handle.canceled()
        else:
            # Mode takeover is an external abort to the former owner, not success.
            handle.abort()
        self.event('GOAL_TERMINAL', '已收到 Nav2 最终结果，允许任务交接',
                   status=status, error_code=result.error_code, error_msg=result.error_msg)
        self.finish(context)
        return result

    def backend_result_received(self, context, future):
        if self.current is not context or context['resolution'].done():
            return
        try:
            context['resolution'].set_result(future.result())
        except Exception as error:
            self.policy.uncertain(context['token'], str(error))
            self.event('STATE_UNKNOWN', '结果服务异常，保留任务并等待匹配 UUID 的终态', error=str(error))

    def finish(self, context):
        if self.current is context:
            self.policy.terminal(context['token'])
            self.current = None
            self.following = False
            pending, self.pending = self.pending, None
            if pending is not None:
                token = self.policy.reserve(pending['owner'])
                if token is None:
                    pending['ready'].set_result(False)
                else:
                    self.activate(pending, token)

    def cancel(self, handle):
        if self.pending is not None and self.pending['upstream'] is handle:
            pending, self.pending = self.pending, None
            pending['ready'].set_result(False)
            self.event('QUEUED_GOAL_CANCELED', '尚未派发的替代目标已取消')
            return CancelResponse.ACCEPT
        if self.current is None or self.current['upstream'] is not handle:
            return CancelResponse.REJECT
        self.policy.request_cancel(self.current['token'])
        self.event('CANCEL_REQUESTED', '取消请求已登记；确认接收不代表任务结束')
        self.send_cancel()
        return CancelResponse.ACCEPT

    def send_cancel(self):
        c = self.current
        if not c or not c['backend'] or c['cancel_sent'] or not self.policy.cancel_requested:
            return
        if (self.policy.owner == 'AUTO' and self.policy.mode != 'AUTO'
                and self.explorer.service_is_ready() and c['mode_changed'] is not None
                and not self.policy.should_force_cancel(time.monotonic(), self.config)
                and not (c['upstream'] and c['upstream'].is_cancel_requested)):
            # Pause frontier scheduling first, so a takeover isn't recorded as
            # navigation failure and followed by a redundant frontier request.
            return
        c['cancel_sent'], c['cancel_at'] = True, time.monotonic()
        self.policy.mark_cancel_sent(c['cancel_at'])
        try:
            future = c['backend'].cancel_goal_async()
            future.add_done_callback(lambda done, c=c: self.cancel_ack(c, done))
        except Exception as error:
            self.policy.uncertain(c['token'], 'cancel transport: '+str(error))

    def cancel_ack(self, context, future):
        if self.current is not context:
            return
        try:
            response = future.result()
            matched = any(bytes(item.goal_id.uuid) == bytes(context['backend'].goal_id.uuid)
                          for item in response.goals_canceling)
            self.event('CANCEL_ACK', '取消响应已到达，继续等待原任务最终结果', accepted=matched)
            if not matched:
                self.policy.uncertain(context['token'], 'Cancellation not acknowledged for current goal')
        except Exception as error:
            self.policy.uncertain(context['token'], str(error))

    def change_mode(self, mode):
        changed = self.policy.set_mode(mode)
        if not changed:
            self.event('MODE_ALREADY_REQUESTED', '重复控制请求已合并，无需再次调用 ROS 服务', requested=mode)
            self.sync_explorer()
            return
        if self.pending is not None:
            pending, self.pending = self.pending, None
            pending['ready'].set_result(False)
            self.event('QUEUED_GOAL_DROPPED', '控制模式切换，撤销待派发目标')
        if self.current:
            self.current['mode_changed'] = time.monotonic()
            self.policy.mark_mode_request(self.current['mode_changed'])
        self.explorer_sync = None
        self.event('MODE_REQUESTED', '控制权切换已登记；旧任务终止前不接收新目标', requested=mode)
        self.sync_explorer()
        self.send_cancel()

    def set_manual(self, request, response):
        self.change_mode('MANUAL' if request.data else 'AUTO')
        response.success, response.message = True, 'Mode requested; inspect /navigation_authority/state for completion'
        return response

    def stop(self, request, response):
        self.change_mode('STOPPED')
        response.success, response.message = True, 'Stop requested; waiting for Nav2 terminal result'
        return response

    def control(self, request, response):
        # Scheduling lives at one owner; silently forwarding delayed commands would race ownership.
        if request.delay_seconds != 0.0 or request.quit_after_stop or request.action not in (1, 2):
            response.accepted, response.message = False, 'Managed control supports immediate start/stop without node shutdown'
            return response
        self.change_mode('AUTO' if request.action == 1 else 'STOPPED')
        response.accepted, response.scheduled = True, False
        response.state = 1 if self.policy.mode == 'AUTO' and self.policy.phase == 'IDLE' else 4
        response.message = 'Request registered; authority state reports actual completion'
        return response

    def manual_pose(self, pose):
        if self.policy.mode != 'MANUAL' or self.policy.phase != 'IDLE':
            self.event('MANUAL_GOAL_REJECTED', 'RViz 目标需先取得手动控制权，且等待旧任务结束')
            return
        future = self.manual.send_goal_async(NavigateToPose.Goal(pose=pose))
        future.add_done_callback(self.manual_goal_response)

    def manual_goal_response(self, future):
        try:
            handle = future.result()
            if handle.accepted:
                handle.get_result_async().add_done_callback(lambda done: done.result())
        except Exception as error:
            self.event('MANUAL_CLIENT_ERROR', '手动目标请求异常', error=str(error))

    def feedback(self, token, message):
        if not self.current or self.current['token'] != token:
            return
        self.current['feedback'] = time.monotonic()
        if self.policy.phase == 'UNKNOWN' and self.policy.fault == 'Feedback stale while simulation advancing':
            self.policy.accepted(token)
            self.event('FEEDBACK_RESUMED', '当前任务反馈恢复，控制权仍属于原任务')
        self.distance = message.distance_remaining
        if self.current['upstream']:
            self.current['upstream'].publish_feedback(message)
        if self.recovery and self.recovery.note_navigation(self.distance, self.following, self.fresh_pose()):
            self.event('NAVIGATION_RESTORED', '恢复动作有实际位移，且原任务重新行驶并接近目标',
                       evidence=asdict(self.recovery))

    def odometry(self, message):
        frame = (message.header.frame_id, message.child_frame_id)
        if self.odom_frame is not None and frame != self.odom_frame and self.recovery:
            self.recovery.start = None
        self.odom_frame = frame
        p, q = message.pose.pose.position, message.pose.pose.orientation
        self.pose = (p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        self.pose_time = message.header.stamp.sec+message.header.stamp.nanosec/1e9

    def fresh_pose(self):
        age = self.get_clock().now().nanoseconds/1e9-self.pose_time
        return self.pose if 0 <= age <= .5 else None

    def action_status(self, kind, message):
        if kind == 'follow_path':
            self.following = any(item.status == GoalStatus.STATUS_EXECUTING for item in message.status_list)
            return
        for item in message.status_list:
            key = (kind, bytes(item.goal_info.goal_id.uuid).hex())
            previous = self.seen.get(key)
            self.seen[key] = item.status
            if previous == item.status or not self.current:
                continue
            if kind == 'navigate_to_pose':
                c = self.current
                if (c['backend'] and bytes(item.goal_info.goal_id.uuid) == bytes(c['backend'].goal_id.uuid)
                    and item.status in (4, 5, 6) and c['terminal_observed'] is None):
                    c['terminal_observed'] = (item.status, time.monotonic())
                continue
            if item.status == GoalStatus.STATUS_EXECUTING:
                if kind == 'wait':
                    self.event('RECOVERY_WAIT', 'Nav2 等待环境更新；等待完成不等于脱困')
                    continue
                evidence = RecoveryEvidence(kind, self.fresh_pose(), self.distance)
                self.recoveries[key] = evidence
                self.recovery = evidence
                self.event('RECOVERY_STARTED', '执行原生恢复动作，原任务保持', kind=kind)
            elif item.status in (4, 5, 6) and key in self.recoveries:
                evidence = self.recoveries.pop(key)
                evidence.finish(item.status == 4, self.fresh_pose())
                self.event('RECOVERY_ACTION_RESULT', '恢复动作结束；需分别核对动作、位移和导航恢复',
                           kind=kind, status=item.status, evidence=asdict(evidence))
        # Status history is bounded independently of the session duration.
        if len(self.seen) > 256:
            self.seen = dict(list(self.seen.items())[-128:])

    def sync_explorer(self):
        desired = self.policy.mode == 'AUTO'
        # Resume only after the previous backend task has actually terminated.
        if desired and self.policy.phase != 'IDLE':
            return
        if self.explorer_inflight is not None or self.explorer_sync == desired:
            return
        if not self.explorer.service_is_ready() or time.monotonic()-self.explorer_request_at < 1.0:
            return
        self.explorer_request_at = time.monotonic()
        self.explorer_request_seq += 1
        request_seq = self.explorer_request_seq
        future = self.explorer.call_async(ControlExploration.Request(action=1 if desired else 2))
        self.explorer_inflight = future
        future.add_done_callback(lambda done, desired=desired, request_seq=request_seq:
                                 self.explorer_synced(done, desired, request_seq))

    def explorer_synced(self, future, desired, request_seq):
        if request_seq != self.explorer_request_seq:
            return
        self.explorer_inflight = None
        try:
            result = future.result()
            if result.accepted:
                self.explorer_sync, self.explorer_error = desired, ''
            else:
                self.explorer_error = result.message
        except Exception as error:
            self.explorer_error = str(error)

    def tick(self):
        now = time.monotonic()
        if self.explorer_inflight is not None and now-self.explorer_request_at > self.config.explorer_sync_timeout_s:
            # A timed-out RPC is uncertain: release the local Future slot, but
            # never claim the remote explorer did not execute the request.
            self.explorer_request_seq += 1
            self.explorer_inflight = None
            self.explorer_error = 'explorer control response timeout'
            self.event('EXPLORER_SYNC_UNKNOWN', '探索控制响应超时；下一轮由当前策略重新协调')
        clock = self.get_clock().now().nanoseconds
        if clock != self.last_clock:
            self.last_clock, self.clock_advanced = clock, now
        c = self.current
        if c:
            if (c['terminal_observed'] is not None and not c['resolution'].done()
                    and now-c['terminal_observed'][1] >= self.config.result_grace_s):
                from types import SimpleNamespace
                status = c['terminal_observed'][0]
                c['resolution'].set_result(SimpleNamespace(status=status,
                    result=NavigateToPose.Result(error_code=901,
                        error_msg='Terminal action status confirmed; result service timed out')))
                self.event('RESULT_DETAIL_TIMEOUT', '已确认任务终态，但 Nav2 结果服务超时；错误详情不可用', status=status)
            reason = ''
            if c['backend'] is None and now-c['created'] > self.config.accept_timeout_s:
                reason = 'Goal acceptance timeout; no replacement allowed'
            elif c['cancel_at'] is not None and now-c['cancel_at'] > self.config.cancel_timeout_s:
                reason = 'Cancel terminal result timeout; no replacement allowed'
            elif c['backend'] and now-c['feedback'] > self.config.feedback_timeout_s and now-self.clock_advanced < 2.0:
                reason = 'Feedback stale while simulation advancing'
            if reason and self.policy.fault != reason:
                self.policy.uncertain(c['token'], reason)
                self.event('STATE_UNKNOWN', '响应超时只表示状态不明确，禁止重复派发', reason=reason)
            self.send_cancel()
        self.sync_explorer()
        state = asdict(self.policy)
        state.update(goal_uuid=bytes(c['backend'].goal_id.uuid).hex() if c and c['backend'] else '',
                     distance_remaining=self.distance, clock_paused=now-self.clock_advanced >= 2.0,
                     following_path=self.following, explorer_error=self.explorer_error,
                     recovery=asdict(self.recovery) if self.recovery else None)
        self.publisher.publish(String(data=json.dumps(state, ensure_ascii=False)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--directory', required=True)
    parser.add_argument('--mode', choices=('AUTO', 'MANUAL'), default='AUTO')
    parser.add_argument('--tree', required=True)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = NavigationAuthority(args.directory, args.mode, args.tree)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.audit.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
