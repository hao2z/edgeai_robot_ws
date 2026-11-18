"""Nav2 execution-slot ownership and confirmed cancellation/mode handoff."""
import argparse
from dataclasses import asdict
import json
import os
from pathlib import Path
import time

import rclpy
from rclpy.action import ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.task import Future
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from authority_policy import Authority, AuthorityConfig
from goal_policy import managed_trees, CONTROLLERS, POLICIES
from navigation_requests import NavigationRequests
from navigation_telemetry import NavigationTelemetry


class NavigationAuthority(Node):
    def __init__(self, directory, mode='STOPPED', tree=''):
        super().__init__('navigation_authority')
        self.config = AuthorityConfig()
        self.policy = Authority(mode=mode)
        self.group = ReentrantCallbackGroup()
        self.backend = ActionClient(self, NavigateToPose,
                                    self.config.backend_action, callback_group=self.group)
        self.tree = tree
        self.goal_trees = managed_trees(tree, directory, namespace=self.get_namespace())
        self.current = None
        self.admissions = {}
        self.distance = None
        self.command_sequence = 0
        self.completed_goals = 0
        self.last_clock = self.get_clock().now().nanoseconds
        self.clock_advanced = time.monotonic()
        self.last_state = None
        self.last_state_sent = 0.
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.audit = (self.directory/'authority-events.jsonl').open('a', buffering=1)
        self.publisher = self.create_publisher(String, 'navigation_authority/state',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.dispatched_goal = self.create_publisher(PoseStamped, 'navigation_authority/dispatched_goal',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.requests = NavigationRequests(self)
        self.telemetry = (NavigationTelemetry(self)
                          if os.environ.get('EDGEAI_RUNTIME_PROFILE', 'debug') != 'performance'
                          else None)
        self.create_service(SetBool, 'navigation_authority/set_manual', self.set_manual,
                            callback_group=self.group)
        self.create_service(Trigger, 'navigation_authority/stop', self.stop,
                            callback_group=self.group)
        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(GoalStatusArray,'navigation_backend/navigate_to_pose/_action/status',
                                 self.terminal_status,status_qos)
        self.create_timer(.25, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME),
                          callback_group=self.group)
        self.event('AUTHORITY_READY', '统一导航仲裁已启动', mode=mode)

    def destroy_node(self):
        self.requests.destroy()
        self.backend.destroy()
        return super().destroy_node()

    def event(self, event_type, message, **details):
        record = dict(event=event_type, message=message, wall_monotonic=time.monotonic(),
                      sim_time=self.get_clock().now().nanoseconds/1e9,
                      task=self.policy.generation, owner=self.policy.owner,
                      mode=self.policy.mode, phase=self.policy.phase)
        record.update(details)
        self.audit.write(json.dumps(record, ensure_ascii=False)+'\n')
        self.get_logger().info(f'{event_type}: {message}; task={self.policy.generation}; '
                               f'owner={self.policy.owner}; phase={self.policy.phase}; {details}')

    def admit(self, request, owner, arrival='position'):
        if request.behavior_tree and request.behavior_tree != self.tree:
            self.event('GOAL_REJECTED', '拒绝客户端替换受管理的恢复行为树', requester=owner)
            return GoalResponse.REJECT
        if not self.backend.server_is_ready():
            return GoalResponse.REJECT
        token = self.policy.reserve(owner)
        if token is None:
            self.event('GOAL_REJECTED', '控制权或交接状态不允许派发', requester=owner)
            return GoalResponse.REJECT
        now = time.monotonic()
        context = dict(token=token, arrival=arrival, upstream=None, backend=None, cancel_sent=False,
                       created=now, feedback=now, cancel_at=None, resolution=Future(),
                       terminal_observed=None)
        self.admissions[id(request)] = context
        self.current = context
        self.distance = None
        if self.telemetry:
            self.telemetry.reset()
        self.event('GOAL_RESERVED', '任务已占用唯一导航执行槽', requester=owner,
                   arrival_policy=arrival)
        return GoalResponse.ACCEPT

    def upstream_accepted(self, handle):
        context = self.admissions[id(handle.request)]
        context['upstream'] = handle
        handle.execute()

    async def execute(self, handle):
        context = self.admissions.pop(id(handle.request))
        token = context['token']
        goal = NavigateToPose.Goal(pose=handle.request.pose,
                                  behavior_tree=self.goal_trees[context['arrival']])
        self.dispatched_goal.publish(goal.pose)
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
                       goal_uuid=bytes(backend.goal_id.uuid).hex(), arrival_policy=context['arrival'],
                       controller_id=CONTROLLERS[context['arrival']],
                       goal_checker_id=POLICIES[context['arrival']][0],
                       behavior_tree=goal.behavior_tree)
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
        if self.telemetry:
            self.telemetry.note_navigation(status == GoalStatus.STATUS_SUCCEEDED)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.completed_goals += 1
            handle.succeed()
        elif status == GoalStatus.STATUS_CANCELED and handle.is_cancel_requested:
            handle.canceled()
        else:
            # Mode takeover is an external abort to the former owner, not success.
            handle.abort()
        self.event('GOAL_TERMINAL', '已收到 Nav2 最终结果，允许任务交接',
                   status=status, error_code=result.error_code, error_msg=result.error_msg,
                   arrival_policy=context['arrival'])
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
            if self.telemetry:
                self.telemetry.following = False

    def cancel(self, handle):
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
        self.command_sequence += 1
        changed = self.policy.set_mode(mode)
        if not changed:
            self.event('MODE_ALREADY_REQUESTED', '重复控制请求已合并，无需再次调用 ROS 服务', requested=mode)
            return
        if self.current:
            self.policy.mark_mode_request(time.monotonic())
        self.event('MODE_REQUESTED', '控制权切换已登记；旧任务终止前不接收新目标', requested=mode)
        self.send_cancel()


    def set_manual(self, request, response):
        # Task coordinators and manual tests share the same execution gate.
        self.change_mode('MANUAL' if request.data else 'STOPPED')
        response.success, response.message = True, 'Mode requested; inspect '+self.get_namespace().rstrip('/')+'/navigation_authority/state for completion'
        return response

    def stop(self, request, response):
        self.change_mode('STOPPED')
        response.success, response.message = True, 'Stop requested; waiting for Nav2 terminal result'
        return response


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
        if self.telemetry:
            self.telemetry.note_navigation()

    def terminal_status(self,message):
        # A matching backend UUID is execution evidence, not a recovery diagnostic.
        c=self.current
        if not c or not c['backend'] or c['terminal_observed'] is not None:return
        for item in message.status_list:
            if bytes(item.goal_info.goal_id.uuid)==bytes(c['backend'].goal_id.uuid) and item.status in (4,5,6):
                c['terminal_observed']=(item.status,time.monotonic());break


    def tick(self):
        now = time.monotonic()
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
        state = asdict(self.policy)
        state.update(command_sequence=self.command_sequence, completed_goals=self.completed_goals,
                     arrival_policy=c['arrival'] if c else None,
                     explorer_paused=True, goal_uuid=bytes(c['backend'].goal_id.uuid).hex() if c and c['backend'] else '',
                     clock_paused=now-self.clock_advanced >= 2.0,
                     following_path=self.telemetry.following if self.telemetry else False,
                     explorer_error='', recovery=self.telemetry.snapshot() if self.telemetry else None)
        if state != self.last_state or now-self.last_state_sent >= 1.0:
            self.last_state = state
            self.last_state_sent = now
            self.publisher.publish(String(data=json.dumps(
                dict(state, distance_remaining=self.distance), ensure_ascii=False)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--directory', required=True)
    parser.add_argument('--mode', choices=('MANUAL', 'STOPPED'), default='STOPPED')
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
