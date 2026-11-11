#!/usr/bin/env python3
"""Bounded retries of one rescue target, followed by independent stop/dwell checks."""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import time
import signal
from datetime import datetime, timezone
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo
from std_srvs.srv import SetBool, Trigger
from nav_msgs.msg import Odometry, OccupancyGrid
from nav2_msgs.action import NavigateToPose, ComputePathToPose, Spin, DriveOnHeading
from action_msgs.msg import GoalStatus
from tf2_ros import Buffer, TransformListener, TransformException
from rescue_policy import select_goal, Dwell, rescue_position, RescueAttemptFailed
from search_log import search_text, goal_text


class Rescue(Node):
    def __init__(self, config, directory):
        super().__init__('normal_rescue')
        self.c, self.directory = config, Path(directory)
        self.state = self.odom = None
        self.odom_history = deque(maxlen=100)
        self.state_received = self.odom_received = 0.0
        self.goal_handle = None
        self.navigation_attempts = []
        self.search_attempts = []
        self.alignment_attempts = []
        self.approach_attempts = []
        self.approach_transient_failures = 0
        self.approach_round_start = 0
        self.rescue_attempts = []
        self.search_map = None
        self.search_camera_info = None
        self.search_suspects = None
        self.rescue_goal = None
        self.target_registry = None
        self.target_received = self.registry_warning_at = 0.0
        self.expected_sequence = None
        self.deadline = time.monotonic()+config['wall_timeout_s']
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/navigation_authority/state', self.authority, qos)
        self.create_subscription(Odometry, '/odom', self.odometry, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/map', self.map_received, qos)
        if (config.get('search_frustum',{}).get('enabled',False)
                or config.get('search_suspects',{}).get('enabled',False)):
            self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_info_received,
                                     qos_profile_sensor_data)
        self.create_subscription(String, '/targets/estimates', self.targets_received, qos)
        self.create_subscription(String, '/targets/suspected', self.suspects_received, qos)
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.heading_client = ActionClient(self, NavigateToPose, '/navigation/heading_to_pose')
        self.rescue_client = ActionClient(self, NavigateToPose, '/navigation/rescue_to_pose')
        self.plan_client = ActionClient(self, ComputePathToPose, '/navigation_backend/compute_path_to_pose')
        self.spin_client = ActionClient(self, Spin, '/navigation_backend/spin')
        self.advance_client = ActionClient(self, DriveOnHeading, '/navigation_backend/drive_on_heading')
        self.manual = self.create_client(SetBool, '/navigation_authority/set_manual')
        self.stop = self.create_client(Trigger, '/navigation_authority/stop')

    def authority(self, msg):
        self.state = json.loads(msg.data)
        self.state_received = time.monotonic()

    def odometry(self, msg):
        self.odom, self.odom_received = msg, time.monotonic()
        self.odom_history.append(msg)

    def map_received(self, msg):
        self.search_map = msg

    def camera_info_received(self, msg):
        self.search_camera_info = msg

    def suspects_received(self, msg):
        self.search_suspects = json.loads(msg.data)

    def select_current_goal(self):
        return select_goal(self.c, self.read_target_registry(allow_handover=True) or {})

    def targets_received(self, msg):
        self.target_registry = json.loads(msg.data)
        self.target_received = time.monotonic()

    def read_target_registry(self, allow_handover=False):
        # Live ROS estimates avoid polling a concurrently replaced file on
        # the Windows/WSL shared filesystem. Files remain a startup fallback.
        if self.target_registry is not None and time.monotonic()-self.target_received < 2.5:
            return self.target_registry
        sources = [self.directory/'targets.json']
        if allow_handover:
            sources.append(self.directory/'handover-targets.json')
        for source in sources:
            try:
                return json.loads(source.read_text())
            except (OSError, json.JSONDecodeError):
                continue
        if time.monotonic()-self.registry_warning_at > 5.:
            self.registry_warning_at = time.monotonic()
            self.event('TARGET_REGISTRY_WAIT', '目标快照暂不可读，等待下一次感知发布；仍受任务总预算限制')
        return None

    def event(self, code, message):
        data=None
        if isinstance(message,str) and message.startswith('{'):
            try:data=json.loads(message)
            except ValueError:pass
        summary=search_text(code,data) if isinstance(data,dict) else None
        with (self.directory/'rescue-events.jsonl').open('a',encoding='utf-8') as stream:
            stream.write(json.dumps(dict(event='RESCUE_'+code,
                sim_time=self.get_clock().now().nanoseconds/1e9,
                data=data if data is not None else message),ensure_ascii=False)+'\n')
        self.get_logger().info(f'RESCUE_{code}: {summary or message}')

    def wait(self, predicate, timeout=20.0, check_owner=False):
        deadline = min(self.deadline, time.monotonic()+timeout)
        while rclpy.ok() and time.monotonic() < deadline:
            if getattr(self,'stop_requested',False) and not getattr(self,'closing',False):raise KeyboardInterrupt()
            rclpy.spin_once(self, timeout_sec=.05)
            if check_owner:
                self.check_owner()
            if predicate():
                return
        raise RuntimeError('Timed out waiting for task transition')

    def check_owner(self):
        if (not self.state or time.monotonic()-self.state_received > 2.0
                or self.state.get('mode') != 'MANUAL'
                or self.state.get('command_sequence') != self.expected_sequence):
            raise RuntimeError('Navigation authority changed or state became stale')

    def fresh_measurement(self, now, goal):
        if time.monotonic()-self.odom_received > self.c['max_data_age_s']:
            raise ValueError('Odometry stale')
        # TF and odometry arrive asynchronously. Use the newest paired sample,
        # within the age bound, instead of resetting dwell on a millisecond TF delay.
        for odom in reversed(self.odom_history):
            stamp = Time.from_msg(odom.header.stamp)
            sample_time = stamp.nanoseconds/1e9
            if not 0 <= now-sample_time <= self.c['max_data_age_s']:
                continue
            try:
                tf = self.buffer.lookup_transform('map', 'base_footprint', stamp).transform
            except TransformException:
                continue
            q, p = tf.rotation, tf.translation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            error = math.atan2(math.sin(yaw-goal['yaw']), math.cos(yaw-goal['yaw']))
            twist = odom.twist.twist
            measurement=dict(distance_m=math.hypot(p.x-goal['x'], p.y-goal['y']),
                x=p.x, y=p.y, yaw=yaw,
                yaw_error_rad=error, linear_mps=math.hypot(twist.linear.x, twist.linear.y),
                angular_rps=twist.angular.z, sample_age_s=now-sample_time)
            distance,limit=rescue_position(self.c,goal,measurement)
            measurement.update(rescue_distance_m=distance,rescue_position_limit_m=limit)
            return sample_time,measurement
        raise ValueError('No fresh odometry/TF pair')

    def check_navigation_budget(self, started):
        if self.get_clock().now().nanoseconds/1e9-started > self.c['navigation_timeout_s']:
            raise RuntimeError('Rescue navigation simulation timeout (shared across retries)')

    def latest_goal(self, goal):
        """Refresh geometry for the selected ID; never change rescue identity."""
        if self.c['goal_source'] != 'detected_target':
            return dict(goal)
        registry = self.read_target_registry()
        if registry is None:
            return dict(goal)
        nominal=select_goal(dict(self.c, target_id=goal['target_id']), registry)
        if 'approach_angle_rad' in goal:
            from rescue_approach import approach_goal
            return approach_goal(nominal,self.c,goal['approach_angle_rad'],goal['approach_distance_m'])
        return nominal

    def choose_approach(self,goal,exclude_current=False,started=None):
        from rescue_approach import approaches, approach_goal, failure_reason, should_change_approach
        from cooperative_policy import footprint_clear
        registry=self.read_target_registry()
        if registry is None:raise RuntimeError('No target estimate for rescue approach selection')
        nominal=select_goal(dict(self.c,target_id=goal['target_id']),registry)
        self.wait(lambda:self.search_map is not None and self.plan_client.server_is_ready(),timeout=10.,check_owner=True)
        runtime=self.directory/'nav2-runtime.yaml'
        if not runtime.is_file():raise RuntimeError('Runtime footprint missing for rescue approach selection')
        geometry=yaml.safe_load(runtime.read_text())['global_costmap']['global_costmap']['ros__parameters']
        footprint=json.loads(geometry['footprint'])
        pad=float(geometry.get('footprint_padding',.01))
        preferred=approach_goal(nominal,self.c,goal.get('approach_angle_rad',0.),goal.get('approach_distance_m'))
        for candidate in [preferred]+approaches(nominal,self.c):
            if len(self.approach_attempts)-self.approach_round_start>=self.c.get('approach_selection',{}).get('max_candidates',12):break
            same=math.hypot(candidate['x']-goal['x'],candidate['y']-goal['y'])<.04
            if exclude_current and same:continue
            if any(math.hypot(candidate['x']-old['goal']['x'],candidate['y']-old['goal']['y'])<.04
                   and abs(math.atan2(math.sin(candidate['yaw']-old['goal']['yaw']),math.cos(candidate['yaw']-old['goal']['yaw'])))<.1
                   for old in self.approach_attempts[self.approach_round_start:] if not old.get('retryable')):continue
            record=dict(goal=dict(candidate),started_sim_s=self.get_clock().now().nanoseconds/1e9)
            self.approach_attempts.append(record)
            if not footprint_clear(self.search_map,[candidate['x'],candidate['y'],candidate['yaw']],footprint,pad):
                record.update(feasible=False,reason='footprint_occupied_unknown_or_outside')
                self.event('APPROACH_REJECTED',f"{goal_text(candidate)}；车身净空不足、未知或越界")
                continue
            request=ComputePathToPose.Goal();request.planner_id='GridBasedHeading'
            request.goal.header.frame_id='map';request.goal.header.stamp=self.get_clock().now().to_msg()
            request.goal.pose.position.x=candidate['x'];request.goal.pose.position.y=candidate['y']
            request.goal.pose.orientation.z=math.sin(candidate['yaw']/2);request.goal.pose.orientation.w=math.cos(candidate['yaw']/2)
            sent=self.plan_client.send_goal_async(request);self.wait(sent.done,timeout=5.,check_owner=True)
            handle=sent.result()
            if not handle.accepted:raise RuntimeError('Rescue approach planning rejected')
            self.goal_handle=handle
            future=handle.get_result_async()
            try:self.wait(future.done,timeout=self.c.get('approach_selection',{}).get('planning_timeout_s',10.),check_owner=True)
            except BaseException:
                cancel=handle.cancel_goal_async()
                self.wait(lambda:cancel.done() and future.done(),timeout=5.)
                self.goal_handle=None
                raise
            terminal=future.result();self.goal_handle=None
            code=terminal.result.error_code
            record.update(status=terminal.status,error_code=code,error_msg=terminal.result.error_msg,
                          reason=failure_reason(code),feasible=terminal.status==4 and code==0 and bool(terminal.result.path.poses))
            if record['feasible']:
                self.event('APPROACH_SELECTED',f"{goal_text(candidate)}；全局规划通过，法线偏转={math.degrees(candidate['approach_angle_rad']):.0f}°")
                return candidate
            self.event('APPROACH_REJECTED',f"{goal_text(candidate)}；{failure_reason(code)}，错误码={code}")
            if terminal.status==GoalStatus.STATUS_ABORTED and code==205:
                self.event('APPROACH_START_OCCUPIED','起点占据；由原导航树执行已知占据退出，尚未判定停车位不可达')
                return candidate
            if (terminal.status==GoalStatus.STATUS_ABORTED and code in (202,207) and
                    self.approach_transient_failures<self.c.get('navigation_max_retries',2)):
                record['retryable']=True
                self.approach_transient_failures+=1
                self.wait_to_retry(goal,started if started is not None else self.get_clock().now().nanoseconds/1e9)
                return self.choose_approach(goal,exclude_current,started)
            if terminal.status!=GoalStatus.STATUS_ABORTED or not should_change_approach(code):
                raise RuntimeError(f'Approach planning requires {failure_reason(code)} handling: error={code}')
        raise RescueAttemptFailed('Rescue approach candidates exhausted; target estimate may be inconsistent with map','approach',208)

    def goal_changed(self, goal, updated):
        distance = math.hypot(updated['x']-goal['x'], updated['y']-goal['y'])
        yaw = abs(math.atan2(math.sin(updated['yaw']-goal['yaw']),
                             math.cos(updated['yaw']-goal['yaw'])))
        return (distance >= self.c.get('target_update_distance_m', .10) or
                yaw >= self.c.get('target_update_yaw_rad', .25))

    def wait_to_retry(self, goal, started):
        beginning = time.monotonic()
        deadline = min(self.deadline, beginning+self.c.get('navigation_retry_wait_s', 5.0))
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.05)
            self.check_owner()
            self.check_navigation_budget(started)
            if time.monotonic()-beginning < self.c.get('navigation_retry_delay_s', 1.0):
                continue
            # The action result may precede the authority's IDLE publication.
            if self.state.get('phase') != 'IDLE' or not self.state.get('explorer_paused'):
                continue
            try:
                _, measurement = self.fresh_measurement(self.get_clock().now().nanoseconds/1e9, goal)
                if measurement['sample_age_s'] <= self.c.get('retry_tf_max_age_s', .15):
                    return
            except (TransformException, ValueError):
                pass
        raise RuntimeError('Rescue retry readiness timeout: authority idle and fresh TF/odometry required')

    def navigate(self, goal):
        started = getattr(self,'navigation_started',self.get_clock().now().nanoseconds/1e9)
        client=self.rescue_client
        max_retries = self.c.get('navigation_max_retries', 2)
        failures = 0
        selection=self.c.get('approach_selection',{}).get('enabled',False) and self.c['goal_source']=='detected_target'
        if selection:goal.update(self.choose_approach(goal,started=started))
        while True:
            self.check_owner()
            self.check_navigation_budget(started)
            goal.update(self.latest_goal(goal))
            attempt = len(self.navigation_attempts)+1
            request = NavigateToPose.Goal()
            request.pose.header.frame_id = 'map'
            request.pose.header.stamp = self.get_clock().now().to_msg()
            request.pose.pose.position.x = goal['x']; request.pose.pose.position.y = goal['y']
            request.pose.pose.orientation.z = math.sin(goal['yaw']/2)
            request.pose.pose.orientation.w = math.cos(goal['yaw']/2)
            future = client.send_goal_async(request)
            self.wait(future.done, check_owner=True)
            self.goal_handle = future.result()
            record = dict(attempt=attempt, goal=dict(goal), arrival_policy='rescue', accepted=self.goal_handle.accepted,
                          started_sim_s=self.get_clock().now().nanoseconds/1e9)
            self.navigation_attempts.append(record)
            if not self.goal_handle.accepted:
                self.goal_handle=None
                raise RescueAttemptFailed('Rescue navigation goal rejected','navigation')
            self.event('NAVIGATING', f'第 {attempt} 次导航已接受；{goal_text(goal)}；检查位置和朝向')
            dispatched_id = bytes(self.goal_handle.goal_id.uuid)
            result = self.goal_handle.get_result_async()
            pending_goal = None
            next_update = time.monotonic()+self.c.get('target_update_interval_s', 1.0)
            cancel = None
            def navigation_ready():
                nonlocal pending_goal, next_update, cancel
                if pending_goal is None and (result.done() or time.monotonic() >= next_update):
                    next_update = time.monotonic()+self.c.get('target_update_interval_s', 1.0)
                    updated_goal = self.latest_goal(goal)
                    if self.goal_changed(goal, updated_goal):
                        pending_goal = updated_goal
                        self.event('TARGET_UPDATED', f'等待旧导航终止后更新偏移点：{goal_text(updated_goal)}')
                        if not result.done():
                            cancel = self.goal_handle.cancel_goal_async()
                return result.done() or self.get_clock().now().nanoseconds/1e9-started > self.c['navigation_timeout_s']
            self.wait(navigation_ready,
                      timeout=self.c['wall_timeout_s'], check_owner=True)
            if not result.done():
                self.check_navigation_budget(started)
                raise RuntimeError('Rescue navigation ended without a terminal result')
            terminal = result.result()
            self.goal_handle = None
            record.update(status=terminal.status, error_code=terminal.result.error_code,
                          error_msg=terminal.result.error_msg,
                          finished_sim_s=self.get_clock().now().nanoseconds/1e9)
            self.check_owner()
            self.check_navigation_budget(started)
            if pending_goal is not None:
                # Only our acknowledged cancellation or a normal terminal result
                # may release this slot for a refreshed goal.
                if terminal.status == GoalStatus.STATUS_CANCELED:
                    if cancel is None:
                        raise RuntimeError('Rescue navigation canceled externally')
                    self.wait(cancel.done, timeout=5.0, check_owner=True)
                    if not any(bytes(g.goal_id.uuid) == dispatched_id
                               for g in cancel.result().goals_canceling):
                        raise RuntimeError('Rescue target update cancellation not acknowledged')
                elif terminal.status not in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED):
                    raise RuntimeError('Unknown navigation terminal status during target update')
                record['target_updated'] = True
                self.wait_to_retry(goal, started)
                if selection:goal.update(self.choose_approach(self.latest_goal(goal),started=started))
                # Read again at dispatch: perception may have advanced while canceling.
                continue
            if terminal.status == GoalStatus.STATUS_SUCCEEDED and terminal.result.error_code == 0:
                return started
            failure = f'status={terminal.status}, error={terminal.result.error_code}'
            from rescue_approach import failure_reason, should_change_approach
            record['failure_reason']=failure_reason(terminal.result.error_code)
            self.event('NAV_FAILURE',f"{record['failure_reason']}；{failure}；{terminal.result.error_msg}")
            if selection and terminal.status==GoalStatus.STATUS_ABORTED and should_change_approach(terminal.result.error_code):
                self.wait_to_retry(goal,started)
                goal.update(self.choose_approach(goal,exclude_current=True,started=started))
                continue
            # Never undo cancellation or retry an action of unknown status.
            if terminal.status != GoalStatus.STATUS_ABORTED or failures >= max_retries:
                message=f'Navigation failed after {attempt} attempt(s): {failure}'
                if terminal.status==GoalStatus.STATUS_ABORTED:
                    raise RescueAttemptFailed(message,'navigation',terminal.result.error_code)
                raise RuntimeError(message)
            failures += 1
            self.event('RETRY_WAIT', f'第 {attempt} 次导航失败（{failure}）；保留目标ID，等待控制槽释放和新鲜TF后按最新定位重试')
            self.wait_to_retry(goal, started)
    def acquire_control(self):
        self.wait(lambda: self.manual.service_is_ready() and self.client.server_is_ready()
                  and self.rescue_client.server_is_ready()
                  and self.state is not None and time.monotonic()-self.state_received < 2.0)
        requested_sequence = self.state['command_sequence']+1
        request = SetBool.Request(); request.data = True
        future = self.manual.call_async(request)
        self.wait(future.done)
        if not future.result().success:
            raise RuntimeError('Authority rejected MANUAL mode')
        def acquired():
            if self.state['command_sequence'] > requested_sequence:
                raise RuntimeError('Navigation authority changed while acquiring manual control')
            return (self.state['command_sequence'] == requested_sequence
                    and self.state['mode'] == 'MANUAL' and self.state['phase'] == 'IDLE'
                    and self.state['explorer_paused'])
        self.wait(acquired)
        self.expected_sequence = requested_sequence

    def find_target(self, goal=None):
        if goal is None:
            try:
                goal = self.select_current_goal()
            except ValueError as exc:
                if ('No eligible localized ' not in str(exc) or
                        not self.c.get('search_enabled', False)):
                    raise
                from target_research import search_target
                goal = search_target(self)
        self.rescue_goal = goal
        return goal

    def execute(self, goal=None):
        self.event('PREPARING', json.dumps(goal))
        self.acquire_control()
        return self.complete_rescue(self.find_target(goal))

    def complete_rescue(self,goal):
        self.navigation_started=self.get_clock().now().nanoseconds/1e9
        navigate=True
        retries=0
        while time.monotonic()<self.deadline:
            record=dict(attempt=len(self.rescue_attempts)+1,navigate=navigate)
            self.rescue_attempts.append(record)
            try:
                result=self.rescue_attempt(goal,navigate)
                record['success']=True
                return result
            except RescueAttemptFailed as exc:
                self.check_owner()
                record.update(success=False,phase=exc.phase,error_code=exc.error_code,reason=str(exc))
                # A failed action may still have brought the robot inside the task region.
                self.wait_to_retry(goal,self.navigation_started)
                goal.update(self.latest_goal(goal))
                _,measurement=self.fresh_measurement(self.get_clock().now().nanoseconds/1e9,goal)
                in_region=measurement['rescue_distance_m']<=measurement['rescue_position_limit_m']
                yaw_ok=abs(measurement['yaw_error_rad'])<=self.c['yaw_tolerance_rad']
                record['recheck']=measurement
                self.event('RECHECK',f"救援动作失败后重检：任务距离={measurement['rescue_distance_m']:.3f} m，"
                    f"范围={measurement['rescue_position_limit_m']:.3f} m，朝向合格={yaw_ok}；{exc}")
                if in_region and yaw_ok:
                    # Even after retry exhaustion, allow task-only checking when no motion is needed.
                    navigate=False
                    continue
                if retries>=self.c.get('rescue_max_retries',2):
                    raise RuntimeError(f'Rescue retries exhausted after {retries+1} attempts: {exc}') from exc
                retries+=1
                self.event('RETRY',f'重新尝试救援，第{retries}次重试；保留目标ID，重新规划可达停车位')
                self.approach_round_start=len(self.approach_attempts)
                self.approach_transient_failures=0
                navigate=True
        raise RuntimeError('Rescue retries exhausted')

    def rescue_attempt(self,goal,navigate=True):
        started=self.navigation_started
        if navigate:self.navigate(goal)
        self.event('DWELL','按救援任务的位置范围、朝向、停车和连续停留检查；不追踪圈内偏移点')
        dwell = Dwell(self.c)
        last = {}
        alignment_started=None
        measurement_after=0.
        while time.monotonic() < self.deadline:
            rclpy.spin_once(self, timeout_sec=.05)
            self.check_owner()
            now = self.get_clock().now().nanoseconds/1e9
            if now-started > self.c['navigation_timeout_s']+self.c['dwell_s']+15:
                raise RuntimeError('Dwell failed to remain in tolerance')
            try:
                updated=self.latest_goal(goal)
                if self.goal_changed(goal,updated):
                    self.event('DWELL_TARGET_UPDATED','停留检查采用最新目标估计，重新检查位置、朝向和连续停留')
                    dwell.update(now,0.,0.,0.,0.,fresh=False)
                goal.update(updated)
                sample_time, last = self.fresh_measurement(now, goal)
                if sample_time<measurement_after:
                    continue
                if self.c.get('alignment_enabled',False):
                    from rescue_alignment import correction, adjust
                    if correction(self.c,goal,last) is not None:
                        if alignment_started is None:alignment_started=now
                        if (now-alignment_started>self.c.get('alignment_timeout_s',60.) or
                            len(self.alignment_attempts)>=self.c.get('alignment_max_actions',20)):
                            raise RuntimeError('Rescue final correction budget exhausted')
                        dwell.update(sample_time,0.,0.,0.,0.,fresh=False)
                        adjust(self,goal,last)
                        measurement_after=self.get_clock().now().nanoseconds/1e9
                        continue
                if dwell.update(sample_time, last['rescue_distance_m'], last['yaw_error_rad'], last['linear_mps'], last['angular_rps'],
                                position_limit=last['rescue_position_limit_m']):
                    self.event('SUCCEEDED', '普通救援完成：到位、停车并连续停留')
                    return dict(success=True, goal=goal, measurement=last, dwell_s=self.c['dwell_s'],
                                sim_time_s=now, target_id=goal['target_id'], target_type='normal',
                                robots=['robot1'], task_state='completed', judge='estimated map pose; development rules')
            except (TransformException, ValueError):
                dwell.update(now, 0, 0, 0, 0, fresh=False)
                if not last or now-last.get('logged_at', -100) > 5:
                    last['logged_at'] = now
                    self.event('WAITING_DATA', '等待新鲜且时间对齐的里程计/TF，停留计时重置')
        raise RuntimeError('Wall timeout during dwell')

    def finish(self):
        if self.goal_handle and self.goal_handle.accepted:
            future = self.goal_handle.cancel_goal_async()
            try:
                self.wait(future.done, timeout=5.0)
            except RuntimeError:
                pass
        # Never restore AUTO or override a new operator command.
        if self.expected_sequence is not None and self.state and self.state.get('command_sequence') == self.expected_sequence:
            if self.stop.service_is_ready():
                future = self.stop.call_async(Trigger.Request())
                try:
                    self.wait(future.done, timeout=5.0)
                    self.wait(lambda: self.state.get('command_sequence', -1) > self.expected_sequence,
                              timeout=5.0)
                except RuntimeError:
                    pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    parser.add_argument('--directory', required=True)
    args, ros_args = parser.parse_known_args()
    c = yaml.safe_load(Path(args.config).read_text())
    directory = Path(args.directory); directory.mkdir(parents=True, exist_ok=True)
    def interrupted(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    existing = directory/'rescue-result.json'
    if existing.exists() and json.loads(existing.read_text()).get('success'):
        print('RESCUE_ALREADY_COMPLETED: 本轮已完成一次救援，不重复派发', flush=True)
        return 0
    rclpy.init(args=ros_args)
    node = None
    result = dict(success=False)
    goal = None
    try:
        from rescue_policy import validate_config
        validate_config(c)
        node = Rescue(c, directory)
        result = node.execute()
    except (Exception, KeyboardInterrupt) as exc:
        if node:
            goal = node.rescue_goal
        result = dict(success=False, reason=str(exc), goal=goal,
                      target_id=goal['target_id'] if goal else None, target_type='normal', robots=['robot1'])
        if node:
            node.event('FAILED', str(exc))
        else:
            print('RESCUE_FAILED: '+str(exc), flush=True)
    finally:
        if node:
            result['navigation_attempts'] = node.navigation_attempts
            result['search_attempts'] = node.search_attempts
            result['alignment_attempts'] = node.alignment_attempts
            result['approach_attempts'] = node.approach_attempts
            result['rescue_attempts'] = node.rescue_attempts
            try:
                node.finish()
            except Exception as exc:
                result.update(success=False, cleanup_error=str(exc))
        result.update(completed_at_utc=datetime.now(timezone.utc).isoformat(),
                      task_state='completed' if result['success'] else 'failed')
        temporary = directory/'rescue-result.json.tmp'
        temporary.write_text(json.dumps(result, indent=2, ensure_ascii=False))
        temporary.replace(directory/'rescue-result.json')
        target = result.get('target_id')
        summary = f"RESCUE_RESULT: {'成功' if result['success'] else '失败'}；目标ID={target if target is not None else '未知'}"
        if result['success']:
            measured = result['measurement']
            summary += (f"；位置误差={measured['distance_m']*100:.1f} cm"
                        f"；朝向误差={abs(math.degrees(measured['yaw_error_rad'])):.1f}°"
                        f"；连续停留={result['dwell_s']:g} s"
                        f"；末端修正={len(result.get('alignment_attempts', []))}次")
            if c.get('goal_source')=='detected_target' and 'target_radius_m' in c:
                summary += (f"；目标距离={measured['rescue_distance_m']*100:.1f} cm"
                            f"/救援半径={c['target_radius_m']*100:.0f} cm")
        else:
            summary += f"；原因={result.get('cleanup_error') or result.get('reason') or '未知'}"
        print(summary, flush=True)
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if result['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
