"""Task-owned fleet allocation; exporters supply safe candidates, Nav2 owns motion.

The policy is ROS independent. ROS imports are deliberately local so its retry
and reservation rules can also be tested without a ROS installation.
"""
import json
import math
from pathlib import Path
import time


from edgeai_tasks.fleet_assignment import ROBOTS, DEFAULTS, distance, AssignmentPolicy


class ExplorationFailure(RuntimeError):
    category = 'fatal'


def candidate_snapshot(payload, now_ns, max_age_s):
    """Return None for unavailable, [] only for a valid, fresh empty source."""
    try:
        if payload['frame_id'] != 'map' or payload['available'] is not True:
            return None
        stamp = payload['stamp_ns']
        if isinstance(stamp, bool) or not isinstance(stamp, int):
            return None
        if not 0 <= (now_ns-stamp)/1e9 <= max_age_s:
            return None
        if not isinstance(payload['candidates'], list):
            return None
        result = []
        for raw in payload['candidates']:
            goal = {k: float(raw[k]) for k in ('x', 'y', 'size')}
            if not all(math.isfinite(v) for v in goal.values()) or goal['size'] < 0:
                return None
            result.append(goal)
        unique = {(g['x'], g['y']): g for g in result}
        return [unique[k] for k in sorted(unique)]
    except (KeyError, TypeError, ValueError, OverflowError):
        return None


def shared_frontiers(snapshots):
    """Union fresh map-frame offers; missing sources cannot prove exhaustion."""
    unique={}
    for goals in snapshots.values():
        for goal in goals or []:
            key=(goal['x'],goal['y'])
            if key not in unique or goal['size']>unique[key]['size']:
                unique[key]=dict(goal)
    if unique:return [unique[key] for key in sorted(unique)]
    return [] if all(goals is not None for goals in snapshots.values()) else None


class FrontierUpdates:
    """Require sustained absence in distinct fresh snapshots before replacement."""
    def __init__(self, settings):
        self.settings = settings
        self.missing = {}

    def obsolete(self, robot, entry, candidates, stamp, wall):
        c = self.settings
        if (entry['record'].get('kind') != 'frontier'
                or entry['cancel_reason'] is not None or candidates is None
                or any(distance(entry['goal'], g) <= c['frontier_match_radius_m']
                       for g in candidates)):
            self.missing.pop(robot, None)
            return False
        previous = self.missing.get(robot)
        # Snapshot age is checked by the caller. A regressed source clock or a
        # gap in observations starts a new confirmation period.
        if (previous is None or previous['entry'] is not entry
                or stamp < previous['stamp']
                or (stamp-previous['stamp'])/1e9 > c['max_candidate_age_s']):
            self.missing[robot] = dict(entry=entry, stamp=stamp, since=wall, samples=1)
            return False
        if stamp == previous['stamp']:
            return False
        previous['stamp'] = stamp
        previous['samples'] += 1
        return (previous['samples'] >= 2
                and wall-previous['since'] >= c['frontier_update_hold_s']
                and wall-entry['sent_at'] >= c['frontier_min_navigation_s'])


class ExplorationUnits:
    task_name = 'explore'
    event_prefix = 'FLEET_EXPLORATION_'
    def __init__(self, c):
        import rclpy
        from rclpy.node import Node
        from rclpy.action import ActionClient
        from rclpy.qos import QoSProfile, DurabilityPolicy
        from std_msgs.msg import String
        from std_srvs.srv import SetBool, Trigger
        from nav2_msgs.action import NavigateToPose
        from nav_msgs.msg import OccupancyGrid
        from tf2_ros import Buffer, TransformListener

        self.robots = tuple(c.get('fleet', {}).get('mapping_robots', ROBOTS))
        self.c = c
        self.directory = Path(c['directory'])
        self.directory.mkdir(parents=True, exist_ok=True)
        self.policy = AssignmentPolicy(c.get('exploration_assignment'))
        self.settings = self.policy.c
        self.frontier_updates = FrontierUpdates(self.settings)
        self.node = Node('fleet_'+self.task_name+'_coordinator')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self.node)
        self.success = False
        self.fatal_reason = None
        self.condition_complete = False
        self.closed = False
        self.closing = False
        self.deadline = time.monotonic()+float(c.get('exploration_timeout_s', 600.))
        self.sources, self.states, self.state_received = {}, {}, {}
        self.expected, self.pending, self.settle_until = {}, {}, {}
        self.assignments, self.failures = [], []
        self.empty_since = None
        self.empty_wall = None
        self.last_state = None
        self.map = None
        self.map_received = 0.
        self.nav, self.manual, self.stops = {}, {}, {}
        self.subscriptions = []
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscriptions.append(self.node.create_subscription(
            OccupancyGrid, '/map', self._map, qos))
        status_root = {'explore':'/exploration', 'find_targets':'/target_search',
                       's1':'/single_rescue'}[self.task_name]
        self.publisher = self.node.create_publisher(String, status_root+'/task_state', qos)
        self.status_publisher = self.node.create_publisher(String, status_root+'/fleet_status', qos)
        self.status_timer = self.node.create_timer(1., self.publish_fleet_status)
        for robot in self.robots:
            prefix = '/'+robot
            self.nav[robot] = ActionClient(self.node, NavigateToPose, prefix+'/navigation/assigned_frontier_to_pose')
            self.manual[robot] = self.node.create_client(SetBool, prefix+'/navigation_authority/set_manual')
            self.stops[robot] = self.node.create_client(Trigger, prefix+'/navigation_authority/stop')
            self.subscriptions.append(self.node.create_subscription(
                String, '/'+robot+'/explore/candidates',
                lambda msg, r=robot: self._source(r, msg), 10))
            self.subscriptions.append(self.node.create_subscription(
                String, prefix+'/navigation_authority/state',
                lambda msg, r=robot: self._authority(r, msg), qos))
        self.event('STARTED')

    def _map(self, msg):
        self.map, self.map_received = msg, time.monotonic()

    def publish_fleet_status(self):
        from std_msgs.msg import String
        robots={}
        snapshots=self.snapshots()
        held = getattr(self, 'held_assignments', {})
        for robot in self.robots:
            entry=self.pending.get(robot) or held.get(robot)
            state=self.states.get(robot,{})
            robots[robot]=dict(task=self.task_name,pose=self.pose(robot),
                authority_fresh=self._fresh_state(robot),mode=state.get('mode'),
                phase=state.get('phase'),command_sequence=state.get('command_sequence'),
                idle=bool(self._fresh_state(robot) and robot not in self.pending and robot not in held
                    and state.get('phase')=='IDLE' and state.get('mode')=='MANUAL'
                    and state.get('command_sequence')==self.expected.get(robot)),
                goal=entry['goal'] if entry else None,
                activity=entry['record'].get('kind','frontier') if entry else 'idle',
                navigation=entry['record']['state'] if entry else None,
                candidates_available=snapshots.get(robot) is not None,
                candidate_count=len(snapshots[robot]) if snapshots.get(robot) is not None else None)
        status=dict(stamp_ns=self.now_ns(),robots=robots)
        if self.task_name=='explore':
            raw={r:candidate_snapshot(self.sources.get(r),self.now_ns(),self.settings['max_candidate_age_s'])
                 for r in self.robots}
            status['frontier_source_counts']={r:len(goals) if goals is not None else None for r,goals in raw.items()}
        self.status_publisher.publish(String(data=json.dumps(status)))

    def now_ns(self):
        return self.node.get_clock().now().nanoseconds

    def event(self, code, **data):
        from std_msgs.msg import String
        record = dict(data)
        record.setdefault('stamp_ns', self.now_ns())
        record['event'] = self.event_prefix+code
        encoded = json.dumps(record, sort_keys=True)
        print(encoded, flush=True)
        self.publisher.publish(String(data=encoded))

    def _source(self, robot, msg):
        try:
            self.sources[robot] = json.loads(msg.data)
        except (ValueError, TypeError):
            self.sources[robot] = None

    def _authority(self, robot, msg):
        try:
            state = json.loads(msg.data)
            self.states[robot] = state if isinstance(state, dict) else {}
        except (ValueError, TypeError):
            self.states[robot] = {}
        self.state_received[robot] = time.monotonic()

    def _save(self):
        path = self.directory/('exploration-result.json' if self.task_name == 'explore' else 'target-search-result.json')
        temporary = path.with_suffix('.tmp')
        temporary.write_text(json.dumps(dict(success=self.success, reason=self.fatal_reason,
            assignments=self.assignments, failures=self.failures,
            failure_history=self.policy.history, arrival_history=self.policy.arrivals), indent=2), encoding='utf-8')
        temporary.replace(path)

    def fail(self, reason):
        if self.fatal_reason is None:
            self.fatal_reason = reason
            self.success = False
            self.event('FAILED', reason=reason)
            self._save()
        raise ExplorationFailure(self.fatal_reason)

    def _wait(self, predicate, timeout, budget=True):
        import rclpy
        end = time.monotonic()+timeout
        if budget:
            end = min(end, self.deadline)
        while rclpy.ok() and time.monotonic() < end:
            if hasattr(self, 'pump_peers'):self.pump_peers()
            if not self.closing and getattr(self.node, 'stop_requested', False):
                self.fail('Exploration task canceled')
            rclpy.spin_once(self.node, timeout_sec=.02)
            if predicate():
                return
        self.fail('Timed out waiting for exploration control/terminal transition')

    def _fresh_state(self, robot):
        return (robot in self.states and time.monotonic()-self.state_received.get(robot, 0.)
                <= self.settings['authority_max_age_s'])

    def check_owner(self):
        if self.fatal_reason:
            raise ExplorationFailure(self.fatal_reason)
        if time.monotonic() >= self.deadline:
            self.fail('Exploration task time budget exhausted')
        if getattr(self.node, 'stop_requested', False):
            self.fail('Exploration task canceled')
        for robot in self.robots:
            state = self.states.get(robot, {})
            if (not self._fresh_state(robot) or robot not in self.expected
                    or state.get('command_sequence') != self.expected[robot]
                    or state.get('mode') != 'MANUAL' or not state.get('explorer_paused')):
                self.fail(robot+': navigation authority changed or stale')
            if state.get('phase') == 'UNKNOWN':
                self.fail(robot+': navigation authority terminal state unknown')

    def acquire(self):
        from std_srvs.srv import SetBool
        if len(self.expected) == len(self.robots):
            self.check_owner()
            return {}
        end = time.monotonic()+20.
        self._wait(lambda: all(self._fresh_state(r) and self.manual[r].service_is_ready()
                              and self.nav[r].server_is_ready() for r in self.robots), 20.)
        # Issue both requests before waiting; retain ownership claims even if a
        # later acquisition fails so close() can stop a partially acquired fleet.
        futures = {}
        for robot in self.robots:
            self.expected[robot] = self.states[robot]['command_sequence']+1
            futures[robot] = self.manual[robot].call_async(SetBool.Request(data=True))
        def acquired():
            for r, future in futures.items():
                state = self.states.get(r, {})
                if state.get('command_sequence', -1) > self.expected[r]:
                    self.fail(r+': operator changed ownership during acquisition')
                if future.done() and not future.result().success:
                    self.fail(r+': MANUAL request rejected')
            return all(f.done() and self._fresh_state(r)
                       and self.states[r].get('command_sequence') == self.expected[r]
                       and self.states[r].get('mode') == 'MANUAL'
                       and self.states[r].get('phase') == 'IDLE'
                       and self.states[r].get('explorer_paused') for r, f in futures.items())
        self._wait(acquired, max(.001, end-time.monotonic()))
        self.event('ACQUIRED', sequences=self.expected)
        return {}

    def snapshots(self, *, require_all_sources=False):
        raw={r: candidate_snapshot(self.sources.get(r), self.now_ns(),
                                  self.settings['max_candidate_age_s']) for r in self.robots}
        # Exporters certify endpoints using their own costmaps, not exclusive
        # ownership. Nav2 still plans and checks clearance for the receiving car.
        pool = None if require_all_sources and any(v is None for v in raw.values()) else shared_frontiers(raw)
        return {r:pool for r in self.robots}

    def pose(self, robot):
        from rclpy.time import Time
        from tf2_ros import TransformException
        try:
            tf = self.buffer.lookup_transform('map', robot+'/base_footprint', Time())
            stamp = tf.header.stamp.sec*10**9+tf.header.stamp.nanosec
            if not 0 <= (self.now_ns()-stamp)/1e9 <= self.settings['max_candidate_age_s']:
                return None
            p = tf.transform.translation
            return dict(x=p.x, y=p.y) if math.isfinite(p.x) and math.isfinite(p.y) else None
        except TransformException:
            return None

    def dispatch(self):
        self.check_owner()
        snapshots = self.snapshots()
        poses = {}
        for r in self.robots:
            if (r in self.pending or snapshots[r] is None
                    or self.now_ns() < self.settle_until.get(r, 0)
                    or self.states[r].get('phase') != 'IDLE' or not self.nav[r].server_is_ready()):
                continue
            pose = self.pose(r)
            if pose is not None:
                poses[r] = pose
        reserved = {r: p['goal'] for r, p in self.pending.items()}
        goals = self.policy.allocate(snapshots, poses, reserved, time.monotonic())
        for robot, goal in goals.items():
            self._send(robot, goal, poses[robot], 'frontier')
        self._report(snapshots)
        return dict(complete=False, dispatched=len(goals))

    def refresh_frontier_goals(self):
        """Explorer updates invalidate goals; allocation remains task-owned."""
        self.check_owner()
        snapshots = self.snapshots()
        raw={r:candidate_snapshot(self.sources.get(r),self.now_ns(),
                                 self.settings['max_candidate_age_s']) for r in self.robots}
        # Both source streams must be fresh before their combined absence can
        # cancel a shared goal. Repeated or one-sided updates are not evidence.
        complete=all(goals is not None for goals in raw.values())
        stamp=min(self.sources[r]['stamp_ns'] for r in self.robots) if complete else None
        for robot in self.robots:
            entry = self.pending.get(robot)
            if entry is None:
                self.frontier_updates.missing.pop(robot, None)
                continue
            candidates = snapshots[robot] if complete else None
            if self.frontier_updates.obsolete(robot, entry, candidates, stamp, time.monotonic()):
                self._cancel(entry, 'frontier_updated')
                self.event('FRONTIER_UPDATE_REQUESTED', robot=robot, goal=entry['goal'],
                    reason='Active frontier absent from sustained fresh candidate updates',
                    candidates=len(candidates))
        return dict(complete=False)

    def _send(self, robot, goal, pose, kind):
        from nav2_msgs.action import NavigateToPose
        yaw = goal.get('yaw', math.atan2(goal['y']-pose['y'], goal['x']-pose['x']))
        request = NavigateToPose.Goal()
        request.pose.header.frame_id = 'map'
        request.pose.header.stamp = self.node.get_clock().now().to_msg()
        request.pose.pose.position.x, request.pose.pose.position.y = goal['x'], goal['y']
        request.pose.pose.orientation.z, request.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        record = dict(robot=robot, kind=kind, goal=dict(goal, yaw=yaw), sent_ns=self.now_ns(), state='SENDING')
        sent = self.navigation_client(robot, goal).send_goal_async(request)
        self.assignments.append(record)
        self.pending[robot] = dict(goal=goal, sent=sent, handle=None, result=None,
            sent_at=time.monotonic(), cancel_at=None, cancel=None, cancel_reason=None, record=record)
        self.event('ASSIGNED', **record)

    def navigation_client(self, robot, goal):
        return self.nav[robot]

    def action_succeeded(self, robot, entry):
        if entry['record'].get('kind') == 'frontier':
            self.policy.reached(robot, entry['goal'], time.monotonic())

    def _cancel(self, entry, reason):
        if entry['cancel_at'] is None:
            entry['cancel_at'] = time.monotonic()
            entry['cancel_reason'] = reason
            if reason != 'frontier_updated':
                self.event('ASSIGNMENT_CANCEL_REQUESTED', robot=entry['record']['robot'],
                           goal=entry['goal'], reason=reason)
        if entry['handle'] is not None and entry['cancel'] is None:
            entry['cancel'] = entry['handle'].cancel_goal_async()
            entry['record']['state'] = 'CANCELING'

    def _failure(self, robot, entry, reason):
        record = dict(robot=robot, kind=entry['record'].get('kind'),
                      goal=dict(entry['goal']), reason=reason, stamp_ns=self.now_ns())
        self.failures.append(record)
        self.policy.failed(robot, entry['goal'], time.monotonic(), reason)
        entry['record'].update(state='FAILED', reason=reason)
        self.event('NAV_FAILED', **record)
        # Peer obstruction recovery is handled before Nav2 returns its terminal result.

    def _advance(self):
        """Pump futures even during cleanup/fatal states; never free unknown slots."""
        unknown = []
        external_cancel = []
        for robot, entry in list(self.pending.items()):
            if entry['handle'] is None and entry['sent'].done():
                try:
                    handle = entry['sent'].result()
                    if not handle.accepted:
                        if entry['cancel_reason'] in ('shutdown', 'frontier_updated', 'target_updated'):
                            entry['record'].update(state='CANCELED', finished_ns=self.now_ns(),
                                                   reason=entry['cancel_reason'])
                            if entry['cancel_reason'] == 'frontier_updated':
                                self.event('FRONTIER_RELEASED', robot=robot, goal=entry['goal'],
                                           reason='Goal rejected after replacement requested')
                        else:
                            self._failure(robot, entry, 'Goal rejected')
                        del self.pending[robot]
                        continue
                    entry['handle'] = handle
                    entry['result'] = handle.get_result_async()
                    entry['record']['state'] = 'ACTIVE'
                except Exception as error:
                    unknown.append(robot+': goal acceptance unknown: '+str(error))
                    continue
            future = entry['result']
            if future is not None and future.done():
                try:
                    terminal = future.result()
                    if terminal.status not in (4, 5, 6):
                        unknown.append(robot+': invalid terminal status')
                        continue
                    code = terminal.result.error_code
                except Exception as error:
                    unknown.append(robot+': terminal result unknown: '+str(error))
                    continue
                entry['record'].update(status=terminal.status, error_code=code, finished_ns=self.now_ns())
                if terminal.status == 4 and code == 0 and entry['cancel_reason'] is None:
                    entry['record']['state'] = 'SUCCEEDED'
                    self.action_succeeded(robot, entry)
                    self.settle_until[robot] = self.now_ns()+int(self.settings['settle_s']*1e9)
                    self.event('ARRIVED', robot=robot, goal=entry['goal'])
                elif entry['cancel_reason'] in ('shutdown', 'frontier_updated', 'target_updated'):
                    entry['record']['state'] = 'CANCELED'
                    entry['record']['reason'] = entry['cancel_reason']
                    if entry['cancel_reason'] == 'frontier_updated':
                        self.settle_until[robot] = self.now_ns()+int(self.settings['settle_s']*1e9)
                        self.event('FRONTIER_RELEASED', robot=robot, goal=entry['goal'],
                                   status=terminal.status, error_code=code)
                else:
                    self._failure(robot, entry, entry['cancel_reason'] or
                                  f'Navigation terminal status={terminal.status}, error={code}')
                    if terminal.status == 5 and entry['cancel_reason'] is None:
                        external_cancel.append(robot+': navigation canceled externally')
                del self.pending[robot]
                continue
            elapsed = time.monotonic()-entry['sent_at']
            limit = (self.settings['acceptance_timeout_s'] if entry['handle'] is None else
                     self.settings['navigation_timeout_s'])
            if elapsed >= limit:
                self._cancel(entry, 'Goal acceptance timeout' if entry['handle'] is None else 'Navigation timeout')
            if entry['cancel_at'] is not None:
                self._cancel(entry, entry['cancel_reason'])
                if time.monotonic()-entry['cancel_at'] >= self.settings['cancel_timeout_s']:
                    unknown.append(robot+': cancellation terminal result unknown; slot retained')
        if unknown or external_cancel:
            self.fail('; '.join(unknown+external_cancel))

    def _report(self, snapshots):
        states = {}
        wall = time.monotonic()
        for r in self.robots:
            candidates = snapshots[r]
            if r in self.pending:
                state = self.pending[r]['record']['state']
            elif candidates is None:
                state = 'SOURCE_UNAVAILABLE'
            elif not candidates:
                state = 'CANDIDATES_EMPTY'
            else:
                kinds = {self.policy.eligibility(r, g, wall) for g in candidates}
                state = 'READY' if 'eligible' in kinds else 'ALL_SUPPRESSED' if 'suppressed' in kinds else 'BUDGET_EXHAUSTED'
            states[r] = state
        if states != self.last_state:
            self.last_state = states
            self.event('STATE', robots=states)
        return states

    def poll(self):
        self._advance()
        self.check_owner()
        self._report(self.snapshots())
        return dict(complete=False, inflight=len(self.pending))

    def check_complete(self):
        self.check_owner()
        # Completion needs fresh reports from every source. A frontier remains
        # actionable during cooldown or while any robot still has attempts.
        snapshots = self.snapshots(require_all_sources=True)
        states = self._report(snapshots)
        now = self.now_ns()
        empty = (not self.pending
                 and all(state in ('CANDIDATES_EMPTY', 'BUDGET_EXHAUSTED') for state in states.values())
                 and all(now >= until for until in self.settle_until.values())
                 and all(self.states[r].get('phase') == 'IDLE' for r in self.robots))
        if not empty:
            self.empty_since = self.empty_wall = None
        elif self.empty_since is None or now < self.empty_since:
            self.empty_since, self.empty_wall = now, time.monotonic()
        hold = self.settings['completion_hold_s']
        self.condition_complete = bool(empty and (now-self.empty_since)/1e9 >= hold
                                      and time.monotonic()-self.empty_wall >= hold)
        return dict(complete=self.condition_complete)

    def record_success(self):
        if not self.check_complete()['complete']:
            self.fail('Exploration completion conditions have not passed')
        self.success = True
        self._save()
        self.event('COMPLETED', assignments=len(self.assignments), failures=len(self.failures))
        return dict(complete=True)

    def cancel_all_navigation(self):
        for entry in self.pending.values():
            self._cancel(entry, 'shutdown')
        end = time.monotonic()+self.settings['acceptance_timeout_s']+self.settings['cancel_timeout_s']
        while self.pending and time.monotonic() < end:
            import rclpy
            rclpy.spin_once(self.node, timeout_sec=.02)
            for entry in self.pending.values():
                self._cancel(entry, 'shutdown')
            try:
                self._advance()
            except ExplorationFailure:
                # Continue pumping late acceptance/results until the cleanup
                # bound, even after an unknown-state fault was recorded.
                if not self.pending:
                    break
        if self.pending:
            self.fail('Cleanup could not confirm all navigation terminal results')
        return dict(complete=False)

    def invoke(self, op):
        operations = dict(acquire=self.acquire, dispatch=self.dispatch, poll=self.poll,
            refresh_frontier_goals=self.refresh_frontier_goals,
            dispatch_navigation=self.dispatch, poll_navigation=self.poll,
            check_complete=self.check_complete, record_success=self.record_success,
            cancel_all_navigation=self.cancel_all_navigation, cancel_navigation=self.cancel_all_navigation)
        if op not in operations:
            raise ExplorationFailure('Unregistered exploration unit: '+op)
        return operations[op]()

    def close(self):
        from std_srvs.srv import Trigger
        if self.closed:
            return
        self.closing = True
        errors = []
        try:
            try:
                self.cancel_all_navigation()
            except Exception as error:
                errors.append(str(error))
            # Stop only authorities still bearing our sequence. An operator's
            # newer command is never overwritten by task cleanup.
            stops = {}
            for r, expected in self.expected.items():
                state = self.states.get(r, {})
                if state.get('command_sequence') != expected:
                    errors.append(r+': ownership changed before stop')
                    continue
                if not self.stops[r].service_is_ready():
                    errors.append(r+': stop service unavailable')
                    continue
                stops[r] = (expected+1, self.stops[r].call_async(Trigger.Request()))
            def stopped():
                return all(f.done() and f.result().success and self._fresh_state(r)
                           and self.states[r].get('command_sequence') == seq
                           and self.states[r].get('mode') == 'STOPPED'
                           and self.states[r].get('phase') == 'IDLE'
                           and self.states[r].get('explorer_paused') for r, (seq, f) in stops.items())
            try:
                self._wait(stopped, self.settings['cancel_timeout_s'], budget=False)
            except Exception as error:
                errors.append(str(error))
            if errors:
                self.success = False
                self.fatal_reason = 'Exploration cleanup failed: '+'; '.join(errors)
            self._save()
            self.event('CLOSED', success=self.success, reason=self.fatal_reason, inflight=len(self.pending))
        finally:
            self.closed = True
            self.node.destroy_node()
        if errors:
            raise ExplorationFailure(self.fatal_reason)
