"""Navigation-owned idle-peer yielding; grid evidence never authorizes motion."""
import math
import time

from nav2_msgs.action import ComputePathToPose, NavigateToPose

from edgeai_nav.yield_routes import GridRoutes


class PeerYieldUnit:
    def __init__(self, node, pending):
        self.node, self.pending = node, pending
        self.context = None
        self.attempts = {}
        self.navigation = {}

    def pose(self, robot):
        return self.node.tf_pose((robot+'/' if robot else '')+'base_footprint')

    def stationary(self, robot):
        n = self.node
        odom = n.odom.get(robot)
        if odom is None:return False
        stamp = odom.header.stamp.sec + odom.header.stamp.nanosec/1e9
        v = odom.twist.twist
        return (0 <= n.now()-stamp <= n.c['max_data_age_s'] and
                math.hypot(v.linear.x,v.linear.y) < .02 and abs(v.angular.z) < .05)

    def idle(self, robot):
        n = self.node
        odom = n.odom.get(robot)
        if odom is None or robot in self.pending or n.states.get(robot, {}).get('phase') != 'IDLE':
            return False
        stamp = odom.header.stamp.sec + odom.header.stamp.nanosec/1e9
        twist = odom.twist.twist
        return (0 <= n.now()-stamp <= n.c['max_data_age_s'] and
                math.hypot(twist.linear.x, twist.linear.y) < .02 and abs(twist.angular.z) < .05)

    def diagnose(self, failures):
        n = self.node; n.check_owner(); self.context = None
        c = n.c.get('yielding', {})
        if not c.get('enabled', False) or n.map is None:
            raise RuntimeError('Peer yielding disabled or map unavailable')
        routes = GridRoutes(n.map, c['yield_body_radius_m'], c['yield_max_cells'])
        seen = set()
        for failure in failures:
            robot, goal = failure['robot'], failure['goal']
            key = (robot, tuple(goal[:2]))
            if key in seen:continue
            seen.add(key)
            peer = 'robot2' if robot == 'robot1' else 'robot1'
            if not self.stationary(robot) or not self.idle(peer):continue
            if self.attempts.get(peer, 0) >= c['yield_max_attempts']:continue
            start, parked = self.pose(robot), self.pose(peer)
            result = routes.diagnose(start, goal, parked)
            n.event('BLOCKAGE_DIAGNOSIS', robot=robot or 'robot1', peer=peer or 'robot1',
                    classification=result['classification'], baseline=result.get('baseline'),
                    with_peer=result.get('with_peer'), evidence='static_map_body_overlay')
            if result['classification'] == 'peer_blocks_static_route':
                self.context = dict(requester=robot, peer=peer, goal=goal,
                                    routes=routes, path=result['path'], start=start, parked=parked)
                return {}
        raise RuntimeError('No confirmed idle-peer blockage')

    def select(self):
        n = self.node; n.check_owner(); c = self.context
        if c is None:raise RuntimeError('No peer blockage selected')
        settings = n.c['yielding']
        candidate = c['routes'].yielding_pose(c['parked'], c['start'], c['path'], settings)
        if candidate is None:raise RuntimeError('No bounded off-route yielding pose')
        goal = [candidate['x'], candidate['y'], c['parked'][2]]
        peer = c['peer']
        # Failed attempts are bounded even when native verification rejects the proposal.
        self.attempts[peer] = self.attempts.get(peer, 0)+1
        terminal = n.action(n.plan[peer], ComputePathToPose.Goal(
            goal=n.goal_pose(goal), planner_id='GridBased'), 15.)
        result = terminal.result
        if (terminal.status != 4 or result.error_code != 0 or not result.path.poses
                or result.path.header.frame_id != 'map'):
            raise RuntimeError('Native planner rejected yielding pose')
        # Native planning and execution retain the live peer-footprint layer.
        c['yield_goal'] = goal
        n.event('YIELD_SELECTED', robot=peer or 'robot1', requester=c['requester'] or 'robot1', goal=goal)
        return {}

    def dispatch(self):
        n = self.node; n.check_owner(); c = self.context
        if c is None or 'yield_goal' not in c:raise RuntimeError('No verified yielding goal')
        if not self.stationary(c['requester']) or not self.idle(c['peer']):
            raise RuntimeError('Yield dispatch requires stopped requester and idle peer')
        peer = c['peer']
        if peer not in self.navigation:
            from rclpy.action import ActionClient
            # Yielding is position-only navigation, not final rescue alignment.
            self.navigation[peer] = ActionClient(n, NavigateToPose,
                '/'+(peer+'/' if peer else '')+'navigation/yield_to_pose')
        client = self.navigation[peer]
        n.wait(client.server_is_ready, 10., True)
        n.begin_yield(peer)
        sent = client.send_goal_async(NavigateToPose.Goal(pose=n.goal_pose(c['yield_goal'])))
        try:n.wait(sent.done, 10., True)
        except RuntimeError as cause:
            # An acceptance timeout does not prove that the robot stayed idle.
            # Cancel a late acceptance and stop task retries until task cleanup.
            def cancel_late(future):
                handle = future.result()
                if handle.accepted:
                    n.handles.append(handle)
                    handle.cancel_goal_async()
            sent.add_done_callback(cancel_late)
            error = RuntimeError('Yield goal acceptance unknown'); error.category = 'fatal'
            raise error from cause
        handle = sent.result()
        if not handle.accepted:raise RuntimeError('Yield navigation rejected')
        self.pending[peer] = (handle, handle.get_result_async(), c['yield_goal'])
        n.handles.append(handle)
        c['deadline'] = time.monotonic()+n.c['yielding']['yield_navigation_timeout_s']
        n.event('YIELD_STARTED', robot=peer or 'robot1', requester=c['requester'] or 'robot1', goal=c['yield_goal'])
        return dict(complete=False)

    def poll(self):
        n = self.node; n.check_owner(); c = self.context
        if c is None:raise RuntimeError('No active yielding operation')
        peer = c['peer']; handle, future, _ = self.pending[peer]
        if not future.done():
            if time.monotonic() >= c['deadline']:
                canceled = handle.cancel_goal_async()
                try:n.wait(lambda:canceled.done() and future.done(), 8., True)
                except RuntimeError as cause:
                    error = RuntimeError('Yield cancellation terminal unknown'); error.category = 'fatal'; raise error from cause
                del self.pending[peer]
                if handle in n.handles:n.handles.remove(handle)
                self.context = None
                n.event('YIELD_RESULT', robot=peer or 'robot1', status=future.result().status,
                        success=False, reason='navigation timeout; cancellation confirmed')
                raise RuntimeError('Yield navigation timed out')
            return dict(complete=False)
        terminal = future.result()
        if terminal.status == 5:
            del self.pending[peer]
            if handle in n.handles:n.handles.remove(handle)
            error = RuntimeError('Yield navigation canceled externally'); error.category = 'fatal'; raise error
        after = self.pose(peer)
        progress = math.dist(after[:2], c['parked'][:2])
        # Action results and TF travel independently. Keep the committed slot
        # while the final measured movement arrives, within the data-age budget.
        terminal_seen = c.setdefault('terminal_seen', time.monotonic())
        if (terminal.status == 4 and terminal.result.error_code == 0 and progress < .05
                and time.monotonic()-terminal_seen < n.c['max_data_age_s']):
            return dict(complete=False)
        del self.pending[peer]
        if handle in n.handles:n.handles.remove(handle)
        released = c['routes'].route(self.pose(c['requester']), c['goal'], after)['status'] == 'reachable'
        okay = terminal.status == 4 and terminal.result.error_code == 0 and progress >= .05 and released
        n.event('YIELD_RESULT', robot=peer or 'robot1', status=terminal.status,
                error=terminal.result.error_code, distance_m=progress, route_released=released, success=okay)
        self.context = None
        if not okay:raise RuntimeError('Yield did not restore the requester route')
        return dict(complete=True)
