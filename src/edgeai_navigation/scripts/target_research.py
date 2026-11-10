"""Bounded camera search through authority-owned navigation in the known map."""
import json
from collections import deque
import math
import time

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose, Spin
from tf2_ros import TransformException
from search_candidates import make_candidates


def search_target(node):
    c = node.c
    started = node.get_clock().now().nanoseconds/1e9
    def expired():
        return (node.get_clock().now().nanoseconds/1e9-started >=
                c.get('search_timeout_s', 120.0) or time.monotonic() >= node.deadline)
    def found():
        try:
            return node.select_current_goal()
        except ValueError as exc:
            if 'No eligible localized ' not in str(exc):
                raise
            return None
    node.event('SEARCH_STARTED', '未定位救援目标，开始在已知空闲区域移动并连续转圈观察')
    pose = None
    def ready():
        nonlocal pose
        if node.search_map is None:
            return False
        try:
            _, pose = node.fresh_measurement(node.get_clock().now().nanoseconds/1e9,
                                            dict(x=0., y=0., yaw=0.))
            return True
        except (ValueError, TransformException):
            return False
    node.wait(ready, timeout=10., check_owner=True)
    grid = node.search_map
    q = grid.info.origin.orientation
    if abs(q.x)+abs(q.y)+abs(q.z) > 1e-6 or grid.header.frame_id != 'map':
        raise RuntimeError('Target search requires an axis-aligned map frame')
    candidates = make_candidates(node,grid,pose)
    if c.get('search_suspects',{}).get('enabled',False):
        from suspect_search import ParallelCandidates
        candidates=ParallelCandidates(candidates,node)
        node.event('SEARCH_SUSPECTS_ENABLED','优先完成当前疑似面板的可用观察方向，再继续视锥搜寻；共享导航槽和搜寻预算')
        # Receive retained mapping hints before choosing the first baseline view.
        grace=time.monotonic()+1.5
        node.wait(lambda:node.search_suspects is not None or time.monotonic()>=grace,
                  timeout=2.,check_owner=True)
    node.event('SEARCH_POSITIONS', json.dumps(dict(robot_map=[pose['x'], pose['y']],
        clearance_m=c.get('search_clearance_m', .40), count=len(candidates.positions),
        positions=candidates.positions, visit_limit=c.get('search_max_positions',6))))
    if not candidates.positions and not c.get('search_suspects',{}).get('enabled',False):
        raise RuntimeError('No known reachable observation positions with search clearance')
    for _ in range(c.get('search_max_positions',6)):
        # Candidate yaw is a view-scoring reference, not an arrival constraint.
        # Position-only navigation is followed by the existing full Spin.
        node.wait(ready,timeout=5.,check_owner=True)
        _, current_pose = node.fresh_measurement(node.get_clock().now().nanoseconds/1e9,
                                                dict(x=0., y=0., yaw=0.))
        candidate=candidates.next_view((current_pose['x'],current_pose['y'],current_pose['yaw']))
        if candidate is None:
            break
        node.event('SEARCH_CANDIDATE',json.dumps(candidate))
        x,y=candidate['x'],candidate['y']
        views = deque([(candidate['yaw'], 0)])
        while views:
            yaw, retries = views.popleft()
            goal = found()
            if goal is not None:
                node.event('SEARCH_FOUND', json.dumps(goal)); return goal
            if expired():
                raise RuntimeError('Target search simulation/wall budget exhausted')
            node.wait(lambda: node.state.get('phase') == 'IDLE' and
                      node.state.get('explorer_paused'), timeout=5., check_owner=True)
            request = NavigateToPose.Goal()
            request.pose.header.frame_id = 'map'
            request.pose.header.stamp = node.get_clock().now().to_msg()
            request.pose.pose.position.x=x; request.pose.pose.position.y=y
            request.pose.pose.orientation.z=math.sin(yaw/2)
            request.pose.pose.orientation.w=math.cos(yaw/2)
            client=node.heading_client if candidate.get('require_heading',False) else node.client
            node.wait(client.server_is_ready,timeout=5.,check_owner=True)
            sent = client.send_goal_async(request)
            node.wait(sent.done, timeout=10., check_owner=True)
            node.goal_handle = sent.result()
            record = dict(x=x, y=y, yaw=yaw, retry=retries, accepted=node.goal_handle.accepted,
                          started_sim_s=node.get_clock().now().nanoseconds/1e9,
                          candidate=dict(candidate), arrival_policy='heading' if candidate.get('require_heading',False) else 'position_only')
            node.search_attempts.append(record)
            node.event('SEARCH_VIEW', json.dumps(record))
            if not node.goal_handle.accepted:
                node.goal_handle=None
                record['outcome'] = 'rejected'
                break
            handle = node.goal_handle
            result = handle.get_result_async()
            step_started = node.get_clock().now().nanoseconds/1e9
            goal = None
            def interrupted():
                nonlocal goal
                goal = found()
                return (result.done() or goal is not None or expired() or
                        node.get_clock().now().nanoseconds/1e9-step_started >=
                        c.get('search_navigation_timeout_s', 20.0))
            node.wait(interrupted, timeout=c['wall_timeout_s'], check_owner=True)
            canceled = None
            if not result.done():
                record['cancel_reason'] = ('target_found' if goal is not None else
                    'search_budget' if expired() else 'view_timeout')
                canceled = handle.cancel_goal_async()
                # A cancel acknowledgement alone is not proof that motion ended.
                node.wait(result.done, timeout=10., check_owner=True)
            terminal = result.result()
            node.goal_handle = None
            record.update(status=terminal.status, error_code=terminal.result.error_code,
                          finished_sim_s=node.get_clock().now().nanoseconds/1e9)
            node.event('SEARCH_VIEW_RESULT', json.dumps(record))
            if terminal.status == GoalStatus.STATUS_CANCELED:
                if canceled is None:
                    raise RuntimeError('Target search canceled externally')
                node.wait(canceled.done, timeout=5., check_owner=True)
                if not any(bytes(g.goal_id.uuid) == bytes(handle.goal_id.uuid)
                           for g in canceled.result().goals_canceling):
                    raise RuntimeError('Target search cancellation not acknowledged')
            elif terminal.status not in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED):
                raise RuntimeError('Target search navigation terminal status unknown')
            node.wait(lambda: node.state.get('phase') == 'IDLE', timeout=5., check_owner=True)
            if goal is not None:
                goal = found()
                if goal is not None:
                    node.event('SEARCH_FOUND', json.dumps(goal)); return goal
            if expired():
                raise RuntimeError('Target search simulation/wall budget exhausted')
            if terminal.status != GoalStatus.STATUS_SUCCEEDED:
                if retries < c.get('navigation_max_retries', 2):
                    node.event('SEARCH_RETRY_WAIT', '观察导航未完成，等待空闲和新鲜TF后重试同一点')
                    node.wait_to_retry(dict(x=x, y=y, yaw=yaw), started)
                    views.appendleft((yaw, retries+1))
                    continue
                node.event('SEARCH_POSITION_SKIPPED', '观察导航重试耗尽，跳过该点转圈观察')
                break  # Try a different position after a failed observation approach.
            if not node.spin_client.wait_for_server(timeout_sec=5.):
                raise RuntimeError('Nav2 Spin server unavailable during target search')
            node.check_owner()
            allowance = c.get('search_spin_timeout_s', 30.0)
            request = Spin.Goal(target_yaw=2*math.pi)
            request.time_allowance.sec = int(allowance)
            request.time_allowance.nanosec = int((allowance-int(allowance))*1e9)
            sent = node.spin_client.send_goal_async(request)
            node.wait(sent.done, timeout=10., check_owner=True)
            node.goal_handle = sent.result()
            sweep = dict(x=x, y=y, action='spin', target_yaw=2*math.pi,
                         strategy=candidate['strategy'],suspect_id=candidate.get('suspect_id'),
                         accepted=node.goal_handle.accepted,
                         started_sim_s=node.get_clock().now().nanoseconds/1e9)
            record['spin'] = sweep
            node.event('SEARCH_SPIN', json.dumps(sweep))
            if not node.goal_handle.accepted:
                node.goal_handle = None
                continue
            handle = node.goal_handle
            result = handle.get_result_async()
            spin_started = node.get_clock().now().nanoseconds/1e9
            goal = None
            def scanned():
                nonlocal goal
                goal = found()
                return (result.done() or goal is not None or expired() or
                        node.get_clock().now().nanoseconds/1e9-spin_started >= allowance)
            node.wait(scanned, timeout=c['wall_timeout_s'], check_owner=True)
            canceled = None
            if not result.done():
                sweep['cancel_reason'] = ('target_found' if goal is not None else
                                         'search_budget' if expired() else 'spin_timeout')
                canceled = handle.cancel_goal_async()
                node.wait(result.done, timeout=10., check_owner=True)
            terminal = result.result()
            node.goal_handle = None
            sweep.update(status=terminal.status, error_code=terminal.result.error_code,
                         finished_sim_s=node.get_clock().now().nanoseconds/1e9)
            node.event('SEARCH_SPIN_RESULT', json.dumps(sweep))
            if terminal.status == GoalStatus.STATUS_CANCELED:
                if canceled is None:
                    raise RuntimeError('Target search spin canceled externally')
                node.wait(canceled.done, timeout=5., check_owner=True)
                if not any(bytes(g.goal_id.uuid) == bytes(handle.goal_id.uuid)
                           for g in canceled.result().goals_canceling):
                    raise RuntimeError('Target search spin cancellation not acknowledged')
            elif terminal.status not in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED):
                raise RuntimeError('Target search spin terminal status unknown')
            node.wait(lambda: node.state.get('phase') == 'IDLE', timeout=5., check_owner=True)
            if goal is not None:
                goal = found()
                if goal is not None:
                    node.event('SEARCH_FOUND', json.dumps(goal)); return goal
            if expired():
                raise RuntimeError('Target search simulation/wall budget exhausted')
            if terminal.status == GoalStatus.STATUS_SUCCEEDED and terminal.result.error_code == 0:
                try:
                    _, actual=node.fresh_measurement(node.get_clock().now().nanoseconds/1e9,
                                                     dict(x=x,y=y,yaw=yaw))
                    candidates.record_sweep((actual['x'],actual['y'],actual['yaw']))
                except (ValueError,TransformException):
                    node.event('SEARCH_COVERAGE_SKIPPED','转圈完成但无新鲜位姿，不计入几何覆盖')
    raise RuntimeError('Target search exhausted observation positions without eligible target')
