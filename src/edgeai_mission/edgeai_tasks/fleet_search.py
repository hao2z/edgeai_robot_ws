"""Task-layer camera search: shared reservations, independent robot actions.

Candidate geometry stays in edgeai_search. This coordinator reuses the fleet
ownership/terminal-state/retry machinery, and credits only completed scans.
"""
import copy
import json
import math
import time
from types import SimpleNamespace
from pathlib import Path

import yaml

from edgeai_tasks.exploration import ExplorationUnits, ROBOTS
from edgeai_tasks.search_candidates import make_candidates


def clone_candidates(planner):
    """Copy selection state, sharing immutable grids and expensive view features."""
    result = copy.copy(planner)
    for name in ('attempted', 'covered', 'used', 'used_positions'):
        if hasattr(planner, name):setattr(result, name, copy.copy(getattr(planner, name)))
    if hasattr(planner, 'baseline'):result.baseline = clone_candidates(planner.baseline)
    return result


def commit_candidate(base, before, after):
    """Commit only one offered view; unselected proposals consume no visits."""
    result=clone_candidates(base)
    if hasattr(after,'baseline'):
        result.baseline=commit_candidate(base.baseline,before.baseline,after.baseline)
        result.used.extend(v for v in after.used if v not in before.used)
        result.used_positions.extend(v for v in after.used_positions if v not in before.used_positions)
        result.visits+=after.visits-before.visits
        result.active_id=after.active_id
    elif hasattr(after,'attempted'):
        result.attempted.update(after.attempted-before.attempted)
    return result


def missing_targets(registry, required, require_confirmation=False):
    valid = {(t.get('id'), t.get('type')) for t in (registry or {}).get('targets', [])
             if t.get('yaw') is not None and all(math.isfinite(t.get(k, math.nan)) for k in ('x','y','yaw'))
             and (not require_confirmation or t.get('confirmed', False))}
    if (registry or {}).get('frame_id') != 'map':valid = set()
    return [t for t in required if (t['id'], t['type']) not in valid]


class FleetSearchUnits(ExplorationUnits):
    task_name = 'find_targets'
    event_prefix = 'FLEET_SEARCH_'

    def __init__(self, c):
        from rclpy.action import ActionClient
        from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo
        from std_msgs.msg import String
        from nav2_msgs.action import NavigateToPose, Spin
        settings = yaml.safe_load((Path(c['directory'])/'rescue-config.yaml').read_text())
        merged = dict(c, exploration_timeout_s=settings['wall_timeout_s'],
                      exploration_assignment=dict(c.get('exploration_assignment', {}), **c.get('search_assignment', {})))
        super().__init__(merged)
        self.search = settings
        self.registry = None
        self.infos, self.suspects, self.planners, self.offers = {}, {}, {}, {}
        self.waiting_sweeps, self.sweep_attempts = {}, []
        self.visited = []
        self.visits = {r: 0 for r in self.robots}
        self.retry_goals = {r: [] for r in self.robots}
        self.started_ns = None
        self.heading, self.spin = {}, {}
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscriptions.append(self.node.create_subscription(String, '/targets/estimates', self.targets, qos))
        for robot in self.robots:
            prefix = '/'+robot
            self.nav[robot].destroy()
            self.nav[robot] = ActionClient(self.node, NavigateToPose, prefix+'/navigate_to_pose')
            self.heading[robot] = ActionClient(self.node, NavigateToPose, prefix+'/navigation/heading_to_pose')
            self.spin[robot] = ActionClient(self.node, Spin, prefix+'/navigation_backend/spin')
            self.subscriptions.append(self.node.create_subscription(CameraInfo, '/'+robot+'/camera/camera_info',
                lambda msg, r=robot: self.infos.update({r: msg}), qos_profile_sensor_data))
            topic = '/'+robot+'/targets/suspected'
            self.subscriptions.append(self.node.create_subscription(String, topic,
                lambda msg, r=robot: self.suspects.update({r: json.loads(msg.data)}), qos))

    def targets(self, msg):
        try:self.registry = json.loads(msg.data)
        except ValueError:self.registry = None

    def missing(self):
        return missing_targets(self.registry, self.c['required_targets'],
                               self.c.get('target_observation_require_confirmation', False))

    def pose(self, robot):
        from rclpy.time import Time
        from tf2_ros import TransformException
        pose = super().pose(robot)
        if pose is not None:
            try:q = self.buffer.lookup_transform('map', robot+'/base_footprint', Time()).transform.rotation
            except TransformException:return None
            pose['yaw'] = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        return pose

    def adapter(self, robot):
        # Both sources contribute hints; prefix ids to avoid detector-local id collisions.
        hints = []
        now = self.now_ns()/1e9
        for source, payload in self.suspects.items():
            if payload.get('frame_id') == 'map' and 0 <= now-payload.get('stamp', -100) < 2.:
                hints.extend(dict(h, id=source+':'+str(h['id'])) for h in payload.get('candidates', []))
        return SimpleNamespace(c=self.search, buffer=self.buffer, search_base_frame=robot+'/base_footprint',
            search_camera_info=self.infos.get(robot), search_map=self.map,
            search_suspects=dict(frame_id='map', stamp=now, candidates=hints),
            get_clock=self.node.get_clock, read_target_registry=lambda **_: self.registry,
            event=lambda code, description: self.event(code, robot=robot, description=description))

    def navigation_client(self, robot, goal):
        return self.heading[robot] if goal.get('require_heading') else self.nav[robot]

    def snapshots(self):
        return {r: [] for r in self.robots}  # Search completion depends on targets, never empty frontiers.

    def _report(self, snapshots):
        pass

    def action_succeeded(self, robot, entry):
        kind = entry['record']['kind']
        if kind == 'search':self.waiting_sweeps[robot] = entry['goal']
        elif kind == 'sweep':
            pose = self.pose(robot)
            if pose is not None:
                actual = (pose['x'], pose['y'], pose['yaw'])
                for planner in self.planners.values():planner.record_sweep(actual)
                self.visited.append(dict(x=pose['x'], y=pose['y']))
            self.waiting_sweeps.pop(robot, None)
            self.retry_goals[robot] = [g for g in self.retry_goals[robot] if g != entry['goal']]
        else:super().action_succeeded(robot, entry)

    def _failure(self, robot, entry, reason):
        super()._failure(robot, entry, reason)
        if entry['record']['kind'] in ('search','sweep') and entry['goal'] not in self.retry_goals[robot]:
            self.retry_goals[robot].append(entry['goal'])
        if entry['record']['kind'] == 'sweep':self.waiting_sweeps.pop(robot, None)

    def start_sweep(self, robot, goal):
        from nav2_msgs.action import Spin
        if not self.spin[robot].server_is_ready():return False
        allowance = float(self.search.get('search_spin_timeout_s', 30.))
        request = Spin.Goal(target_yaw=2*math.pi)
        request.time_allowance.sec = int(allowance)
        request.time_allowance.nanosec = int((allowance-int(allowance))*1e9)
        record = dict(robot=robot, kind='sweep', goal=goal, sent_ns=self.now_ns(), state='SENDING')
        sent = self.spin[robot].send_goal_async(request)
        self.sweep_attempts.append(record)
        self.pending[robot] = dict(goal=goal, sent=sent, handle=None, result=None, sent_at=time.monotonic(),
            cancel_at=None, cancel=None, cancel_reason=None, record=record)
        self.event('SWEEP_STARTED', robot=robot, goal=goal)
        return True

    def poll(self):
        self._advance()
        self.check_owner()
        if self.started_ns is None:self.started_ns = self.now_ns()
        if not self.missing():
            self.cancel_all_navigation()
            self.waiting_sweeps.clear()
            self.condition_complete = True
            return dict(complete=True)
        if (self.now_ns()-self.started_ns)/1e9 >= self.search.get('search_timeout_s',120.):
            self.fail('Fleet target search simulation budget exhausted; missing '+str(self.missing()))
        for robot, entry in self.pending.items():
            limit = (self.search.get('search_spin_timeout_s',30.) if entry['record']['kind']=='sweep'
                     else self.search.get('search_navigation_timeout_s',20.))
            if (self.now_ns()-entry['record']['sent_ns'])/1e9 >= limit:
                self._cancel(entry, 'Search action simulation timeout')
        for robot, goal in list(self.waiting_sweeps.items()):
            if (robot not in self.pending and self.states[robot].get('phase') == 'IDLE'
                    and self.now_ns() >= self.settle_until.get(robot, 0)):
                self.start_sweep(robot, goal)
        return dict(complete=False, missing=self.missing(), inflight=len(self.pending))

    def dispatch(self):
        self.check_owner()
        if self.condition_complete:return {}
        if self.map is None:return {}
        q = self.map.info.origin.orientation
        if self.map.header.frame_id != 'map' or abs(q.x)+abs(q.y)+abs(q.z)>1e-6:
            self.fail('Fleet target search requires an axis-aligned map')
        candidates, poses, self.offers = {}, {}, {}
        occupied = [p['goal'] for p in self.pending.values()]+list(self.waiting_sweeps.values())
        def blocked(goal):
            spacing = (self.search.get('search_suspects',{}).get('min_view_spacing_m',.10)
                       if goal.get('strategy')=='suspected_panel' else self.policy.radius)
            return (any(math.hypot(goal['x']-p['x'],goal['y']-p['y'])<spacing for p in self.visited)
                    or any(math.hypot(goal['x']-p['x'],goal['y']-p['y'])<self.policy.radius for p in occupied))
        for robot in self.robots:
            if robot in self.pending or robot in self.waiting_sweeps:continue
            if self.states[robot].get('phase')!='IDLE' or self.now_ns()<self.settle_until.get(robot,0):continue
            pose = self.pose(robot)
            if pose is None or robot not in self.infos:continue
            adapter = self.adapter(robot)
            if robot not in self.planners:
                self.planners[robot] = make_candidates(adapter, self.map, pose)
                if self.search.get('search_suspects',{}).get('enabled'):
                    from edgeai_search.suspect_search import ParallelCandidates
                    self.planners[robot] = ParallelCandidates(self.planners[robot], adapter)
            planner = clone_candidates(self.planners[robot])
            if hasattr(planner,'node'):planner.node = adapter
            offers = []
            for _ in range(self.search.get('search_frustum',{}).get('candidate_pool_size',24)+8):
                before=clone_candidates(planner)
                goal = planner.next_view((pose['x'],pose['y'],pose['yaw']))
                if goal is None:break
                selected = commit_candidate(self.planners[robot],before,planner)
                if hasattr(planner,'baseline'):
                    # Enumerating proposals advances identities, not actual visit budgets
                    # or observation spacing. Only dispatch commits those changes.
                    planner.visits = before.visits
                    planner.used_positions = list(before.used_positions)
                if blocked(goal):continue
                if self.policy.eligibility(robot,goal,time.monotonic())!='eligible':continue
                offers.append(goal)
                self.offers[robot,goal['x'],goal['y']] = selected
                if len(offers)>=4:break
            candidates[robot], poses[robot] = offers, pose
            if self.visits[robot]>=self.search.get('search_max_positions',6):candidates[robot]=[]
            candidates[robot].extend(g for g in self.retry_goals[robot]
                if self.policy.eligibility(robot,g,time.monotonic())=='eligible'
                and not blocked(g))
        # Suspect views finish before baseline views whenever any robot can inspect one.
        if (any(g.get('strategy')=='suspected_panel' for gs in candidates.values() for g in gs)
                or any(g.get('strategy')=='suspected_panel' for g in
                       [p['goal'] for p in self.pending.values()]+list(self.waiting_sweeps.values()))):
            candidates = {r:[g for g in gs if g.get('strategy')=='suspected_panel'] for r,gs in candidates.items()}
        reserved = {r:p['goal'] for r,p in self.pending.items()}
        for robot, goal in self.policy.allocate(candidates,poses,reserved,time.monotonic()).items():
            if not self.navigation_client(robot,goal).server_is_ready():continue
            # Commit only the selected proposal. Examining alternatives never consumes real state.
            if goal in self.retry_goals[robot]:self.retry_goals[robot].remove(goal)
            else:
                selected = self.offers[robot,goal['x'],goal['y']]
                # A proposal may predate the other robot's completed sweep.
                # Preserve shared coverage while committing this robot's visit.
                current = self.planners[robot]
                old_base = getattr(current, 'baseline', current)
                new_base = getattr(selected, 'baseline', selected)
                if hasattr(new_base, 'covered'):
                    new_base.covered.update(old_base.covered)
                if hasattr(new_base, 'attempted'):
                    new_base.attempted.update(old_base.attempted)
                self.planners[robot] = selected
                self.visits[robot] += 1
            self._send(robot,goal,poses[robot],'search')
        if (not self.pending and not self.waiting_sweeps and
            all(self.visits[r]>=self.search.get('search_max_positions',6) for r in self.robots)
            and not any(self.policy.eligibility(r,g,time.monotonic())!='exhausted'
                        for r in self.robots for g in self.retry_goals[r])):
            self.fail('Fleet observation visit budgets exhausted; missing '+str(self.missing()))
        return dict(complete=False)

    def check_complete(self):
        return dict(complete=self.condition_complete and not self.pending)

    def _save(self):
        super()._save()
        if not hasattr(self,'sweep_attempts'):return
        path=self.directory/'target-search-result.json'
        data=json.loads(path.read_text())
        data.update(required_targets=self.c['required_targets'], missing=self.missing(),
                    search_attempts=self.assignments, sweeps=self.sweep_attempts, visits=self.visits)
        tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(data,indent=2));tmp.replace(path)
