"""Dispatch distinct normal targets concurrently using independent rescue checks."""
import json
import math
import time
from pathlib import Path

import yaml

from edgeai_tasks.exploration import ExplorationUnits
from edgeai_tasks.rescue_approach import approaches
from edgeai_tasks.rescue_policy import select_goal
from edgeai_tasks.cooperative_policy import footprint_clear, separated


class FleetRescueUnits(ExplorationUnits):
    task_name = 's1'
    event_prefix = 'FLEET_RESCUE_'
    operations = ['acquire', 'select_target', 'check_rescue', 'record_success']

    def __init__(self, c, unit_factory):
        settings = yaml.safe_load((Path(c['directory'])/'rescue-config.yaml').read_text())
        fleet = c.get('fleet')
        if fleet:
            fleet = dict(fleet, mapping_robots=fleet['task_robots']['s1'])
        super().__init__(dict(c, **({'fleet': fleet} if fleet else {}),
            exploration_timeout_s=settings['wall_timeout_s'],
            exploration_assignment=dict(c.get('exploration_assignment', {}),
                                        **c.get('single_rescue_assignment', {}))))
        self.children = {r:unit_factory('s1', dict(c, robot=r)) for r in self.robots}
        self.data_nodes = [self.node]+[u.node for u in self.children.values()]
        self.held_assignments = {}
        self.remaining = {}
        self.completed = {}
        self.skipped = {}
        self.waiting_since = {}
        self.candidates = {r:[] for r in self.robots}
        self.work_deadline = self.deadline - min(10., settings['wall_timeout_s']/4)
        geometry = yaml.safe_load((self.directory/'nav2-runtime.yaml').read_text())[
            'global_costmap']['global_costmap']['ros__parameters']
        self.footprint = json.loads(geometry['footprint'])
        self.padding = float(geometry.get('footprint_padding', .01))
        self.pump_peers = lambda:self._pump_except(self.node)
        for u in self.children.values():
            u.node.deadline = self.deadline
            u.node.pump_peers = lambda node=u.node:self._pump_except(node)

    def _pump_except(self, active):
        import rclpy
        for node in self.data_nodes:
            if node is not active:
                for _ in range(8):rclpy.spin_once(node, timeout_sec=0.)
        if getattr(self.node, 'stop_requested', False):
            for u in self.children.values():u.node.stop_requested = True
            return
        # A synchronous preplan/correction for one robot must not create a gap
        # in the other robot's stationary dwell samples.
        wall = time.monotonic()
        for robot, entry in self.held_assignments.items():
            child = self.children[robot]
            if (self.closing or child.node is active or robot in self.pending or child.condition_complete
                    or entry['record']['state'] != 'CHECKING' or 'check_error' in entry
                    or wall < entry.get('next_peer_check', 0.)):
                continue
            entry['next_peer_check'] = wall + .1
            try:
                child.invoke('check_rescue')
            except Exception as error:
                entry['check_error'] = error

    def _save(self):
        path = self.directory/'s1-assignment-result.json'
        tmp = path.with_suffix('.tmp')
        tmp.write_text(json.dumps(dict(success=self.success, reason=self.fatal_reason,
            assignments=self.assignments, failures=self.failures, failure_history=self.policy.history,
            completed_targets=list(self.completed.values()), skipped_targets=list(self.skipped.values())), indent=2))
        tmp.replace(path)

    def snapshots(self):
        # Camera/search and frontier snapshots do not determine rescue completion.
        return getattr(self, 'candidates', {r:[] for r in self.robots})

    def pose(self, robot):
        from rclpy.time import Time
        from tf2_ros import TransformException
        pose = super().pose(robot)
        if pose is None:return None
        try:
            q = self.buffer.lookup_transform('map', robot+'/base_footprint', Time()).transform.rotation
            return dict(pose, yaw=math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        except TransformException:
            return None

    def acquire(self):
        result = super().acquire()
        self._pump_except(self.node)
        for r, u in self.children.items():
            u.has_control = True
            u.node.expected_sequence = self.expected[r]
            u.node.navigation_started = self.now_ns()/1e9
        return result

    def select_targets(self):
        self.check_owner()
        node = self.children[self.robots[0]].node
        required = [t['id'] for t in self.c.get('required_targets', []) if t['type'] == 'normal']
        if not required or node.c['goal_source'] == 'map_pose':
            goal = node.select_current_goal()
            self.remaining[goal['target_id']] = goal
        else:
            registry = node.read_target_registry(allow_handover=True) or {}
            for target_id in required:
                try:
                    self.remaining[target_id] = select_goal(dict(node.c, target_id=target_id), registry)
                except ValueError as error:
                    self.skip(target_id, str(error))
        return dict(target_ids=list(self.remaining))

    def skip(self, target_id, reason):
        self.remaining.pop(target_id, None)
        self.waiting_since.pop(target_id, None)
        self.skipped[target_id] = dict(target_id=target_id, reason=reason)
        self.event('TARGET_SKIPPED', target_id=target_id, reason=reason)

    @staticmethod
    def pose_values(goal):
        return [goal[k] for k in ('x', 'y', 'yaw')]

    def compatible(self, a, b):
        return separated(self.pose_values(a), self.pose_values(b), self.footprint, self.padding)

    def navigation_client(self, robot, goal):
        return self.children[robot].node.rescue_client

    def action_succeeded(self, robot, entry):
        child = self.children[robot]
        child.navigation_after = self.now_ns()/1e9
        child.reset_dwell()

    def _failure(self, robot, entry, reason):
        super()._failure(robot, entry, reason)
        entry['failure_charged'] = True

    def release(self, robot, *, reason=None):
        entry = self.held_assignments[robot]
        child = self.children[robot]
        try:
            child.cancel_navigation(True, shutting_down=True)
        except Exception as error:
            self.fail(f'{robot}: rescue action terminal result unknown: {error}')
        if reason and not entry.get('failure_charged'):
            self._failure(robot, entry, reason)
        del self.held_assignments[robot]

    def start_rescue(self, robot, goal, pose, options):
        child = self.children[robot]
        node = child.node
        child.goal = dict(goal)
        child.last = None
        child.reset_dwell()
        child.alignment_started = None
        node.navigation_started = self.now_ns()/1e9
        node.navigation_attempts = []
        node.alignment_attempts = []
        node.approach_round_start = len(node.approach_attempts)
        node.approach_transient_failures = 0
        entry = dict(goal=dict(goal), record=dict(kind='rescue', state='SELECTED'))
        self.held_assignments[robot] = entry
        try:
            updated = node.latest_goal(goal)
            if node.goal_changed(goal, updated):
                self.release(robot)
                return
            if node.c.get('approach_selection', {}).get('enabled'):
                child.goal.update(node.choose_approach(goal, started=node.navigation_started, candidates=options))
            child.node.rescue_goal = child.goal
            self._send(robot, dict(child.goal), pose, 'rescue')
            self.held_assignments[robot] = self.pending[robot]
            node.navigation_attempts.append(self.pending[robot]['record'])
        except Exception as error:
            self.check_owner()
            if getattr(error, 'category', '') == 'fatal':
                self.fail(str(error))
            self.release(robot, reason=str(error))

    def dispatch_rescues(self):
        wall = time.monotonic()
        reserved = {r: entry['goal'] for r, entry in self.held_assignments.items()}
        targets = [g for key, g in self.remaining.items()
                   if all(other['target_id'] != key for other in reserved.values())]
        if not targets:
            return
        poses = {r: self.pose(r) for r in self.robots}
        ready = {r: p for r, p in poses.items() if p is not None and r not in reserved
                 and self.states[r].get('phase') == 'IDLE' and self.now_ns() >= self.settle_until.get(r, 0)
                 and self.children[r].node.rescue_client.server_is_ready()}
        candidates = {r: [] for r in self.robots}
        if self.map is not None and all(p is not None for p in poses.values()):
            for r in ready:
                node = self.children[r].node
                for goal in targets:
                    if self.policy.eligibility(r, goal, wall) != 'eligible':
                        continue
                    try:
                        nominal = node.latest_goal(goal)
                    except ValueError:
                        continue
                    options = approaches(nominal, node.c) if node.c.get('approach_selection', {}).get('enabled') else [nominal]
                    candidates[r].extend(g for g in options
                        if footprint_clear(self.map, self.pose_values(g), self.footprint, self.padding)
                        and all(self.compatible(g, p) for peer, p in poses.items() if peer != r))
        self.candidates = candidates
        chosen = self.policy.allocate_rescues(candidates, ready, reserved, wall, self.compatible)
        for robot, goal in chosen.items():
            other_goals = [g for peer, g in (reserved | chosen).items() if peer != robot]
            options = [goal] + [g for g in candidates[robot] if g is not goal
                       and g['target_id'] == goal['target_id']
                       and all(self.compatible(g, other) for other in other_goals)]
            self.waiting_since.pop(goal['target_id'], None)
            self.start_rescue(robot, goal, ready[robot], options)
            # Reserve the approach actually selected by preplanning.
            reserved[robot] = self.held_assignments.get(robot, {}).get('goal', goal)
            chosen[robot] = reserved[robot]
        assigned_ids = {entry['goal']['target_id'] for entry in self.held_assignments.values()}
        for goal in targets:
            target_id = goal['target_id']
            if target_id in assigned_ids:
                continue
            if all(self.policy.eligibility(r, goal, wall) == 'exhausted' for r in self.robots):
                self.skip(target_id, 'All robots exhausted the rescue retry budget')
            elif self.held_assignments:
                self.waiting_since.pop(target_id, None)
            elif wall - self.waiting_since.setdefault(target_id, wall) >= self.settings['assignment_timeout_s']:
                self.skip(target_id, 'No feasible available rescue assignment before timeout')

    def tick_rescues(self):
        self._advance()
        self.check_owner()
        if time.monotonic() >= self.work_deadline:
            self.cancel_all_navigation()
            for robot in list(self.held_assignments):
                self.release(robot)
            for target_id in list(self.remaining):
                self.skip(target_id, 'Normal rescue stage time budget exhausted')
        for robot, entry in list(self.held_assignments.items()):
            child = self.children[robot]
            node = child.node
            try:
                if robot in self.pending:
                    if entry['cancel_reason'] is None:
                        try:
                            node.check_navigation_budget(node.navigation_started)
                        except RuntimeError as error:
                            self._cancel(entry, str(error))
                        else:
                            try:
                                updated = node.latest_goal(child.goal)
                            except ValueError:
                                continue  # Keep the last valid estimate during a registry update.
                            if node.goal_changed(child.goal, updated):
                                self._cancel(entry, 'target_updated')
                    continue
                state = entry['record']['state']
                if state == 'CANCELED':
                    self.release(robot)
                    continue
                if state == 'FAILED':
                    # A recovered navigation can abort after entering the region.
                    # Keep the rescue only if current measurements prove entry.
                    if str(entry['record'].get('reason', '')).endswith('Goal rejected'):
                        self.release(robot)
                        continue
                    try:
                        child.invoke('inside_rescue_region')
                    except Exception as error:
                        if getattr(error, 'category', '') == 'fatal':
                            raise
                        self.release(robot)
                        continue
                    child.navigation_after = self.now_ns()/1e9
                    child.reset_dwell()
                entry['record']['state'] = 'CHECKING'
                if 'check_error' in entry:
                    raise entry.pop('check_error')
                result = dict(complete=True) if child.condition_complete else child.invoke('check_rescue')
                if result['complete']:
                    target_id = child.goal['target_id']
                    child.result_path = self.directory/f'rescue-target-{target_id}.json'
                    child.invoke('record_success')
                    self.completed[target_id] = json.loads(child.result_path.read_text())
                    self.remaining.pop(target_id)
                    self.release(robot)
                    self.event('TARGET_COMPLETED', robot=robot, target_id=target_id)
                elif result.get('needs_correction'):
                    entry['record']['state'] = 'CORRECTING'
                    child.invoke('correct')
                    entry['record']['state'] = 'CHECKING'
            except Exception as error:
                self.check_owner()
                if getattr(error, 'category', '') == 'fatal':
                    self.fail(str(error))
                if robot in self.pending:
                    self._cancel(entry, str(error))
                else:
                    self.release(robot, reason=str(error))
        self.dispatch_rescues()
        return dict(complete=not self.remaining, inflight=len(self.held_assignments),
                    rescued=len(self.completed), skipped=len(self.skipped))

    def record_success(self):
        if self.remaining or self.held_assignments:
            self.fail('Normal rescue targets or actions still outstanding')
        all_rescued = not self.skipped
        # In an integrated mission, finish the normal stage and let the C1 tree
        # handle any available cooperative target. The per-target ledger retains failures.
        can_continue = self.c.get('post_mapping_localization') and self.c.get('cooperative')
        if not all_rescued and not can_continue:
            self.fail('Normal rescue targets exhausted or skipped')
        results = list(self.completed.values())
        summary = dict(results[0]) if len(results) == 1 else {}
        summary.update(success=all_rescued, stage_completed=True, target_results=results,
                       skipped_targets=list(self.skipped.values()))
        path = self.directory/'rescue-result.json'
        temporary = path.with_suffix('.tmp')
        temporary.write_text(json.dumps(summary, indent=2))
        temporary.replace(path)
        self.success = True
        self._save()
        self.event('COMPLETED', rescued=len(results), skipped=len(self.skipped))
        return dict(complete=True)

    def invoke(self, op):
        return {'acquire':self.acquire, 'select_target':self.select_targets,
                'check_rescue':self.tick_rescues, 'record_success':self.record_success}[op]()

    def close(self):
        if self.closed:
            return
        errors = []
        for r, child in self.children.items():
            child.node.closing = True
            child.node.deadline = max(self.deadline, time.monotonic()+self.settings['cancel_timeout_s'])
            try:
                child.cancel_navigation(True, shutting_down=True)
            except Exception as error:
                errors.append(f'{r}: {error}')
        try:
            if errors:
                self.success = False
                self.fatal_reason = 'Rescue adapter cleanup failed: '+'; '.join(errors)
            super().close()
        finally:
            for child in self.children.values():
                child.node.destroy_node()
        if errors:
            raise RuntimeError(self.fatal_reason)
