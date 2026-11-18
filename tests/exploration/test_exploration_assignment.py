"""ROS-independent scheduling, source validity, and persistent retry budgets."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'src/edgeai_mission'))
from edgeai_tasks.exploration import AssignmentPolicy, candidate_snapshot, shared_frontiers


def point(x, y=0.):
    return dict(x=float(x), y=float(y), size=1.)


class AssignmentTests(unittest.TestCase):
    def test_rescue_minimizes_longest_approach_and_keeps_targets_distinct(self):
        policy = AssignmentPolicy(dict(max_navigation_attempts=1))
        near, far = dict(point(1), target_id=1), dict(point(-2), target_id=2)
        poses = dict(robot1=point(0), robot2=point(3))
        candidates = {robot: [near, far] for robot in poses}
        compatible = lambda _a, _b: True
        self.assertEqual(policy.allocate_rescues(candidates, poses, {}, 0., compatible),
                         dict(robot1=far, robot2=near))
        policy.failed('robot1', dict(point(-2.2), target_id=2), 0., 'no path')
        self.assertEqual(policy.allocate_rescues(candidates, poses, {}, 1., compatible),
                         dict(robot1=near, robot2=far))

    def test_history_spacing_is_independent_of_concurrent_reservations(self):
        p=AssignmentPolicy(dict(history_match_radius_m=.1,max_navigation_attempts=1))
        p.failed('robot1',point(1),0.,'blocked')
        self.assertEqual(p.eligibility('robot1',point(1.05),1.),'suppressed')
        self.assertEqual(p.eligibility('robot1',point(1.2),1.),'eligible')
        self.assertEqual(p.allocate(dict(robot1=[point(1.2)]),dict(robot1=point(0)),
                                    dict(robot2=point(1)),1.),{})
        for radius in (0.,float('nan'),-1.):
            with self.assertRaises(ValueError):AssignmentPolicy(dict(history_match_radius_m=radius))

    def test_single_task_uses_same_nearest_rule_and_switches_robot_after_failure(self):
        from edgeai_tasks.fleet_assignment import AssignmentPolicy as SharedPolicy
        self.assertIs(SharedPolicy, AssignmentPolicy)
        policy = SharedPolicy(dict(max_navigation_attempts=1, reservation_radius_m=.15))
        candidates = dict(robot1=[point(2)], robot2=[point(2)])
        poses = dict(robot1=point(1), robot2=point(4))
        self.assertEqual(policy.allocate(candidates, poses, {}, 0., limit=1),dict(robot1=point(2)))
        policy.failed('robot1',point(2),0.,'no_valid_path')
        self.assertEqual(policy.allocate(candidates, poses, {}, 1., limit=1),dict(robot2=point(2)))
        self.assertEqual(policy.history['robot1'][0]['attempts'],1)

    def test_two_distinct_nearest_goals_and_deterministic_ties(self):
        p = AssignmentPolicy()
        candidates = {r: [point(1), point(4)] for r in ('robot1', 'robot2')}
        poses = dict(robot1=point(0), robot2=point(5))
        self.assertEqual(p.allocate(candidates, poses, {}, 0),
                         dict(robot1=point(1), robot2=point(4)))
        poses = dict(robot2=point(0), robot1=point(0))
        a = p.allocate(candidates, poses, {}, 0)
        self.assertEqual(a['robot1'], point(1))
        self.assertEqual(a['robot2'], point(4))
        self.assertEqual(a, p.allocate({r: list(reversed(v)) for r, v in candidates.items()}, poses, {}, 0))

    def test_active_and_sending_reservations_win_over_nearer_free_robot(self):
        p = AssignmentPolicy()
        candidates = dict(robot1=[point(1)], robot2=[point(1.1), point(10)])
        chosen = p.allocate(candidates, dict(robot1=point(0), robot2=point(1)),
                            dict(robot1=point(1)), 0)
        self.assertEqual(chosen, dict(robot2=point(10)))

    def test_no_geographic_partition_and_configured_endpoint_separation(self):
        p = AssignmentPolicy(dict(reservation_radius_m=.2))
        chosen = p.allocate(dict(robot1=[point(10)], robot2=[point(10.3)]),
                            dict(robot1=point(0), robot2=point(11)), {}, 0)
        self.assertEqual(len(chosen), 2)
        p = AssignmentPolicy(dict(reservation_radius_m=1.))
        chosen = p.allocate(dict(robot2=[point(10.3), point(15)]),
                            dict(robot2=point(11)), dict(robot1=point(10)), 0)
        self.assertEqual(chosen['robot2'], point(15))

    def test_neighborhood_failures_persist_and_are_per_robot(self):
        p = AssignmentPolicy(dict(failure_cooldown_s=10.))
        p.failed('robot1', point(1), 0, 'aborted')
        self.assertEqual(p.eligibility('robot1', point(1.2), 1), 'eligible')
        p.failed('robot1', point(1.2), 1, 'rejected')
        self.assertEqual(p.eligibility('robot1', point(1.4), 10), 'suppressed')
        self.assertEqual(p.eligibility('robot2', point(1.4), 2), 'eligible')
        self.assertEqual(p.eligibility('robot1', point(1.4), 11), 'eligible')
        p.failed('robot1', point(1.1), 11, 'aborted')
        p.failed('robot1', point(1.1), 12, 'aborted')
        self.assertEqual(p.eligibility('robot1', point(1.1), 10000), 'exhausted')
        self.assertEqual(p.history['robot1'][0]['attempts'], 4)
        self.assertEqual(p.allocate(dict(robot1=[point(1.1)]), dict(robot1=point(0)), {}, 10000), {})

    def test_suppression_does_not_block_distinct_farther_candidate(self):
        p = AssignmentPolicy()
        for _ in range(2):
            p.failed('robot1', point(1), 0, 'failed')
        chosen = p.allocate(dict(robot1=[point(1), point(8)]), dict(robot1=point(0)), {}, 1)
        self.assertEqual(chosen, dict(robot1=point(8)))

    def test_invalid_config(self):
        for settings in (dict(reservation_radius_m=0), dict(max_retry_rounds=1.5),
                         dict(failure_cooldown_s=float('nan'))):
            with self.assertRaises(ValueError):
                AssignmentPolicy(settings)

    def test_arrival_waits_for_map_update_and_bounds_repeated_endpoints(self):
        p = AssignmentPolicy(dict(arrival_cooldown_s=3., max_candidate_arrivals=2))
        p.reached('robot1', point(1), 0.)
        self.assertEqual(p.eligibility('robot1', point(1.1), 1.), 'suppressed')
        self.assertEqual(p.eligibility('robot2', point(1.1), 1.), 'eligible')
        self.assertEqual(p.eligibility('robot1', point(1.1), 3.), 'eligible')
        p.reached('robot1', point(1.1), 4.)
        self.assertEqual(p.eligibility('robot1', point(1), 100.), 'exhausted')
        self.assertEqual(p.allocate(dict(robot1=[point(1), point(8)]),
                                    dict(robot1=point(0)), {}, 100.), dict(robot1=point(8)))


class SnapshotTests(unittest.TestCase):
    def test_pool_is_deterministic_and_deduplicates_shared_endpoints(self):
        self.assertEqual(shared_frontiers(dict(robot1=[point(2),point(1)],robot2=[point(1)])),
                         [point(1),point(2)])

    def test_idle_car_can_take_other_exporters_goal_with_independent_retry_history(self):
        pool=shared_frontiers(dict(robot1=[],robot2=[point(2)]))
        p=AssignmentPolicy(dict(max_navigation_attempts=1))
        poses=dict(robot1=point(1),robot2=point(5))
        offers={r:pool for r in poses}
        self.assertEqual(p.allocate(offers,poses,{},0.),dict(robot1=point(2)))
        p.failed('robot1',point(2),0.,'unreachable')
        self.assertEqual(p.allocate(offers,poses,{},1.),dict(robot2=point(2)))

    def test_missing_source_can_supply_no_false_completion(self):
        self.assertIsNone(shared_frontiers(dict(robot1=None,robot2=[])))
        self.assertEqual(shared_frontiers(dict(robot1=None,robot2=[point(2)])),[point(2)])
        self.assertEqual(shared_frontiers(dict(robot1=[],robot2=[])),[])

    def snapshot(self, **changes):
        return dict(frame_id='map', stamp_ns=10**9, available=True, candidates=[], **changes)

    def test_empty_is_exhaustion_only_when_fresh_and_available(self):
        valid = self.snapshot()
        self.assertEqual(candidate_snapshot(valid, 2*10**9, 1.5), [])
        for update in (dict(available=False), dict(stamp_ns=0), dict(stamp_ns=3*10**9),
                       dict(frame_id='odom'), dict(stamp_ns=1.), dict(candidates=None)):
            self.assertIsNone(candidate_snapshot(dict(valid, **update), 2*10**9, 1.5))
        self.assertIsNone(candidate_snapshot(None, 2*10**9, 1.5))

    def test_invalid_geometry_never_becomes_empty(self):
        for goal in (dict(x=1), point(float('nan')), dict(x=0, y=0, size=-1)):
            payload = dict(self.snapshot(), candidates=[goal])
            self.assertIsNone(candidate_snapshot(payload, 10**9, 1.5))
        payload = dict(self.snapshot(), candidates=[point(2), point(1), point(2)])
        self.assertEqual(candidate_snapshot(payload, 10**9, 1.5), [point(1), point(2)])


if __name__ == '__main__':
    unittest.main()
