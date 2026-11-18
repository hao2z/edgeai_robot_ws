"""Frontier update hysteresis and the cancellation/redispatch boundary."""
from concurrent.futures import Future
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'src/edgeai_mission'))
from edgeai_tasks.exploration import (AssignmentPolicy, ExplorationFailure,
                                     ExplorationUnits, FrontierUpdates)


def point(x):
    return dict(x=float(x), y=0., size=1.)


def entry(kind='frontier'):
    return dict(goal=point(1), sent_at=0., cancel_at=None, cancel=None,
                cancel_reason=None, record=dict(kind=kind, state='ACTIVE'),
                sent=Future(), handle=None, result=None)


class FrontierUpdateTests(unittest.TestCase):
    def setUp(self):
        self.policy = AssignmentPolicy()
        self.watch = FrontierUpdates(self.policy.c)
        self.entry = entry()

    def observe(self, stamp, candidates, wall=None):
        return self.watch.obsolete('robot1', self.entry, candidates,
                                   int(stamp*1e9), stamp if wall is None else wall)

    def test_sustained_distinct_updates_and_minimum_goal_lifetime(self):
        for stamp in (1., 2., 3., 4.):
            self.assertFalse(self.observe(stamp, [point(4)]))
        self.assertTrue(self.observe(5., [point(4)]))

    def test_repeated_snapshot_is_not_new_evidence(self):
        self.assertFalse(self.observe(10., []))
        self.assertFalse(self.observe(10., [], wall=100.))

    def test_drift_and_temporary_disappearance_do_not_preempt(self):
        self.assertFalse(self.observe(10., []))
        self.assertFalse(self.observe(11., [point(1.5), point(4)]))
        self.assertFalse(self.observe(12., []))
        self.assertFalse(self.observe(13., []))
        self.assertTrue(self.observe(14., []))

    def test_unavailable_gap_or_clock_regression_resets_confirmation(self):
        for following in (None, 'gap', 'regression'):
            with self.subTest(following=following):
                self.watch.missing.clear()
                self.assertFalse(self.observe(10., []))
                if following is None:
                    self.assertFalse(self.observe(11., None))
                    self.assertFalse(self.observe(12., []))
                elif following == 'gap':
                    self.assertFalse(self.observe(13., []))
                else:
                    self.assertFalse(self.observe(9., [], wall=13.))

    def test_other_action_kinds_and_already_canceling_are_untouched(self):
        for kind in ('search', 'sweep', 'yield'):
            self.entry['record']['kind'] = kind
            self.assertFalse(self.observe(10., []))
            self.assertFalse(self.observe(11., []))
            self.assertFalse(self.observe(12., []))
        self.entry['record']['kind'] = 'frontier'
        self.entry['cancel_reason'] = 'Navigation timeout'
        self.assertFalse(self.observe(20., []))

    def test_new_goal_needs_its_own_confirmation(self):
        self.observe(10., [])
        self.observe(11., [])
        self.entry = entry()
        self.assertFalse(self.observe(12., []))


class NavigationTransitionTests(unittest.TestCase):
    def setUp(self):
        self.units = u = ExplorationUnits.__new__(ExplorationUnits); u.robots=('robot1','robot2')
        u.policy = AssignmentPolicy()
        u.settings = u.policy.c
        u.frontier_updates = FrontierUpdates(u.settings)
        u.pending = {'robot1': entry()}
        u.states = dict(robot1=dict(phase='ACTIVE'), robot2=dict(phase='IDLE'))
        u.settle_until = {}
        u.yield_context = None
        u.failures = []
        u.blockage_queue = []
        u.check_owner = Mock()
        u.event = Mock()
        u.now_ns = Mock(return_value=10*10**9)
        u.snapshots = Mock(return_value=dict(robot1=[point(4)], robot2=[]))
        u.sources = {r:dict(stamp_ns=10*10**9,frame_id='map',available=True,candidates=[])
                     for r in ('robot1','robot2')}
        u.nav = {r: Mock(server_is_ready=Mock(return_value=True)) for r in u.states}
        u.pose = Mock(return_value=point(0))
        u._send = Mock()
        u._report = Mock()

    def test_completion_skips_frontier_only_after_all_robots_exhaust_retries(self):
        u = self.units
        del u.snapshots, u._report
        u.pending.clear()
        u.states['robot1']['phase'] = 'IDLE'
        u.last_state = None
        u.empty_since = u.empty_wall = None
        u.sources['robot1']['candidates'] = [point(4)]
        for _ in range(4):
            u.policy.failed('robot1', point(4), 10., 'no path')
        for _ in range(2):
            u.policy.failed('robot2', point(4), 10., 'no path')
        with patch('edgeai_tasks.exploration.time.monotonic', return_value=11.):
            self.assertFalse(u.check_complete()['complete'])
            self.assertIsNone(u.empty_since)
        for _ in range(2):
            u.policy.failed('robot2', point(4), 21., 'no path')
        with patch('edgeai_tasks.exploration.time.monotonic', return_value=22.):
            self.assertFalse(u.check_complete()['complete'])
            self.assertEqual(u.empty_since, 10*10**9)
        u.now_ns.return_value = 12*10**9
        u.sources['robot1']['stamp_ns'] = 12*10**9
        with patch('edgeai_tasks.exploration.time.monotonic', return_value=24.):
            self.assertFalse(u.check_complete()['complete'])  # robot2's source is stale
            self.assertIsNone(u.empty_since)
            u.sources['robot2']['stamp_ns'] = 12*10**9
            self.assertFalse(u.check_complete()['complete'])
        u.now_ns.return_value = 14*10**9
        for source in u.sources.values():
            source['stamp_ns'] = 14*10**9
        with patch('edgeai_tasks.exploration.time.monotonic', return_value=26.):
            self.assertTrue(u.check_complete()['complete'])
            u.sources['robot2']['candidates'] = [point(8)]
            self.assertFalse(u.check_complete()['complete'])

    def request_replacement(self):
        with patch('edgeai_tasks.exploration.time.monotonic', return_value=10.):
            self.units.refresh_frontier_goals()
        for source in self.units.sources.values():source['stamp_ns']=11*10**9
        self.units.now_ns.return_value=11*10**9
        with patch('edgeai_tasks.exploration.time.monotonic', return_value=11.):
            self.units.refresh_frontier_goals()
        for source in self.units.sources.values():source['stamp_ns']=12*10**9
        self.units.now_ns.return_value=12*10**9
        with patch('edgeai_tasks.exploration.time.monotonic', return_value=12.):
            self.units.refresh_frontier_goals()

    def test_waits_for_terminal_and_idle_then_uses_latest_candidates(self):
        u = self.units
        e = u.pending['robot1']
        handle = Mock(accepted=True)
        terminal = Future()
        handle.get_result_async.return_value = terminal
        e['sent'].set_result(handle)
        # Replacement may be requested before the acceptance callback is pumped.
        self.request_replacement()
        self.assertEqual(e['cancel_reason'], 'frontier_updated')
        with patch('edgeai_tasks.exploration.time.monotonic', return_value=12.1):
            u._advance()
            u._advance()
        handle.cancel_goal_async.assert_called_once()
        u.dispatch()
        u._send.assert_not_called()  # cancel acceptance alone does not free slot
        u.snapshots.return_value['robot1'] = [point(8)]
        terminal.set_result(SimpleNamespace(status=5, result=SimpleNamespace(error_code=0)))
        u._advance()
        self.assertNotIn('robot1', u.pending)
        self.assertEqual(e['record']['state'], 'CANCELED')
        self.assertEqual(u.policy.history['robot1'], [])
        self.assertEqual(u.failures, [])
        self.assertEqual(u.blockage_queue, [])
        u.now_ns.return_value += 10**9
        u.dispatch()
        u._send.assert_not_called()  # authority must also report IDLE
        u.states['robot1']['phase'] = 'IDLE'
        u.dispatch()
        u._send.assert_called_once_with('robot1', point(8), point(0), 'frontier')

    def test_other_source_keeps_shared_goal_alive_and_missing_source_cannot_cancel(self):
        u=self.units
        u.snapshots=ExplorationUnits.snapshots.__get__(u)
        u.sources['robot2']['candidates']=[point(1)]
        self.request_replacement()
        self.assertIsNone(u.pending['robot1']['cancel_reason'])
        u.sources['robot2']['available']=False
        self.request_replacement()
        self.assertIsNone(u.pending['robot1']['cancel_reason'])

    def test_terminal_races_do_not_charge_normal_replacement_to_failure_budget(self):
        for status in (4, 5, 6):
            with self.subTest(status=status):
                self.setUp()
                u = self.units
                e = u.pending['robot1']
                e['handle'] = Mock()
                e['cancel_reason'] = 'frontier_updated'
                e['result'] = Future()
                e['result'].set_result(SimpleNamespace(status=status, result=SimpleNamespace(error_code=0)))
                u._advance()
                self.assertFalse(u.pending)
                self.assertFalse(u.failures)
                self.assertFalse(u.policy.arrivals['robot1'])

    def test_late_rejection_after_replacement_does_not_charge_retry_budget(self):
        u = self.units
        self.request_replacement()
        u.pending['robot1']['sent'].set_result(Mock(accepted=False))
        u._advance()
        self.assertFalse(u.pending)
        self.assertFalse(u.failures)

    def test_failed_terminal_without_replacement_still_triggers_failure_handling(self):
        u = self.units
        e = u.pending['robot1']
        e['handle'] = Mock()
        e['result'] = Future()
        e['result'].set_result(SimpleNamespace(status=6, result=SimpleNamespace(error_code=208)))
        u._advance()
        self.assertEqual(len(u.failures), 1)
        self.assertEqual(len(u.blockage_queue), 0)  # Navigation already owns peer recovery.
        self.assertEqual(u.policy.history['robot1'][0]['attempts'], 1)

    def test_cancel_timeout_retains_slot_and_reports_unknown(self):
        u = self.units
        e = u.pending['robot1']
        self.request_replacement()
        u.fail = Mock(side_effect=ExplorationFailure('unknown'))
        with patch('edgeai_tasks.exploration.time.monotonic', return_value=30.):
            with self.assertRaises(ExplorationFailure):
                u._advance()
        self.assertIs(u.pending['robot1'], e)
        u.fail.assert_called_once()


if __name__ == '__main__':
    unittest.main()
