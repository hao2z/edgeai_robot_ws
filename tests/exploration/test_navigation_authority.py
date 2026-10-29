"""Ownership and recovery invariants; run without ROS."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'src/edgeai_navigation/scripts'))
from authority_policy import Authority, AuthorityConfig, RecoveryEvidence


class AuthorityTests(unittest.TestCase):
    def test_owner_gate_and_single_inflight(self):
        p = Authority()
        self.assertIsNone(p.reserve('MANUAL'))
        token = p.reserve('AUTO')
        self.assertIsNotNone(token)
        self.assertIsNone(p.reserve('AUTO'))
        p.accepted(token)
        self.assertIsNone(p.reserve('AUTO'))

    def test_takeover_waits_for_terminal_not_cancel_ack(self):
        p = Authority()
        token = p.reserve('AUTO')
        p.accepted(token)
        p.set_mode('MANUAL')
        self.assertTrue(p.cancel_requested)
        self.assertEqual(p.phase, 'CANCELING')
        self.assertIsNone(p.reserve('MANUAL'))
        p.request_cancel(token)
        self.assertIsNone(p.reserve('MANUAL'))
        p.terminal(token)
        self.assertIsNotNone(p.reserve('MANUAL'))

    def test_stop_while_sending_honors_late_acceptance(self):
        p = Authority()
        token = p.reserve('AUTO')
        p.set_mode('STOPPED')
        p.uncertain(token, 'no ack')
        self.assertTrue(p.accepted(token))
        self.assertEqual(p.phase, 'CANCELING')
        self.assertTrue(p.cancel_requested)

    def test_timeout_and_late_old_result_do_not_release_new_task(self):
        p = Authority()
        first = p.reserve('AUTO')
        p.uncertain(first, 'feedback timeout')
        self.assertIsNone(p.reserve('AUTO'))
        p.terminal(first)
        second = p.reserve('AUTO')
        self.assertFalse(p.terminal(first))
        self.assertFalse(p.accepted(first))
        self.assertEqual(p.generation, second)
        self.assertEqual(p.phase, 'SENDING')

    def test_same_mode_request_keeps_active_task(self):
        p = Authority()
        token = p.reserve('AUTO')
        p.accepted(token)
        p.set_mode('AUTO')
        self.assertFalse(p.cancel_requested)
        self.assertEqual(p.phase, 'ACTIVE')

    def test_duplicate_takeover_does_not_extend_cancel_grace(self):
        p = Authority()
        token = p.reserve('AUTO')
        p.accepted(token)
        self.assertTrue(p.set_mode('MANUAL'))
        p.mark_mode_request(10.0)
        self.assertFalse(p.set_mode('MANUAL'))
        self.assertFalse(p.should_force_cancel(10.9))
        self.assertTrue(p.should_force_cancel(11.0))
        self.assertEqual(p.mode_requested_at, 10.0)

    def test_policy_owns_endpoint_and_deadline_contract(self):
        config = AuthorityConfig()
        self.assertEqual(config.cancel_timeout_s, 5.0)
        self.assertEqual(config.auto_action, '/exploration/navigate_to_pose')
        self.assertEqual(config.backend_action, '/navigation_backend/navigate_to_pose')


class RecoveryTests(unittest.TestCase):
    def test_action_success_without_motion_is_not_recovery(self):
        r = RecoveryEvidence('backup', (0, 0, 0), 2)
        r.finish(True, (0, 0, 0))
        self.assertFalse(r.note_navigation(0, True, (1, 0, 0), True))
        self.assertEqual(r.outcome, 'UNVERIFIED_NO_MOTION')

    def test_failed_action_with_partial_motion_is_not_success(self):
        r = RecoveryEvidence('backup', (0, 0, 0), 2)
        r.finish(False, (-.05, 0, 0))
        self.assertFalse(r.note_navigation(0, True, (1, 0, 0), True))

    def test_translation_needs_resumed_path_and_progress(self):
        r = RecoveryEvidence('backup', (0, 0, 0), 2)
        r.finish(True, (-.08, 0, 0))
        self.assertEqual(r.outcome, 'AWAITING_NAVIGATION')
        self.assertFalse(r.note_navigation(2.08, True, (-.08, 0, 0)))
        self.assertFalse(r.note_navigation(1.8, False, (.1, 0, 0)))
        self.assertTrue(r.note_navigation(1.8, True, (.1, 0, 0)))
        self.assertFalse(r.note_navigation(1.5, True, (.3, 0, 0)))

    def test_rotation_and_final_goal_result(self):
        r = RecoveryEvidence('spin', (0, 0, 0), 2)
        r.finish(True, (0, 0, .5))
        self.assertTrue(r.note_navigation(None, False, None, goal_succeeded=True))

    def test_missing_odometry_does_not_prove_recovery(self):
        r = RecoveryEvidence('backup', None, 2)
        r.finish(True, (-.08, 0, 0))
        self.assertFalse(r.note_navigation(0, True, (1, 0, 0), True))
        self.assertEqual(r.outcome, 'UNVERIFIED_NO_ODOMETRY')


if __name__ == '__main__':
    unittest.main()
