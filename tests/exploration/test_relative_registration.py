"""Synthetic measured geometry and fail-closed initialization checks; no simulator."""
import importlib.util
import math
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import Mock, patch

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'src/edgeai_mapping'))
from edgeai_mapping import relative_registration as rr


def room():
    # Unequal walls and interior diagonal: no rotational or corridor symmetry.
    segments = [((-2., -1.4), (2.5, -1.4)), ((2.5, -1.4), (2.5, 2.1)),
                ((2.5, 2.1), (-.8, 2.1)), ((-2., -1.4), (-2., .8)),
                ((-.8, .2), (.5, 1.2))]
    return np.vstack([np.linspace(a, b, 36) for a, b in segments])


class RegistrationTests(unittest.TestCase):
    def assertPose(self, actual, expected, tolerance=.025):
        delta = rr.compose(rr.inverse(expected), actual)
        self.assertLess(np.linalg.norm(delta[:2]), tolerance)
        self.assertLess(abs(delta[2]), tolerance)

    def test_recovers_noisy_offset_with_outliers_and_partial_overlap(self):
        rng = np.random.default_rng(9)
        target = room()
        actual = [0.08, -.62, .09]
        source = rr.transform(target[12:], rr.inverse(actual))
        source += rng.normal(0, .004, source.shape)
        source = np.vstack((source, rng.uniform(-4, 4, (12, 2))))
        result = rr.register_scans(target, source, [0., -.6, 0.])
        self.assertTrue(result['accepted'], result)
        self.assertPose(result['odom1_from_odom2'], actual)
        self.assertGreater(result['overlap'], .6)
        self.assertEqual(len(result['covariance']), 9)
        self.assertGreaterEqual(np.linalg.eigvalsh(np.reshape(result['covariance'], (3, 3)))[0], 0)

    def test_corridor_rejected_even_with_zero_residual(self):
        x = np.linspace(-4, 4, 90)
        target = np.vstack((np.column_stack((x, x*0-1)), np.column_stack((x, x*0+1))))
        result = rr.register_scans(target, target, [0., 0., 0.])
        self.assertFalse(result['accepted'])
        self.assertEqual(result['reason'], 'degenerate_geometry')

    def test_tiny_patch_no_overlap_and_empty_fail(self):
        for target, source in [(room()*.02, room()*.02), (room(), room()+20),
                               (np.empty((0, 2)), room())]:
            self.assertFalse(rr.register_scans(target, source, [0., 0., 0.])['accepted'])

    def test_correction_limit_independent_of_fit_quality(self):
        cfg = rr.RegistrationConfig(max_correction_m=.05, coarse_xy_m=.04)
        target = room()
        source = rr.transform(target, rr.inverse([.12, 0., 0.]))
        result = rr.register_scans(target, source, [0., 0., 0.], cfg)
        self.assertFalse(result['accepted'], result)
        self.assertEqual(result['reason'], 'correction_limit')

    def test_residual_threshold_rejects_noisy_measurements(self):
        rng = np.random.default_rng(28)
        result = rr.register_scans(room(), room()+rng.normal(0, .02, room().shape),
                                  [0., 0., 0.], rr.RegistrationConfig(max_residual_m=.002))
        self.assertFalse(result['accepted'], result)
        self.assertEqual(result['reason'], 'high_residual')

    def test_distinctiveness_gate_is_applied(self):
        # Offsets far below the measurement noise cannot be distinguished.
        source = room()+np.random.default_rng(12).normal(0, .003, room().shape)
        result = rr.register_scans(room(), source, [0., 0., 0.],
                                  rr.RegistrationConfig(distinct_translation_m=.00001,
                                                        distinct_yaw_rad=.00001))
        self.assertFalse(result['accepted'], result)
        self.assertEqual(result['reason'], 'ambiguous_geometry')

    def test_repeated_geometry_is_ambiguous_despite_strong_local_information(self):
        tile = np.array([[0., 0.], [.02, 0.], [.04, 0.], [.06, 0.],
                         [0., .02], [0., .04], [0., .06]])
        period = 2*math.pi/24
        points = np.vstack([rr.transform(tile, [2*math.cos(a), 2*math.sin(a), a])
                            for a in np.arange(24)*period])
        result = rr.register_scans(points, points, [0., 0., 0.],
                                  rr.RegistrationConfig(normal_neighbors=3,
                                      distinct_yaw_rad=period, coarse_yaw_rad=period))
        self.assertGreater(result['information_ratio'], .5)
        self.assertGreater(result['overlap'], .9)
        self.assertFalse(result['accepted'], result)
        self.assertEqual(result['reason'], 'ambiguous_geometry')

    def test_perfect_fit_does_not_imply_sub_noise_distinctiveness(self):
        result = rr.register_scans(room(), room(), [0., 0., 0.],
                                  rr.RegistrationConfig(distinct_translation_m=.00001,
                                                        distinct_yaw_rad=.00001))
        self.assertFalse(result['accepted'], result)
        self.assertEqual(result['reason'], 'ambiguous_geometry')

    def test_numpy_fallback_and_point_budget(self):
        with patch.object(rr, 'cKDTree', None):
            result = rr.register_scans(room(), room(), [0., 0., 0.])
        self.assertTrue(result['accepted'], result)
        self.assertLessEqual(max(rr.bounded_points(np.tile(room(), (10, 1))).shape), 180)

    def test_scan_validity_and_peer_endpoint_removal(self):
        points = rr.scan_points([.6, 2., float('inf'), float('nan'), -.1],
                                0., math.pi/2, .05, 10.)
        original = points.copy()
        masked = rr.remove_peer(points, [-.6, 0., 0.], [-.3, .1, -.16, .16], .05)
        np.testing.assert_array_equal(points, original)
        self.assertEqual(len(masked), 1)
        np.testing.assert_allclose(masked[0], [0., 2.], atol=1e-12)

    def test_initial_frame_and_laser_offset_composition(self):
        base1, base2 = [1., 2., .3], [-.3, .8, -.2]
        initial = [0., -.6, .1]
        prior = rr.compose(rr.compose(base1, initial), rr.inverse(base2))
        laser2 = rr.compose(base2, [.16, -.03, .2])
        in_base2 = rr.transform([[.7, .2]], [.16, -.03, .2])
        expected = rr.transform(in_base2, rr.compose(base1, initial))
        actual = rr.transform(rr.transform([[.7, .2]], laser2), prior)
        np.testing.assert_allclose(actual, expected)

    def test_invalid_config_prior(self):
        for kwargs in ({'max_points': 181}, {'trim_fraction': 0}, {'iterations': 1.5},
                       {'normal_neighbors': 31}, {'max_seeds': 28},
                       {'min_overlap': 1.1}, {'max_residual_m': float('nan')}):
            with self.assertRaises(ValueError):
                rr.RegistrationConfig(**kwargs)
        with self.assertRaises(ValueError):
            rr.register_scans(room(), room(), [0, float('nan'), 0])


class ConfirmationTests(unittest.TestCase):
    def candidate(self, x=0.):
        return dict(accepted=True, odom1_from_odom2=[x, -.6, 0.], reason='candidate')

    def test_distinct_both_timestamps_interval_and_freeze(self):
        gate = rr.TemporalConfirmation()
        self.assertIsNone(gate.observe(self.candidate(), [1., 1.]))
        for stamps in ([1., 1.], [2., 1.], [1.1, 1.1]):
            self.assertIsNone(gate.observe(self.candidate(), stamps))
        self.assertEqual(len(gate.samples), 1)
        self.assertIsNone(gate.observe(self.candidate(.01), [2., 2.]))
        result = gate.observe(self.candidate(.02), [3., 3.])
        self.assertTrue(result['accepted'])
        self.assertIs(gate.observe(self.candidate(4.), [4., 4.]), result)

    def test_failure_and_inconsistent_candidate_restart_sequence(self):
        gate = rr.TemporalConfirmation()
        gate.observe(self.candidate(), [1., 1.])
        gate.observe(dict(accepted=False), [2., 2.])
        self.assertEqual(len(gate.samples), 0)
        gate.observe(self.candidate(), [3., 3.])
        gate.observe(self.candidate(.2), [4., 4.])
        self.assertEqual(len(gate.samples), 1)
        self.assertIsNone(gate.confirmed)


def load_node():
    """Import with minimal ROS types so lifecycle tests run without ROS installed."""
    class FakeTime:
        def __init__(self, nanoseconds=0): self.nanoseconds = nanoseconds
        @classmethod
        def from_msg(cls, msg): return cls(msg.sec*10**9+msg.nanosec)

    class TransformException(Exception): pass
    modules = {}
    for name in ('rclpy', 'rclpy.node', 'rclpy.qos', 'rclpy.time',
                 'sensor_msgs', 'sensor_msgs.msg', 'std_msgs', 'std_msgs.msg', 'tf2_ros'):
        modules[name] = types.ModuleType(name)
    modules['rclpy.node'].Node = object
    qos = modules['rclpy.qos']
    qos.QoSProfile = Mock()
    qos.ReliabilityPolicy = types.SimpleNamespace(RELIABLE=1)
    qos.DurabilityPolicy = types.SimpleNamespace(TRANSIENT_LOCAL=1)
    qos.qos_profile_sensor_data = object()
    modules['rclpy.time'].Time = FakeTime
    modules['sensor_msgs.msg'].LaserScan = object
    modules['std_msgs.msg'].String = lambda **kw: types.SimpleNamespace(**kw)
    tf = modules['tf2_ros']
    tf.Buffer = tf.TransformListener = object
    tf.TransformException = TransformException
    spec = importlib.util.spec_from_file_location('initializer_under_test',
        ROOT/'src/edgeai_mapping/scripts/joint_mapping_initializer.py')
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


class NodeLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.module = load_node()
        self.node = self.module.JointMappingInitializer.__new__(self.module.JointMappingInitializer)
        n = self.node
        n.p = dict(self.module.DEFAULTS)
        n.cfg = rr.RegistrationConfig()
        n.confirmation = rr.TemporalConfirmation()
        n.started = self.module.time.monotonic()
        n.done, n.prior, n.initial_samples, n.last_attempt = False, None, None, None
        n.latest = {}
        n.publisher = Mock()
        n.report_path = None
        n.get_clock = lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=10**10))
        n.get_logger = Mock()
        n.publish(dict(accepted=False, reason='waiting_for_scans', state='waiting'))

    def scan(self, seconds):
        return types.SimpleNamespace(header=types.SimpleNamespace(
            stamp=types.SimpleNamespace(sec=seconds, nanosec=0)))

    def test_tf_catchup_retries_same_pair_then_stops_after_confirmation(self):
        n = self.node
        n.receive('primary', self.scan(10))
        n.receive('secondary', self.scan(10))
        n.sample = Mock(side_effect=self.module.TransformException('catchup'))
        n.process()
        self.assertIsNone(n.last_attempt)
        def sample(scan, index, alias=False):
            return room(), dict(odom_from_base=[0., 0., 0.],
                                odom_frame=f'robot{index}/odom')
        n.sample = Mock(side_effect=sample)
        n.p['initial_relative_pose'] = [0., 0., 0.]
        n.process()
        self.assertEqual(len(n.confirmation.samples), 1)
        n.process()
        self.assertEqual(len(n.confirmation.samples), 1)
        for stamp in (11, 12):
            n.receive('primary', self.scan(stamp))
            n.receive('secondary', self.scan(stamp))
            n.process()
        self.assertTrue(n.done)
        calls = n.sample.call_count
        n.process()
        self.assertEqual(calls, n.sample.call_count)
        self.assertTrue(n.last_record['accepted'])

    def test_deadline_and_persisted_record(self):
        import json
        with tempfile.TemporaryDirectory() as folder:
            n = self.node
            n.report_path = Path(folder)/'joint-initialization.json'
            n.started -= 46
            n.process()
            record = json.loads(n.report_path.read_text())
            self.assertEqual(record['state'], 'failed')
            self.assertEqual(record['reason'], 'deadline')
            self.assertFalse(record['accepted'])
            self.assertIsNone(record['odom1_from_odom2'])

    def test_timestamped_tf_includes_laser_offset_and_sample_metadata(self):
        n = self.node
        scan = self.scan(10)
        scan.header.frame_id = 'robot1/laser'
        scan.ranges, scan.angle_min, scan.angle_increment = [1.], 0., .1
        scan.range_min, scan.range_max = .05, 10.
        def tf(parent, child, stamp):
            self.assertEqual(stamp.nanoseconds, 10**10)
            x = {'robot1/odom': {'robot1/laser': 1.2, 'robot1/base_footprint': 1.},
                 'robot1/base_footprint': {'robot1/laser': .2}}[parent][child]
            return types.SimpleNamespace(header=scan.header, transform=types.SimpleNamespace(
                translation=types.SimpleNamespace(x=x, y=0.),
                rotation=types.SimpleNamespace(w=1., x=0., y=0., z=0.)))
        n.buffer = Mock()
        n.buffer.lookup_transform.side_effect = tf
        points, metadata = n.sample(scan, 1)
        np.testing.assert_allclose(points, [[2.2, 0.]])
        self.assertEqual(metadata['base_from_laser'], [.2, 0., 0.])
        self.assertEqual(metadata['odom_pose_stamp_ns'], 10**10)
        self.assertEqual(n.buffer.lookup_transform.call_count, 3)
        scan.header.stamp.sec = 1
        with self.assertRaisesRegex(ValueError, 'stale'):
            n.sample(scan, 1)

    def test_config_file_accepts_integrated_and_nested_layouts(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'config.yaml'
            path.write_text('initialization:\n  processing_hz: 1.0\n  initial_relative_pose: [0.0, -0.6, 0.0]\n')
            self.assertEqual(self.module.config_defaults(path)['processing_hz'], 1.)
            path.write_text('joint_mapping:\n  initial_relative_pose: [0.0, -0.6, 0.0]\n  initializer:\n    timeout_s: 12.0\n')
            self.assertEqual(self.module.config_defaults(path)['timeout_s'], 12.)
            path.write_text('initialization:\n  processing_typo: 1.0\n')
            with self.assertRaisesRegex(ValueError, 'Unknown'):
                self.module.config_defaults(path)


if __name__ == '__main__':
    unittest.main()
