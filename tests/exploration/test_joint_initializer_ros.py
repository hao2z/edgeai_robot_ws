"""Real initializer subprocess and ROS transport/TF, synthetic rays, no Gazebo.

Run from the workspace (only idle domain 198 is permitted):
  bash scripts/ros/ros-command.sh env ROS_DOMAIN_ID=198 python3 -m unittest \
      discover -s tests/exploration -p test_joint_initializer_ros.py -v
The overlay supplies edgeai_mapping; no source-path substitution or threshold overrides.
Only this test's own subprocess is terminated.
"""
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                      qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
import yaml

from edgeai_mapping.relative_registration import RegistrationConfig, compose, inverse


ROOT = Path(__file__).resolve().parents[2]


class JointInitializerROS(unittest.TestCase):
    actual = [0., -.6, 0.]
    mounts = ([.21, .04, .12], [-.17, .08, -.23])
    # Unequal dimensions and asymmetric sensor locations disambiguate room axes.
    room = (-3., 4., -2.3, 2.8)

    def setUp(self):
        if os.environ.get('ROS_DOMAIN_ID') != '198':
            self.skipTest('This test is restricted to ROS_DOMAIN_ID=198')
        self.directory = tempfile.TemporaryDirectory(prefix='joint-initializer-ros-')
        self.addCleanup(self.directory.cleanup)
        self.log_path = Path(self.directory.name)/'initializer.log'
        self.log = self.log_path.open('w+')
        self.addCleanup(self.log.close)
        self.report_path = Path(self.directory.name)/'joint-initialization.json'
        # Default frame/topic names and ALL registration/confirmation thresholds.
        params = dict(use_sim_time=False, initial_relative_pose=[0., -.55, .04],
                      output_directory=self.directory.name)
        parameter_path = Path(self.directory.name)/'initializer.yaml'
        parameter_path.write_text(yaml.safe_dump(
            {'joint_mapping_initializer': {'ros__parameters': params}}))
        rclpy.init()
        self.addCleanup(rclpy.shutdown)
        self.node = rclpy.create_node('joint_initializer_ros_test')
        self.addCleanup(self.node.destroy_node)
        self.records = []
        self.latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscription = self.node.create_subscription(
            String, '/joint_mapping/alignment',
            lambda msg: self.records.append(json.loads(msg.data)), self.latched)
        self.scans = [self.node.create_publisher(LaserScan, f'/robot{i}/scan',
                                                qos_profile_sensor_data) for i in (1, 2)]
        self.static = StaticTransformBroadcaster(self.node)
        self.dynamic = TransformBroadcaster(self.node)
        self.static.sendTransform([
            self.make_tf(f'robot{i+1}/base_footprint', f'robot{i+1}/lidar_link', mount)
            for i, mount in enumerate(self.mounts)])
        self.process = subprocess.Popen(
            [sys.executable, '-u', str(ROOT/'src/edgeai_mapping/scripts/joint_mapping_initializer.py'),
             '--ros-args', '--params-file', str(parameter_path)],
            stdout=self.log, stderr=subprocess.STDOUT)
        self.addCleanup(self.stop_initializer)
        self.wait(lambda: all(p.get_subscription_count() for p in self.scans),
                  'initializer scan subscriptions', 15.)
        self.last_scans = None
        self.timer = self.node.create_timer(.1, self.sample)

    def stop_initializer(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=5.)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=3.)

    def wait(self, predicate, description, timeout=15.):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=.02)
            if predicate():
                return
            if self.process.poll() is not None:
                break
        self.log.flush()
        self.fail(f'Timed out: {description}; last records={self.records[-3:]}; '
                  f'initializer log={self.log_path.read_text()[-6000:]}')

    def spin(self, duration):
        deadline = time.monotonic()+duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=.02)
            self.assertIsNone(self.process.poll(), self.log_path.read_text())

    def make_tf(self, parent, child, pose, stamp=None):
        result = TransformStamped()
        result.header.frame_id, result.child_frame_id = parent, child
        result.header.stamp = stamp or self.node.get_clock().now().to_msg()
        result.transform.translation.x, result.transform.translation.y = map(float, pose[:2])
        result.transform.rotation.z = math.sin(pose[2]/2)
        result.transform.rotation.w = math.cos(pose[2]/2)
        return result

    def sample(self):
        stamp = self.node.get_clock().now().to_msg()
        scans = []
        transforms = []
        for index in (0, 1):
            # Stationary bases, but timestamped dynamic wheel-odom TF at 10 Hz.
            local = [0., 0., 0.]
            common = local if index == 0 else compose(self.actual, local)
            sensor = compose(common, self.mounts[index])
            scan = LaserScan()
            scan.header.frame_id = f'robot{index+1}/lidar_link'
            scan.header.stamp = stamp
            scan.angle_min, scan.angle_max = -math.pi, math.pi
            scan.angle_increment = math.pi/180
            scan.range_min, scan.range_max = .05, 12.
            scan.scan_time, scan.time_increment = .1, 0.
            xmin, xmax, ymin, ymax = self.room
            readings = []
            for beam in range(361):
                angle = sensor[2]+scan.angle_min+beam*scan.angle_increment
                dx, dy = math.cos(angle), math.sin(angle)
                distances = []
                for coordinate, direction, bounds in (
                        (sensor[0], dx, (xmin, xmax)), (sensor[1], dy, (ymin, ymax))):
                    if abs(direction) > 1e-9:
                        distances.extend((bound-coordinate)/direction for bound in bounds
                                         if (bound-coordinate)/direction > 0.)
                readings.append(min(distances))
            scan.ranges = readings
            scans.append(scan)
            transforms.append(self.make_tf(f'robot{index+1}/odom',
                f'robot{index+1}/base_footprint', local, stamp))
        self.dynamic.sendTransform(transforms)
        for publisher, scan in zip(self.scans, scans):
            publisher.publish(scan)
        self.last_scans = scans

    def test_real_initializer_confirms_persists_and_ignores_stale_replays(self):
        self.wait(lambda: any(r.get('accepted') for r in self.records),
                  'three independent confirmations', 25.)
        accepted = next(r for r in self.records if r.get('accepted'))
        self.timer.cancel()
        self.assertEqual(accepted['state'], 'confirmed')
        self.assertEqual(accepted['parent_frame'], 'robot1/odom')
        self.assertEqual(accepted['child_frame'], 'robot2/odom')
        correction = compose(inverse(self.actual), accepted['odom1_from_odom2'])
        self.assertLess(math.hypot(*correction[:2]), .04, accepted)
        self.assertLess(abs(correction[2]), .03, accepted)
        self.assertEqual(accepted['confirmations'], 3)
        pairs = accepted['confirmation_scan_stamps_ns']
        self.assertEqual(len(pairs), 3)
        # The confirmation audit field round-trips through float seconds. Source
        # samples preserve exact epoch nanoseconds; validate those independently.
        confirmed_samples = {}
        for record in self.records:
            count = record.get('confirmations', 0)
            if count and record.get('source_samples'):
                confirmed_samples.setdefault(count, record['source_samples'])
        self.assertEqual(set(confirmed_samples), {1, 2, 3})
        for robot in (0, 1):
            stamps = [confirmed_samples[count][robot]['scan_stamp_ns'] for count in (1, 2, 3)]
            self.assertEqual(len(set(stamps)), 3)
            self.assertTrue(all(b-a >= 400_000_000-1 for a, b in zip(stamps, stamps[1:])))
            for exact, pair in zip(stamps, pairs):
                self.assertLessEqual(abs(exact-pair[robot]), 512)
        for key, expected in vars(RegistrationConfig()).items():
            self.assertEqual(accepted['parameters'][key], expected, key)
        for index, sample in enumerate(accepted['source_samples']):
            self.assertEqual(sample['scan_stamp_ns'], confirmed_samples[3][index]['scan_stamp_ns'])
            self.assertEqual(sample['tf_query_stamp_ns'], sample['scan_stamp_ns'])
            self.assertEqual(sample['odom_pose_stamp_ns'], sample['scan_stamp_ns'])
            for actual, expected in zip(sample['base_from_laser'], self.mounts[index]):
                self.assertAlmostEqual(actual, expected, places=6)
        self.assertEqual(len(accepted['covariance']), 9)
        self.assertGreaterEqual(accepted['overlap'], RegistrationConfig().min_overlap)
        self.assertLessEqual(accepted['residual'], RegistrationConfig().max_residual_m)
        persisted = self.report_path.read_bytes()
        self.assertEqual(json.loads(persisted), accepted)
        report_mtime = self.report_path.stat().st_mtime_ns
        # Make saved packets stale, then repeatedly replay their original stamps.
        self.spin(1.8)
        record_count = len(self.records)
        for _ in range(10):
            for publisher, scan in zip(self.scans, self.last_scans):
                publisher.publish(scan)
            self.spin(.1)
        self.assertEqual(len(self.records), record_count)
        self.assertEqual(self.report_path.read_bytes(), persisted)
        self.assertEqual(self.report_path.stat().st_mtime_ns, report_mtime)
        # A late consumer receives exactly the frozen reliable/transient result.
        late_records = []
        late_subscription = self.node.create_subscription(String, '/joint_mapping/alignment',
            lambda msg: late_records.append(json.loads(msg.data)), self.latched)
        self.wait(lambda: bool(late_records), 'late subscriber frozen alignment', 3.)
        self.assertEqual(late_records, [accepted])
        self.node.destroy_subscription(late_subscription)
        print('INITIALIZER_ROS_EVIDENCE '+json.dumps(dict(
            odom1_from_odom2=accepted['odom1_from_odom2'], residual=accepted['residual'],
            overlap=accepted['overlap'], confirmations=pairs,
            translation_error_m=math.hypot(*correction[:2]), yaw_error_rad=abs(correction[2]),
            report_persisted=True, stale_replays_ignored=True), sort_keys=True), flush=True)


if __name__ == '__main__':
    unittest.main()
