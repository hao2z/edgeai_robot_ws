"""Installed joint SLAM backend, synthetic scans/TF, no simulator.

Run after building edgeai_mapping and sourcing its install/setup.bash:
  ROS_DOMAIN_ID=197 python3 -m unittest discover -s tests/exploration \
      -p test_joint_slam_ros.py -v
Only the child backend started by this test is terminated. Use an idle domain.
"""
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
import time
import unittest

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
import yaml


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c*b[0] - s*b[1], a[1] + s*b[0] + c*b[1], a[2] + b[2])


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c*a[0] - s*a[1], s*a[0] - c*a[1], -a[2])


def yaw(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))


class JointSlamROS(unittest.TestCase):
    alignment = (2.0, -0.6, 0.25)
    mounts = ((0.21, 0.04, 0.12), (-0.17, 0.08, -0.23))
    local = ((0., 0., 0.), (-1., 0.2, 0.1))

    def setUp(self):
        if os.environ.get('ROS_DOMAIN_ID', '0') in ('0', '86'):
            self.skipTest('Use an isolated ROS_DOMAIN_ID, e.g. 197')
        executable = Path(get_package_prefix('edgeai_mapping')) / 'lib/edgeai_mapping/joint_slam_toolbox'
        self.assertTrue(executable.is_file(), f'Build backend first: {executable}')
        self.directory = tempfile.TemporaryDirectory(prefix='joint-slam-test-')
        self.addCleanup(self.directory.cleanup)
        log_path = Path(self.directory.name) / 'backend.log'
        self.log = log_path.open('w+')
        self.addCleanup(self.log.close)
        params = {
            'use_sim_time': False, 'use_lifecycle_manager': False,
            'transform_publish_period': 0.0, 'enable_interactive_mode': False,
            'use_map_saver': False, 'map_frame': 'map', 'map_name': '/map',
            'solver_plugin': 'solver_plugins::CeresSolver',
            'ceres_linear_solver': 'SPARSE_NORMAL_CHOLESKY',
            'ceres_preconditioner': 'SCHUR_JACOBI',
            'ceres_trust_strategy': 'LEVENBERG_MARQUARDT',
            'ceres_dogleg_type': 'TRADITIONAL_DOGLEG', 'ceres_loss_function': 'None',
            'minimum_time_interval': 0.0, 'minimum_travel_distance': 0.0,
            'minimum_travel_heading': 0.0, 'throttle_scans': 1,
            'map_update_interval': 0.15, 'anchor_graph_publish_period_s': 0.05,
            'joint_tf_publish_period_s': 0.02, 'joint_scan_max_age_s': 3.0,
            'joint_tf_wait_s': 0.2, 'joint_scan_queue_size': 3,
            'max_laser_range': 12.0, 'resolution': 0.05,
            'do_loop_closing': True,
        }
        for robot in ('robot1', 'robot2'):
            params.update({f'{robot}_base_frame': f'{robot}/base_footprint',
                           f'{robot}_odom_frame': f'{robot}/odom',
                           f'{robot}_sensor_frame': f'{robot}/lidar_link',
                           f'{robot}_scan_topic': f'/{robot}/scan_slam'})
        parameter_path = Path(self.directory.name) / 'params.yaml'
        parameter_path.write_text(yaml.safe_dump({'slam_toolbox': {'ros__parameters': params}}))
        rclpy.init()
        self.addCleanup(rclpy.shutdown)
        self.node = rclpy.create_node('joint_backend_test')
        self.addCleanup(self.node.destroy_node)
        self.status, self.graph, self.grid = None, None, None
        self.graph_history, self.status_history, self.transforms = [], [], []
        self.poses = {}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(String, '/joint_mapping/status', self.on_status, latched)
        self.node.create_subscription(String, '/slam_toolbox/anchor_graph', self.on_graph, latched)
        self.node.create_subscription(OccupancyGrid, '/map', lambda m: setattr(self, 'grid', m), latched)
        self.node.create_subscription(TFMessage, '/tf', self.on_tf, 100)
        for robot in ('robot1', 'robot2'):
            self.node.create_subscription(PoseWithCovarianceStamped, f'/{robot}/slam_pose',
                lambda m, robot=robot: self.poses.__setitem__(robot, m), 5)
        self.scans = [self.node.create_publisher(LaserScan, f'/robot{i}/scan_slam', qos_profile_sensor_data)
                      for i in (1, 2)]
        self.align_pub = self.node.create_publisher(String, '/joint_mapping/alignment', latched)
        self.static = StaticTransformBroadcaster(self.node)
        self.dynamic = TransformBroadcaster(self.node)
        self.static.sendTransform([self.make_tf(f'robot{i+1}/base_footprint',
            f'robot{i+1}/lidar_link', mount) for i, mount in enumerate(self.mounts)])
        # Deliberately publish alignment BEFORE configure: test transient latch.
        self.align_pub.publish(String(data=json.dumps(
            {'accepted': True, 'odom1_from_odom2': self.alignment})))
        self.process = subprocess.Popen([str(executable), '--ros-args', '--params-file', str(parameter_path)],
                                        stdout=self.log, stderr=subprocess.STDOUT)
        self.addCleanup(self.stop_backend)
        self.change = self.node.create_client(ChangeState, '/slam_toolbox/change_state')
        self.finalize = self.node.create_client(Trigger, '/slam_toolbox/finalize_anchors')
        self.wait(lambda: self.change.service_is_ready(), 'lifecycle service', timeout=15)
        self.transition(Transition.TRANSITION_CONFIGURE)
        self.transition(Transition.TRANSITION_ACTIVATE)
        self.wait(lambda: self.status and self.status['alignment_accepted'], 'alignment bootstrap')
        self.wait(lambda: self.graph is not None, 'initial graph')
        self.epoch = self.graph['epoch']
        self.wait(lambda: len(self.map_transforms()) >= 2, 'provisional map TF')

    def stop_backend(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=3)

    def on_status(self, msg):
        self.status = json.loads(msg.data)
        self.status_history.append(self.status)

    def on_graph(self, msg):
        self.graph = json.loads(msg.data)
        self.graph_history.append(self.graph)

    def on_tf(self, msg):
        self.transforms.extend(msg.transforms)

    def wait(self, predicate, description, timeout=8):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if predicate():
                return
            if self.process.poll() is not None:
                break
        self.log.flush()
        self.fail(f'Timed out: {description}; status={self.status}; '
                  f'backend log={Path(self.log.name).read_text()[-5000:]}')

    def spin(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def transition(self, transition):
        request = ChangeState.Request()
        request.transition.id = transition
        future = self.change.call_async(request)
        self.wait(future.done, f'lifecycle transition {transition}')
        self.assertTrue(future.result().success)

    def make_tf(self, parent, child, p, stamp=None):
        msg = TransformStamped()
        msg.header.frame_id, msg.child_frame_id = parent, child
        msg.header.stamp = stamp or self.node.get_clock().now().to_msg()
        msg.transform.translation.x, msg.transform.translation.y = p[:2]
        msg.transform.rotation.z, msg.transform.rotation.w = math.sin(p[2]/2), math.cos(p[2]/2)
        return msg

    def map_transforms(self):
        return [tf for tf in self.transforms if tf.header.frame_id == 'map']

    def sample(self, index, mask=False, local=None):
        local = local or self.local[index]
        common = local if index == 0 else compose(self.alignment, local)
        sensor = compose(common, self.mounts[index])
        scan = LaserScan()
        scan.header.frame_id = f'robot{index+1}/lidar_link'
        scan.header.stamp = self.node.get_clock().now().to_msg()
        scan.angle_min, scan.angle_max, scan.angle_increment = -math.pi, math.pi, math.pi/180
        scan.range_min, scan.range_max = 0.05, 12.0
        readings = []
        for beam in range(361):
            angle = sensor[2] + scan.angle_min + beam*scan.angle_increment
            dx, dy = math.cos(angle), math.sin(angle)
            distances = []
            for coordinate, direction, bounds in ((sensor[0], dx, (-5., 6.)),
                                                   (sensor[1], dy, (-4., 4.5))):
                if abs(direction) > 1e-9:
                    distances.extend((bound-coordinate)/direction for bound in bounds
                                     if (bound-coordinate)/direction > 0)
            distance = min(distances)
            if mask and abs(math.atan2(dy, dx)) < 0.25:
                distance = float('nan')
            readings.append(distance)
        scan.ranges = readings
        self.dynamic.sendTransform(self.make_tf(f'robot{index+1}/odom',
            f'robot{index+1}/base_footprint', local, scan.header.stamp))
        self.scans[index].publish(scan)
        return scan

    def send_accepted(self, index, **kwargs):
        robot = f'robot{index+1}'
        before = self.status['sources'][robot]['accepted']
        scan = self.sample(index, **kwargs)
        self.wait(lambda: self.status['sources'][robot]['accepted'] > before, f'{robot} accepted scan')
        return scan

    def assert_pose_close(self, actual, expected, places=5):
        self.assertAlmostEqual(actual[0], expected[0], places=places)
        self.assertAlmostEqual(actual[1], expected[1], places=places)
        self.assertAlmostEqual(math.atan2(math.sin(actual[2]-expected[2]),
                                        math.cos(actual[2]-expected[2])), 0., places=places)

    def test_joint_graph_transforms_finalize_and_lifecycle(self):
        self.assertEqual(self.status['nodes'], 0)
        for robot, expected in (('robot1', (0., 0., 0.)), ('robot2', self.alignment)):
            self.assertTrue(self.status['sources'][robot]['provisional_tf'])
            self.assert_pose_close(self.status['sources'][robot]['map_from_odom'], expected)
        # Robot2 arrives first; adding lexically earlier robot1 must not change epoch.
        self.wait(lambda: all(p.get_subscription_count() for p in self.scans), 'scan subscriptions')
        self.send_accepted(1)
        self.send_accepted(0)
        self.wait(lambda: self.status['cross_robot_edges'] > 0, 'stationary first-pair cross edge')
        self.assertEqual(self.status['nodes'], 2)
        for index in (1, 0, 1, 0):
            self.send_accepted(index)
        self.wait(lambda: len(self.graph['nodes']) >= 6 and self.status['cross_robot_edges'] > 0,
                  'joint graph edges')
        self.assertEqual({node['robot_id'] for node in self.graph['nodes']}, {'robot1', 'robot2'})
        ids = [node['id'] for node in self.graph['nodes']]
        self.assertEqual(len(ids), len(set(ids)))
        self.assertEqual({g['epoch'] for g in self.graph_history}, {self.epoch})
        for index, robot in enumerate(('robot1', 'robot2')):
            source = self.status['sources'][robot]
            self.assert_pose_close(source['laser_mount'], self.mounts[index])
            self.assertFalse(source['provisional_tf'])
            node = [n for n in self.graph['nodes'] if n['robot_id'] == robot][-1]
            self.assertEqual(node['sensor_id'], f'{robot}/lidar_link')
            self.assert_pose_close(node['odom'], self.local[index])
            common = self.local[index] if index == 0 else compose(self.alignment, self.local[index])
            self.assert_pose_close(node['global_odom'], common)
            self.assert_pose_close(source['map_from_odom'], compose(node['map'], inverse(node['odom'])))
        self.wait(lambda: len(self.poses) == 2 and self.grid is not None, 'map and evidence poses')
        self.assertEqual(self.grid.header.frame_id, 'map')
        self.assertAlmostEqual(self.grid.info.resolution, 0.05)
        self.assertEqual(self.grid.info.origin.orientation.w, 1.)
        self.assertEqual({tf.child_frame_id for tf in self.map_transforms()},
                         {'robot1/odom', 'robot2/odom'})
        self.assertTrue(all(math.isfinite(v) for p in self.poses.values() for v in p.pose.covariance))

        # Replayed timestamp and broken frame cannot become accepted keyframes.
        scan = self.send_accepted(0)
        accepted = self.status['sources']['robot1']['accepted']
        dropped = self.status['sources']['robot1']['dropped']
        self.scans[0].publish(scan)
        self.wait(lambda: self.status['sources']['robot1']['dropped'] > dropped, 'timestamp rejection')
        self.assertEqual(self.status['sources']['robot1']['accepted'], accepted)

        future = self.finalize.call_async(Trigger.Request())
        self.wait(future.done, 'finalization')
        self.assertTrue(future.result().success, future.result().message)
        final_graph = json.loads(future.result().message)
        self.assertTrue(final_graph['finalized'])
        self.assertTrue(final_graph['frozen'])
        self.wait(lambda: self.status['finalized'] and self.status['frozen'], 'frozen status')
        final_nodes = len(final_graph['nodes'])
        for i in (0, 1):
            self.sample(i)
        self.spin(.25)
        self.assertEqual(self.status['nodes'], final_nodes)
        self.assertTrue(self.graph['finalized'])
        self.assertGreater(self.status['map_update_stamp_ns'], 0)

        self.transition(Transition.TRANSITION_DEACTIVATE)
        self.spin(.15)  # Drain DDS samples already in flight.
        count = len(self.map_transforms())
        self.spin(.25)
        self.assertEqual(len(self.map_transforms()), count, 'map TF continued after deactivation')
        self.transition(Transition.TRANSITION_CLEANUP)
        self.transition(Transition.TRANSITION_CONFIGURE)
        self.transition(Transition.TRANSITION_ACTIVATE)
        self.wait(lambda: self.graph['epoch'] != self.epoch, 'new epoch after cleanup')
        self.wait(lambda: not self.status['frozen'] and self.status['nodes'] == 0, 'fresh graph')
        self.send_accepted(1)

    def test_masked_peer_does_not_clear_behind_peer(self):
        self.wait(lambda: self.scans[0].get_subscription_count() > 0, 'scan subscription')
        for _ in range(6):
            self.send_accepted(0, mask=True)
        self.wait(lambda: self.grid is not None and self.status['map_update_stamp_ns'] > 0, 'masked map')
        self.spin(.3)

        def cell(x, y):
            grid = self.grid
            ix = int((x-grid.info.origin.position.x)/grid.info.resolution)
            iy = int((y-grid.info.origin.position.y)/grid.info.resolution)
            self.assertTrue(0 <= ix < grid.info.width and 0 <= iy < grid.info.height)
            return grid.data[iy*grid.info.width+ix]

        # The central masked cone extends across the peer at x~1 and beyond it.
        # Neighboring observed rays must clear, while the masked cone stays unknown.
        for x in (1., 2., 3., 4.):
            self.assertEqual(cell(x, .04), -1, f'Masked beam fabricated free space at x={x}')
        self.assertEqual(cell(2., 1.), 0, 'Control observed ray did not clear')
        self.assertEqual(self.status['sources']['robot2']['accepted'], 0)


if __name__ == '__main__':
    unittest.main()
