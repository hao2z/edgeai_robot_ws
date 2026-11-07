"""Timestamp pairing and invalid-data behavior, with real ROS messages/TF buffer."""
import json
import math
from pathlib import Path
import sys
import time
import unittest
from unittest.mock import Mock

root = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(root/'src/edgeai_simulation/tools'))
import rclpy
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker
from truth_diagnostics import TruthDiagnostics


def transform(parent, child, seconds, x=0., y=0., angle=0.):
    msg = TransformStamped()
    msg.header.frame_id, msg.child_frame_id = parent, child
    msg.header.stamp.sec = int(seconds)
    msg.header.stamp.nanosec = round((seconds-int(seconds))*1e9)
    msg.transform.translation.x, msg.transform.translation.y = float(x), float(y)
    msg.transform.rotation.z, msg.transform.rotation.w = math.sin(angle/2), math.cos(angle/2)
    return msg


class TruthTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(args=['--ros-args', '-p', 'world_file:='+str(root/'src/edgeai_simulation/worlds/midterm.sdf'),
                        '-p', 'targets_file:='+str(root/'src/edgeai_simulation/config/targets_midterm.yaml')])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = TruthDiagnostics()
        self.node.summary = Mock()
        self.node.markers = Mock()

    def tearDown(self):
        self.node.destroy_node()

    def read(self):
        self.node.tick()
        return json.loads(self.node.summary.publish.call_args.args[0].data)

    def populate(self, truth_stamp=1.5):
        self.node.receive_tf(TFMessage(transforms=[
            transform('map', 'odom', 1), transform('map', 'odom', 2),
            transform('odom', 'base_footprint', 1, x=2),
            transform('odom', 'base_footprint', 2, x=4)]))
        self.node.receive_truth(TFMessage(transforms=[
            transform('world', 'edgeai_robot_real', truth_stamp, 10, 20, math.pi/2)]))

    def test_interpolates_tf_at_truth_stamp_instead_of_latest(self):
        self.populate()
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x, goal.pose.orientation.w = 4., 1.
        self.node.receive_goal(goal)
        row = self.read()
        self.assertEqual(row['status'], 'ok')
        self.assertAlmostEqual(row['robot_estimated'][0], 3.)
        self.assertAlmostEqual(row['goal_world'][0], 10.)
        self.assertAlmostEqual(row['goal_world'][1], 21.)

    def test_future_truth_has_no_latest_tf_fallback(self):
        self.populate(truth_stamp=2.5)
        self.node.sim_time = 2.6
        row = self.read()
        self.assertEqual(row['status'], 'unavailable')
        self.assertNotIn('world_from_reference', row)
        self.assertEqual([m.action for m in self.node.markers.publish.call_args.args[0].markers],
                         [Marker.DELETEALL])

    def test_stopped_stream_is_invalid_even_when_sim_time_stops(self):
        self.populate()
        self.assertEqual(self.read()['status'], 'ok')
        self.node.truth_received = time.monotonic()-4
        self.assertIn('stream stopped', self.read()['reason'])


if __name__ == '__main__':
    unittest.main()
