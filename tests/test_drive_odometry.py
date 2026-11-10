import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
import numpy as np
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory
from edgeai_autonomy.drive_odometry import AXLE_OFFSET_X, chassis_odometry, axle_goal


class DriveOdometryTests(unittest.TestCase):
    def test_offset_matches_robot_model(self):
        model = Path(get_package_share_directory('edgeai_robot_description'))/'urdf/edgeai_robot_real.urdf.xacro'
        prop = ET.parse(model).getroot().find(".//*[@name='wheel_x']")
        self.assertEqual(AXLE_OFFSET_X, float(prop.attrib['value']))

    def test_pure_spin_keeps_axle_fixed_and_moves_chassis(self):
        for heading in (0., math.pi/2, math.pi, -math.pi/2):
            raw = Odometry()
            raw.pose.pose.orientation.z = math.sin(heading/2)
            raw.pose.pose.orientation.w = math.cos(heading/2)
            raw.twist.twist.angular.z = .3
            corrected = chassis_odometry(raw)
            p = corrected.pose.pose.position
            np.testing.assert_allclose(axle_goal(p.x,p.y,heading), [AXLE_OFFSET_X,0.], atol=1e-12)
            self.assertAlmostEqual(corrected.twist.twist.linear.y, -AXLE_OFFSET_X*.3)
            self.assertEqual(raw.pose.pose.position.x, 0.)
            self.assertEqual(raw.twist.twist.linear.y, 0.)

    def test_straight_motion_and_yaw_covariance(self):
        raw = Odometry()
        raw.pose.pose.orientation.w = 1.
        raw.pose.pose.position.x = .7
        raw.twist.twist.linear.x = .1
        raw.pose.covariance[35] = .04
        corrected = chassis_odometry(raw)
        self.assertAlmostEqual(corrected.pose.pose.position.x, .7)
        self.assertAlmostEqual(corrected.pose.pose.position.y, 0.)
        self.assertAlmostEqual(corrected.twist.twist.linear.x, .1)
        self.assertAlmostEqual(corrected.pose.covariance[7], AXLE_OFFSET_X**2*.04)
        self.assertEqual(corrected.header.frame_id, 'odom')
        self.assertEqual(corrected.child_frame_id, 'base_footprint')


if __name__ == '__main__':
    unittest.main()
