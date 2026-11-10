"""Safety behavior: lost commands/sensors/mission heartbeat must output zero."""
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import inspect
import math
from pathlib import Path
import yaml
import json
from edgeai_autonomy.drive_odometry import AXLE_OFFSET_X
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from edgeai_autonomy.guard import Guard
from edgeai_autonomy.geometry import frontier_goals


class GuardTests(unittest.TestCase):
    def test_lidar_offset_obstacle_triggers_stop(self):
        # A point at (0.28, 0.21) in base coordinates is inside the guard.
        # The sensor itself is mounted at (0.10, 0.08).
        scan = LaserScan()
        scan.angle_min = math.atan2(0.13, 0.18)
        scan.angle_increment = 0.01
        scan.range_min = 0.05
        scan.ranges = [math.hypot(0.18, 0.13)]
        state = self.state()
        for name in ('STOP_FRONT', 'STOP_REAR', 'STOP_HALF_WIDTH'):
            setattr(state, name, getattr(Guard, name))
        Guard.scan(state, scan)
        self.assertTrue(state.blocked)
        scan.ranges = [1.0]
        Guard.scan(state, scan)
        self.assertFalse(state.blocked)

    def test_transit_and_observation_have_separate_safety_margins(self):
        config = yaml.safe_load((Path(get_package_share_directory('edgeai_autonomy'))/'config/nav2.yaml').read_text())
        practice = yaml.safe_load((Path(get_package_share_directory('edgeai_autonomy'))/'config/practice.yaml').read_text())
        guard_radius = math.hypot(max(Guard.STOP_FRONT, Guard.STOP_REAR), Guard.STOP_HALF_WIDTH)
        goal_tolerance = config['controller_server']['ros__parameters']['general_goal_checker']['xy_goal_tolerance']
        exploration_clearance = inspect.signature(frontier_goals).parameters['clearance'].default
        self.assertEqual(exploration_clearance, practice['exploration_clearance_m'])
        self.assertGreater(practice['observation_clearance_m'], guard_radius+goal_tolerance+0.10)
        for name in ('local_costmap', 'global_costmap'):
            params = config[name][name]['ros__parameters']
            footprint = json.loads(params['footprint'])
            xs = [point[0]+AXLE_OFFSET_X for point in footprint]
            ys = [point[1] for point in footprint]
            self.assertGreaterEqual(max(xs), Guard.STOP_FRONT)
            self.assertLessEqual(min(xs), -Guard.STOP_REAR)
            self.assertGreaterEqual(max(ys), Guard.STOP_HALF_WIDTH)
            self.assertLessEqual(min(ys), -Guard.STOP_HALF_WIDTH)
            self.assertGreater(params['footprint_padding'], math.sqrt(2)*params['resolution']/2)
            self.assertEqual(params['robot_base_frame'], 'drive_axle')
            self.assertGreater(params['inflation_layer']['inflation_radius'], guard_radius)
            self.assertGreater(exploration_clearance, guard_radius)

    def state(self):
        raw=Twist();raw.linear.x=0.8;raw.angular.z=1.2
        return SimpleNamespace(raw=raw,command_at=10.,scan_at=10.,heartbeat_at=10.,
                               blocked=False,publisher=Mock(),status=Mock())

    @patch('edgeai_autonomy.guard.time.monotonic',return_value=10.)
    def test_limits_fresh_velocity(self, clock):
        state=self.state();Guard.tick(state)
        output=state.publisher.publish.call_args.args[0]
        self.assertEqual(output.linear.x,0.15)
        self.assertEqual(output.angular.z,0.4)

    @patch('edgeai_autonomy.guard.time.monotonic',return_value=10.)
    def test_each_lost_input_stops_robot(self, clock):
        for field in ['command_at','scan_at','heartbeat_at']:
            with self.subTest(field=field):
                state=self.state();setattr(state,field,0.)
                Guard.tick(state)
                output=state.publisher.publish.call_args.args[0]
                self.assertEqual(output.linear.x,0.)
                self.assertEqual(output.angular.z,0.)
                self.assertEqual(state.status.publish.call_args.args[0].data,'stale_input')

    @patch('edgeai_autonomy.guard.time.monotonic',return_value=10.)
    def test_obstacle_overrides_fresh_navigation(self, clock):
        state=self.state();state.blocked=True;Guard.tick(state)
        output=state.publisher.publish.call_args.args[0]
        self.assertEqual(output.linear.x,0.)
        self.assertEqual(output.angular.z,0.)
        self.assertEqual(state.status.publish.call_args.args[0].data,'footprint_obstacle')


if __name__=='__main__':unittest.main()
