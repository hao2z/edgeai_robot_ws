"""Construct real launch actions without starting Gazebo or ROS nodes."""
import importlib.util
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
import yaml

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location(
    'simulation_launch', ROOT/'src/edgeai_simulation/launch/simulation_real.launch.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class SimulationLaunchConfig(unittest.TestCase):
    def construct(self, temporary, clock_rate='100', **environment):
        share = ROOT/'src/edgeai_simulation'
        context = LaunchContext()
        context.launch_configurations.update(
            robot_count='2', multi_robot='true',
            robots_file=str(share/'config/robots_midterm.yaml'),
            targets_file=str(share/'config/targets_midterm.yaml'),
            world_file=str(share/'worlds/midterm.sdf'), imu_file=str(share/'config/imu.yaml'),
            gui='true', truth='true', navigation_test='false', slam='true',
            use_robot_pose='true', rviz='false', clock_rate=clock_rate)
        def package(name):
            local = ROOT/'src'/name
            return str(local) if local.is_dir() else get_package_share_directory(name)
        with patch.object(module, 'get_package_share_directory', side_effect=package), \
                patch.object(module.tempfile, 'mkdtemp', return_value=temporary), \
                patch.object(module, 'Node', wraps=module.Node) as nodes, \
                patch.object(module, 'ExecuteProcess', wraps=module.ExecuteProcess) as processes, \
                patch.dict(os.environ, environment):
            actions = module.setup(context)
            self.node_specs = [call.kwargs for call in nodes.call_args_list]
            self.process_specs = [call.kwargs for call in processes.call_args_list]
            return actions

    def test_two_robot_slam_launch_with_lazy_cloud(self):
        with tempfile.TemporaryDirectory() as temporary:
            actions = self.construct(temporary, EDGEAI_EAGER_POINT_CLOUD='false', EDGEAI_JOINT_MAPPING='false')
            directory = Path(temporary)
            cloud = yaml.safe_load((directory/'robot1-cloud-bridge.yaml').read_text())
            self.assertTrue(cloud[0]['lazy'])
            self.assertEqual(cloud[0]['ros_topic_name'], '/robot1/camera/points')
            self.assertFalse((directory/'robot2-cloud-bridge.yaml').exists())
            filtering = yaml.safe_load((directory/'fleet-filter.yaml').read_text())
            self.assertIn('relative_pose', filtering['fleet_scan_filter']['ros__parameters'])
            self.assertEqual(filtering['fleet_scan_filter']['ros__parameters']['root_odom_frame'], 'robot1/odom')
            command = next(p['cmd'] for p in self.process_specs if any('fleet_scan_filter.py' in str(a) for a in p['cmd']))
            for remap in ('scan:=/robot1/scan', 'odom:=/robot1/odom',
                          'scan_slam:=/robot1/scan_slam', 'robot2/odom:=/robot2/odom'):
                self.assertIn(remap, command)
            slam = next(n for n in self.node_specs if n.get('executable') == 'anchored_slam_toolbox')
            self.assertEqual(slam['parameters'][0]['scan_topic'], '/robot1/scan_slam')
            self.assertTrue(actions)
            self.assertIsNotNone(ET.parse(directory/'arena.sdf').find('world'))
            clocks = [n for n in self.node_specs if n.get('name') in ('clock_bridge', 'simulation_clock')]
            self.assertEqual(len(clocks), 2)
            self.assertEqual(clocks[0]['remappings'], [('/clock', '/simulation/clock_raw')])
            self.assertEqual(clocks[1]['arguments'], ['messages', '/simulation/clock_raw', '100.0', '/clock'])
            self.assertEqual(clocks[1]['parameters'][0], dict(use_sim_time=False, use_wall_clock=True, lazy=False))

    def test_camera_retains_original_30_hz_and_follower_has_no_camera(self):
        for value in (None, '30', 'invalid'):
            with self.subTest(value=value), tempfile.TemporaryDirectory() as temporary, patch.dict(os.environ):
                os.environ.pop('EDGEAI_CAMERA_RATE', None)
                settings = {} if value is None else dict(EDGEAI_CAMERA_RATE=value)
                self.construct(temporary, **settings)
                robots = [n for n in self.node_specs if n.get('name') == 'robot_state_publisher']
                first, second = [ET.fromstring(n['parameters'][0]['robot_description']) for n in robots]
                camera = first.find(".//sensor[@name='rgbd_camera']")
                self.assertEqual(float(camera.findtext('update_rate')), 30.)
                self.assertIsNone(second.find(".//sensor[@name='rgbd_camera']"))
                self.assertEqual(first.findtext(".//sensor[@name='lidar_sensor']/update_rate"), '20')

    def test_unthrottled_clock_is_explicitly_available(self):
        with tempfile.TemporaryDirectory() as temporary:
            self.construct(temporary, clock_rate='0')
            clocks = [n for n in self.node_specs if n.get('name') in ('clock_bridge', 'simulation_clock')]
            self.assertEqual(len(clocks), 1)
            self.assertEqual(clocks[0]['remappings'], [])

    def test_invalid_clock_rate_rejected_before_launch(self):
        for value in ('nan', '-1', '5'):
            with self.subTest(value=value), tempfile.TemporaryDirectory() as temporary:
                with self.assertRaises(ValueError):
                    self.construct(temporary, clock_rate=value)

    def test_eager_cloud_does_not_create_lazy_bridge(self):
        with tempfile.TemporaryDirectory() as temporary:
            self.construct(temporary, EDGEAI_EAGER_POINT_CLOUD='true')
            self.assertFalse((Path(temporary)/'robot1-cloud-bridge.yaml').exists())


if __name__ == '__main__':
    unittest.main()
