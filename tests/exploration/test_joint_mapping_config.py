"""Build runtime launch descriptions without starting any ROS process."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
import yaml

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/edgeai_navigation'))


class JointRuntimeConfig(unittest.TestCase):
    def test_joint_launch_uses_independent_sensors_shared_map_and_consistent_handover_frames(self):
        from launch import LaunchContext
        from edgeai_nav.fleet_config import namespaced_config
        env=dict(os.environ,EDGEAI_JOINT_MAPPING='true',EDGEAI_COOPERATIVE_RESCUE='true',
                 EDGEAI_CONTROLLER='rpp',EDGEAI_NAV_PROFILE='rectangle')
        with tempfile.TemporaryDirectory() as tmp,patch.dict(os.environ,env):
            directory=Path(tmp);runtime=directory/'nav2-runtime.yaml'
            subprocess.run([sys.executable,str(ROOT/'scripts/ros/prepare_navigation_runtime.py'),
                str(ROOT/'src/edgeai_navigation/config/nav2_exploration_rectangle.yaml'),str(runtime)],check=True)
            subprocess.run([sys.executable, str(ROOT/'src/edgeai_mission/scripts/prepare_mission.py'),
                '--root', str(ROOT), '--directory', str(directory),
                '--rescue', str(ROOT/'src/edgeai_navigation/config/normal_rescue.yaml'),
                '--map', str(directory/'map'), '--duration', '600', '--backend', 'native',
                '--cooperative', 'true', '--robots-file',
                str(ROOT/'src/edgeai_simulation/config/robots_joint_development.yaml'), '--targets-file',
                str(ROOT/'src/edgeai_simulation/config/targets_competition.yaml')], check=True)
            config=json.loads((directory/'mission-runtime.json').read_text())
            self.assertEqual(config['initial_relative_pose'], [0., .8, 0.])
            self.assertEqual(config['fleet']['robots']['robot2']['initial_relative_pose'], [0., .8, 0.])
            self.assertIn(dict(id=10, type='cooperative'), config['required_targets'])
            self.assertEqual(config['fleet']['task_robots']['c1'], ['robot1', 'robot2'])
            spec=importlib.util.spec_from_file_location('joint_launch',ROOT/'src/edgeai_mission/launch/joint_mapping_mission.launch.py')
            launch=importlib.util.module_from_spec(spec);spec.loader.exec_module(launch)
            context=LaunchContext();context.launch_configurations['directory']=str(directory)
            with patch.object(launch,'ExecuteProcess',wraps=launch.ExecuteProcess) as processes:
                actions=launch.setup(context)
            detector=[c.kwargs['cmd'] for c in processes.call_args_list
                      if any('suspect_targets.py' in str(v) for v in c.kwargs['cmd'])]
            self.assertFalse(detector, 'Perception owns the shared image subscriptions')
            self.assertTrue(any(dict(getattr(a, 'launch_arguments', ())).get('initial_relative_pose')
                                == '[0.0, 0.8, 0.0]' for a in actions))
            first=yaml.safe_load(runtime.read_text())
            second=yaml.safe_load((directory/'robot2/nav2-runtime.yaml').read_text())
            for name,c in [('robot1',first),('robot2',second)]:
                self.assertEqual(c['amcl']['ros__parameters']['odom_frame_id'],name+'/odom')
                self.assertEqual(c['local_costmap']['local_costmap']['ros__parameters']['robot_base_frame'],name+'/base_footprint')
                self.assertEqual(c['global_costmap']['global_costmap']['ros__parameters']['global_frame'],'map')
                self.assertEqual(c['velocity_smoother']['ros__parameters']['odom_topic'],'odom')
            self.assertNotIn('depth_layer',second['local_costmap']['local_costmap']['ros__parameters']['plugins'])
            for name in ('robot1','robot2'):
                explorer=yaml.safe_load((directory/name/'frontier-runtime.yaml').read_text())['/**']['ros__parameters']
                self.assertEqual(explorer['completion_event_topic'],'/'+name+'/exploration_complete')
                self.assertEqual(explorer['costmap_topic'],'/' + name + '/global_costmap/costmap')
                self.assertFalse(explorer['autostart'])
                self.assertEqual(explorer['robot_base_frame'],name+'/base_footprint')
                self.assertAlmostEqual(min(explorer['frontier_footprint'][::2]), -.3225)
                self.assertAlmostEqual(max(explorer['frontier_footprint'][1::2]), .175)
                self.assertEqual(len(explorer['frontier_heading_angles']), 16)

    def test_mapping_launch_has_no_navigation_or_task_dependency(self):
        from launch import LaunchContext
        from launch_ros.actions import Node
        from launch.actions import ExecuteProcess
        spec=importlib.util.spec_from_file_location('mapping_launch',ROOT/'src/edgeai_mapping/launch/joint_mapping.launch.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as tmp,patch.dict(os.environ,EDGEAI_RUNTIME_PROFILE='debug'):
            context=LaunchContext()
            context.launch_configurations.update(directory=tmp,body_bounds='[-0.3125,0.1125,-0.165,0.165]',
                                                 initial_relative_pose='[0.0, 0.8, 0.0]')
            with patch.object(module,'Node',wraps=Node) as nodes:
                actions=module.setup(context)
            self.assertEqual(len(actions),5)
            self.assertEqual([c.kwargs['package'] for c in nodes.call_args_list],
                             ['edgeai_mapping','nav2_lifecycle_manager'])
            runtime=yaml.safe_load((Path(tmp)/'joint-runtime.yaml').read_text())['/**']['ros__parameters']
            self.assertEqual(runtime['bounds'],[-.3125,.1125,-.165,.165])
            initializer=yaml.safe_load((Path(tmp)/'joint-initializer.yaml').read_text())['/**']['ros__parameters']
            self.assertEqual(initializer['initial_relative_pose'], [0., .8, 0.])

    def test_performance_profile_omits_only_mapping_diagnostics(self):
        from launch import LaunchContext
        spec=importlib.util.spec_from_file_location('mapping_profile_launch',ROOT/'src/edgeai_mapping/launch/joint_mapping.launch.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        for profile,expected in (('debug',True),('performance',False)):
            with self.subTest(profile=profile), tempfile.TemporaryDirectory() as tmp,patch.dict(
                    os.environ,EDGEAI_RUNTIME_PROFILE=profile):
                context=LaunchContext()
                context.launch_configurations.update(directory=tmp,body_bounds='[-0.3125,0.1125,-0.165,0.165]')
                with patch.object(module,'ExecuteProcess',wraps=module.ExecuteProcess) as processes,patch.object(
                        module,'Node',wraps=module.Node) as nodes:
                    actions=module.setup(context)
                scripts=[Path(c.kwargs['cmd'][2]).name for c in processes.call_args_list]
                self.assertEqual('mapping_diagnostics.py' in scripts,expected)
                self.assertIn('joint_mapping_initializer.py',scripts)
                self.assertIn('joint_mapping_runtime.py',scripts)
                self.assertEqual([c.kwargs['executable'] for c in nodes.call_args_list],
                                 ['joint_slam_toolbox','lifecycle_manager'])
                self.assertEqual(len(actions),5 if expected else 4)

    def test_frontier_monitor_preserves_unavailable_and_stale_states(self):
        spec=importlib.util.spec_from_file_location('monitor',ROOT/'src/edgeai_mission/scripts/joint_frontier_monitor.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        times=dict(robot1=10.,robot2=10.)
        self.assertEqual(module.aggregate(dict(robot1=1,robot2=2),times,11.,2.),3)
        self.assertEqual(module.aggregate(dict(robot1=0,robot2=-1),times,11.,2.),-1)
        self.assertEqual(module.aggregate(dict(robot1=0,robot2=0),times,13.,2.),-1)
        self.assertEqual(module.aggregate(dict(robot1=0),dict(robot1=10.),11.,2.),-1)


if __name__=='__main__':unittest.main()
