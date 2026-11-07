"""Guard the non-rescue/final-rescue policy across profiles and generation."""
from pathlib import Path
import os
import subprocess
import sys
import tempfile
import unittest
import yaml
from itertools import product

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/edgeai_navigation/scripts'))
from goal_policy import configure_controllers
from navigation_config import load_navigation


class GoalPolicyConfig(unittest.TestCase):
    def test_rpp_keeps_position_and_heading_goals_separate(self):
        config=load_navigation(ROOT/'src/edgeai_navigation/config','rpp')
        control=config['controller_server']['ros__parameters']
        control['heading_goal_checker']=dict(control['general_goal_checker'])
        control['rescue_goal_checker']=dict(control['general_goal_checker'],xy_goal_tolerance=.10)
        configure_controllers(config)
        self.assertFalse(control['FollowPath']['use_rotate_to_heading'])
        for name in ('FollowPathHeading','FollowPathRescue'):
            self.assertFalse(control[name]['use_rotate_to_heading'])
            self.assertTrue(control[name]['rotate_to_goal_heading'])
            self.assertTrue(control[name]['use_path_orientations'])
            self.assertFalse(control[name]['allow_reversing'])
        self.assertFalse(control['FollowPath']['rotate_to_goal_heading'])
        self.assertEqual(config['local_costmap']['local_costmap']['ros__parameters']['global_frame'],
                         config['global_costmap']['global_costmap']['ros__parameters']['global_frame'])
        self.assertEqual(config['behavior_server']['ros__parameters']['local_frame'],'map')

    def test_behavior_frame_follows_costmap_when_shim_orientation_is_disabled(self):
        config=load_navigation(ROOT/'src/edgeai_navigation/config','rpp')
        control=config['controller_server']['ros__parameters']
        control['FollowPath']['use_path_orientations']=False
        control['heading_goal_checker']=dict(control['general_goal_checker'])
        control['rescue_goal_checker']=dict(control['general_goal_checker'])
        local=config['local_costmap']['local_costmap']['ros__parameters']
        local['global_frame']='odom'
        configure_controllers(config)
        self.assertEqual(config['behavior_server']['ros__parameters']['local_frame'],'odom')

    def test_lattice_reverse_only_for_position(self):
        config=load_navigation(ROOT/'src/edgeai_navigation/config','rpp')
        control=config['controller_server']['ros__parameters']
        control['heading_goal_checker']=dict(control['general_goal_checker'])
        control['rescue_goal_checker']=dict(control['general_goal_checker'])
        config['planner_server']['ros__parameters']['GridBased']['allow_reverse_expansion']=True
        configure_controllers(config)
        self.assertTrue(control['FollowPath']['allow_reversing'])
        self.assertFalse(control['FollowPath']['use_rotate_to_heading'])
        self.assertEqual(control['FollowPath']['plugin'],
                         'nav2_rotation_shim_controller::RotationShimController')
        for name in ('FollowPathHeading','FollowPathRescue'):
            self.assertFalse(control[name]['allow_reversing'])
            self.assertFalse(control[name]['use_rotate_to_heading'])
            self.assertTrue(control[name]['rotate_to_goal_heading'])
            self.assertEqual(control[name]['plugin'],
                             'nav2_rotation_shim_controller::RotationShimController')

    def test_all_controller_profiles_keep_yaw_only_when_requested(self):
        for profile, controller in product(('rectangle','baseline','safe'), ('dwb','mppi')):
            with self.subTest(profile=profile,controller=controller):
                config=load_navigation(ROOT/'src/edgeai_navigation/config',controller,profile)
                control=config['controller_server']['ros__parameters']
                general=control['general_goal_checker']
                control['heading_goal_checker']=dict(general)
                control['rescue_goal_checker']=dict(general,xy_goal_tolerance=.10)
                base=control['FollowPath']
                dwb=base.get('primary_controller',base['plugin'])=='dwb_core::DWBLocalPlanner'
                critic='RotateToGoal' if dwb else 'GoalAngleCritic'
                configure_controllers(config)
                self.assertNotIn(critic,control['FollowPath']['critics'])
                self.assertIn(critic,control['FollowPathHeading']['critics'])
                self.assertIn(critic,control['FollowPathRescue']['critics'])
                if dwb:
                    self.assertEqual(control['FollowPathHeading']['xy_goal_tolerance'],general['xy_goal_tolerance'])
                    self.assertEqual(control['FollowPathRescue']['xy_goal_tolerance'],.10)

    def assert_ordinary(self,config):
        p=config['controller_server']['ros__parameters']['general_goal_checker']
        self.assertEqual(p['plugin'],'nav2_controller::PositionGoalChecker')
        self.assertNotIn('yaw_goal_tolerance',p)
        self.assertTrue(config['planner_server']['ros__parameters']['GridBased']['use_final_approach_orientation'])

    def test_all_exploration_profiles(self):
        for profile in ('rectangle','baseline','safe'):
            for controller in ('dwb','mppi','rpp'):
                with self.subTest(profile=profile,controller=controller):
                    self.assert_ordinary(load_navigation(ROOT/'src/edgeai_navigation/config',controller,profile))

    def test_generator_and_actual_launcher_runtime_config(self):
        with tempfile.TemporaryDirectory() as tmp:
            launcher=(ROOT/'scripts/simulation/run-open-source-exploration.sh').read_text()
            code=launcher.split("<<'PY'\n",1)[1].split('\nPY\n',1)[0]
            active=Path(tmp)/'nav2-runtime.yaml'
            env=dict(os.environ,EDGEAI_CONTROLLER='dwb',EDGEAI_NAV_PROFILE='rectangle',EDGEAI_RESCUE_CONFIG=str(ROOT/'src/edgeai_navigation/config/midterm_single_rescue.yaml'))
            env['EDGEAI_GLOBAL_PLANNER']='navfn'
            env.pop('EDGEAI_MOTION_ABLATION',None)
            subprocess.run([sys.executable,'-',str(ROOT/'src/edgeai_navigation/config/nav2_exploration_rectangle.yaml'),
                            str(active)],input=code,text=True,env=env,check=True)
            config=yaml.safe_load(active.read_text())
            self.assert_ordinary(config)
            rescue=config['controller_server']['ros__parameters']['rescue_goal_checker']
            self.assertEqual(rescue['plugin'],'nav2_controller::StoppedGoalChecker')
            self.assertEqual(rescue['yaw_goal_tolerance'],.25)
            heading=config['controller_server']['ros__parameters']['heading_goal_checker']
            self.assertEqual(heading['plugin'],'nav2_controller::SimpleGoalChecker')
            planner=config['planner_server']['ros__parameters']
            self.assertIn('GridBasedHeading',planner['planner_plugins'])
            self.assertFalse(planner['GridBasedHeading']['use_final_approach_orientation'])
            control=config['controller_server']['ros__parameters']
            self.assertNotIn('RotateToGoal',control['FollowPath']['critics'])
            self.assertNotIn('RotateToGoal.scale',control['FollowPath'])
            for plugin,checker in [('FollowPathHeading','heading_goal_checker'),('FollowPathRescue','rescue_goal_checker')]:
                self.assertIn(plugin,control['controller_plugins'])
                self.assertIn('RotateToGoal',control[plugin]['critics'])
                self.assertEqual(control[plugin]['xy_goal_tolerance'],control[checker]['xy_goal_tolerance'])
            self.assertEqual(control['FollowPath']['xy_goal_tolerance'],control['general_goal_checker']['xy_goal_tolerance'])


if __name__=='__main__':
    unittest.main()
