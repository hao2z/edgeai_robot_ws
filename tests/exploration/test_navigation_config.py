"""No ROS processes: check composition, isolation and actual launcher generation."""
from pathlib import Path
import os
import subprocess
import sys
import tempfile
import unittest
import yaml

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT/'src/edgeai_navigation/config'
sys.path.insert(0, str(ROOT/'src/edgeai_navigation/scripts'))
from navigation_config import load_navigation


class NavigationConfig(unittest.TestCase):
    def test_global_dynamic_obstacles_merge_before_shared_inflation(self):
        from fleet_config import namespaced_config
        for controller in ('rpp','dwb','mppi'):
            config=load_navigation(CONFIG,controller)
            for namespace,derived in [('',config),('robot2',namespaced_config(config,'robot2'))]:
                params=derived['global_costmap']['global_costmap']['ros__parameters']
                self.assertEqual(params['plugins'],['static_layer','obstacle_layer','inflation_layer'])
                self.assertEqual(params['obstacle_layer']['combination_method'],1)
                scan=params['obstacle_layer']['scan']
                self.assertEqual(scan['topic'],'/robot2/scan' if namespace else '/scan')
                self.assertTrue(scan['marking']);self.assertTrue(scan['clearing'])

    def test_common_config_has_no_controller_tuning(self):
        common = yaml.safe_load((CONFIG/'nav2_exploration_rectangle.yaml').read_text())
        self.assertNotIn('FollowPath', common['controller_server']['ros__parameters'])
        for controller in ('dwb', 'rpp', 'mppi'):
            config = load_navigation(CONFIG, controller)
            follow = config['controller_server']['ros__parameters']['FollowPath']
            if controller == 'dwb':
                self.assertIn('critics', follow)
                self.assertNotIn('lookahead_dist', follow)
            elif controller == 'rpp':
                self.assertNotIn('critics', follow)
                self.assertIn('lookahead_dist', follow)
            else:
                self.assertEqual(follow['plugin'], 'nav2_mppi_controller::MPPIController')
                self.assertNotIn('vx_samples', follow)

    def test_profiles_do_not_mutate_next_load(self):
        safe = load_navigation(CONFIG, 'dwb', 'safe')
        safe['controller_server']['ros__parameters']['FollowPath']['max_vel_x'] = 999
        self.assertNotEqual(load_navigation(CONFIG)['controller_server']['ros__parameters']['FollowPath']['max_vel_x'], 999)
        self.assertEqual(load_navigation(CONFIG, 'dwb', 'safe'), load_navigation(CONFIG, 'dwb', 'conservative'))
        for args in [('unknown','rectangle'), ('dwb','unknown')]:
            with self.assertRaises(ValueError):
                load_navigation(CONFIG, *args)

    def test_launcher_generates_all_profiles_without_starting_ros(self):
        launcher = (ROOT/'scripts/simulation/run-open-source-exploration.sh').read_text()
        code = launcher.split("<<'PY'\n", 1)[1].split('\nPY\n', 1)[0]
        cases = [(profile, controller, 'navfn')
                 for profile in ('rectangle', 'baseline', 'safe')
                 for controller in ('dwb', 'rpp', 'mppi')]
        try:
            from ament_index_python.packages import get_package_share_directory
            get_package_share_directory('nav2_smac_planner')
        except (ImportError, LookupError):
            pass
        else:
            cases += [('rectangle', controller, 'lattice') for controller in ('dwb', 'rpp', 'mppi')]
        for profile, controller, planner in cases:
            with self.subTest(profile=profile, controller=controller, planner=planner), tempfile.TemporaryDirectory() as tmp:
                output = Path(tmp)/'nav2-runtime.yaml'
                env = dict(os.environ, EDGEAI_CONTROLLER=controller, EDGEAI_NAV_PROFILE=profile,
                           EDGEAI_GLOBAL_PLANNER=planner, EDGEAI_RESCUE_CONFIG=str(CONFIG/'normal_rescue.yaml'))
                for key in ('EDGEAI_MOTION_ABLATION', 'EDGEAI_POINT_TEST_PARAMS'):
                    env.pop(key, None)
                subprocess.run([sys.executable, '-', str(CONFIG/'nav2_exploration_rectangle.yaml'), str(output)],
                               input=code, text=True, env=env, check=True)
                config = yaml.safe_load(output.read_text())
                control = config['controller_server']['ros__parameters']
                self.assertEqual(set(control['controller_plugins']),
                                 {'FollowPath','FollowPathHeading','FollowPathRescue',
                                  'FollowPathFrontier','FollowPathCooperative'})
                primary = control['FollowPath'].get('primary_controller', control['FollowPath']['plugin'])
                expected = {'dwb':'dwb_core::DWBLocalPlanner',
                            'rpp':'edgeai_navigation::MatchedRPP',
                            'mppi':'nav2_mppi_controller::MPPIController'}[controller]
                self.assertEqual(primary, expected)
                if controller=='rpp':
                    self.assertTrue(output.with_name('motion-limits.json').is_file())
                    smooth=config['velocity_smoother']['ros__parameters']
                    for name in control['controller_plugins']:
                        follow=control[name]
                        self.assertLessEqual(follow['motion_angular_limit'],smooth['max_velocity'][2])
                        self.assertLessEqual(follow['motion_reverse_limit'],-smooth['min_velocity'][0])
                if planner == 'lattice':
                    self.assertEqual(config['planner_server']['ros__parameters']['GridBased']['plugin'],
                                     'edgeai_navigation::XYLatticePlanner')
                    self.assertIn('footprint', config['global_costmap']['global_costmap']['ros__parameters'])


if __name__ == '__main__':
    unittest.main()
