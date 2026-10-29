"""Cross-module constraints for the predictive circular comparison controller."""
from pathlib import Path
import unittest
import yaml

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT/'src/edgeai_navigation/config'


class PredictiveProfileTests(unittest.TestCase):
    def test_shared_geometry_and_actuator_limits(self):
        baseline = yaml.safe_load((CONFIG/'nav2_exploration_rectangle.yaml').read_text())
        config = yaml.safe_load((CONFIG/'nav2_exploration_rectangle_mppi.yaml').read_text())
        c = config['controller_server']['ros__parameters']
        f = c['FollowPath']
        s = config['velocity_smoother']['ros__parameters']
        self.assertEqual(f['plugin'], 'nav2_mppi_controller::MPPIController')
        # Profiles share a planner policy, but independently tuned tolerances and
        # expected frequencies need not match.
        self.assertEqual(config['collision_monitor'], baseline['collision_monitor'])
        for profile in (baseline, config):
            planner = profile['planner_server']['ros__parameters']['GridBased']
            self.assertEqual(planner['plugin'], 'nav2_navfn_planner::NavfnPlanner')
            self.assertTrue(planner['allow_unknown'])
            self.assertFalse(planner['use_astar'])
            self.assertGreaterEqual(planner['tolerance'], 0.0)
        self.assertEqual(f['vx_max'], s['max_velocity'][0])
        self.assertEqual(f['vx_min'], s['min_velocity'][0])
        self.assertEqual(f['wz_max'], s['max_velocity'][2])
        self.assertEqual(f['ax_max'], s['max_accel'][0])
        self.assertEqual(f['ax_min'], s['max_decel'][0])
        self.assertEqual(f['az_max'], s['max_accel'][2])
        self.assertAlmostEqual(f['model_dt'], 1/c['controller_frequency'])
        horizon = f['time_steps']*f['model_dt']
        local = config['local_costmap']['local_costmap']['ros__parameters']
        self.assertGreater(horizon, 1.7)
        self.assertLess(horizon*f['vx_max'] + 0.37, min(local['width'], local['height'])/2)
        self.assertFalse(f['CostCritic']['consider_footprint'])
        for section in ('local_costmap', 'global_costmap'):
            geometry = config[section][section]['ros__parameters']
            self.assertNotIn('footprint', geometry)
            self.assertEqual(geometry['robot_radius'], 0.37)
            self.assertEqual(geometry['footprint_padding'], 0.0)
            self.assertGreaterEqual(geometry['inflation_layer']['inflation_radius'],
                                    geometry['robot_radius'])
        self.assertFalse(f['enforce_path_inversion'])
        self.assertFalse(f['PathAlignCritic']['use_path_orientations'])
        self.assertEqual(f['PathAngleCritic']['mode'], 0)
        self.assertTrue(f['regenerate_noises'])
        self.assertLess(f['PreferForwardCritic']['cost_weight'], f['PathFollowCritic']['cost_weight'])


if __name__ == '__main__':
    unittest.main()
