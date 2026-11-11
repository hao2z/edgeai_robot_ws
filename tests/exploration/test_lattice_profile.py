from pathlib import Path
import json,sys,unittest
import yaml
from ament_index_python.packages import get_package_share_directory
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/edgeai_navigation/scripts'))
from planner_profile import configure_lattice
from navigation_config import load_navigation
from goal_policy import configure_controllers

class LatticeProfile(unittest.TestCase):
    def test_native_heading_and_free_heading_use_same_diff_geometry(self):
        resource=ROOT/'src/edgeai_navigation/config'
        config=load_navigation(resource)
        configure_lattice(config,resource,get_package_share_directory('nav2_smac_planner'))
        planners=config['planner_server']['ros__parameters']
        self.assertEqual(planners['GridBased']['plugin'],'edgeai_navigation::XYLatticePlanner')
        self.assertNotIn('heading_search_time',planners['GridBased'])
        self.assertEqual(planners['GridBasedHeading']['plugin'],'nav2_smac_planner::SmacPlannerLattice')
        self.assertEqual(planners['GridBased']['goal_heading_mode'],'ALL_DIRECTION')
        self.assertEqual(planners['GridBased']['goal_heading_count'],4)
        self.assertNotIn('goal_heading_count',planners['GridBasedHeading'])
        self.assertEqual(planners['GridBased']['heading_search_budget_s'],2.)
        self.assertNotIn('goal_heading_mode',planners['GridBasedHeading'])
        metadata=json.loads(Path(planners['GridBased']['lattice_filepath']).read_text())['lattice_metadata']
        self.assertEqual(metadata['motion_model'],'diff')
        self.assertEqual(metadata['grid_resolution'],.05)
        self.assertTrue(planners['GridBased']['allow_reverse_expansion'])
        self.assertFalse(planners['GridBasedHeading']['allow_reverse_expansion'])
        local=config['local_costmap']['local_costmap']['ros__parameters']
        glob=config['global_costmap']['global_costmap']['ros__parameters']
        self.assertEqual(local['footprint'],glob['footprint'])
        self.assertEqual(glob['footprint_padding'],local['footprint_padding'])
        self.assertEqual(glob['footprint_padding'],.01)
        self.assertFalse(config['controller_server']['ros__parameters']['FollowPath']['use_path_orientations'])

    def test_peer_and_parking_checks_derive_runtime_padding(self):
        from edgeai_nav.fleet_config import configure_peer_costmaps,parking_geometry
        config=load_navigation(ROOT/'src/edgeai_navigation/config')
        grid=config['global_costmap']['global_costmap']['ros__parameters']
        grid['footprint_padding']=.017
        configure_peer_costmaps(config,'',{})
        settings={'parked_peer_clearance_m':.02}
        parking_geometry(config,settings)
        self.assertEqual(settings['footprint_padding_m'],.017)
        self.assertAlmostEqual(settings['parking_peer_clearance_m'],.034+2**-.5*.05)
        for name in ('local_costmap','global_costmap'):
            layer=config[name][name]['ros__parameters']['peer_footprint']
            self.assertEqual(layer['padding_m'],.017)

    def test_resolution_mismatch_fails_before_startup(self):
        resource=ROOT/'src/edgeai_navigation/config'
        config=load_navigation(resource)
        config['global_costmap']['global_costmap']['ros__parameters']['resolution']=.1
        with self.assertRaisesRegex(ValueError,'resolution'):
            configure_lattice(config,resource,get_package_share_directory('nav2_smac_planner'))

    def test_rpp_reverse_switches_agree_with_planner(self):
        resource=ROOT/'src/edgeai_navigation/config'
        for enabled in (True,False):
            config=load_navigation(resource,'rpp')
            control=config['controller_server']['ros__parameters']
            control['heading_goal_checker']=dict(control['general_goal_checker'])
            control['rescue_goal_checker']=dict(control['general_goal_checker'])
            configure_lattice(config,resource,get_package_share_directory('nav2_smac_planner'))
            import tempfile,shutil,yaml
            with tempfile.TemporaryDirectory() as directory:
                profiles=Path(directory)/'tasks'
                shutil.copytree(resource/'navigation_tasks',profiles)
                for name in ('position','frontier'):
                    p=profiles/(name+'.yaml');settings=yaml.safe_load(p.read_text())
                    settings['allow_reverse']=enabled;p.write_text(yaml.safe_dump(settings))
                configure_controllers(config,profiles)
            planners=config['planner_server']['ros__parameters']
            self.assertEqual(planners['GridBased']['allow_reverse_expansion'],enabled)
            self.assertEqual(control['FollowPath']['allow_reversing'],enabled)
            self.assertFalse(planners['GridBasedHeading']['allow_reverse_expansion'])
            for name in ('FollowPathHeading','FollowPathRescue'):
                self.assertFalse(control[name]['allow_reversing'])
                # Shim owns heading alignment; the nested RPP must not repeat it.
                self.assertTrue(control[name]['rotate_to_goal_heading'])
                self.assertFalse(control[name]['use_rotate_to_heading'])

if __name__=='__main__':unittest.main()
