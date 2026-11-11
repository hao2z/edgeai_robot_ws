from pathlib import Path
import sys
import math
import tempfile
from types import SimpleNamespace as Obj
import unittest
import yaml

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/edgeai_navigation/scripts'))
from cooperative_policy import compose, relative, JointDwell, parking_pairs, rescue_yaw_error, path_clear_of_peer
from fleet_config import namespaced_config
from goal_policy import managed_trees


class CooperativePolicy(unittest.TestCase):
    def test_peer_blocks_path_between_clear_waypoints_and_during_rotation(self):
        body=[(.4,.1),(-.4,.1),(-.4,-.1),(.4,-.1)]
        self.assertFalse(path_clear_of_peer([(-2,0,0),(2,0,0)],(0,0,0),body))
        self.assertTrue(path_clear_of_peer([(-2,1,0),(2,1,0)],(0,0,0),body))
        self.assertFalse(path_clear_of_peer([(0,0,-math.pi/4),(0,0,math.pi/4)],(.76,0,0),body))

    def test_rescue_heading_uses_surface_normal_and_wraps(self):
        self.assertAlmostEqual(rescue_yaw_error([0,0,-math.pi],dict(yaw=0.)),0.)
        self.assertAlmostEqual(rescue_yaw_error([0,0,-math.pi+.3],dict(yaw=0.)),.3)
        self.assertAlmostEqual(rescue_yaw_error([0,0,-math.pi/2],dict(yaw=math.pi/2)),0.)

    def test_configured_wider_parking_pair_retains_rescue_margin(self):
        info=Obj(resolution=.05,width=120,height=120,origin=Obj(position=Obj(x=0.,y=0.),orientation=Obj(x=0.,y=0.,z=0.,w=1.)))
        grid=Obj(info=info,data=[0]*14400)
        config=yaml.safe_load((ROOT/'src/edgeai_navigation/config/cooperative_rescue.yaml').read_text())
        footprint=[[.1125,.165],[-.3125,.165],[-.3125,-.165],[.1125,-.165]]
        pairs=parking_pairs(grid,dict(x=3.,y=3.,yaw=0.),footprint,config)
        self.assertTrue(pairs)
        for a,b in pairs:
            self.assertGreaterEqual(math.dist(a[:2],b[:2]),config['min_separation_m'])
            for p in (a,b):self.assertLessEqual(math.dist(p[:2],[3.,3.]),.550001)
        config['parking_radii_m']=[.4]
        config['min_separation_m']=.65
        pairs=parking_pairs(grid,dict(x=3.,y=3.,yaw=0.),footprint,config)
        self.assertTrue(pairs)
        self.assertTrue(all(abs(math.dist(p[:2],[3.,3.])-.4)<1e-6 for pair in pairs for p in pair))

    def test_relative_initialization_uses_starting_robot_frame(self):
        pose=compose([.45,.55,math.pi/2],[0.,-.60,0.])
        self.assertAlmostEqual(pose[0],1.05)
        self.assertAlmostEqual(pose[1],.55)
        # Propagating through current odometry recovers the original relative prior.
        current=compose(pose,[.4,.1,.3])
        self.assertAlmostEqual(compose(current,relative([.4,.1,.3],[0,0,0]))[0],pose[0])

    def test_joint_time_resets_when_either_robot_exits_or_data_invalid(self):
        dwell=JointDwell(.7,1.)
        for t in (0,.25,.5):self.assertFalse(dwell.update(t,[.5,.6]))
        self.assertFalse(dwell.update(.75,[.5,.8]))
        for t in (1.,1.25,1.5,1.75):self.assertFalse(dwell.update(t,[.5,.6]))
        self.assertTrue(dwell.update(2.,[.5,.6]))
        self.assertFalse(dwell.update(2.25,[.5,.6],False))
        self.assertIsNone(dwell.start)

    def test_clock_rewind_and_missing_robot_reset_joint_time(self):
        dwell=JointDwell(.7,1.)
        dwell.update(5.,[.5,.5]);dwell.update(5.25,[.5,.5])
        self.assertFalse(dwell.update(4.,[.5,.5]))
        self.assertIsNone(dwell.start)
        self.assertFalse(dwell.update(4.25,[.5]))
        self.assertFalse(dwell.update(5.5,[.5,.5]))

    def test_namespace_keeps_shared_map_and_isolates_scans_frames_and_bt(self):
        source=yaml.safe_load((ROOT/'src/edgeai_navigation/config/nav2_exploration_rectangle.yaml').read_text())
        original=source['bt_navigator']['ros__parameters']['robot_base_frame']
        config=namespaced_config(source,'robot2')
        self.assertEqual(config['bt_navigator']['ros__parameters']['robot_base_frame'],'robot2/base_footprint')
        self.assertEqual(config['bt_navigator']['ros__parameters']['odom_topic'],'/robot2/odom')
        self.assertEqual(config['amcl']['ros__parameters']['map_topic'],'/map')
        self.assertEqual(source['bt_navigator']['ros__parameters']['robot_base_frame'],original)
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory);(p/'nav2-runtime.yaml').write_text(yaml.safe_dump(config))
            trees=managed_trees(ROOT/'src/edgeai_navigation/config/behavior_trees/explore_recovery.xml',p)
            self.assertIn('robot_frame="robot2/base_footprint"',Path(trees['rescue']).read_text())

    def test_parking_pair_stays_in_circle_and_avoids_unknown_cells(self):
        info=Obj(resolution=.05,width=120,height=120,origin=Obj(position=Obj(x=0.,y=0.),orientation=Obj(x=0.,y=0.,z=0.,w=1.)))
        grid=Obj(info=info,data=[0]*14400)
        config=dict(radius_m=.7,position_margin_m=.1,min_separation_m=.65)
        footprint=[[.1125,.165],[-.3125,.165],[-.3125,-.165],[.1125,-.165]]
        pairs=parking_pairs(grid,dict(x=3.,y=3.,yaw=0.),footprint,config)
        self.assertTrue(pairs)
        self.assertTrue(all(math.dist(p[:2],[3.,3.])<=.600001 for pair in pairs for p in pair))
        grid.data=[-1]*14400
        with self.assertRaisesRegex(ValueError,'No clear'):parking_pairs(grid,dict(x=3.,y=3.,yaw=0.),footprint,config)


if __name__=='__main__':unittest.main()
