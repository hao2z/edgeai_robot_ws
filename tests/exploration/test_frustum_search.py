"""Functional camera-direction, height, occlusion and feedback checks, no ROS."""
from pathlib import Path
import math
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'src/edgeai_navigation/scripts'))
from edgeai_search.frustum_search import Camera,FrustumPlanner,line_clear,settings

FORWARD=(0,0,1,-1,0,0,0,-1,0)  # optical +Z = base +X


class FrustumTests(unittest.TestCase):
    def planner(self,data=None,heights=None,positions=None):
        return FrustumPlanner([0]*2500 if data is None else data,50,50,.1,(-2.5,-2.5),
            positions or [(-1.,0.),(1.,0.)],Camera(640,480,400.,400.,320.,240.,(0.,0.,.35),FORWARD),
            dict(heights_m=heights or [.35],sample_step_m=.2,far_m=2.))

    def target_keys(self,p,view,xy):
        index=int(np.argmin(np.linalg.norm(p.xy-xy,axis=1)))
        return {k for k in p.visible_keys(view) if k//(len(p.c['heights_m'])*p.c['direction_bins'])==index}

    def test_camera_heading_and_range_limit_visibility(self):
        p=self.planner()
        self.assertTrue(self.target_keys(p,(-1.,0.,0.),(0.,0.)))
        self.assertFalse(self.target_keys(p,(-1.,0.,math.pi),(0.,0.)))
        self.assertFalse(self.target_keys(p,(-1.,0.,0.),(2.,0.)))

    def test_vertical_frustum_rejects_wrong_height(self):
        p=self.planner(heights=[2.])
        self.assertFalse(self.target_keys(p,(-1.,0.,0.),(0.,0.)))

    def test_obstacle_and_unknown_block_line_of_sight(self):
        for value in (100,-1):
            data=[0]*2500
            for y in range(50):
                data[y*50+25]=value
            p=self.planner(data)
            self.assertFalse(self.target_keys(p,(-1.,0.,0.),(1.,0.)))
        data=[0]*9;data[1]=100
        self.assertFalse(line_clear(data,3,3,(0,0),(2,2)))

    def test_opposite_sides_of_same_location_keep_distinct_coverage(self):
        p=self.planner()
        west=self.target_keys(p,(-1.,0.,0.),(0.,0.))
        east=self.target_keys(p,(1.,0.,math.pi),(0.,0.))
        self.assertTrue(west and east)
        self.assertFalse(west&east)
        self.assertEqual({k//4 for k in west},{k//4 for k in east})

    def test_planning_does_not_credit_observation(self):
        p=self.planner()
        first=p.next_view((-1.,0.,0.))
        self.assertGreater(first['gain_m2'],0)
        self.assertFalse(p.covered)
        p.record_sweep((-1.,0.,0.))
        self.assertTrue(p.covered)
        second=p.next_view((-1.,0.,0.))
        self.assertNotEqual((first['x'],first['y']),(second['x'],second['y']))
        self.assertIsNone(p.next_view((-1.,0.,0.)))

    def test_best_heading_faces_into_open_room(self):
        p=self.planner(positions=[(-2.,0.)])
        candidate=p.next_view((-2.,0.,math.pi))
        self.assertLessEqual(abs(candidate['yaw']),math.pi/4)

    def test_bad_config_and_calibration_are_rejected(self):
        for c in [dict(yaw_samples=0),dict(far_m=.1,near_m=.2),dict(heights_m=[]),
                  dict(sample_step_m=.01),dict(enabled='yes'),dict(typo=1)]:
            with self.assertRaises(ValueError):
                settings(c)
        with self.assertRaises(ValueError):
            Camera(640,480,0.,400.,320.,240.,(0.,0.,.35),FORWARD)


if __name__=='__main__':
    unittest.main()
