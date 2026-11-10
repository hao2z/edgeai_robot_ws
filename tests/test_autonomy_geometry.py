"""Meaningful checks independent of ROS and the practice arena coordinates."""
import math
from types import SimpleNamespace
import unittest
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from edgeai_autonomy.geometry import frontier_goals, observation_goals, coverage_goals, matrix, rescue_goal
from edgeai_autonomy.acceptance import rectangle, polygon_distance, segment_hits_polygon


class GeometryTests(unittest.TestCase):
    def test_transit_can_cross_085m_door_but_rejects_075m_door(self):
        grid = np.zeros((120,120),dtype=int)
        grid[:2,:]=grid[-2:,:]=100
        grid[:,:2]=grid[:,-2:]=100
        grid[:,60:64]=100
        grid[43:77,60:64]=0  # 34 cells * 0.025 m = 0.85 m.
        hint={'id':23,'position':[2.2,1.5,0.2],'normal':[0.,1.,0.]}
        self.assertTrue(observation_goals(grid,.025,(0,0),(.8,1.5),[],[hint],clearance=.40))
        self.assertEqual(observation_goals(grid,.025,(0,0),(.8,1.5),[],[hint],clearance=.60),[])
        grid[43:45,60:64]=grid[75:77,60:64]=100
        self.assertEqual(observation_goals(grid,.025,(0,0),(.8,1.5),[],[hint],clearance=.40),[])

    def test_camera_coverage_continues_after_lidar_map_is_complete(self):
        grid = np.zeros((100,100), dtype=int)
        grid[:2,:] = grid[-2:,:] = 100
        grid[:,:2] = grid[:,-2:] = 100
        self.assertEqual(frontier_goals(grid, 0.05, (0,0), (1.,1.), []), [])
        goals = coverage_goals(grid, 0.05, (0,0), (1.,1.), [])
        self.assertTrue(goals)
        self.assertTrue(all(g['source'] == 'mapped_surface_coverage' for g in goals))
        grid[:,39:42] = 100
        goals = coverage_goals(grid, 0.05, (0,0), (1.,1.), [])
        self.assertTrue(goals)
        self.assertTrue(all(g['x'] < 1.95 for g in goals))

    def test_visual_hint_uses_cleared_incremental_path(self):
        grid = np.full((100,100), -1)
        grid[2:98,2:98] = 0
        hint = {'id': 23, 'position': [3.,3.,0.2], 'normal': [0.,1.,0.]}
        goals = observation_goals(grid, 0.05, (0,0), (1.,1.), [], [hint])
        self.assertTrue(goals)
        self.assertLess(math.hypot(goals[0]['x']-1, goals[0]['y']-1), 1.05)
        self.assertEqual(goals[0]['hint_id'], 23)
        grid[:,39:42] = 100
        self.assertEqual(observation_goals(grid, 0.05, (0,0), (1.,1.), [], [hint]), [])

    def test_segment_occlusion(self):
        box = rectangle(0, 0, math.pi/4, 1, 0.3)
        self.assertTrue(segment_hits_polygon((-2, 0), (2, 0), box))
        self.assertTrue(segment_hits_polygon((0, 0), (2, 0), box))
        self.assertFalse(segment_hits_polygon((-2, 2), (2, 2), box))
        self.assertFalse(segment_hits_polygon((-2, 0), (-1, 0), box))

    def test_clearance_detects_rotation_and_overlap(self):
        a=rectangle(0,0,0,1,1)
        self.assertAlmostEqual(polygon_distance(a,rectangle(2,0,0,1,1)),1.)
        self.assertEqual(polygon_distance(a,rectangle(0.7,0,math.pi/4,1,1)),0.)

    def test_rescue_offset_uses_observed_normal(self):
        target={'position':[2.,3.,0.2],'normal':[-1.,0.,0.]}
        x,y,heading=rescue_goal(target,0.6)
        self.assertAlmostEqual(x,1.4);self.assertAlmostEqual(y,3.)
        self.assertAlmostEqual(heading,0.)
        with self.assertRaises(ValueError):
            rescue_goal({'position':[0.,0.,0.], 'normal':[0.,0.,1.]},0.6)

    def test_frontiers_do_not_cross_wall_or_unknown(self):
        grid=np.full((80,80),-1)
        grid[10:70,10:36]=0
        grid[10:70,44:70]=0
        grid[:,39:41]=100
        goals=frontier_goals(grid,0.05,(0,0),(0.8,1.8),[])
        self.assertTrue(goals)
        for goal in goals:
            self.assertLess(goal['x'],1.8)
            self.assertLess(math.hypot(goal['x']-0.8,goal['y']-1.8),1.05)
            self.assertEqual(grid[int(goal['y']/0.05),int(goal['x']/0.05)],0)
        repeated=frontier_goals(grid,0.05,(0,0),(0.8,1.8),[(g['frontier_x'],g['frontier_y']) for g in goals])
        self.assertEqual(repeated,[])

    def test_frontier_route_can_revisit_a_transit_waypoint(self):
        grid = np.full((100,100), -1)
        grid[2:98,2:98] = 0
        robot = (1.,1.)
        goals = frontier_goals(grid,.05,(0,0),robot,[])
        # Revisiting the first metre must not exclude distant unexplored goals.
        transit = [(g['x'],g['y']) for g in goals
                   if math.hypot(g['frontier_x']-g['x'],g['frontier_y']-g['y']) > 1.]
        self.assertTrue(transit)
        remaining = frontier_goals(grid,.05,(0,0),robot,transit)
        self.assertTrue(remaining)
        self.assertTrue(any(any(math.hypot(g['x']-x,g['y']-y)<.1 for x,y in transit)
                            for g in remaining))

    def test_optical_pnp_map_transform_round_trip(self):
        half=0.12
        obj=np.array([[-half,half,0],[half,half,0],[half,-half,0],[-half,-half,0]],np.float32)
        k=np.array([[554.,0,320.],[0,554.,240.],[0,0,1.]])
        rvec=np.array([math.pi-0.15,0.1,0.05]);tvec=np.array([0.15,-0.05,1.7])
        pixels,_=cv2.projectPoints(obj,rvec,tvec,k,np.zeros(5))
        ok,r,t=cv2.solvePnP(obj,pixels.reshape(4,2),k,np.zeros(5),flags=cv2.SOLVEPNP_IPPE_SQUARE)
        self.assertTrue(ok)
        np.testing.assert_allclose(t.flatten(),tvec,atol=1e-5)
        rotation=Rotation.from_euler('xyz',[-math.pi/2,0,-math.pi/2])
        q=rotation.as_quat()
        tr=SimpleNamespace(rotation=SimpleNamespace(x=q[0],y=q[1],z=q[2],w=q[3]),
                           translation=SimpleNamespace(x=1.,y=2.,z=0.1))
        actual=(matrix(tr)@np.r_[t.flatten(),1])[:3]
        # Optical +z -> body +x; optical +x -> body -y; optical +y -> body -z.
        np.testing.assert_allclose(actual,[2.7,1.85,0.15],atol=1e-5)


if __name__=='__main__':unittest.main()
