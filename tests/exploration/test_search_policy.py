from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'src/edgeai_navigation/scripts'))
from edgeai_search.search_policy import observation_positions


class SearchPolicyTests(unittest.TestCase):
    def test_unknown_and_disconnected_rooms_are_excluded(self):
        width = height = 40
        data = [0]*(width*height)
        for y in range(height):
            data[y*width+20] = 100  # separates the two rooms
        for y in range(10):
            for x in range(10):
                data[y*width+x] = -1
        points = observation_positions(data, width, height, .1, (0.,0.),
                                       (1.,2.), .3, .7, 6)
        self.assertGreater(len(points), 1)
        self.assertTrue(all(x < 1.7 for x,y in points))
        self.assertTrue(all(x > 1.3 or y > 1.3 for x,y in points))
        self.assertLessEqual(len(points), 6)

    def test_no_clearance_returns_no_viewpoints(self):
        self.assertEqual(observation_positions([-1]*100,10,10,.1,(0.,0.),(.5,.5)), [])

    def test_robot_near_wall_can_search_clear_interior(self):
        width = height = 30
        data = [0]*(width*height)
        for y in range(height):
            data[y*width] = 100
        points = observation_positions(data,width,height,.1,(0.,0.),(.2,1.5),.42,.7,6)
        self.assertGreater(len(points), 1)
        self.assertTrue(all(x >= .55 for x,y in points))

    def test_clear_endpoints_across_narrow_known_passage_are_proposed_to_nav2(self):
        width, height = 50, 30
        data = [0]*(width*height)
        for y in range(height):
            if not 12 <= y <= 17:
                data[y*width+25] = 100
        points = observation_positions(data,width,height,.1,(0.,0.),(1.,1.5),.42,.8,6)
        self.assertTrue(any(x > 3. for x,y in points))
        self.assertTrue(all(abs(x-2.55) >= .4 for x,y in points))

    def test_views_are_spatially_separated(self):
        import math
        points = observation_positions([0]*3600,60,60,.1,(-1.,-1.),(1.,2.),.4,1.,6)
        self.assertEqual(len(points),6)
        for i,p in enumerate(points):
            self.assertTrue(all(math.dist(p,q) >= 1. for q in points[:i]))


if __name__ == '__main__':
    unittest.main()
