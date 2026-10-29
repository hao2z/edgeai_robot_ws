from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'src/edgeai_perception/scripts'))
from scan_policy import open_disk, new_view


class ScanPolicyTests(unittest.TestCase):
    def test_free_space_and_inflation(self):
        a=np.full((40,40),60)
        self.assertTrue(open_disk(a,40,40,.05,(-1,-1),(0,0),.5))
    def test_wall_unknown_and_map_edge(self):
        for value in (100,-1):
            a=np.zeros((40,40),dtype=int);a[20,29]=value
            self.assertFalse(open_disk(a,40,40,.05,(-1,-1),(0,0),.5))
        a=np.zeros((40,40),dtype=int)
        self.assertFalse(open_disk(a,40,40,.05,(-1,-1),(.8,0),.5))
    def test_corner_intersection_is_checked(self):
        a=np.zeros((40,40),dtype=int);a[27,27]=100
        self.assertFalse(open_disk(a,40,40,.05,(-1,-1),(0,0),.5))
    def test_narrow_door_not_open_rotation_disk(self):
        a=np.zeros((60,60),dtype=int)
        a[:,22]=100;a[:,37]=100
        self.assertFalse(open_disk(a,60,60,.05,(-1.5,-1.5),(0,0),.5))
    def test_view_spacing(self):
        self.assertFalse(new_view((.5,0),[(0,0)],1.2))
        self.assertTrue(new_view((1.3,0),[(0,0)],1.2))


if __name__=='__main__':unittest.main()
