from pathlib import Path
import sys
import unittest
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'src/edgeai_perception/scripts'))
from suspect_geometry import panels, prefer_surface_normal, laser_support


def scene(width=80,black=True):
    rgb=np.full((240,320,3),35,np.uint8)
    left=160-width//2
    rgb[80:160,left:left+width]=240
    if black and width>20:rgb[96:144,left+16:left+width-16]=5
    depth=np.full((240,320),np.nan,np.float32)
    depth[80:160,left:left+width]=1.
    k=np.array([[400.,0,160],[0,400.,120],[0,0,1]])
    r=np.array([[0,0,1],[-1,0,0],[0,-1,0]])
    return rgb,depth,k,r,[0,0,.25]


class Panels(unittest.TestCase):
    def test_surface_normal_survives_later_edge_observation(self):
        surface=dict(normal_kind='surface',normal=[1.,0.],normal_ambiguous=False,width_m=.18,confidence=.85)
        edge=dict(normal_kind='thin_edge',normal=[0.,1.],normal_ambiguous=True,width_m=.006,confidence=.6,position=[2.,3.,.25])
        merged=prefer_surface_normal(surface,edge)
        self.assertEqual(merged['normal'],surface['normal'])
        self.assertEqual(merged['normal_kind'],'surface')
        self.assertEqual(merged['position'],edge['position'])
        self.assertEqual(prefer_surface_normal(edge,surface)['normal'],surface['normal'])

    def test_front_geometry_has_no_identity(self):
        found=panels(*scene())
        self.assertEqual(len(found),1)
        p=found[0]
        self.assertAlmostEqual(p['position'][0],1.,places=2)
        self.assertAlmostEqual(p['position'][2],.25,delta=.01)
        self.assertAlmostEqual(p['height_m'],.2,delta=.03)
        self.assertGreater(p['black_fraction'],.1)
        self.assertNotIn('id',p)
        self.assertFalse(p['normal_ambiguous'])

    def test_white_side_without_print_retains_normal_ambiguity(self):
        found=panels(*scene(8,False))
        self.assertEqual(len(found),1)
        self.assertTrue(found[0]['normal_ambiguous'])

    def test_floor_wall_and_invalid_depth_rejected(self):
        args=list(scene());args[3]=np.eye(3)
        self.assertFalse(panels(*args)) # Horizontal board in odom.
        args=list(scene());args[1][:]=np.nan
        self.assertFalse(panels(*args))
        args=list(scene());args[1][:]=4.
        self.assertFalse(panels(*args)) # Outside bounded search range.
        args=list(scene());args[0][:]=240;args[1][:]=1.
        self.assertFalse(panels(*args)) # Large white wall.

    def test_dark_board_in_front_of_wall_uses_depth_without_white(self):
        args=list(scene());args[0][:]=35
        args[1][~np.isfinite(args[1])]=2.
        self.assertFalse(panels(*args,depth_geometry=False))
        found=panels(*args)
        self.assertEqual(len(found),1)
        self.assertEqual(found[0]['source'],'depth')
        self.assertAlmostEqual(found[0]['position'][0],1.,places=2)

    def test_modalities_do_not_double_count_one_frame(self):
        found=panels(*scene())
        self.assertEqual(len(found),1)
        self.assertTrue(laser_support(found[0],[[1.,0.,.25]]))
        self.assertFalse(laser_support(found[0],[[1.,0.,0.]])) # Scan below board.
        self.assertFalse(laser_support(found[0],[[2.,0.,.25]])) # Background wall.
        self.assertFalse(laser_support(found[0],[]))

    def test_oblique_plane_is_not_rejected_as_depth_jump(self):
        rgb,depth,k,r,t=scene(52)
        u=np.arange(134,186)
        depth[80:160,134:186]=1./(1.-1.2*(u-160)/400.)
        found=panels(rgb,depth,k,r,t)
        self.assertEqual(len(found),1)
        self.assertGreater(abs(found[0]['normal'][1]),.4)


if __name__=='__main__':unittest.main()
