from pathlib import Path
import sys,tempfile,unittest
import importlib.util
ROOT=Path(__file__).resolve().parents[2]
SHARE=ROOT/'src/edgeai_simulation'
sys.path.insert(0,str(SHARE/'tools'))
from generate_navigation_truth_map import generate


class TruthMap(unittest.TestCase):
    def test_world_boxes_and_panel_includes_in_true_coordinates(self):
        spec=importlib.util.spec_from_file_location('support',SHARE/'launch/simulation_support.py')
        support=importlib.util.module_from_spec(spec);spec.loader.exec_module(support)
        with tempfile.TemporaryDirectory() as tmp:
            world=Path(tmp)/'world.sdf'
            support.make_world(SHARE,SHARE/'config/targets_midterm.yaml',world,SHARE/'worlds/midterm.sdf')
            image,origin,boxes=generate(world,Path(tmp)/'map')
            def value(x,y):return image[int((y-origin[1])/.05),int((x-origin[0])/.05)]
            self.assertEqual(value(3.4,1.4),0) # W2
            self.assertEqual(value(2.2,2.),0) # O2
            self.assertEqual(value(2.85,1.),254)
            self.assertEqual(value(4.5,1.4),254)
            self.assertEqual(value(1.09,4.1),0) # Raised S1 panel beyond its small base.
            self.assertEqual(value(4.19,1.),0) # Raised C1 panel.
            self.assertTrue(any(b['name']=='aruco_1' for b in boxes))
            self.assertTrue((Path(tmp)/'map.yaml').exists())


if __name__=='__main__':unittest.main()
