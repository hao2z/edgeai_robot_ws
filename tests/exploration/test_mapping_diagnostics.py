"""Graph anomalies are bounded evidence, never a command to reject SLAM."""
import copy
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'src/edgeai_mapping'))
from edgeai_mapping.graph_diagnostics import compare_graphs


class GraphDiagnosticsTests(unittest.TestCase):
    def graph(self):
        return dict(epoch='a',revision=1,stamp_ns=1,nodes=[
            dict(id=1,robot_id='robot1',map=[0.,0.,0.],sensor_map=[0.,0.,0.]),
            dict(id=2,robot_id='robot2',map=[1.,0.,0.],sensor_map=[1.,0.,0.])],constraints=[])

    def test_unchanged_graph_or_new_epoch_is_not_a_jump(self):
        before=self.graph();after=copy.deepcopy(before)
        self.assertIsNone(compare_graphs(before,after))
        after['epoch']='b';after['nodes'][0]['map'][0]=10.
        self.assertIsNone(compare_graphs(before,after))

    def test_jump_records_new_cross_robot_constraint_and_sensor_residual(self):
        before=self.graph();after=copy.deepcopy(before)
        after.update(revision=2,stamp_ns=2)
        after['nodes'][1]['map'][0]=1.5;after['nodes'][1]['sensor_map'][0]=1.5
        after['constraints']=[dict(source=1,target=2,delta=[1.5,0.,0.])]
        result=compare_graphs(before,after)
        self.assertEqual(result['changed_robots'],['robot2'])
        self.assertAlmostEqual(result['max_translation_m'],.5)
        self.assertTrue(result['added_constraints'][0]['cross_robot'])
        self.assertAlmostEqual(result['added_constraints'][0]['translation_residual_m'],0.)


if __name__=='__main__':unittest.main()
