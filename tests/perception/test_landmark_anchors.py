"""Historical graph corrections act locally; current robot TF is not an input."""
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'src/edgeai_perception/scripts'))
from landmark_anchors import AnchorGraph,yaw_rotation,pose_point


def graph(nodes,epoch='map-a',revision=1,finalized=False):
    return dict(frame_id='map',epoch=epoch,revision=revision,finalized=finalized,nodes=nodes)


def node(key,odom,corrected,stamp=1_000_000_000):
    return dict(id=key,stamp_ns=stamp,odom=odom,map=corrected)


class LandmarkAnchors(unittest.TestCase):
    def test_unrelated_current_robot_correction_does_not_move_old_target(self):
        g=AnchorGraph();a=node(0,[0.,0.,0.],[1.,2.,0.]);b=node(1,[4.,0.,0.],[5.,2.,0.],5_000_000_000)
        g.update(graph([a,b]));binding=g.bind(1_000_000_000,[.5,.2,.15],np.eye(3),[0.,0.])
        before=g.project(binding)[0]
        b['map']=[9.,8.,1.];g.update(graph([a,b],revision=2))
        np.testing.assert_allclose(g.project(binding)[0],before)

    def test_anchor_loop_correction_moves_position_and_orientation_together(self):
        g=AnchorGraph();a=node(0,[0.,0.,0.],[0.,0.,0.]);g.update(graph([a]))
        binding=g.bind(1_000_000_000,[1.,0.,.15],np.eye(3),[0.,0.])
        a['map']=[2.,3.,np.pi/2];g.update(graph([a],revision=2))
        point,rotation=g.project(binding)
        np.testing.assert_allclose(point,[2.,4.,.15],atol=1e-10)
        np.testing.assert_allclose(rotation,yaw_rotation(np.pi/2),atol=1e-10)

    def test_different_odometry_drift_is_removed_before_fusion(self):
        g=AnchorGraph();a=node(0,[0.,0.,0.],[0.,0.,0.]);b=node(1,[1.4,.2,.2],[1.,0.,0.],2_000_000_000)
        g.update(graph([a,b]));target=np.array([2.,.4,.15])
        point_b=pose_point(b['odom'],pose_point(b['map'],target,inverse=True))
        first=g.bind(a['stamp_ns'],target,np.eye(3),a['odom'])
        second=g.bind(b['stamp_ns'],point_b,yaw_rotation(.2),b['odom'])
        np.testing.assert_allclose(g.project(first)[0],g.project(second)[0],atol=1e-10)
        np.testing.assert_allclose(g.project(second)[1],np.eye(3),atol=1e-10)

    def test_map_reset_invalidates_anchors_but_frozen_map_stays_fixed(self):
        g=AnchorGraph();a=node(0,[0.,0.,0.],[1.,2.,0.]);g.update(graph([a]))
        binding=g.bind(a['stamp_ns'],[.5,0.,.15],np.eye(3),a['odom']);frozen=g.freeze(binding)
        self.assertTrue(g.update(graph([a],epoch='map-b')))
        with self.assertRaises(ValueError):g.project(binding)
        np.testing.assert_allclose(g.project(frozen)[0],[1.5,2.,.15])

    def test_missing_or_distant_anchor_has_no_latest_tf_fallback(self):
        g=AnchorGraph()
        with self.assertRaises(ValueError):g.bind(1,[0.,0.,.15],np.eye(3),[0.,0.])
        g.update(graph([node(0,[0.,0.,0.],[0.,0.,0.])]))
        with self.assertRaises(ValueError):g.bind(1,[5.,0.,.15],np.eye(3),[5.,0.])


if __name__=='__main__':unittest.main()
