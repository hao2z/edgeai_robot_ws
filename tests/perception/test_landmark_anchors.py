"""Historical graph corrections act locally; current robot TF is not an input."""
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'src/edgeai_perception/scripts'))
from landmark_anchors import AnchorGraph,yaw_rotation,pose_point


def graph(nodes,epoch='map-a',revision=1,finalized=False):
    return dict(frame_id='map',epoch=epoch,revision=revision,finalized=finalized,nodes=nodes)


def node(key,odom,corrected,stamp=1_000_000_000,**metadata):
    return dict(id=key,stamp_ns=stamp,odom=odom,map=corrected,**metadata)


def shared_graph(nodes,graph_id='shared-a',revision=1,**metadata):
    return dict(frame_id='map',graph_id=graph_id,revision=revision,nodes=nodes,**metadata)


class LandmarkAnchors(unittest.TestCase):
    def test_old_frame_does_not_reapply_drift_to_new_map_observation(self):
        g=AnchorGraph();a=node(8,[0.,0.,0.],[1.,2.,0.],1)
        g.update(graph([a]))
        binding=g.bind(77_000_000_000,[2.4,2.,.15],np.eye(3),[2.,2.],frame='map')
        np.testing.assert_allclose(g.project(binding)[0],[2.4,2.,.15])
        a['map']=[2.,3.,np.pi/2];g.update(graph([a],revision=2))
        np.testing.assert_allclose(g.project(binding)[0],[2.,4.4,.15],atol=1e-10)

    def test_revisited_old_region_binds_nearest_frame_without_age_gate(self):
        g=AnchorGraph();g.update(graph([
            node(1,[0.,0.,0.],[1.,0.,0.],1),
            node(2,[2.,0.,0.],[3.,0.,0.],299_000_000_000)]))
        binding=g.bind(300_000_000_000,[1.,.2,.15],np.eye(3),[.8,0.])
        self.assertEqual(binding['node_id'],1)
        np.testing.assert_allclose(g.project(binding)[0],[2.,.2,.15])

    def test_spatial_limit_is_configurable(self):
        g=AnchorGraph(max_distance=3.);g.update(graph([node(1,[0.,0.,0.],[0.,0.,0.],1)]))
        self.assertEqual(g.bind(300_000_000_000,[2.5,0.,.15],np.eye(3),[2.,0.])['node_id'],1)
        g.max_distance=1.
        with self.assertRaises(ValueError):g.bind(300_000_000_000,[2.5,0.,.15],np.eye(3),[2.,0.])

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

    def test_robot_selection_precedes_distance_and_age_in_both_frames(self):
        for frame in ('odom','map'):
            with self.subTest(frame=frame):
                g=AnchorGraph()
                own=node(10,[1.,0.,0.],[1.,0.,0.],1,robot_id='a',sensor_id='front')
                foreign=node(20,[0.,0.,0.],[0.,0.,0.],99,robot_id='b',sensor_id='rear')
                g.update(shared_graph([foreign,own]))
                binding=g.bind(99,[2.,0.,.15],np.eye(3),[0.,0.],frame=frame,robot_id='a')
                self.assertEqual(binding['node_id'],10)
                self.assertEqual(binding['robot_id'],'a')
                self.assertEqual(binding['sensor_id'],'front')
                np.testing.assert_allclose(g.project(binding)[0],[2.,0.,.15])
                unscoped=g.bind(99,[2.,0.,.15],np.eye(3),[0.,0.],frame=frame)
                self.assertEqual(unscoped['node_id'],20)
                self.assertEqual(unscoped['robot_id'],'b')

    def test_explicit_robot_never_falls_back_to_foreign_or_legacy_node(self):
        g=AnchorGraph()
        g.update(shared_graph([
            node(1,[0.,0.,0.],[0.,0.,0.],robot_id='b'),
            node(2,[0.,0.,0.],[0.,0.,0.]),
            node(3,[10.,0.,0.],[10.,0.,0.],robot_id='a')]))
        for robot in ('a','missing',0):
            for frame in ('odom','map'):
                with self.subTest(robot=robot,frame=frame):
                    with self.assertRaisesRegex(ValueError,'No nearby'):
                        g.bind(1,[1.,0.,.15],np.eye(3),[0.,0.],frame=frame,robot_id=robot)

    def test_independent_robot_anchors_with_duplicate_local_ids(self):
        g=AnchorGraph()
        a=node(7,[0.,0.,0.],[10.,0.,np.pi/2],robot_id='a',sensor_id='front')
        b=node(7,[0.,0.,0.],[0.,20.,-np.pi/2],robot_id='b',sensor_id='front')
        g.update(shared_graph([a,b]))
        first=g.bind(a['stamp_ns'],[1.,2.,.15],yaw_rotation(.3),[0.,0.],robot_id='a')
        second=g.bind(b['stamp_ns'],[1.,2.,.15],yaw_rotation(.3),[0.,0.],robot_id='b')
        np.testing.assert_allclose(g.project(first)[0],[8.,1.,.15],atol=1e-10)
        np.testing.assert_allclose(g.project(first)[1],yaw_rotation(np.pi/2+.3),atol=1e-10)
        np.testing.assert_allclose(g.project(second)[0],[2.,19.,.15],atol=1e-10)
        np.testing.assert_allclose(g.project(second)[1],yaw_rotation(-np.pi/2+.3),atol=1e-10)
        a['map']=[30.,40.,np.pi]
        self.assertFalse(g.update(shared_graph([b,a],revision=2)))
        np.testing.assert_allclose(g.project(first)[0],[29.,38.,.15],atol=1e-10)
        np.testing.assert_allclose(g.project(first)[1],yaw_rotation(np.pi+.3),atol=1e-10)
        np.testing.assert_allclose(g.project(second)[0],[2.,19.,.15],atol=1e-10)
        np.testing.assert_allclose(g.project(second)[1],yaw_rotation(-np.pi/2+.3),atol=1e-10)
        # An identical id on another robot cannot replace a removed anchor.
        g.update(shared_graph([b],revision=3))
        with self.assertRaisesRegex(ValueError,'anchor missing'):g.project(first)
        np.testing.assert_allclose(g.project(second)[0],[2.,19.,.15],atol=1e-10)

    def test_robot_odometry_and_map_observations_agree_after_correction(self):
        g=AnchorGraph()
        a=node(1,[2.,3.,np.pi/2],[10.,20.,0.],robot_id='a')
        b=node(1,[-4.,5.,-np.pi/2],[10.,20.,np.pi/2],robot_id='b')
        g.update(shared_graph([a,b]))
        # Both observations describe map point (11, 22) with heading pi/4.
        first=g.bind(1,[0.,4.,.15],yaw_rotation(3*np.pi/4),[2.,3.],robot_id='a')
        second=g.bind(1,[-5.,3.,.15],yaw_rotation(-3*np.pi/4),[-4.,5.],robot_id='b')
        mapped=g.bind(1,[11.,22.,.15],yaw_rotation(np.pi/4),[10.,20.],frame='map',robot_id='b')
        for binding in (first,second,mapped):
            np.testing.assert_allclose(g.project(binding)[0],[11.,22.,.15],atol=1e-10)
            np.testing.assert_allclose(g.project(binding)[1],yaw_rotation(np.pi/4),atol=1e-10)
        b['map']=[30.,40.,np.pi]
        g.update(shared_graph([a,b],revision=2))
        np.testing.assert_allclose(g.project(first)[0],[11.,22.,.15],atol=1e-10)
        for binding in (second,mapped):
            np.testing.assert_allclose(g.project(binding)[0],[28.,41.,.15],atol=1e-10)
            np.testing.assert_allclose(g.project(binding)[1],yaw_rotation(3*np.pi/4),atol=1e-10)

    def test_shared_graph_epoch_invalidates_all_robots_but_preserves_frozen_sources(self):
        g=AnchorGraph()
        nodes=[node(1,[0.,0.,0.],[2.,3.,np.pi/2],robot_id=r,sensor_id='camera-'+r)
               for r in ('a','b')]
        self.assertFalse(g.update(shared_graph(nodes,epoch='legacy-epoch')))
        bindings=[g.bind(1,[1.,0.,.15],np.eye(3),[0.,0.],robot_id=r) for r in ('a','b')]
        frozen=[g.freeze(binding) for binding in bindings]
        self.assertFalse(g.update(shared_graph(nodes,revision=2,epoch='changed-legacy-epoch')))
        for binding in bindings:
            self.assertEqual(binding['epoch'],'shared-a')
            np.testing.assert_allclose(g.project(binding)[0],[2.,4.,.15],atol=1e-10)
        self.assertTrue(g.update(shared_graph(nodes,graph_id='shared-b',revision=1)))
        for robot,binding,fixed in zip(('a','b'),bindings,frozen):
            with self.assertRaisesRegex(ValueError,'anchor missing'):g.project(binding)
            self.assertEqual(fixed['source_node_id'],1)
            self.assertEqual(fixed['source_robot_id'],robot)
            self.assertEqual(fixed['source_sensor_id'],'camera-'+robot)
            self.assertEqual(fixed['source_epoch'],'shared-a')
            for preserved in (fixed,g.freeze(fixed)):
                self.assertEqual(preserved['source_robot_id'],robot)
                self.assertEqual(preserved['source_sensor_id'],'camera-'+robot)
                self.assertEqual(preserved['source_epoch'],'shared-a')
                self.assertEqual(preserved['source_node_id'],1)
                np.testing.assert_allclose(g.project(preserved)[0],[2.,4.,.15],atol=1e-10)
                np.testing.assert_allclose(g.project(preserved)[1],yaw_rotation(np.pi/2),atol=1e-10)
        self.assertEqual(g.bind(1,[1.,0.,.15],np.eye(3),[0.,0.],robot_id='a')['epoch'],'shared-b')

    def test_legacy_numeric_keys_and_robot_zero_coexist(self):
        g=AnchorGraph()
        legacy=node(1,[0.,0.,0.],[1.,0.,0.])
        tagged=node(1,[0.,0.,0.],[0.,2.,0.],robot_id=0,sensor_id=0)
        g.update(graph([legacy,tagged]))
        self.assertIs(g.nodes[1],legacy)
        self.assertIs(g.nodes[(0,1)],tagged)
        old=g.bind(1,[1.,0.,.15],np.eye(3),[0.,0.])
        self.assertNotIn('robot_id',old)
        np.testing.assert_allclose(g.project(old)[0],[2.,0.,.15])
        binding=g.bind(1,[1.,0.,.15],np.eye(3),[0.,0.],robot_id=0)
        self.assertEqual(binding['robot_id'],0)
        self.assertEqual(binding['sensor_id'],0)
        np.testing.assert_allclose(g.project(binding)[0],[1.,2.,.15])

    def test_duplicate_ids_within_one_robot_are_rejected_atomically(self):
        g=AnchorGraph()
        a=node(1,[0.,0.,0.],[0.,0.,0.],robot_id='a',sensor_id='front')
        g.update(shared_graph([a]))
        for duplicate in (dict(a),dict(a,sensor_id='rear')):
            with self.subTest(sensor=duplicate['sensor_id']):
                with self.assertRaisesRegex(ValueError,'Invalid anchor keyframe'):
                    g.update(shared_graph([a,duplicate],graph_id='bad',revision=2))
                self.assertEqual(g.epoch,'shared-a')
                self.assertEqual(g.revision,1)
                self.assertEqual(list(g.nodes),[('a',1)])
        legacy=node(1,[0.,0.,0.],[0.,0.,0.])
        with self.assertRaises(ValueError):g.update(graph([legacy,legacy]))

    def test_invalid_graph_id_does_not_fall_back_to_stale_legacy_epoch(self):
        g=AnchorGraph()
        for graph_id in (None,''):
            with self.subTest(graph_id=graph_id):
                with self.assertRaisesRegex(ValueError,'frame/epoch'):
                    g.update(shared_graph([],graph_id=graph_id,epoch='legacy'))


if __name__=='__main__':unittest.main()
