"""ROS target publishing and asynchronous finalization without simulator truth."""
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from std_srvs.srv import Trigger
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/edgeai_perception/scripts'))
from aruco_targets import Targets
from test_aruco_targets import panel
from test_landmark_anchors import graph,node,shared_graph


class LandmarkAnchorsROS(unittest.TestCase):
    def test_reobservation_can_replace_biased_history_and_attach_again(self):
        rclpy.init();target=Targets()
        try:
            stamp=target.get_clock().now().nanoseconds
            for i in range(3):
                target.accept(1,stamp+i*10_000_000,
                              dict(kind='map',point=np.array([1.,2.,.15]),rotation=np.eye(3)),{})
            for i in range(3):
                target.accept(1,stamp+(i+3)*10_000_000,
                              dict(kind='map',point=np.array([1.6,2.,.15]),rotation=np.eye(3),
                                   pending_anchor=True,robot_map=np.array([1.,2.])),{})
                if i<2:self.assertAlmostEqual(target.records[1][-1][1]['point'][0],1.)
            self.assertEqual(len(target.records[1]),3)
            self.assertAlmostEqual(target.records[1][-1][1]['point'][0],1.6)
            self.assertEqual(target.records[1][-1][0],stamp+50_000_000)
            target.graph_received(String(data=json.dumps(graph([node(5,[0.,0.,0.],[1.,2.,0.],stamp)]))))
            self.assertTrue(all(item[1]['node_id']==5 for item in target.records[1]))
            np.testing.assert_allclose(target.graph.project(target.records[1][-1][1])[0],[1.6,2.,.15])
        finally:target.destroy_node();rclpy.shutdown()

    def test_delayed_observation_preserves_latest_timestamp(self):
        rclpy.init();target=Targets()
        try:
            stamp=target.get_clock().now().nanoseconds
            binding=dict(kind='map',point=np.array([1.,2.,1.]),rotation=np.eye(3))
            for offset in (0,2,1):target.accept(1,stamp+int(offset*1e9),binding,{})
            self.assertEqual([item[0] for item in target.records[1]],
                             [stamp,stamp+int(1e9),stamp+int(2e9)])
            target.accept(1,stamp-int(30e9),binding,{})
            self.assertEqual(len(target.records[1]),3)
        finally:target.destroy_node();rclpy.shutdown()

    def setup_target(self,target):
        info=CameraInfo(width=640,height=480);info.k=[600.,0.,320.,0.,600.,240.,0.,0.,1.];info.d=[0.]*5
        target.camera(info)
        correction=TransformStamped();correction.header.frame_id='map';correction.child_frame_id='odom'
        correction.transform.rotation.w=1.;correction.transform.translation.x=-1.;correction.transform.translation.y=2.
        target.buffer.set_transform_static(correction,'test')
        for child,x,y in [('base_footprint',2.,0.),(target.p['optical_frame'],3.,.5)]:
            tf=TransformStamped();tf.header.frame_id='odom';tf.child_frame_id=child
            tf.transform.rotation.w=1.;tf.transform.translation.x=x;tf.transform.translation.y=y
            target.buffer.set_transform_static(tf,'test')
        for _ in range(3):
            msg=target.bridge.cv2_to_imgmsg(panel(),'mono8');msg.header.frame_id=target.p['optical_frame']
            msg.header.stamp=target.get_clock().now().to_msg();target.image(msg);target.process()

    def test_pending_observations_bind_and_follow_only_their_keyframe(self):
        rclpy.init();target=Targets()
        try:
            with tempfile.TemporaryDirectory() as tmp:
                target.output=Path(tmp);self.setup_target(target)
                target.publish()
                initial=json.loads((target.output/'targets.json').read_text())['targets'][0]
                self.assertTrue(initial['confirmed'])
                self.assertEqual(initial['coordinate_source'],'map_observation')
                self.assertEqual(initial['pending_anchor_observations'],3)
                self.assertEqual(initial['anchor_nodes'],[])
                np.testing.assert_allclose(initial['position'],[2.,2.5,1.],atol=.03)
                stamp=target.get_clock().now().nanoseconds
                a=node(5,[2.,0.,0.],[1.,2.,0.],stamp)
                target.graph_received(String(data=json.dumps(graph([a]))));target.publish()
                result=json.loads((target.output/'targets.json').read_text())['targets'][0]
                np.testing.assert_allclose(result['position'],[2.,2.5,1.],atol=.03)
                self.assertEqual(result['anchor_nodes'],[5]);self.assertTrue(result['confirmed'])
                self.assertEqual(result['observations'],3)
                self.assertEqual(result['pending_anchor_observations'],0)
                self.assertEqual(target.stats['position_accepted'],3)
                self.assertEqual(target.stats['anchor_attached'],3)
                # A current robot TF jump must not move historical landmarks.
                tf=TransformStamped();tf.header.frame_id='map';tf.child_frame_id='odom'
                tf.transform.rotation.w=1.;tf.transform.translation.x=20.
                target.buffer.set_transform_static(tf,'test');target.publish()
                np.testing.assert_allclose(json.loads((target.output/'targets.json').read_text())['targets'][0]['position'],result['position'])
                a['map']=[2.,3.,np.pi/2]
                target.graph_received(String(data=json.dumps(graph([a],revision=2))));target.publish()
                corrected=json.loads((target.output/'targets.json').read_text())['targets'][0]
                np.testing.assert_allclose(corrected['position'],[1.5,4.,1.],atol=.03)
                self.assertEqual(corrected['last_observed_ns'],result['last_observed_ns'])
        finally:target.destroy_node();rclpy.shutdown()

    def test_robot_camera_waits_for_own_anchor_and_audits_compound_source(self):
        rclpy.init()
        target=Targets(parameter_overrides=[Parameter('robot_id',value='robot1'),
                       Parameter('optical_frame',value='robot1/camera_optical_frame')])
        try:
            with tempfile.TemporaryDirectory() as tmp:
                target.output=Path(tmp)
                self.assertEqual(target.p['image_topic'],'/camera/image')
                self.assertEqual(target.p['camera_info_topic'],'/camera/camera_info')
                stamp=target.get_clock().now().nanoseconds
                other=node(5,[2.,0.,0.],[1.,2.,0.],stamp,robot_id='robot2',sensor_id='camera2')
                target.graph_received(String(data=json.dumps(shared_graph([other]))))
                self.setup_target(target);target.update_anchors();target.publish()
                initial=json.loads((target.output/'targets.json').read_text())['targets'][0]
                self.assertEqual(initial['pending_anchor_observations'],3)
                self.assertEqual(initial['anchor_nodes'],[])
                self.assertEqual(initial['anchor_sources'],[])
                observations=[json.loads(line) for line in (target.output/'target-observations.jsonl').read_text().splitlines()]
                self.assertEqual(len(observations),3)
                self.assertTrue(all(row['robot_id']=='robot1' for row in observations))
                self.assertTrue(all(row['frame_id']=='robot1/camera_optical_frame' for row in observations))
                own=node(5,[2.,0.,0.],[1.2,2.,0.],stamp,robot_id='robot1',sensor_id='camera1')
                target.graph_received(String(data=json.dumps(shared_graph([other,own],revision=2))))
                target.publish()
                result=json.loads((target.output/'targets.json').read_text())['targets'][0]
                source=dict(node_id=5,robot_id='robot1',sensor_id='camera1')
                self.assertEqual(result['anchor_nodes'],[5])
                self.assertEqual(result['anchor_sources'],[source])
                self.assertEqual(result['pending_anchor_observations'],0)
                self.assertTrue(all(binding['robot_id']=='robot1' for _,binding in target.records[3]))
                np.testing.assert_allclose(result['position'],[2.,2.5,1.],atol=.03)
                audit=json.loads((target.output/'target-anchor-history.jsonl').read_text().splitlines()[-1])
                self.assertEqual(audit['anchor_sources'],[source])
                self.assertEqual([item['robot_id'] for item in audit['nodes']],['robot1'])
                # The closer foreign node must not influence the historical target.
                other['map']=[50.,60.,np.pi]
                target.graph_received(String(data=json.dumps(shared_graph([own,other],revision=3))))
                target.publish()
                unchanged=json.loads((target.output/'targets.json').read_text())['targets'][0]
                np.testing.assert_allclose(unchanged['position'],result['position'])
                own['map']=[2.,3.,np.pi/2]
                target.graph_received(String(data=json.dumps(shared_graph([own,other],revision=4))))
                target.publish()
                corrected=json.loads((target.output/'targets.json').read_text())['targets'][0]
                np.testing.assert_allclose(corrected['position'],[1.5,3.8,1.],atol=.03)
                # A shared graph reset clears old source-specific target history.
                target.graph_received(String(data=json.dumps(shared_graph([own,other],graph_id='shared-b'))))
                self.assertFalse(target.records);self.assertFalse(target.confirmed)
        finally:target.destroy_node();rclpy.shutdown()

    def test_audit_distinguishes_duplicate_local_ids_and_legacy_keys_after_freeze(self):
        rclpy.init();target=Targets()
        try:
            with tempfile.TemporaryDirectory() as tmp:
                target.output=Path(tmp);stamp=target.get_clock().now().nanoseconds
                a=node(5,[0.,0.,0.],[1.,0.,0.],stamp,robot_id='robot1',sensor_id='camera1')
                b=node(5,[0.,0.,0.],[0.,2.,0.],stamp,robot_id='robot2',sensor_id='camera2')
                legacy=node(5,[0.,0.,0.],[0.,0.,0.],stamp)
                target.update_graph(shared_graph([legacy,a,b]))
                for marker_id,robot in ((1,None),(2,'robot1'),(3,'robot2')):
                    binding=target.graph.bind(stamp,[1.,0.,.15],np.eye(3),[0.,0.],robot_id=robot)
                    target.accept(marker_id,stamp,binding,{})
                target.publish()
                before=json.loads((target.output/'targets.json').read_text())['targets']
                self.assertTrue(all(row['anchor_nodes']==[5] for row in before))
                self.assertEqual([row['anchor_sources'][0].get('robot_id') for row in before],
                                 [None,'robot1','robot2'])
                audit=json.loads((target.output/'target-anchor-history.jsonl').read_text().splitlines()[-1])
                self.assertEqual(len(audit['nodes']),3)
                self.assertEqual(len(audit['anchor_sources']),3)
                observations=[json.loads(line) for line in (target.output/'target-observations.jsonl').read_text().splitlines()]
                self.assertEqual(observations[1]['anchor_robot_id'],'robot1')
                self.assertEqual(observations[2]['anchor_sensor_id'],'camera2')
                for history in target.records.values():
                    for index,(when,binding) in enumerate(history):history[index]=(when,target.graph.freeze(binding))
                target.mapping=False;target.frozen=True
                # Frozen source metadata can still be published without live nodes.
                target.graph.nodes.clear();target.publish()
                after=json.loads((target.output/'targets.json').read_text())['targets']
                for old,new in zip(before,after):
                    self.assertEqual(new['anchor_sources'],old['anchor_sources'])
                    self.assertEqual(new['anchor_nodes'],old['anchor_nodes'])
                    np.testing.assert_allclose(new['position'],old['position'])
                    np.testing.assert_allclose(new['orientation'],old['orientation'])
                audit=json.loads((target.output/'target-anchor-history.jsonl').read_text().splitlines()[-1])
                self.assertEqual(audit['nodes'],[])
                self.assertEqual(len(audit['anchor_sources']),3)
        finally:target.destroy_node();rclpy.shutdown()

    def test_finalization_uses_last_graph_and_survives_later_tf_corrections(self):
        self.check_finalization('')

    def test_source_specific_finalization_preserves_robot_and_sensor_audit(self):
        self.check_finalization('robot1')

    def check_finalization(self,robot_id):
        rclpy.init();target=Targets(parameter_overrides=[Parameter('robot_id',value=robot_id)])
        peer=rclpy.create_node('fake_slam_finalizer')
        executor=SingleThreadedExecutor();executor.add_node(peer);executor.add_node(target)
        try:
            with tempfile.TemporaryDirectory() as tmp:
                target.output=Path(tmp);self.setup_target(target)
                a=node(5,[2.,0.,0.],[1.,2.,0.],target.get_clock().now().nanoseconds)
                if robot_id:a.update(robot_id=robot_id,sensor_id='front-camera')
                target.graph_received(String(data=json.dumps(graph([a]))))
                requests=[]
                def finalize(_,response):
                    requests.append(True);a['map']=[2.,3.,np.pi/2]
                    response.success=True;response.message=json.dumps(graph([a],revision=2,finalized=True));return response
                peer.create_service(Trigger,'/slam_toolbox/finalize_anchors',finalize)
                client=peer.create_client(Trigger,'/targets/finalize_map')
                end=time.monotonic()+5.
                while not target.finalize_client.service_is_ready() or not client.service_is_ready():
                    self.assertLess(time.monotonic(),end);executor.spin_once(timeout_sec=.02)
                future=client.call_async(Trigger.Request())
                while not future.done():
                    self.assertLess(time.monotonic(),end);executor.spin_once(timeout_sec=.02)
                self.assertTrue(future.result().success,future.result().message)
                result=json.loads((target.output/'targets.json').read_text())
                self.assertEqual(result['map_binding'],dict(mode='fixed_map',epoch='map-a',graph_revision=2,finalized=True))
                np.testing.assert_allclose(result['targets'][0]['position'],[1.5,4.,1.],atol=.03)
                expected=dict(node_id=5)
                if robot_id:expected.update(robot_id=robot_id,sensor_id='front-camera')
                self.assertEqual(result['targets'][0]['anchor_sources'],[expected])
                if robot_id:
                    for _,binding in target.records[3]:
                        self.assertEqual(binding['source_robot_id'],robot_id)
                        self.assertEqual(binding['source_sensor_id'],'front-camera')
                audit=json.loads((target.output/'target-anchor-history.jsonl').read_text().splitlines()[-1])
                self.assertEqual(audit['anchor_sources'],[expected])
                self.assertEqual(audit['nodes'][0].get('robot_id'),robot_id or None)
                a['map']=[99.,99.,0.];target.graph_received(String(data=json.dumps(graph([a],revision=3))))
                target.publish();after=json.loads((target.output/'targets.json').read_text())
                self.assertEqual(result['targets'],after['targets']);self.assertEqual(len(requests),1)
        finally:
            executor.shutdown();peer.destroy_node();target.destroy_node();rclpy.shutdown()


if __name__=='__main__':unittest.main()
