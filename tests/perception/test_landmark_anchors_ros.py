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
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from std_srvs.srv import Trigger
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/edgeai_perception/scripts'))
from aruco_targets import Targets
from test_aruco_targets import panel
from test_landmark_anchors import graph,node


class LandmarkAnchorsROS(unittest.TestCase):
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
        for child,x,y in [('base_footprint',2.,0.),('camera_optical_frame',3.,.5)]:
            tf=TransformStamped();tf.header.frame_id='odom';tf.child_frame_id=child
            tf.transform.rotation.w=1.;tf.transform.translation.x=x;tf.transform.translation.y=y
            target.buffer.set_transform_static(tf,'test')
        for _ in range(3):
            msg=target.bridge.cv2_to_imgmsg(panel(),'mono8');msg.header.frame_id='camera_optical_frame'
            msg.header.stamp=target.get_clock().now().to_msg();target.image(msg);target.process()

    def test_pending_observations_bind_and_follow_only_their_keyframe(self):
        rclpy.init();target=Targets()
        try:
            with tempfile.TemporaryDirectory() as tmp:
                target.output=Path(tmp);self.setup_target(target)
                self.assertFalse(target.records);self.assertEqual(len(target.pending),3)
                stamp=target.get_clock().now().nanoseconds
                a=node(5,[2.,0.,0.],[1.,2.,0.],stamp)
                target.graph_received(String(data=json.dumps(graph([a]))));target.publish()
                result=json.loads((target.output/'targets.json').read_text())['targets'][0]
                np.testing.assert_allclose(result['position'],[2.,2.5,1.],atol=.03)
                self.assertEqual(result['anchor_nodes'],[5]);self.assertTrue(result['confirmed'])
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

    def test_finalization_uses_last_graph_and_survives_later_tf_corrections(self):
        rclpy.init();target=Targets();peer=rclpy.create_node('fake_slam_finalizer')
        executor=SingleThreadedExecutor();executor.add_node(peer);executor.add_node(target)
        try:
            with tempfile.TemporaryDirectory() as tmp:
                target.output=Path(tmp);self.setup_target(target)
                a=node(5,[2.,0.,0.],[1.,2.,0.],target.get_clock().now().nanoseconds)
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
                a['map']=[99.,99.,0.];target.graph_received(String(data=json.dumps(graph([a],revision=3))))
                target.publish();after=json.loads((target.output/'targets.json').read_text())
                self.assertEqual(result['targets'],after['targets']);self.assertEqual(len(requests),1)
        finally:
            executor.shutdown();peer.destroy_node();target.destroy_node();rclpy.shutdown()


if __name__=='__main__':unittest.main()
