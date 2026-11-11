"""Continuous RGB/depth/laser/TF perception, memory and conservative rejection."""
from pathlib import Path
import sys
import json
import time
import unittest
import os
import selectors
import subprocess
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Image,CameraInfo,LaserScan
from std_msgs.msg import Bool,String
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from cv_bridge import CvBridge
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'src/edgeai_perception/scripts'))
from suspect_targets import Suspects
from aruco_geometry import rotation_quaternion
from test_suspect_geometry import scene


class RGBDNode(unittest.TestCase):
    def test_idle_process_handles_sigterm(self):
        script=Path(__file__).resolve().parents[2]/'src/edgeai_perception/scripts/suspect_targets.py'
        process=subprocess.Popen([sys.executable,'-u',str(script),'--ros-args','-p','use_sim_time:=false'],
            stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,env=dict(os.environ,ROS_DOMAIN_ID='97'))
        try:
            with selectors.DefaultSelector() as selector:
                selector.register(process.stdout,selectors.EVENT_READ)
                self.assertTrue(selector.select(timeout=10.),'Detector did not start')
                self.assertIn('SUSPECT_READY',process.stdout.readline())
            process.terminate()
            output,_=process.communicate(timeout=5.)
            self.assertEqual(process.returncode,0,output)
            self.assertNotIn('Traceback',output)
        finally:
            if process.poll() is None:process.kill();process.wait()

    def test_mapping_memory_handover_reprojection_and_known_exclusion(self):
        rclpy.init(args=['--ros-args','-p','use_sim_time:=false','-p','processing_hz:=10.0'])
        node=Suspects();feed=Node('suspect_test_camera');executor=SingleThreadedExecutor()
        executor.add_node(node);executor.add_node(feed)
        pubs=[feed.create_publisher(t,topic,10) for t,topic in
              ((Image,'/camera/image'),(Image,'/camera/depth_image'),(CameraInfo,'/camera/camera_info'),(Bool,'/target_search/active'))]
        results=[]
        known_pub=feed.create_publisher(String,'/targets/estimates',10)
        authority_pub=feed.create_publisher(String,'/navigation_authority/state',
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        feed.create_subscription(String,'/targets/suspected',lambda m:results.append(json.loads(m.data)),10)
        rgb,depth,k,r,t=scene(8,False);bridge=CvBridge()
        transforms=[]
        for parent,child,rot,translation in (('odom','camera_optical_frame',r,t),('map','odom',np.eye(3),[2.,0.,0.]),
                                             ('odom','test_laser',np.eye(3),[0.,0.,.25])):
            tf=TransformStamped();tf.header.frame_id=parent;tf.child_frame_id=child
            tf.header.stamp=feed.get_clock().now().to_msg()
            q=rotation_quaternion(rot)
            tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w=map(float,q)
            tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=map(float,translation)
            transforms.append(tf)
        broadcaster=StaticTransformBroadcaster(feed);broadcaster.sendTransform(transforms)
        enabled=False;bad=False;mode=None;visible=True
        scan_pub=feed.create_publisher(LaserScan,'/scan',10)
        def publish():
            stamp=feed.get_clock().now().to_msg()
            for pub,array,encoding in ((pubs[0],rgb if visible else np.zeros_like(rgb),'bgr8'),
                                       (pubs[1],depth if visible else np.full_like(depth,np.nan),'32FC1')):
                msg=bridge.cv2_to_imgmsg(array,encoding);msg.header.frame_id='camera_optical_frame';msg.header.stamp=stamp
                if bad and encoding=='32FC1':msg.header.stamp.sec-=1
                pub.publish(msg)
            info=CameraInfo();info.header.frame_id='camera_optical_frame';info.header.stamp=stamp
            info.width=320;info.height=240;info.k=k.flatten().tolist();pubs[2].publish(info)
            pubs[3].publish(Bool(data=enabled))
            scan=LaserScan();scan.header.stamp=stamp;scan.header.frame_id='test_laser'
            scan.angle_increment=.01;scan.range_min=.05;scan.range_max=10.;scan.ranges=[1.]
            scan_pub.publish(scan)
            if mode is not None:
                authority_pub.publish(String(data=json.dumps(dict(mode=mode,explorer_paused=mode!='AUTO'))))
        feed.create_timer(.05,publish)
        def spin(seconds,predicate=lambda:False):
            end=time.monotonic()+seconds
            while time.monotonic()<end and not predicate():executor.spin_once(timeout_sec=.02)
        try:
            # No authority or search heartbeat is needed to start perception.
            spin(4.,lambda:any(m['candidates'] for m in results))
            hints=[m for m in results if m['candidates']]
            self.assertTrue(hints)
            hint=hints[-1]['candidates'][0]
            self.assertAlmostEqual(hint['position'][0],3.,delta=.03)
            self.assertTrue(hint['normal_ambiguous']);self.assertGreaterEqual(hint['observations'],2)
            self.assertTrue(node.detecting())
            self.assertTrue(hint['retained']);self.assertIn('position_spread_m',hint)
            spin(.3)
            self.assertGreater(node.stats['laser_supported'],0)
            # A mapping observation spin pauses exploration and sets STOPPED.
            mode='STOPPED';before=node.stats['frames'];spin(.3)
            self.assertGreater(node.stats['frames'],before)
            depth[np.isfinite(depth)]=1.04
            spin(.3)
            fused=results[-1]['candidates'][0]
            self.assertGreater(fused['observations'],2)
            self.assertGreater(fused['position'][0],3.)
            self.assertLess(fused['position'][0],3.04) # Fusion, not last-frame replacement.
            mode='MANUAL';visible=False;spin(.3)
            before=node.stats['frames']
            node.records[0]['last_seen']-=20. # No longer in view; older than live TTL.
            spin(.3)
            self.assertGreater(node.stats['frames'],before)
            self.assertTrue(results[-1]['candidates'])
            # A weak single-frame hint must still expire instead of occupying memory.
            now=node.get_clock().now().nanoseconds/1e9
            node.records.append(dict(id=999,observations=1,last_seen=now-9.))
            node.process()
            self.assertFalse(any(r['id']==999 for r in node.records))
            # Apply a global SLAM correction: persistent odom point reprojects.
            transforms[1].transform.translation.x=3.
            transforms[1].header.stamp=feed.get_clock().now().to_msg()
            broadcaster.sendTransform(transforms)
            spin(.4)
            self.assertAlmostEqual(results[-1]['candidates'][0]['position'][0],4.,delta=.03)
            enabled=True;spin(.3)
            self.assertTrue(results[-1]['candidates']) # Search start must not clear memory.
            self.assertGreater(results[-1]['stamp']-results[-1]['candidates'][0]['last_seen'],8.)
            known_pub.publish(String(data=json.dumps(dict(frame_id='map',targets=[
                dict(id=2,type='cooperative',position=[4.,0.,.25])]))))
            spin(.4)
            self.assertFalse(results[-1]['candidates'])
            self.assertIn(2,node.known_odom)
            enabled=False;spin(.3);self.assertFalse(results[-1]['candidates'])
            before=node.stats['frames'];spin(.2);self.assertGreater(node.stats['frames'],before)
            enabled=True;bad=True;spin(.4)
            self.assertFalse(node.records);self.assertGreater(node.stats['stale_or_unsynced'],0)
            # Missing/stale laser never blocks otherwise valid RGB-D proposals.
            node.scans.clear();self.assertEqual(node.scan_points(0.),[])
            old=LaserScan();old.header.frame_id='test_laser';node.scans.append(old)
            self.assertEqual(node.scan_points(10.),[])
            self.assertGreater(node.stats['laser_unsynced'],0)
        finally:
            executor.shutdown();node.destroy_node();feed.destroy_node();rclpy.shutdown()


if __name__=='__main__':unittest.main()
