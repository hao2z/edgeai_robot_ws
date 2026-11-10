"""Exercise the installed SLAM library wrapper with synthetic laser/TF inputs."""
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
import time
import unittest
import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from std_srvs.srv import Trigger
from lifecycle_msgs.srv import ChangeState
from tf2_ros import StaticTransformBroadcaster


class SlamAnchorsROS(unittest.TestCase):
    def test_real_keyframes_and_final_map_snapshot(self):
        rclpy.init();peer=rclpy.create_node('anchor_scan_peer');process=None
        pub=peer.create_publisher(LaserScan,'/scan',10);graphs=[];maps=[]
        peer.create_subscription(String,'/slam_toolbox/anchor_graph',lambda m:graphs.append(json.loads(m.data)),10)
        peer.create_subscription(OccupancyGrid,'/map',lambda m:maps.append(m),10)
        tf=StaticTransformBroadcaster(peer);transforms=[]
        for parent,child in [('odom','base_footprint'),('base_footprint','laser')]:
            t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child;t.transform.rotation.w=1.;transforms.append(t)
        tf.sendTransform(transforms)
        binary=Path(get_package_prefix('edgeai_navigation'))/'lib/edgeai_navigation/anchored_slam_toolbox'
        try:
            with tempfile.TemporaryDirectory() as temporary:
                with (Path(temporary)/'slam.log').open('w+') as log:
                    process=subprocess.Popen([str(binary),'--ros-args','-p','use_sim_time:=false','-p','use_lifecycle_manager:=false',
                        '-p','minimum_time_interval:=0.0','-p','map_update_interval:=0.2','-p','min_pass_through:=1',
                        '-p','transform_publish_period:=0.05','-p','scan_topic:=/scan','-p','use_map_saver:=false'],stdout=log,stderr=log)
                    transition=peer.create_client(ChangeState,'/slam_toolbox/change_state')
                    def spin_until(predicate,seconds=10.):
                        end=time.monotonic()+seconds
                        while not predicate() and time.monotonic()<end:rclpy.spin_once(peer,timeout_sec=.02)
                        log.flush();log.seek(0);self.assertTrue(predicate(),log.read())
                    spin_until(transition.service_is_ready)
                    for key in (1,3):
                        request=ChangeState.Request();request.transition.id=key
                        future=transition.call_async(request);spin_until(future.done);self.assertTrue(future.result().success)
                    # Lifecycle can be active before Gazebo delivers any scan.
                    # An empty graph must publish safely rather than dereference null.
                    spin_until(lambda:bool(graphs),4.)
                    self.assertIsNone(process.poll());self.assertFalse(graphs[-1]['nodes'])
                    initial_epoch=graphs[-1]['epoch']
                    def scan():
                        m=LaserScan();m.header.frame_id='laser';m.header.stamp=peer.get_clock().now().to_msg()
                        m.angle_min=-math.pi;m.angle_increment=2*math.pi/360;m.angle_max=m.angle_min+359*m.angle_increment
                        m.range_min=.1;m.range_max=8.;m.ranges=[2./max(abs(math.sin(m.angle_min+i*m.angle_increment)),abs(math.cos(m.angle_min+i*m.angle_increment))) for i in range(360)]
                        pub.publish(m)
                    peer.create_timer(.05,scan)
                    spin_until(lambda:bool(graphs and graphs[-1]['nodes']),15.)
                    item=graphs[-1]['nodes'][0]
                    self.assertEqual(graphs[-1]['epoch'],initial_epoch)
                    self.assertGreater(item['stamp_ns'],0);self.assertEqual(len(item['map']),3);self.assertEqual(len(item['odom']),3)
                    finalize=peer.create_client(Trigger,'/slam_toolbox/finalize_anchors');spin_until(finalize.service_is_ready)
                    future=finalize.call_async(Trigger.Request());spin_until(future.done)
                    self.assertTrue(future.result().success,future.result().message)
                    result=json.loads(future.result().message);self.assertTrue(result['finalized']);self.assertTrue(maps)
                    self.assertTrue(result['nodes']);self.assertTrue(result['epoch'])
        finally:
            if process and process.poll() is None:process.terminate();process.wait(timeout=10.)
            peer.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    if os.environ.get('ROS_DOMAIN_ID','0') in ('0','86'):raise SystemExit('Use an isolated test domain')
    unittest.main()
