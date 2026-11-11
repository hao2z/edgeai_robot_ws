"""Bidirectional real ROS filtering with estimated TF; no simulator or motion."""
import math
import os
from pathlib import Path
import sys
import time
import unittest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped,PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'src/edgeai_navigation/scripts'))
from fleet_scan_filter import LocalizationScanFilter


class LocalizationScanROS(unittest.TestCase):
    def test_two_directions_bootstrap_tracking_and_degraded_passthrough(self):
        rclpy.init();peer=rclpy.create_node('localization_filter_peer')
        first=LocalizationScanFilter()
        second=LocalizationScanFilter(namespace='robot2',parameter_overrides=[
            Parameter('other_base_frame',value='base_footprint'),Parameter('own_odom_frame',value='robot2/odom'),
            Parameter('other_odom_frame',value='odom'),Parameter('other_pose_topic',value='/amcl_pose')])
        executor=SingleThreadedExecutor()
        for node in (peer,first,second):executor.add_node(node)
        tf=TransformBroadcaster(peer);static=StaticTransformBroadcaster(peer)
        def transform(parent,child,x=0.,yaw=0.,stamp=None):
            t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child
            t.header.stamp=stamp or peer.get_clock().now().to_msg()
            t.transform.translation.x=x;t.transform.rotation.z=math.sin(yaw/2);t.transform.rotation.w=math.cos(yaw/2)
            return t
        static.sendTransform([transform('base_footprint','laser'),transform('robot2/base_footprint','robot2/laser')])
        inputs=[peer.create_publisher(LaserScan,topic,qos_profile_sensor_data) for topic in ('/scan','/robot2/scan')]
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        covs=[peer.create_publisher(PoseWithCovarianceStamped,topic,qos) for topic in ('/amcl_pose','/robot2/amcl_pose')]
        output=[[],[]];raw=[[],[]]
        subs=[]
        for i,ns in enumerate(('','robot2/')):
            subs.append(peer.create_subscription(LaserScan,'/'+ns+'scan_localization',lambda m,i=i:output[i].append(m),qos_profile_sensor_data))
            subs.append(peer.create_subscription(LaserScan,'/'+ns+'scan',lambda m,i=i:raw[i].append(m),qos_profile_sensor_data))
        position=.6;broadcast=True;age=.04;missing_frame=False
        def publish():
            stamp=peer.get_clock().now().to_msg()
            if broadcast:
                tf.sendTransform([transform('map','odom',stamp=stamp),transform('odom','base_footprint',stamp=stamp),
                    transform('map','robot2/odom',position,math.pi,stamp),
                    transform('robot2/odom','robot2/base_footprint',stamp=stamp)])
            for i,pub in enumerate(inputs):
                scan=LaserScan();scan.header.frame_id=('robot2/laser' if i else 'laser') if not missing_frame else 'absent_laser'
                scan.header.stamp=(peer.get_clock().now()-rclpy.duration.Duration(seconds=age)).to_msg()
                scan.range_min=.05;scan.range_max=10.;scan.angle_increment=math.pi/2
                scan.ranges=[.6,2.,float('inf'),float('nan')];pub.publish(scan)
        timer=peer.create_timer(.04,publish)
        def wait(predicate,timeout=4.):
            end=time.monotonic()+timeout
            while time.monotonic()<end:
                executor.spin_once(timeout_sec=.01)
                if predicate():return
            self.fail('No expected localization scan: '+str([dict(n.stats) for n in (first,second)]))
        def covariance(variance):
            m=PoseWithCovarianceStamped();m.header.frame_id='map'
            m.pose.covariance[0]=m.pose.covariance[7]=m.pose.covariance[35]=variance
            for p in covs:p.publish(m)
        try:
            wait(lambda:all(output))
            self.assertTrue(all(abs(v[-1].ranges[0]-.6)<1e-5 for v in output),'Bootstrap must retain raw scans')
            covariance(.0025)
            wait(lambda:all(v and math.isnan(v[-1].ranges[0]) for v in output))
            for v in output:
                self.assertEqual(v[-1].ranges[1],2.);self.assertTrue(math.isinf(v[-1].ranges[2]))
            self.assertTrue(all(all(abs(m.ranges[0]-.6)<1e-5 for m in v) for v in raw),'Navigation raw scans changed')
            # A new peer position must replace the original relative placement.
            position=1.6
            wait(lambda:all(v and math.isfinite(v[-1].ranges[0]) for v in output))
            position=.6
            wait(lambda:all(v and math.isnan(v[-1].ranges[0]) for v in output))
            covariance(1.)
            wait(lambda:all(v and math.isfinite(v[-1].ranges[0]) for v in output))
            covariance(.0025)
            wait(lambda:all(v and math.isnan(v[-1].ranges[0]) for v in output))
            previous=[n.stats['fallback_missing_tf'] for n in (first,second)]
            missing_frame=True
            wait(lambda:all(n.stats['fallback_missing_tf']>old for n,old in zip((first,second),previous)))
            wait(lambda:all(v[-1].header.frame_id=='absent_laser' and math.isfinite(v[-1].ranges[0]) for v in output))
            missing_frame=False
            # Latest TF must never substitute for the requested scan time.
            age=2.
            wait(lambda:all(n.stats['fallback_stale_scan']>0 for n in (first,second)))
            age=.04;broadcast=False
            wait(lambda:all(n.stats['fallback_stale_localization_tf']>0 for n in (first,second)))
            wait(lambda:all(math.isfinite(v[-1].ranges[0]) for v in output))
        finally:
            executor.shutdown()
            for node in (first,second,peer):node.destroy_node()
            rclpy.shutdown()


if __name__=='__main__':
    if os.environ.get('ROS_DOMAIN_ID','0') in ('0','86'):raise SystemExit('Use an isolated test domain')
    unittest.main()
