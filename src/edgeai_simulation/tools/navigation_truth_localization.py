#!/usr/bin/env python3
"""Explicit benchmark-only map->odom from time-paired Gazebo world and odom poses."""
from collections import deque
import json
import math
import signal
import time
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import TransformStamped,PoseStamped
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener,TransformBroadcaster,TransformException
from truth_geometry import compose,inverse,planar


class TruthLocalization(Node):
    def __init__(self):
        super().__init__('navigation_truth_localization')
        self.declare_parameter('entity','edgeai_robot_real')
        self.entity=self.get_parameter('entity').value
        self.buffer=Buffer(cache_time=Duration(seconds=3));self.listener=TransformListener(self.buffer,self)
        self.broadcaster=TransformBroadcaster(self);self.pending=deque(maxlen=12)
        self.pose_pub=self.create_publisher(PoseStamped,'/navigation_test/robot_pose',1)
        self.status_pub=self.create_publisher(String,'/navigation_test/status',1)
        self.last_status=0.;self.received=0.
        self.create_subscription(TFMessage,'/truth/poses',self.receive,qos_profile_sensor_data)
        self.create_timer(.05,self.tick)
        self.get_logger().info('NAVIGATION_TEST: 真值定位启用，map等于仿真world；本模式不使用SLAM/AMCL')

    def receive(self,msg):
        for tf in msg.transforms:
            if tf.child_frame_id==self.entity:
                self.pending.append(tf);self.received=time.monotonic()

    def tick(self):
        if time.monotonic()-self.received>1.:return
        now=self.get_clock().now()
        for sample in reversed(self.pending):
            stamp=Time.from_msg(sample.header.stamp)
            if not 0<=(now-stamp).nanoseconds/1e9<.25:continue
            try:odom=self.buffer.lookup_transform('odom','base_footprint',stamp).transform
            except TransformException:continue
            world=planar(sample.transform.translation,sample.transform.rotation)
            transform=compose(world,inverse(planar(odom.translation,odom.rotation)))
            tf=TransformStamped();tf.header.frame_id='map';tf.child_frame_id='odom';tf.header.stamp=sample.header.stamp
            tf.transform.translation.x,tf.transform.translation.y=transform[:2]
            tf.transform.rotation.z=math.sin(transform[2]/2);tf.transform.rotation.w=math.cos(transform[2]/2)
            self.broadcaster.sendTransform(tf)
            p=PoseStamped();p.header.frame_id='map';p.header.stamp=sample.header.stamp
            p.pose.position.x,p.pose.position.y=world[:2]
            p.pose.orientation.z=math.sin(world[2]/2);p.pose.orientation.w=math.cos(world[2]/2)
            self.pose_pub.publish(p)
            if time.monotonic()-self.last_status>1.:
                self.status_pub.publish(String(data=json.dumps(dict(mode='simulation_truth_navigation',
                    sample_time=stamp.nanoseconds/1e9,age_s=(now-stamp).nanoseconds/1e9,
                    map_from_odom=transform,robot_world=world))))
                self.last_status=time.monotonic()
            break


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    def stop(*args):raise KeyboardInterrupt
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    node=TruthLocalization()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
