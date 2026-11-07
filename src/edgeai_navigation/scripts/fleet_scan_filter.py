#!/usr/bin/env python3
"""Remove known follower footprints from SLAM scans; retain raw navigation scans."""
import copy
import json
import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException
from cooperative_policy import compose, relative


def planar(p):
    q=p.orientation
    return [p.position.x,p.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]


def mask_scan(scan,laser_in_other,bounds,margin):
    """NaN means unobserved, never fabricate free space behind the robot."""
    result=copy.deepcopy(scan);count=0
    xmin,xmax,ymin,ymax=bounds;c,s=math.cos(laser_in_other[2]),math.sin(laser_in_other[2])
    for i,distance in enumerate(scan.ranges):
        if not math.isfinite(distance) or not scan.range_min<=distance<=scan.range_max:continue
        a=scan.angle_min+i*scan.angle_increment;x,y=distance*math.cos(a),distance*math.sin(a)
        x,y=laser_in_other[0]+c*x-s*y,laser_in_other[1]+s*x+c*y
        if xmin-margin<=x<=xmax+margin and ymin-margin<=y<=ymax+margin:
            result.ranges[i]=float('nan');count+=1
    return result,count


class FleetScanFilter(Node):
    def __init__(self):
        super().__init__('fleet_scan_filter')
        for key,value in [('relative_pose',[0.,-.6,0.]),('root_odom_frame','odom'),
                          ('bounds',[-.3125,.1125,-.165,.165]),('margin_m',.15),('max_age_s',.5)]:
            self.declare_parameter(key,value)
        self.prior=self.get_parameter('relative_pose').value
        self.frame=self.get_parameter('root_odom_frame').value
        self.bounds=self.get_parameter('bounds').value
        self.margin=self.get_parameter('margin_m').value;self.age=self.get_parameter('max_age_s').value
        self.initial={};self.odom={};self.pending=deque(maxlen=10)
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.pub=self.create_publisher(LaserScan,'scan_slam',qos_profile_sensor_data)
        for name,topic in [('root','odom'),('other','robot2/odom')]:
            self.create_subscription(Odometry,topic,lambda m,n=name:self.receive_odom(n,m),qos_profile_sensor_data)
        self.create_subscription(LaserScan,'scan',lambda m:self.pending.append((time.monotonic(),m)),qos_profile_sensor_data)
        self.create_timer(.02,self.process)
        self.last_log=0.;self.removed=0;self.scans=0;self.dropped=0

    def receive_odom(self,name,msg):
        self.odom[name]=msg;self.initial.setdefault(name,planar(msg.pose.pose))

    def process(self):
        while self.pending:
            received,scan=self.pending[0];stamp=Time.from_msg(scan.header.stamp)
            age=self.get_clock().now().nanoseconds/1e9-stamp.nanoseconds/1e9
            ready=len(self.initial)==2 and 0<=age<=self.age
            if ready:
                other=self.odom['other'];other_stamp=Time.from_msg(other.header.stamp).nanoseconds/1e9
                ready=abs(other_stamp-stamp.nanoseconds/1e9)<=self.age
            try:
                if not ready:raise TransformException('Waiting for fresh follower odometry')
                tf=self.buffer.lookup_transform(self.frame,scan.header.frame_id,stamp).transform
                laser=[tf.translation.x,tf.translation.y,2*math.atan2(tf.rotation.z,tf.rotation.w)]
                other_pose=compose(compose(self.initial['root'],self.prior),
                    relative(self.initial['other'],planar(other.pose.pose)))
                output,count=mask_scan(scan,relative(other_pose,laser),self.bounds,self.margin)
            except TransformException:
                if time.monotonic()-received<self.age:return
                self.pending.popleft();self.dropped+=1
                self.get_logger().warning('FLEET_SCAN_DEFERRED: missing fresh odometry/TF; scan omitted from SLAM',throttle_duration_sec=5.)
                continue
            self.pending.popleft();self.pub.publish(output);self.removed+=count;self.scans+=1
            if time.monotonic()-self.last_log>=5.:
                self.last_log=time.monotonic()
                self.get_logger().info('FLEET_SCAN_FILTER: '+json.dumps(dict(scans=self.scans,removed_points=self.removed,
                    dropped_scans=self.dropped,margin_m=self.margin)))


def main():
    rclpy.init();node=FleetScanFilter()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
