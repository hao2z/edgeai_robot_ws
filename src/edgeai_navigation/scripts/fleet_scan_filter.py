#!/usr/bin/env python3
"""Peer footprint masks for mapping and localization; never change navigation scans."""
import argparse
import copy
import json
import math
import time
from collections import Counter, deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener, TransformException
from edgeai_nav.geometry import compose, relative


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


def covariance_reliable(covariance,max_xy,max_yaw):
    values=[covariance[i] for i in (0,1,6,7,35)]
    if not all(math.isfinite(v) for v in values) or min(values[0],values[3],values[4])<0:
        return False
    xx,xy,yx,yy,yaw=values
    # Largest XY eigenvalue includes correlated position uncertainty.
    cross=(xy+yx)/2
    largest=(xx+yy+math.hypot(xx-yy,2*cross))/2
    return xx+yy-largest>=-1e-10 and largest<=max_xy**2 and yaw<=max_yaw**2


class LocalizationScanFilter(Node):
    """One direction, using only time-aligned estimated TF and AMCL covariance."""
    def __init__(self,**node_options):
        super().__init__('localization_scan_filter',**node_options)
        defaults=dict(other_base_frame='robot2/base_footprint',own_odom_frame='odom',
            other_odom_frame='robot2/odom',other_pose_topic='/robot2/amcl_pose',
            bounds=[-.3125,.1125,-.165,.165],margin_m=.05,max_age_s=.5,max_wait_s=.1,
            max_std_xy_m=.15,max_std_yaw_rad=.20)
        for key,value in defaults.items():self.declare_parameter(key,value)
        self.p={key:self.get_parameter(key).value for key in defaults}
        bounds=self.p['bounds']
        if (len(bounds)!=4 or not all(math.isfinite(v) for v in bounds)
                or bounds[0]>=bounds[1] or bounds[2]>=bounds[3]
                or any(not math.isfinite(self.p[k]) or self.p[k]<0 for k in
                    ('margin_m','max_wait_s','max_age_s','max_std_xy_m','max_std_yaw_rad'))
                or self.p['max_age_s']<=0 or self.p['max_wait_s']>self.p['max_age_s']):
            raise ValueError('Invalid localization scan filter settings')
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.covariances={};self.pending=deque();self.stats=Counter();self.last_log=0.
        self.pub=self.create_publisher(LaserScan,'scan_localization',qos_profile_sensor_data)
        pose_qos=rclpy.qos.QoSProfile(depth=1,durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseWithCovarianceStamped,'amcl_pose',
            lambda m:self.covariances.update(own=m.pose.covariance),pose_qos)
        self.create_subscription(PoseWithCovarianceStamped,self.p['other_pose_topic'],
            lambda m:self.covariances.update(other=m.pose.covariance),pose_qos)
        self.create_subscription(LaserScan,'scan',self.receive,qos_profile_sensor_data)
        self.create_timer(.02,self.process)
        self.get_logger().info('LOCALIZATION_SCAN_READY: peer='+self.p['other_base_frame']+
            '; raw scan retained for navigation; uncertain/missing TF passes through')

    def receive(self,scan):
        if len(self.pending)>=10:
            _,old=self.pending.popleft();self.pub.publish(old);self.stats['queue_fallback']+=1
        self.pending.append((time.monotonic(),scan))

    def transform(self,scan):
        if (len(self.covariances)!=2 or not all(covariance_reliable(c,
                self.p['max_std_xy_m'],self.p['max_std_yaw_rad']) for c in self.covariances.values())):
            raise ValueError('uncertain_localization')
        stamp=Time.from_msg(scan.header.stamp)
        age=(self.get_clock().now().nanoseconds-stamp.nanoseconds)/1e9
        if stamp.nanoseconds==0 or not 0<=age<=self.p['max_age_s']:
            raise ValueError('stale_scan')
        # A fresh wheel TF alone does not prove that either map->odom estimate is current.
        tf_ages=[]
        for odom in (self.p['own_odom_frame'],self.p['other_odom_frame']):
            latest=self.buffer.lookup_transform('map',odom,Time())
            tf_age=(self.get_clock().now().nanoseconds-Time.from_msg(latest.header.stamp).nanoseconds)/1e9
            tf_ages.append(tf_age);self.stats['last_tf_age_s']=round(max(tf_ages),3)
            if not -self.p['max_age_s']<=tf_age<=self.p['max_age_s']:
                raise ValueError('stale_localization_tf')
        tf=self.buffer.lookup_transform(self.p['other_base_frame'],scan.header.frame_id,stamp).transform
        q=tf.rotation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        return [tf.translation.x,tf.translation.y,yaw],age

    def process(self):
        while self.pending:
            received,scan=self.pending[0]
            try:
                relative_pose,age=self.transform(scan)
                output,count=mask_scan(scan,relative_pose,self.p['bounds'],self.p['margin_m'])
                self.stats['filtered_scans']+=1;self.stats['removed_points']+=count
                self.stats['last_scan_age_s']=round(age,3)
            except (TransformException,ValueError) as exc:
                # Allow a bounded interval for TF to catch up to the scan timestamp.
                if isinstance(exc,TransformException) and time.monotonic()-received<self.p['max_wait_s']:
                    break
                output=scan;reason='missing_tf' if isinstance(exc,TransformException) else str(exc)
                self.stats['fallback_'+reason]+=1
            self.pending.popleft();self.pub.publish(output)
        if time.monotonic()-self.last_log>=5.:
            self.last_log=time.monotonic()
            self.get_logger().info('LOCALIZATION_SCAN_FILTER: '+json.dumps(dict(self.stats)))


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
    parser=argparse.ArgumentParser();parser.add_argument('--localization',action='store_true')
    args,ros_args=parser.parse_known_args()
    rclpy.init(args=ros_args);node=LocalizationScanFilter() if args.localization else FleetScanFilter()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
