#!/usr/bin/env python3
"""Mask peer scans for public mapping and publish map telemetry."""
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rclpy.clock import Clock, ClockType
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
from edgeai_mapping.scan_mask import mask_scan


def planar(t):
    q=t.rotation
    return [t.translation.x,t.translation.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]


class JointMappingRuntime(Node):
    def __init__(self):
        super().__init__('joint_mapping_runtime')
        defaults=dict(max_age_s=.6,mask_margin_m=.05,bounds=[-.3125,.1125,-.165,.165],
                      processing_hz=2.,directory='')
        for k,v in defaults.items():self.declare_parameter(k,v)
        self.p={k:self.get_parameter(k).value for k in defaults}
        for k in ('max_age_s','processing_hz'):
            if not math.isfinite(self.p[k]) or self.p[k]<=0:raise ValueError('Invalid '+k)
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.slam={};self.aligned=False
        self.fresh_qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.scan_pubs={}
        self.status=self.create_publisher(String,'/joint_mapping/sensor_status',self.fresh_qos)
        self.create_subscription(String,'/joint_mapping/alignment',self.alignment,self.fresh_qos)
        self.create_subscription(String,'/joint_mapping/status',self.slam_status,self.fresh_qos)
        self.pending={}
        for robot in ('robot1','robot2'):
            self.scan_pubs[robot]=self.create_publisher(LaserScan,f'/{robot}/scan_slam',qos_profile_sensor_data)
            self.create_subscription(LaserScan,f'/{robot}/scan',lambda m,r=robot:self.pending.update({r:(time.monotonic(),m)}),qos_profile_sensor_data)
        self.create_timer(.02,self.scan_tick)
        self.create_timer(1./self.p['processing_hz'],self.update,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.last_status=None
        self.mask_stats={r:dict(scans=0,masked_returns=0,tf_waits=0) for r in self.scan_pubs}

    def alignment(self,msg):
        self.aligned=json.loads(msg.data).get('accepted',False)

    def slam_status(self,msg):
        self.slam=json.loads(msg.data)

    def scan_tick(self):
        if not self.aligned or self.slam.get('finalized',False) or self.slam.get('frozen',False):return
        for robot,(received,scan) in list(self.pending.items()):
            peer='robot2' if robot=='robot1' else 'robot1'
            stamp=Time.from_msg(scan.header.stamp)
            age=(self.get_clock().now()-stamp).nanoseconds/1e9
            if not 0<=age<=self.p['max_age_s']:
                if time.monotonic()-received>self.p['max_age_s']:self.pending.pop(robot,None)
                continue
            try:
                tf=self.buffer.lookup_transform(peer+'/base_footprint',scan.header.frame_id,stamp).transform
                filtered,removed=mask_scan(scan,planar(tf),self.p['bounds'],self.p['mask_margin_m'])
            except TransformException:
                self.mask_stats[robot]['tf_waits']+=1
                # Never feed the peer into the persistent map while awaiting TF.
                if time.monotonic()-received>self.p['max_age_s']:self.pending.pop(robot,None)
                continue
            self.mask_stats[robot]['scans']+=1
            self.mask_stats[robot]['masked_returns']+=removed
            self.scan_pubs[robot].publish(filtered);self.pending.pop(robot,None)

    def update(self):
        now=time.monotonic();poses={}
        for robot in self.scan_pubs:
            try:
                tf=self.buffer.lookup_transform('map',robot+'/base_footprint',Time())
                stamp=Time.from_msg(tf.header.stamp)
                age=(self.get_clock().now()-stamp).nanoseconds/1e9
                if not -.2<=age<=self.p['max_age_s']:continue
                poses[robot]=planar(tf.transform)
            except TransformException:continue
        frozen=self.slam.get('finalized',False) or self.slam.get('frozen',False)
        state=dict(aligned=self.aligned,finalized=frozen,
                   poses_ready=list(poses),masking=self.mask_stats)
        self.status.publish(String(data=json.dumps(state)))
        key=(self.aligned,frozen,tuple(poses))
        if key!=self.last_status:
            self.get_logger().info('JOINT_MAPPING_SENSORS: '+json.dumps(state));self.last_status=key


def main():
    rclpy.init();node=JointMappingRuntime()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
