#!/usr/bin/env python3
"""Central read-only mapping audit; no navigation, SLAM mutation or truth control."""
import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.clock import Clock, ClockType
from rclpy.time import Time
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from tf2_ros import Buffer, TransformListener, TransformException
from edgeai_mapping.graph_diagnostics import compare_graphs


class MappingDiagnostics(Node):
    def __init__(self):
        super().__init__('mapping_diagnostics')
        for k,v in dict(directory='',processing_hz=1.,jump_translation_m=.3,jump_yaw_rad=.25).items():
            self.declare_parameter(k,v)
        self.p={k:self.get_parameter(k).value for k in ('directory','processing_hz','jump_translation_m','jump_yaw_rad')}
        self.directory=Path(self.p['directory']) if self.p['directory'] else None
        self.slam={};self.sensors={};self.graph=None;self.latest_graph=None;self.last_anomaly=None
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String,'/joint_mapping/status',lambda m:setattr(self,'slam',json.loads(m.data)),qos)
        self.create_subscription(String,'/joint_mapping/sensor_status',lambda m:setattr(self,'sensors',json.loads(m.data)),qos)
        self.create_subscription(String,'/slam_toolbox/anchor_graph',self.on_graph,qos)
        self.events=self.create_publisher(String,'/joint_mapping/diagnostic_events',qos)
        self.diagnostics=self.create_publisher(DiagnosticArray,'/diagnostics',10)
        self.create_timer(1./self.p['processing_hz'],self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def write(self,name,row):
        if self.directory:
            with (self.directory/name).open('a',encoding='utf-8') as stream:stream.write(json.dumps(row)+'\n')

    def on_graph(self,msg):
        # Compare every supplied graph revision, not only the low-rate audit.
        current=json.loads(msg.data)
        anomaly=compare_graphs(self.graph,current,self.p['jump_translation_m'],self.p['jump_yaw_rad'])
        if anomaly:
            anomaly['slam']=self.slam;anomaly['sensor_filter']=self.sensors
            self.write('mapping-diagnostics.jsonl',anomaly)
            self.last_anomaly=anomaly
            self.events.publish(String(data=json.dumps(anomaly)))
            self.get_logger().warning('SLAM_GRAPH_JUMP: '+json.dumps({k:anomaly[k] for k in
                ('stamp_ns','changed_nodes','changed_robots','max_translation_m')}))
        self.graph=current

    def tick(self):
        import math
        poses={}
        for robot in ('robot1','robot2'):
            try:
                tf=self.buffer.lookup_transform('map',robot+'/base_footprint',Time())
                if not 0<=(self.get_clock().now()-Time.from_msg(tf.header.stamp)).nanoseconds/1e9<=.6:continue
                p,q=tf.transform.translation,tf.transform.rotation
                poses[robot]=[p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
            except TransformException:pass
        self.write('joint-mapping-status.jsonl',dict(wall_monotonic_s=time.monotonic(),
            sim_time=self.get_clock().now().nanoseconds/1e9,slam=self.slam,exploration=self.sensors,poses=poses))
        status=DiagnosticStatus(name='joint_mapping/graph',hardware_id='public_slam',
            level=DiagnosticStatus.WARN if self.last_anomaly else DiagnosticStatus.OK,
            message='Graph jump recorded; inspect mapping-diagnostics.jsonl' if self.last_anomaly else 'Monitoring')
        status.values=[KeyValue(key='last_jump_stamp_ns',value=str(self.last_anomaly['stamp_ns']))] if self.last_anomaly else []
        msg=DiagnosticArray(status=[status]);msg.header.stamp=self.get_clock().now().to_msg();self.diagnostics.publish(msg)


def main():
    rclpy.init();node=MappingDiagnostics()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
