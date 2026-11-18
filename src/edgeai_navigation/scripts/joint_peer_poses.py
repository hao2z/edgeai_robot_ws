#!/usr/bin/env python3
"""Estimated peer obstacle poses while public SLAM owns localization."""
import json
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException


class JointPeerPoses(Node):
    def __init__(self):
        super().__init__('joint_peer_poses')
        self.declare_parameter('max_age_s',.6)
        self.declare_parameter('publish_hz',5.)
        self.max_age=float(self.get_parameter('max_age_s').value)
        hz=float(self.get_parameter('publish_hz').value)
        if not all(math.isfinite(v) and v>0 for v in (self.max_age,hz)):
            raise ValueError('Invalid peer pose freshness or frequency')
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.mapping_active=False
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String,'/joint_mapping/status',self.status,qos)
        self.pose_publishers={r:self.create_publisher(PoseStamped,'/cooperative_rescue/peer_pose/'+r,1)
                         for r in ('robot1','robot2')}
        self.create_timer(1./hz,self.update)

    def status(self,msg):
        state=json.loads(msg.data)
        self.mapping_active=(state.get('active',False) and state.get('alignment_accepted',False)
                             and (not state.get('frozen',False) or state.get('finalized',False)))

    def update(self):
        # Finalized SLAM supplies peer obstacles only during the stopped handover.
        # Deactivation hands these topics to mission-owned AMCL before rescue.
        if not self.mapping_active:return
        poses={}
        for robot in self.pose_publishers:
            try:
                tf=self.buffer.lookup_transform('map',robot+'/base_footprint',Time())
                age=(self.get_clock().now()-Time.from_msg(tf.header.stamp)).nanoseconds/1e9
                if not -.2<=age<=self.max_age:return
                msg=PoseStamped();msg.header=tf.header
                t=tf.transform
                msg.pose.position.x,msg.pose.position.y=t.translation.x,t.translation.y
                msg.pose.orientation=t.rotation
                poses[robot]=msg
            except TransformException:return
        for robot,msg in poses.items():self.pose_publishers[robot].publish(msg)


def main():
    rclpy.init();node=JointPeerPoses()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
