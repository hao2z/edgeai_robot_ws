#!/usr/bin/env python3
"""Aggregate passive frontier counts for the task-tree mapping supplement gate."""
import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


def aggregate(counts, received, now, max_age):
    if any(r not in received or not 0<=now-received[r]<=max_age
           or counts.get(r,-1)<0 for r in ('robot1','robot2')):
        return -1
    return sum(counts[r] for r in ('robot1','robot2'))


class JointFrontierMonitor(Node):
    def __init__(self):
        super().__init__('joint_frontier_monitor')
        self.declare_parameter('max_age_s',2.)
        self.declare_parameter('publish_hz',2.)
        self.max_age=float(self.get_parameter('max_age_s').value)
        hz=float(self.get_parameter('publish_hz').value)
        if not all(math.isfinite(v) and v>0 for v in (self.max_age,hz)):
            raise ValueError('Invalid frontier monitor freshness or frequency')
        self.counts={};self.received={}
        self.publisher=self.create_publisher(Int32,'/explore/available_frontiers',1)
        for robot in ('robot1','robot2'):
            self.create_subscription(Int32,'/'+robot+'/explore/available_frontiers',
                lambda msg,r=robot:self.receive(r,msg),1)
        self.create_timer(1./hz,self.update)

    def receive(self,robot,msg):
        self.counts[robot]=msg.data;self.received[robot]=time.monotonic()

    def update(self):
        self.publisher.publish(Int32(data=aggregate(self.counts,self.received,time.monotonic(),self.max_age)))


def main():
    rclpy.init();node=JointFrontierMonitor()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
