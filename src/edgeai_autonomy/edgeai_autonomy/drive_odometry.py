"""Convert wheel-integrated axle motion to the chassis reference used by SLAM.

The model's wheel_x is +0.0875 m. Gazebo DiffDrive integrates an ideal axle
pose but labels it base_footprint. Keep the raw stream separate and correct
both pose and twist; do not use simulator ground truth.
"""
import copy
import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import qos_profile_sensor_data
from .geometry import yaw

AXLE_OFFSET_X = 0.0875


def axle_goal(x, y, heading):
    return x + AXLE_OFFSET_X*math.cos(heading), y + AXLE_OFFSET_X*math.sin(heading)


def chassis_odometry(raw):
    msg = copy.deepcopy(raw)
    heading = yaw(raw.pose.pose.orientation)
    a = AXLE_OFFSET_X
    # Preserve base_footprint=(0,0,0) at the initial heading of zero.
    msg.pose.pose.position.x += a*(1-math.cos(heading))
    msg.pose.pose.position.y -= a*math.sin(heading)
    msg.twist.twist.linear.y -= a*msg.twist.twist.angular.z
    pose_jacobian = np.eye(6)
    pose_jacobian[0,5], pose_jacobian[1,5] = a*math.sin(heading), -a*math.cos(heading)
    twist_jacobian = np.eye(6)
    twist_jacobian[1,5] = -a
    for value, jacobian in ((msg.pose, pose_jacobian), (msg.twist, twist_jacobian)):
        covariance = np.asarray(value.covariance).reshape(6,6)
        value.covariance = (jacobian @ covariance @ jacobian.T).flatten().tolist()
    msg.header.frame_id, msg.child_frame_id = 'odom', 'base_footprint'
    return msg


class DriveOdometry(Node):
    def __init__(self):
        super().__init__('drive_odometry')
        self.publisher = self.create_publisher(Odometry, '/odom', 10)
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/odom_raw', self.receive, qos_profile_sensor_data)

    def receive(self, raw):
        msg = chassis_odometry(raw)
        self.publisher.publish(msg)
        transform = TransformStamped()
        transform.header, transform.child_frame_id = msg.header, msg.child_frame_id
        position = msg.pose.pose.position
        transform.transform.translation.x = position.x
        transform.transform.translation.y = position.y
        transform.transform.translation.z = position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = DriveOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
