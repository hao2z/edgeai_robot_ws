#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def normalize_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Convert a 3D quaternion into a 2D yaw angle (in radians)."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class SquarePatrolNode(Node):
    """
    Drives the robot in a continuous square loop using closed-loop /odom feedback:
      1. Drive forward 1.0 meter
      2. Stop briefly
      3. Turn left 90 degrees (pi/2 rad)
      4. Stop briefly and repeat
    """

    def __init__(self):
        super().__init__('square_patrol')

        # Target parameters
        self.side_length = 1.0          # 1.0 meter straight segment
        self.turn_angle = math.pi / 2.0  # 90 degrees left turn
        self.linear_speed = 0.15        # m/s forward speed
        self.angular_speed = 0.35       # rad/s turning speed
        self.pause_duration = 0.5       # seconds to pause between moves

        # State machine variables: 'FORWARD', 'PAUSE_1', 'TURN_LEFT', 'PAUSE_2', 'COMPLETED'
        self.state = 'FORWARD'
        self.step_count = 0
        self.max_circles = 2
        self.max_steps = self.max_circles * 4  # 4 sides per circle -> 8 sides total

        # Current odometry pose
        self.curr_x = None
        self.curr_y = None
        self.curr_yaw = None

        # Reference pose at the start of the current segment/turn
        self.start_x = None
        self.start_y = None
        self.start_yaw = None
        self.pause_start_time = None

        # Publisher & Subscriber
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Control loop timer at 20 Hz (every 0.05s)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info(
            'Square Patrol started: 1.0m forward -> 90 deg left turn (repeating). '
            'Waiting for /odom...'
        )

    def odom_callback(self, msg: Odometry):
        self.curr_x = msg.pose.pose.position.x
        self.curr_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.curr_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

        # Initialize start pose on first odometry message
        if self.start_x is None:
            self.start_x = self.curr_x
            self.start_y = self.curr_y
            self.start_yaw = self.curr_yaw
            self.get_logger().info(
                f'Initial pose locked at (x={self.curr_x:.2f}, y={self.curr_y:.2f}, '
                f'yaw={math.degrees(self.curr_yaw):.1f} deg). Moving forward 1.0m...'
            )

    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def control_loop(self):
        if self.curr_x is None:
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        cmd = Twist()

        if self.state == 'FORWARD':
            dist = math.hypot(self.curr_x - self.start_x, self.curr_y - self.start_y)
            remaining = self.side_length - dist

            if remaining <= 0.01:
                self.stop_robot()
                self.state = 'PAUSE_1'
                self.pause_start_time = now_sec
                self.get_logger().info(
                    f'Completed 1.0m forward (at x={self.curr_x:.2f}, y={self.curr_y:.2f}). '
                    'Preparing to turn left 90 deg...'
                )
            else:
                # Slow down smoothly in the final 15 cm
                cmd.linear.x = max(0.05, min(self.linear_speed, remaining * 1.0))
                self.cmd_pub.publish(cmd)

        elif self.state == 'PAUSE_1':
            self.stop_robot()
            if now_sec - self.pause_start_time >= self.pause_duration:
                self.start_yaw = self.curr_yaw
                self.state = 'TURN_LEFT'

        elif self.state == 'TURN_LEFT':
            turned = normalize_angle(self.curr_yaw - self.start_yaw)
            remaining_angle = self.turn_angle - turned

            if remaining_angle <= math.radians(1.0):
                self.stop_robot()
                self.state = 'PAUSE_2'
                self.pause_start_time = now_sec
                self.step_count += 1
                self.get_logger().info(
                    f'Completed 90 deg left turn (heading={math.degrees(self.curr_yaw):.1f} deg, '
                    f'side #{self.step_count}). Moving forward 1.0m...'
                )
            else:
                # Slow down smoothly in the final 15 degrees
                cmd.angular.z = max(0.08, min(self.angular_speed, remaining_angle * 1.2))
                self.cmd_pub.publish(cmd)

        elif self.state == 'PAUSE_2':
            self.stop_robot()
            if now_sec - self.pause_start_time >= self.pause_duration:
                # Stop after completing 2 full circles (8 sides)
                if self.step_count >= self.max_steps:
                    self.state = 'COMPLETED'
                    self.get_logger().info(
                        f'🎉 Patrol finished! Successfully completed {self.max_circles} full circles '
                        f'({self.step_count} sides). Robot halted.'
                    )
                    return
                self.start_x = self.curr_x
                self.start_y = self.curr_y
                self.state = 'FORWARD'

        elif self.state == 'COMPLETED':
            self.stop_robot()


def main(args=None):
    rclpy.init(args=args)
    node = SquarePatrolNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # Always send an explicit stop command before exiting
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

