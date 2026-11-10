"""The only cmd_vel publisher in the autonomy stack; fail closed on stale input."""
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, String
from rclpy.qos import qos_profile_sensor_data


class Guard(Node):
    STOP_FRONT = 0.29
    STOP_REAR = 0.25
    STOP_HALF_WIDTH = 0.22

    def __init__(self):
        super().__init__('velocity_guard')
        self.raw = Twist()
        self.command_at = self.scan_at = self.heartbeat_at = 0.0
        self.blocked = False
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status = self.create_publisher(String, '/autonomy/guard', 10)
        self.create_subscription(Twist, '/cmd_vel_nav', self.command, 10)
        self.create_subscription(LaserScan, '/scan', self.scan, qos_profile_sensor_data)
        self.create_subscription(Empty, '/autonomy/heartbeat', self.heartbeat, 10)
        # Wall/steady timer keeps stopping commands flowing even if /clock freezes.
        self.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def command(self, msg):
        self.raw, self.command_at = msg, time.monotonic()

    def heartbeat(self, msg):
        self.heartbeat_at = time.monotonic()

    def scan(self, msg):
        self.scan_at = time.monotonic()
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges))*msg.angle_increment
        valid = np.isfinite(ranges) & (ranges >= msg.range_min)
        # Fixed lidar_link offset in this robot: +0.10 forward, +0.08 left.
        x, y = ranges[valid]*np.cos(angles[valid])+0.10, ranges[valid]*np.sin(angles[valid])+0.08
        self.blocked = bool(np.any((x > -self.STOP_REAR) & (x < self.STOP_FRONT)
                                   & (np.abs(y) < self.STOP_HALF_WIDTH)))

    def tick(self):
        now = time.monotonic()
        valid = now-self.command_at < 0.7 and now-self.scan_at < 2.0 and now-self.heartbeat_at < 2.0
        output = Twist()
        reason = 'stale_input'
        if valid and not self.blocked:
            output.linear.x = max(-0.08, min(0.15, self.raw.linear.x))
            output.angular.z = max(-0.4, min(0.4, self.raw.angular.z))
            reason = 'ready'
        elif self.blocked:
            reason = 'footprint_obstacle'
        self.publisher.publish(output)
        self.status.publish(String(data=reason))


def main():
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    node = Guard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
