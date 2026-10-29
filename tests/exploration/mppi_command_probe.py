"""Opt-in experiment: invert MPPI linear output before smoothing/monitoring."""
import json
import time
import rclpy
from geometry_msgs.msg import Twist


def main():
    rclpy.init()
    node = rclpy.create_node('mppi_command_probe')
    output = node.create_publisher(Twist, '/cmd_vel_nav', 10)
    last = [0.0, True]

    def receive(message):
        command = Twist()
        command.linear.x = -message.linear.x
        command.angular.z = message.angular.z
        output.publish(command)
        last[:] = [time.monotonic(), False]
        print(json.dumps(dict(sim_time=node.get_clock().now().nanoseconds/1e9,
                              raw=[message.linear.x,message.angular.z],
                              modified=[command.linear.x,command.angular.z])), flush=True)

    subscription = node.create_subscription(Twist, '/cmd_vel_mppi_raw', receive, 10)

    def watchdog():
        if not last[1] and time.monotonic()-last[0] > .5:
            output.publish(Twist())
            last[1] = True

    node.create_timer(.1, watchdog)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok(): output.publish(Twist())
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
