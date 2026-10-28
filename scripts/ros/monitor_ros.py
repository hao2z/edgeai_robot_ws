"""Read-only live ROS log viewer; print to the console without saving logs."""
from datetime import datetime

import rclpy
from rcl_interfaces.msg import Log
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def main():
    rclpy.init()
    node = rclpy.create_node('edgeai_live_monitor', enable_rosout=False)
    levels = {Log.DEBUG: 'DEBUG', Log.INFO: 'INFO', Log.WARN: 'WARN',
              Log.ERROR: 'ERROR', Log.FATAL: 'FATAL'}

    def receive(message):
        print(f'[{message.stamp.sec}.{message.stamp.nanosec:09d}] '
              f'[{levels.get(message.level, message.level)}] '
              f'[{message.name}] {message.msg}', flush=True)

    # Keep the subscription alive for the lifetime of the node.
    subscription = node.create_subscription(
        Log, '/rosout', receive,
        QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.VOLATILE))

    previous_names = None

    def status():
        nonlocal previous_names
        names = sorted({('/' + ns.strip('/') + '/' + name).replace('//', '/')
                        for name, ns in node.get_node_names_and_namespaces()
                        if name != node.get_name()})
        prefix = f'[{datetime.now().astimezone():%H:%M:%S}] Visible ROS nodes: {len(names)}'
        if names != previous_names:
            print(prefix + '\n  ' + '\n  '.join(names or ['none; waiting for discovery']), flush=True)
            previous_names = names
        else:
            print(prefix + ' (graph unchanged)', flush=True)

    node.create_timer(10.0, status)
    print('Waiting for ROS discovery and messages; graph status refreshes every 10 seconds.', flush=True)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
