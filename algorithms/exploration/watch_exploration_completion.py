"""Consume the upstream reliable transient-local completion event."""
import sys
import time
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Empty

rclpy.init()
node = rclpy.create_node('edgeai_completion_watch', enable_rosout=False)
flag = Path(sys.argv[1])
subscription = node.create_subscription(
    Empty, '/exploration_complete', lambda _: flag.write_text('frontier_exhausted'),
    QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
               durability=DurabilityPolicy.TRANSIENT_LOCAL))
try:
    while rclpy.ok() and not flag.exists():
        rclpy.spin_once(node, timeout_sec=.25)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
