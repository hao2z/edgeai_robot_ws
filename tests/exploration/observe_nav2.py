"""Read-only short observation of the navigation command chain and odometry."""
import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

rclpy.init()
node = rclpy.create_node('edgeai_navigation_observer', enable_rosout=False)
values = {}
subscriptions = []

def record(topic, msg):
    if isinstance(msg, Twist):
        value = (msg.linear.x, msg.angular.z)
    elif isinstance(msg, Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        value = (p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
    else:
        finite = [v for v in msg.ranges if math.isfinite(v)]
        value = (min(finite), max(finite), len(finite)) if finite else (None, None, 0)
    entry = values.setdefault(topic, {'count': 0, 'first': value, 'last': value, 'max_abs_linear': 0.0})
    entry['count'] += 1
    entry['last'] = value
    if isinstance(msg, Twist):
        entry['max_abs_linear'] = max(entry['max_abs_linear'], abs(msg.linear.x))

for topic, typ in (('/odom', Odometry), ('/scan', LaserScan), ('/cmd_vel_nav', Twist),
                   ('/cmd_vel_smoothed', Twist), ('/cmd_vel', Twist)):
    subscriptions.append(node.create_subscription(typ, topic, lambda msg, t=topic: record(t, msg), qos_profile_sensor_data))
end = time.monotonic() + 20.0
try:
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
finally:
    for topic in ('/odom', '/scan', '/cmd_vel_nav', '/cmd_vel_smoothed', '/cmd_vel'):
        print(topic, values.get(topic, 'no messages'), flush=True)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
