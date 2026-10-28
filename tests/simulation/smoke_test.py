"""Bounded baseline checks; run only against the isolated simulator."""
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan, Image, CameraInfo, PointCloud2, JointState
from rosgraph_msgs.msg import Clock
from tf2_ros import Buffer, TransformListener

parser = argparse.ArgumentParser()
parser.add_argument('phase', choices=['sensors', 'motion', 'map'])
parser.add_argument('--output', required=True)
args = parser.parse_args()
rclpy.init()
node = rclpy.create_node('edgeai_environment_smoke_test')
messages = {}
counts = {}
subscriptions = []
report = {'phase': args.phase}

def subscribe(topic, msg_type, qos=qos_profile_sensor_data):
    def receive(msg):
        messages[topic] = msg
        counts[topic] = counts.get(topic, 0) + 1
    subscriptions.append(node.create_subscription(msg_type, topic, receive, qos))

def spin_for(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

def until(predicate, timeout=90):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        rclpy.spin_once(node, timeout_sec=0.1)
    raise AssertionError('Timed out waiting for data: ' + str(counts))

try:
    if args.phase == 'sensors':
        for topic, kind in [('/clock', Clock), ('/odom', Odometry),
                            ('/joint_states', JointState), ('/scan', LaserScan),
                            ('/camera/image', Image), ('/camera/depth_image', Image),
                            ('/camera/camera_info', CameraInfo), ('/camera/points', PointCloud2)]:
            subscribe(topic, kind)
        buffer = Buffer()
        listener = TransformListener(buffer, node)
        until(lambda: len(messages) == 8 and counts.get('/clock', 0) > 2)
        t0 = messages['/clock'].clock
        spin_for(2)
        t1 = messages['/clock'].clock
        assert (t1.sec, t1.nanosec) > (t0.sec, t0.nanosec), 'Clock not advancing'
        scan = messages['/scan']
        finite = [r for r in scan.ranges if math.isfinite(r)]
        assert len(scan.ranges) == 360 and finite
        assert scan.header.frame_id == 'lidar_link'
        report['scan'] = {'samples': len(scan.ranges), 'finite_returns': len(finite), 'nearest_m': min(finite)}
        report['camera'] = {}
        for topic in ['/camera/image', '/camera/depth_image', '/camera/camera_info', '/camera/points']:
            msg = messages[topic]
            assert (msg.width, msg.height) == (640, 480)
            if hasattr(msg, 'data'):
                assert len(msg.data) > 0
            report['camera'][topic] = {'width': msg.width, 'height': msg.height, 'frame': msg.header.frame_id,
                                     'encoding': getattr(msg, 'encoding', None)}
        assert messages['/joint_states'].name
        report['transforms'] = {}
        for parent, child in [('odom', 'base_footprint'), ('base_footprint', 'lidar_link'),
                              ('base_footprint', 'camera_link'),
                              ('camera_link', messages['/camera/points'].header.frame_id)]:
            until(lambda: buffer.can_transform(parent, child, rclpy.time.Time()), 15)
            tf = buffer.lookup_transform(parent, child, rclpy.time.Time()).transform
            report['transforms'][parent + ' -> ' + child] = [tf.translation.x, tf.translation.y, tf.translation.z]
        report['message_counts'] = counts
    elif args.phase == 'motion':
        subscribe('/odom', Odometry)
        subscribe('/scan', LaserScan)
        pub = node.create_publisher(Twist, '/cmd_vel', 10)
        until(lambda: '/odom' in messages and '/scan' in messages and pub.get_subscription_count() > 0)
        scan = messages['/scan']
        forward = [r for i, r in enumerate(scan.ranges)
                   if abs(scan.angle_min + i * scan.angle_increment) < 0.35 and math.isfinite(r)]
        assert forward and min(forward) > 0.7, 'Insufficient forward clearance'
        start = messages['/odom'].pose.pose.position
        cmd = Twist()
        cmd.linear.x = 0.1
        try:
            end = time.monotonic() + 3
            while time.monotonic() < end:
                pub.publish(cmd)
                spin_for(0.1)
        finally:
            for _ in range(15):
                pub.publish(Twist())
                spin_for(0.1)
        finish = messages['/odom'].pose.pose.position
        stopped_start = (finish.x, finish.y)
        distance = math.hypot(finish.x-start.x, finish.y-start.y)
        assert 0.005 < distance < 0.5, f'Unexpected displacement: {distance}'
        spin_for(2)
        odom = messages['/odom']
        stopped_drift = math.hypot(odom.pose.pose.position.x-stopped_start[0], odom.pose.pose.position.y-stopped_start[1])
        assert stopped_drift < 0.01 and abs(odom.twist.twist.linear.x) < 0.01
        report.update(distance_m=distance, stopped_drift_m=stopped_drift,
                      final_linear_velocity=odom.twist.twist.linear.x, forward_clearance_m=min(forward))
    else:
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        subscribe('/map', OccupancyGrid, qos)
        until(lambda: '/map' in messages)
        grid = messages['/map']
        assert abs(grid.info.resolution - 0.05) < 1e-6
        assert grid.info.width > 0 and grid.info.height > 0
        known = sum(v >= 0 for v in grid.data)
        occupied = sum(v >= 65 for v in grid.data)
        assert known > 0 and occupied > 0
        report.update(frame=grid.header.frame_id, resolution=grid.info.resolution,
                      width=grid.info.width, height=grid.info.height, known_cells=known, occupied_cells=occupied)
    report['passed'] = True
except Exception as exc:
    report.update(passed=False, error=str(exc))
    raise
finally:
    Path(args.output).write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2), flush=True)
    node.destroy_node()
    rclpy.shutdown()
