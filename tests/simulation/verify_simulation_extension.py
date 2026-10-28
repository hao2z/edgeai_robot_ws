"""Live checks against Gazebo: frames, noisy IMU, rendered tags, command isolation."""
import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import statistics
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Imu, LaserScan, Image, CameraInfo, PointCloud2
from tf2_ros import Buffer, TransformListener


parser = argparse.ArgumentParser()
parser.add_argument('--count', type=int, default=2)
parser.add_argument('--single', action='store_true')
parser.add_argument('--motion', action='store_true')
parser.add_argument('--maps', action='store_true')
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
rclpy.init()
node = rclpy.create_node('simulation_extension_verification', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
buffer = Buffer()
listener = TransformListener(buffer, node)
latest, samples = {}, defaultdict(list)
subscriptions = []
publishers = []
report = {'passed': False, 'robots': {}}
args.output.parent.mkdir(parents=True, exist_ok=True)


def receive(key, message):
    latest[key] = message
    if isinstance(message, Imu):
        samples[key].append((message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
                             message.angular_velocity.z, message.linear_acceleration.z))


def spin(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def wait(predicate, timeout=70):
    end = time.monotonic() + timeout
    last_report = 0
    while time.monotonic() < end:
        if predicate():
            return
        if time.monotonic() - last_report > 10:
            print('Received topics: ' + ', '.join(latest), flush=True)
            last_report = time.monotonic()
        rclpy.spin_once(node, timeout_sec=0.1)
    raise AssertionError('Timeout. Received: ' + ', '.join(latest))


names = [''] if args.single else [f'robot{i+1}' for i in range(args.count)]
try:
    for ns in names:
        pre = '/' + ns if ns else ''
        for suffix, kind in [('odom', Odometry), ('scan', LaserScan), ('imu/data_raw', Imu),
                             ('camera/image', Image), ('camera/camera_info', CameraInfo), ('camera/points', PointCloud2)]:
            topic = pre + '/' + suffix
            qos = QoSProfile(depth=1) if kind in (Image, PointCloud2) else qos_profile_sensor_data
            subscriptions.append(node.create_subscription(kind, topic, lambda m, k=topic: receive(k, m), qos))
        if args.maps:
            topic = pre + '/map'
            subscriptions.append(node.create_subscription(OccupancyGrid, topic, lambda m, k=topic: receive(k, m),
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
        publishers.append(node.create_publisher(Twist, pre + '/cmd_vel', 10))
    expected = len(names) * (7 if args.maps else 6)
    wait(lambda: len(latest) == expected)
    # Sample after sensor delivery and initial physics settling.
    samples.clear()
    spin(5)
    detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
    for ns in names:
        pre = '/' + ns if ns else ''
        frame = ns + '/' if ns else ''
        imu = latest[pre + '/imu/data_raw']
        scan = latest[pre + '/scan']
        odom = latest[pre + '/odom']
        assert imu.header.frame_id == frame + 'imu_link', imu.header.frame_id
        assert scan.header.frame_id == frame + 'lidar_link', scan.header.frame_id
        assert odom.header.frame_id == frame + 'odom', odom.header.frame_id
        assert odom.child_frame_id == frame + 'base_footprint', odom.child_frame_id
        assert len(scan.ranges) == 360 and any(math.isfinite(v) for v in scan.ranges)
        values = samples[pre + '/imu/data_raw']
        assert len(values) > 30, len(values)
        gyro_sd = statistics.stdev(v[1] for v in values)
        assert 0.0005 < gyro_sd < 0.008, gyro_sd
        dt = [b[0] - a[0] for a, b in zip(values, values[1:]) if b[0] > a[0]]
        rate = 1 / statistics.median(dt)
        assert 90 < rate < 110, rate
        accel = statistics.mean(v[2] for v in values)
        assert 9.5 < accel < 10.1, accel
        for child in ('imu_link', 'lidar_link', 'camera_optical_frame'):
            assert buffer.can_transform(frame + 'base_footprint', frame + child, rclpy.time.Time()), child
        assert buffer.can_transform(frame + 'odom', frame + 'base_footprint', rclpy.time.Time())
        image = latest[pre + '/camera/image']
        raw = np.frombuffer(image.data, np.uint8).reshape(image.height, image.step)
        pixels = raw[:, :image.width * 3].reshape(image.height, image.width, 3)
        bgr = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR) if image.encoding == 'rgb8' else pixels
        corners, ids, _ = detector.detectMarkers(bgr)
        found = [] if ids is None else ids.flatten().tolist()
        cv2.aruco.drawDetectedMarkers(bgr, corners, ids)
        cv2.imwrite(str(args.output.parent / ((ns or 'single') + '-camera.png')), bgr)
        if ns in ('robot1', 'robot2', 'robot3', 'robot4'):
            wanted = {'robot1': 1, 'robot2': 2, 'robot3': 10, 'robot4': 3}[ns]
            assert wanted in found, f'{ns}: expected rendered marker {wanted}, got {found}'
        if args.maps:
            grid = latest[pre + '/map']
            assert grid.header.frame_id == frame + 'map'
            assert any(v >= 0 for v in grid.data)
        report['robots'][ns or 'single'] = {
            'imu_samples': len(values), 'imu_rate_sim_hz': rate, 'gyro_z_stddev_rad_s': gyro_sd,
            'gyro_z_mean_rad_s': statistics.mean(v[1] for v in values),
            'accel_z_mean_m_s2': accel, 'imu_orientation_covariance': list(imu.orientation_covariance),
            'scan_samples': len(scan.ranges), 'detected_marker_ids': found,
            'camera_frame': image.header.frame_id,
            'cloud_frame': latest[pre + '/camera/points'].header.frame_id,
            'odom_pose': [odom.pose.pose.position.x, odom.pose.pose.position.y],
        }
    if len(names) > 1:
        assert not buffer.can_transform('robot1/base_footprint', 'robot2/base_footprint', rclpy.time.Time()), 'Invented cross-robot localization'
    if args.motion:
        before = {ns: latest['/' + ns + '/odom'] for ns in names}
        samples.clear()
        command = Twist()
        command.angular.z = 0.2
        try:
            end = time.monotonic() + 3
            while time.monotonic() < end:
                publishers[0].publish(command)
                spin(0.05)
        finally:
            for _ in range(12):
                for publisher in publishers:
                    publisher.publish(Twist())
                spin(0.05)
        moving = samples['/robot1/imu/data_raw']
        assert any(abs(v[1]) > 0.1 for v in moving), 'IMU did not respond to actual rotation'
        for ns in names[1:]:
            old, new = before[ns], latest['/' + ns + '/odom']
            assert abs(old.pose.pose.orientation.z - new.pose.pose.orientation.z) < 0.01, 'Command leaked to peer'
        spin(1)
        assert abs(latest['/robot1/odom'].twist.twist.angular.z) < 0.01
        report['command_isolation_and_explicit_stop'] = True
    report['passed'] = True
finally:
    for publisher in publishers:
        if rclpy.ok():
            publisher.publish(Twist())
    args.output.write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2), flush=True)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
