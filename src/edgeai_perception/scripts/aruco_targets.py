#!/usr/bin/env python3
"""Observe targets without owning navigation or using Gazebo target truth."""
import json
from collections import deque
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException
from aruco_geometry import detector, estimate, transform_point


class Targets(Node):
    def __init__(self):
        super().__init__('aruco_targets')
        defaults = dict(image_topic='/camera/image', camera_info_topic='/camera/camera_info',
                        optical_frame='camera_optical_frame', dictionary='DICT_4X4_50',
                        marker_size_m=.20, processing_hz=5.0, max_image_age_s=.5,
                        min_marker_side_px=12.0, max_reprojection_error_px=2.5,
                        min_distance_m=.15, max_distance_m=5.0, confirmation_observations=3,
                        fusion_window_s=20.0, max_observations_per_id=20,
                        outlier_distance_m=.30, stale_after_s=10.0, output_directory='')
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {k: self.get_parameter(k).value for k in defaults}
        if self.p['marker_size_m'] <= 0 or self.p['processing_hz'] <= 0:
            raise ValueError('Marker size and processing rate must be positive')
        if self.p['confirmation_observations'] < 1 or self.p['max_observations_per_id'] < self.p['confirmation_observations']:
            raise ValueError('Invalid confirmation/window capacity')
        self.detect = detector(self.p['dictionary'])
        self.bridge = CvBridge()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.latest = self.info = None
        self.last_stamp = None
        self.records = {}
        self.confirmed = set()
        self.started = time.monotonic()
        self.last_warning = {}
        self.output = Path(self.p['output_directory']) if self.p['output_directory'] else None
        if self.output:
            self.output.mkdir(parents=True, exist_ok=True)
        self.create_subscription(Image, self.p['image_topic'], self.image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.p['camera_info_topic'], self.camera, qos_profile_sensor_data)
        self.observations = self.create_publisher(String, 'targets/observations', 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.estimates = self.create_publisher(String, 'targets/estimates', qos)
        self.markers = self.create_publisher(MarkerArray, 'targets/markers', qos)
        self.create_timer(1/self.p['processing_hz'], self.process)
        self.create_timer(1.0, self.publish)
        self.get_logger().info('TARGET_SEARCH_READY: ArUco探测已启用；仅使用图像、内参和TF，不控制导航')

    def warn(self, reason):
        now = time.monotonic()
        if now-self.last_warning.get(reason, -100) >= 10:
            self.get_logger().warning(reason)
            self.last_warning[reason] = now

    def image(self, msg):
        self.latest = msg

    def camera(self, msg):
        self.info = msg

    def process(self):
        if self.latest is None or self.info is None:
            if time.monotonic()-self.started > 10:
                self.warn('TARGET_INPUT_MISSING: 等待相机图像/CameraInfo；检查相机是否被禁用')
            return
        msg, info = self.latest, self.info
        stamp = Time.from_msg(msg.header.stamp)
        if stamp.nanoseconds == self.last_stamp:
            return
        now = self.get_clock().now()
        if not 0 <= (now-stamp).nanoseconds*1e-9 <= self.p['max_image_age_s']:
            self.warn('TARGET_STALE_IMAGE: 图像时间无效或过期，跳过')
            return
        if (msg.width, msg.height) != (info.width, info.height) or info.k[0] <= 0 or info.k[4] <= 0:
            self.warn('TARGET_BAD_CALIBRATION: 图像尺寸或内参无效')
            return
        if msg.header.frame_id != self.p['optical_frame']:
            self.warn('TARGET_FRAME_MISMATCH: 图像frame与配置光学frame不一致，拒绝定位：'+msg.header.frame_id)
            return
        if info.distortion_model not in ('plumb_bob', 'rational_polynomial', ''):
            self.warn('TARGET_BAD_CALIBRATION: 暂不支持该畸变模型')
            return
        try:
            image = self.bridge.imgmsg_to_cv2(msg, 'mono8')
            corners, ids, _ = self.detect(image)
            self.last_stamp = stamp.nanoseconds
            if ids is None:
                return
            tf = self.buffer.lookup_transform('odom', self.p['optical_frame'], stamp)
            k = np.array(info.k, dtype=float).reshape(3, 3)
            d = np.array(info.d, dtype=float)
            for corner, marker_id in zip(corners, ids.flatten()):
                result = estimate(corner, self.p['marker_size_m'], k, d,
                                  self.p['min_marker_side_px'], self.p['max_reprojection_error_px'],
                                  self.p['min_distance_m'], self.p['max_distance_m'])
                if result is None:
                    continue
                error, rvec, tvec = result
                point = transform_point(tf.transform, tvec)
                key = int(marker_id)
                history = self.records.setdefault(key, deque(maxlen=self.p['max_observations_per_id']))
                while history and stamp.nanoseconds-history[0][0] > self.p['fusion_window_s']*1e9:
                    history.popleft()
                if len(history) >= self.p['confirmation_observations']:
                    center = np.median([h[1] for h in history], axis=0)
                    if np.linalg.norm(point-center) > self.p['outlier_distance_m']:
                        self.warn(f'TARGET_OUTLIER: ID={key} 观测偏离历史，拒绝融合')
                        continue
                first = not history
                history.append((stamp.nanoseconds, point))
                row = dict(id=key, stamp_ns=stamp.nanoseconds, frame_id=self.p['optical_frame'],
                           camera_position=tvec.tolist(), camera_rvec=rvec.tolist(),
                           odom_position=point.tolist(), reprojection_error_px=error)
                self.observations.publish(String(data=json.dumps(row)))
                if self.output:
                    with (self.output/'target-observations.jsonl').open('a') as f:
                        f.write(json.dumps(row)+'\n')
                if first:
                    self.get_logger().info(f'TARGET_FOUND: 发现 ArUco ID={key}')
                if len(history) >= self.p['confirmation_observations'] and key not in self.confirmed:
                    self.confirmed.add(key)
                    self.get_logger().info(f'TARGET_CONFIRMED: 多帧确认 ArUco ID={key}')
        except TransformException:
            self.warn('TARGET_TF_UNAVAILABLE: 图像时刻的相机TF不可用，跳过定位')
        except (ValueError, RuntimeError, CvBridgeError, cv2.error) as exc:
            self.warn('TARGET_PROCESS_ERROR: '+str(exc))

    def publish(self):
        now = self.get_clock().now()
        try:
            tf = self.buffer.lookup_transform('map', 'odom', now)
        except TransformException:
            if self.records:
                self.warn('TARGET_MAP_TF_UNAVAILABLE: 当前map到odom变换不可用，暂缓地图定位发布')
            return
        rows, markers = [], MarkerArray()
        for key, history in self.records.items():
            if not history:
                continue
            center = np.median([h[1] for h in history], axis=0)
            point = transform_point(tf.transform, center)
            stale = (now.nanoseconds-history[-1][0])*1e-9 > self.p['stale_after_s']
            state = 'stale' if stale else ('confirmed' if key in self.confirmed else 'candidate')
            spread = float(np.sqrt(np.mean([np.sum((h[1]-center)**2) for h in history])))
            rows.append(dict(id=key, position=point.tolist(), frame_id='map', state=state,
                             last_observed_ns=history[-1][0], observations=len(history), spread_m=spread))
            m = Marker()
            m.header.frame_id = 'map'; m.header.stamp = now.to_msg()
            m.ns = 'aruco_target'; m.id = key; m.type = Marker.SPHERE; m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, point)
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = .12
            m.color.a = .85
            m.color.r = 1.0 if state != 'confirmed' else .1
            m.color.g = .7 if state != 'stale' else .2
            markers.markers.append(m)
            label = Marker()
            label.header = m.header; label.ns = 'aruco_label'; label.id = key
            label.type = Marker.TEXT_VIEW_FACING; label.action = Marker.ADD
            label.pose.position.x, label.pose.position.y, label.pose.position.z = map(float, point+[0, 0, .2])
            label.pose.orientation.w = 1.0; label.scale.z = .13
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = f'ID {key} [{state}]'
            markers.markers.append(label)
        output = dict(frame_id='map', stamp_ns=now.nanoseconds, targets=rows)
        self.estimates.publish(String(data=json.dumps(output)))
        self.markers.publish(markers)
        if self.output:
            temporary = self.output/'targets.json.tmp'
            temporary.write_text(json.dumps(output, indent=2))
            temporary.replace(self.output/'targets.json')


def main():
    rclpy.init()
    node = Targets()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
