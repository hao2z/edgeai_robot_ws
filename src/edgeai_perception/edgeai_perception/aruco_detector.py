#!/usr/bin/env python3
import json
import math
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import tf2_ros
import tf2_geometry_msgs  # noqa: F401
import cv2
import numpy as np


def rotation_matrix_to_quaternion(R: np.ndarray):
    """Convert a 3x3 rotation matrix to quaternion (qx, qy, qz, qw)."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S
    return qx, qy, qz, qw


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Extract 2D yaw angle in radians from quaternion."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def odom_to_arena(ox: float, oy: float, oyaw: float, sx: float, sy: float, syaw: float):
    """Convert odom/map coordinates to Gazebo 5x5m Arena reference coordinates."""
    cos_s = math.cos(syaw)
    sin_s = math.sin(syaw)
    ax = sx + ox * cos_s - oy * sin_s
    ay = sy + ox * sin_s + oy * cos_s
    ayaw = math.atan2(math.sin(syaw + oyaw), math.cos(syaw + oyaw))
    return ax, ay, ayaw


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # Declare spawn offset parameters so we can report both odom/map and Arena coordinates
        self.declare_parameter('spawn_x', 0.7)
        self.declare_parameter('spawn_y', 0.5)
        self.declare_parameter('spawn_yaw', 0.0)
        self.declare_parameter('evidence_dir', 'evidence')

        self.spawn_x = float(self.get_parameter('spawn_x').value)
        self.spawn_y = float(self.get_parameter('spawn_y').value)
        self.spawn_yaw = float(self.get_parameter('spawn_yaw').value)
        self.evidence_dir = str(self.get_parameter('evidence_dir').value)
        os.makedirs(self.evidence_dir, exist_ok=True)

        self.bridge = CvBridge()

        # Marker physical size in meters (0.16 m per official Midterm specification)
        self.marker_size = 0.16
        s = self.marker_size / 2.0
        self.obj_points = np.array([
            [-s,  s, 0.0],
            [ s,  s, 0.0],
            [ s, -s, 0.0],
            [-s, -s, 0.0]
        ], dtype=np.float32)

        # ArUco detector setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # Camera intrinsics storage
        self.camera_matrix = None
        self.dist_coeffs = None

        # Persistent Target Map
        self.targets = {}
        self.saved_snapshots = set()

        # Target classification rules per Midterm specification:
        # ID 1 -> Single Rescue Target S1 ('normal')
        # ID 2 -> Cooperative Rescue Target C1 ('coop')
        self.coop_target_ids = {2}
        self.target_labels = {
            1: 'S1 (Single Rescue)',
            2: 'C1 (Cooperative Rescue)',
        }

        # TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers
        self.target_list_pub = self.create_publisher(String, '/detected_targets', 10)
        self.marker_array_pub = self.create_publisher(MarkerArray, '/target_markers', 10)
        self.debug_img_pub = self.create_publisher(
            Image, '/camera/aruco_debug', qos_profile_sensor_data
        )

        # Periodic timer (2 Hz) to publish persistent targets to RViz and Coordinator
        self.pub_timer = self.create_timer(0.5, self.publish_target_list)

        # Subscribers
        self.info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            qos_profile_sensor_data
        )
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info(
            'ArUco Detector & Persistent Target Map Node started. '
            'Listening to /camera/image and publishing HUD to /camera/aruco_debug...'
        )

    def camera_info_callback(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d, dtype=np.float64)
            self.get_logger().info('Received camera intrinsics from /camera/camera_info.')

    def image_callback(self, msg: Image):
        if self.camera_matrix is None:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        debug_img = cv_image.copy()
        corners, ids, _ = self.detector.detectMarkers(cv_image)

        sim_time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if sim_time_sec <= 0.0:
            sim_time_sec = self.get_clock().now().nanoseconds * 1e-9

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(debug_img, corners, ids)

            for marker_id, corner in zip(ids.flatten(), corners):
                marker_id = int(marker_id)
                success, rvec, tvec = cv2.solvePnP(
                    self.obj_points,
                    corner[0],
                    self.camera_matrix,
                    self.dist_coeffs
                )

                if success:
                    cv2.drawFrameAxes(
                        debug_img, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.08
                    )

                    # Convert rotation vector to 3x3 rotation matrix
                    R_marker_to_cam, _ = cv2.Rodrigues(rvec)
                    qx, qy, qz, qw = rotation_matrix_to_quaternion(R_marker_to_cam)
                    x_cam, y_cam, z_cam = [float(v) for v in tvec.flatten()]

                    # Relative position in robot base_footprint frame:
                    # camera_optical_frame has +Z forward, +X right, +Y down
                    # camera_link is at +0.175m ahead of base_footprint
                    rel_forward_m = z_cam + 0.175
                    rel_left_m = -x_cam
                    rel_dist_m = math.hypot(rel_forward_m, rel_left_m)

                    # Draw overlay text on the marker in debug_img
                    pts = corner[0].astype(int)
                    top_pt = (int(pts[0][0]), max(45, int(pts[0][1]) - 12))
                    label_str = self.target_labels.get(marker_id, f'ID {marker_id}')
                    cv2.putText(
                        debug_img,
                        f'{label_str} dist={rel_dist_m:.2f}m',
                        top_pt,
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (0, 255, 0),
                        2
                    )

                    # Create PoseStamped in camera optical frame
                    pose_cam = PoseStamped()
                    pose_cam.header.stamp = rclpy.time.Time().to_msg()
                    pose_cam.header.frame_id = 'camera_optical_frame'
                    pose_cam.pose.position.x = x_cam
                    pose_cam.pose.position.y = y_cam
                    pose_cam.pose.position.z = z_cam
                    pose_cam.pose.orientation.x = qx
                    pose_cam.pose.orientation.y = qy
                    pose_cam.pose.orientation.z = qz
                    pose_cam.pose.orientation.w = qw

                    # Transform to map (or fallback to odom)
                    target_frame = (
                        'map'
                        if self.tf_buffer.can_transform(
                            'map', 'camera_optical_frame', rclpy.time.Time()
                        )
                        else 'odom'
                    )

                    try:
                        pose_world = self.tf_buffer.transform(pose_cam, target_frame)
                        wx = pose_world.pose.position.x
                        wy = pose_world.pose.position.y
                        wz = pose_world.pose.position.z
                        q_w = pose_world.pose.orientation
                        wyaw = quaternion_to_yaw(q_w.x, q_w.y, q_w.z, q_w.w)

                        # Update persistent target memory
                        self.update_target(
                            marker_id=marker_id,
                            x=wx,
                            y=wy,
                            z=wz,
                            yaw=wyaw,
                            frame_id=target_frame,
                            rel_forward=rel_forward_m,
                            rel_left=rel_left_m,
                            rel_dist=rel_dist_m,
                            sim_time_sec=sim_time_sec,
                            debug_img=debug_img,
                        )

                    except tf2_ros.TransformException as ex:
                        self.get_logger().warn(f'TF transform failed for target {marker_id}: {ex}')

        # Draw top HUD banner on debug_img
        cv2.rectangle(debug_img, (0, 0), (debug_img.shape[1], 34), (25, 25, 25), -1)
        found_ids = sorted(self.targets.keys())
        hud_text = (
            f'ArUco HUD | Targets Mapped: {len(found_ids)}/2 {found_ids} | '
            f't={sim_time_sec:.1f}s'
        )
        cv2.putText(
            debug_img,
            hud_text,
            (10, 23),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            2
        )

        debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
        debug_msg.header = msg.header
        self.debug_img_pub.publish(debug_msg)

    def update_target(
        self,
        marker_id: int,
        x: float,
        y: float,
        z: float,
        yaw: float,
        frame_id: str,
        rel_forward: float,
        rel_left: float,
        rel_dist: float,
        sim_time_sec: float,
        debug_img: np.ndarray,
    ):
        target_type = 'coop' if marker_id in self.coop_target_ids else 'normal'
        target_name = 'C1' if marker_id in self.coop_target_ids else 'S1'
        ax, ay, ayaw = odom_to_arena(
            x, y, yaw, self.spawn_x, self.spawn_y, self.spawn_yaw
        )

        if marker_id not in self.targets:
            w_init = 1.0 / (max(rel_dist, 0.35) ** 2)
            self.targets[marker_id] = {
                'id': marker_id,
                'name': target_name,
                'type': target_type,
                'x': x,
                'y': y,
                'z': z,
                'yaw': yaw,
                'arena_x': ax,
                'arena_y': ay,
                'frame_id': frame_id,
                'min_dist_m': round(rel_dist, 3),
                'weight_sum': w_init,
                'relative_base_footprint': {
                    'forward_x_m': round(rel_forward, 3),
                    'left_y_m': round(rel_left, 3),
                    'distance_m': round(rel_dist, 3),
                },
                'first_detected_sim_time_sec': round(sim_time_sec, 2),
                'last_detected_sim_time_sec': round(sim_time_sec, 2),
                'count': 1
            }
            self.get_logger().info(
                f'*** NEW TARGET DISCOVERED: {target_name} (ID {marker_id}, type={target_type.upper()}) | '
                f'Relative (base_footprint): (x={rel_forward:+.2f}m, y={rel_left:+.2f}m, d={rel_dist:.2f}m) | '
                f'{frame_id}: ({x:+.2f}, {y:+.2f}) | Arena: ({ax:.2f}, {ay:.2f}) | '
                f't_sim={sim_time_sec:.2f}s ***'
            )
        else:
            t = self.targets[marker_id]
            # Inverse-distance-squared weighting + far-observation outlier gating
            w_obs = 1.0 / (max(rel_dist, 0.35) ** 2)
            if rel_dist <= 2.4 or t.get('min_dist_m', 9.9) > 1.8:
                t['weight_sum'] = t.get('weight_sum', 1.0) + w_obs
                alpha = max(0.04, min(0.35, w_obs / t['weight_sum']))
                t['x'] = (1.0 - alpha) * t['x'] + alpha * x
                t['y'] = (1.0 - alpha) * t['y'] + alpha * y
                t['z'] = (1.0 - alpha) * t['z'] + alpha * z
                yaw_diff = math.atan2(math.sin(yaw - t['yaw']), math.cos(yaw - t['yaw']))
                t['yaw'] = math.atan2(
                    math.sin(t['yaw'] + alpha * yaw_diff),
                    math.cos(t['yaw'] + alpha * yaw_diff),
                )
                ax_s, ay_s, _ = odom_to_arena(
                    t['x'], t['y'], t['yaw'], self.spawn_x, self.spawn_y, self.spawn_yaw
                )
                t['arena_x'] = ax_s
                t['arena_y'] = ay_s
            if rel_dist < t.get('min_dist_m', 9.9):
                t['min_dist_m'] = round(rel_dist, 3)
            t['frame_id'] = frame_id
            t['relative_base_footprint'] = {
                'forward_x_m': round(rel_forward, 3),
                'left_y_m': round(rel_left, 3),
                'distance_m': round(rel_dist, 3),
            }
            t['last_detected_sim_time_sec'] = round(sim_time_sec, 2)
            t['count'] += 1

        # Save evidence snapshot image when we get a close/clear observation
        if marker_id not in self.saved_snapshots and rel_dist <= 1.8:
            self.saved_snapshots.add(marker_id)
            img_path = os.path.join(
                self.evidence_dir, f'{target_name}_id{marker_id}_detection.png'
            )
            cv2.imwrite(img_path, debug_img)
            self.get_logger().info(f'Saved Part 1 visual detection evidence to {img_path}')

        self.save_evidence_json()

    def save_evidence_json(self):
        evidence_path = os.path.join(self.evidence_dir, 'detected_targets.json')
        summary_path = os.path.join(self.evidence_dir, 'part1_detection_summary.json')
        payload = [
            {
                'id': t['id'],
                'name': t['name'],
                'type': t['type'],
                'x': round(t['x'], 3),
                'y': round(t['y'], 3),
                'arena_x': round(t['arena_x'], 3),
                'arena_y': round(t['arena_y'], 3),
                'yaw': round(t['yaw'], 3),
                'frame_id': t['frame_id'],
                'min_dist_m': t.get('min_dist_m', 0.0),
                'relative_base_footprint': t['relative_base_footprint'],
                'first_detected_sim_time_sec': t['first_detected_sim_time_sec'],
                'last_detected_sim_time_sec': t['last_detected_sim_time_sec'],
                'observations': t['count'],
            }
            for t in self.targets.values()
        ]
        summary_dict = {
            t['name']: {
                'id': t['id'],
                'name': t['name'],
                'type': t['type'],
                'frame_id': t['frame_id'],
                'odom_map_pose': {'x': round(t['x'], 3), 'y': round(t['y'], 3), 'yaw': round(t['yaw'], 3)},
                'arena_pose': {'x': round(t['arena_x'], 3), 'y': round(t['arena_y'], 3), 'yaw': round(t['yaw'], 3)},
                'relative_to_robot_base': {
                    'forward_m': t['relative_base_footprint']['forward_x_m'],
                    'left_m': t['relative_base_footprint']['left_y_m'],
                    'distance_m': t['relative_base_footprint']['distance_m'],
                },
                'first_detected_sim_time_sec': t['first_detected_sim_time_sec'],
                'last_detected_sim_time_sec': t['last_detected_sim_time_sec'],
                'observations': t['count'],
            }
            for t in self.targets.values()
        }
        try:
            with open(evidence_path, 'w') as f:
                json.dump(payload, f, indent=2)
            with open(summary_path, 'w') as f:
                json.dump(summary_dict, f, indent=2)
        except OSError:
            pass

    def publish_target_list(self):
        # 1. Publish JSON Target List for Coordinator / Navigation nodes
        target_list = [
            {
                'id': t['id'],
                'name': t['name'],
                'type': t['type'],
                'x': round(t['x'], 3),
                'y': round(t['y'], 3),
                'arena_x': round(t['arena_x'], 3),
                'arena_y': round(t['arena_y'], 3),
                'yaw': round(t['yaw'], 3),
                'frame_id': t['frame_id'],
                'relative_base_footprint': t['relative_base_footprint'],
                'first_detected_sim_time_sec': t['first_detected_sim_time_sec'],
                'observations': t['count']
            }
            for t in self.targets.values()
        ]
        json_msg = String()
        json_msg.data = json.dumps(target_list)
        self.target_list_pub.publish(json_msg)

        if not self.targets:
            return

        # 2. Publish 3D MarkerArray for RViz visualization
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        for t in self.targets.values():
            m_id = t['id']
            frame = t['frame_id']
            is_coop = (t['type'] == 'coop')

            # --- Visual 1: 3D Cylinder / Base Pin ---
            base_marker = Marker()
            base_marker.header.stamp = now
            base_marker.header.frame_id = frame
            base_marker.ns = 'target_pins'
            base_marker.id = m_id
            base_marker.type = Marker.CYLINDER
            base_marker.action = Marker.ADD
            base_marker.pose.position.x = t['x']
            base_marker.pose.position.y = t['y']
            base_marker.pose.position.z = 0.15
            base_marker.scale.x = 0.25
            base_marker.scale.y = 0.25
            base_marker.scale.z = 0.30
            if is_coop:
                base_marker.color.r = 0.0
                base_marker.color.g = 0.8
                base_marker.color.b = 1.0
            else:
                base_marker.color.r = 0.0
                base_marker.color.g = 1.0
                base_marker.color.b = 0.0
            base_marker.color.a = 0.85
            marker_array.markers.append(base_marker)

            # --- Visual 2: Floating 3D Text Label ---
            text_marker = Marker()
            text_marker.header.stamp = now
            text_marker.header.frame_id = frame
            text_marker.ns = 'target_labels'
            text_marker.id = m_id
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = t['x']
            text_marker.pose.position.y = t['y']
            text_marker.pose.position.z = 0.50
            text_marker.scale.z = 0.16
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = (
                f"{t['name']} [ID:{m_id} {t['type'].upper()}]\n"
                f"Arena:({t['arena_x']:.2f},{t['arena_y']:.2f})"
            )
            marker_array.markers.append(text_marker)

            # --- Visual 3: Rescue Radius Ring on Floor (0.5m for S1, 0.7m for C1) ---
            ring_marker = Marker()
            ring_marker.header.stamp = now
            ring_marker.header.frame_id = frame
            ring_marker.ns = 'rescue_zones'
            ring_marker.id = m_id
            ring_marker.type = Marker.CYLINDER
            ring_marker.action = Marker.ADD
            ring_marker.pose.position.x = t['x']
            ring_marker.pose.position.y = t['y']
            ring_marker.pose.position.z = 0.01
            diameter = 1.40 if is_coop else 1.00
            ring_marker.scale.x = diameter
            ring_marker.scale.y = diameter
            ring_marker.scale.z = 0.01
            if is_coop:
                ring_marker.color.r = 0.2
                ring_marker.color.g = 0.6
                ring_marker.color.b = 1.0
            else:
                ring_marker.color.r = 1.0
                ring_marker.color.g = 0.3
                ring_marker.color.b = 0.3
            ring_marker.color.a = 0.25
            marker_array.markers.append(ring_marker)

        self.marker_array_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
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
