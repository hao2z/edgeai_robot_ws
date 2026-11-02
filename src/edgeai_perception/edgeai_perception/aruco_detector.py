#!/usr/bin/env python3
import json
import math
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


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')

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

        # Persistent Target Map: {marker_id: {'id': int, 'type': str, 'x': float, 'y': float, 'yaw': float, 'count': int}}
        self.targets = {}

        # Target classification rules per Midterm specification:
        # ID 1 -> Single Rescue Target S1 ('normal')
        # ID 2 -> Cooperative Rescue Target C1 ('coop')
        self.coop_target_ids = {2}

        # TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers
        self.target_list_pub = self.create_publisher(String, '/detected_targets', 10)
        self.marker_array_pub = self.create_publisher(MarkerArray, '/target_markers', 10)

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

        self.get_logger().info('ArUco Detector & Persistent Target Map Node started. Listening to /camera/image...')

    def camera_info_callback(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d, dtype=np.float64)
            self.get_logger().info('Received camera intrinsics from /camera/camera_info.')

    def image_callback(self, msg: Image):
        if self.camera_matrix is None:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        corners, ids, rejected = self.detector.detectMarkers(cv_image)

        if ids is not None:
            for marker_id, corner in zip(ids.flatten(), corners):
                marker_id = int(marker_id)
                success, rvec, tvec = cv2.solvePnP(
                    self.obj_points,
                    corner[0],
                    self.camera_matrix,
                    self.dist_coeffs
                )

                if success:
                    # Convert rotation vector to 3x3 rotation matrix
                    R_marker_to_cam, _ = cv2.Rodrigues(rvec)
                    qx, qy, qz, qw = rotation_matrix_to_quaternion(R_marker_to_cam)
                    x_cam, y_cam, z_cam = tvec.flatten()

                    # Create PoseStamped in camera optical frame
                    pose_cam = PoseStamped()
                    pose_cam.header.stamp = rclpy.time.Time().to_msg()
                    pose_cam.header.frame_id = 'camera_optical_frame'
                    pose_cam.pose.position.x = float(x_cam)
                    pose_cam.pose.position.y = float(y_cam)
                    pose_cam.pose.position.z = float(z_cam)
                    pose_cam.pose.orientation.x = qx
                    pose_cam.pose.orientation.y = qy
                    pose_cam.pose.orientation.z = qz
                    pose_cam.pose.orientation.w = qw

                    # Transform to map (or fallback to odom)
                    target_frame = (
                        'map'
                        if self.tf_buffer.can_transform('map', 'camera_optical_frame', rclpy.time.Time())
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
                        self.update_target(marker_id, wx, wy, wz, wyaw, target_frame)

                    except tf2_ros.TransformException as ex:
                        self.get_logger().warn(f'TF transform failed for target {marker_id}: {ex}')

    def update_target(self, marker_id: int, x: float, y: float, z: float, yaw: float, frame_id: str):
        target_type = 'coop' if marker_id in self.coop_target_ids else 'normal'

        if marker_id not in self.targets:
            # First time seeing this target!
            self.targets[marker_id] = {
                'id': marker_id,
                'type': target_type,
                'x': x,
                'y': y,
                'z': z,
                'yaw': yaw,
                'frame_id': frame_id,
                'count': 1
            }
            self.get_logger().info(
                f'*** NEW TARGET DISCOVERED: ID {marker_id} ({target_type.upper()}) at '
                f'{frame_id}: (x={x:+.2f}, y={y:+.2f}, yaw={math.degrees(yaw):.1f} deg) ***'
            )
        else:
            # Running average to smooth out noise
            t = self.targets[marker_id]
            alpha = 0.15
            t['x'] = (1.0 - alpha) * t['x'] + alpha * x
            t['y'] = (1.0 - alpha) * t['y'] + alpha * y
            t['z'] = (1.0 - alpha) * t['z'] + alpha * z
            t['yaw'] = (1.0 - alpha) * t['yaw'] + alpha * yaw
            t['count'] += 1

    def publish_target_list(self):
        # 1. Publish JSON Target List for Coordinator / Navigation nodes
        target_list = [
            {
                'id': t['id'],
                'type': t['type'],
                'x': round(t['x'], 3),
                'y': round(t['y'], 3),
                'yaw': round(t['yaw'], 3),
                'frame_id': t['frame_id'],
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
            base_marker.pose.position.z = 0.15  # On ground / obstacle height
            base_marker.scale.x = 0.25
            base_marker.scale.y = 0.25
            base_marker.scale.z = 0.30
            # Green for Normal, Cyan for Cooperative
            if is_coop:
                base_marker.color.r = 0.0
                base_marker.color.g = 0.8
                base_marker.color.b = 1.0
            else:
                base_marker.color.r = 0.0
                base_marker.color.g = 1.0
                base_marker.color.b = 0.0
            base_marker.color.a = 0.8
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
            text_marker.pose.position.z = 0.45  # Hovering above cylinder
            text_marker.scale.z = 0.15  # Text height
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = f"ID: {m_id} ({t['type'].upper()})"
            marker_array.markers.append(text_marker)

            # --- Visual 3: Orientation Arrow (Facing Direction) ---
            arrow_marker = Marker()
            arrow_marker.header.stamp = now
            arrow_marker.header.frame_id = frame
            arrow_marker.ns = 'target_arrows'
            arrow_marker.id = m_id
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            arrow_marker.pose.position.x = t['x']
            arrow_marker.pose.position.y = t['y']
            arrow_marker.pose.position.z = 0.30
            # Convert yaw to quaternion for arrow orientation
            arrow_marker.pose.orientation.z = math.sin(t['yaw'] / 2.0)
            arrow_marker.pose.orientation.w = math.cos(t['yaw'] / 2.0)
            arrow_marker.scale.x = 0.30  # Arrow length
            arrow_marker.scale.y = 0.05  # Arrow width
            arrow_marker.scale.z = 0.05  # Arrow height
            arrow_marker.color.r = 1.0
            arrow_marker.color.g = 0.9
            arrow_marker.color.b = 0.0
            arrow_marker.color.a = 0.9
            marker_array.markers.append(arrow_marker)

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
