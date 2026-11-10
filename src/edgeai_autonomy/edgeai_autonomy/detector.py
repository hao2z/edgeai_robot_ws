"""Image-based ArUco detections, timestamped TF localization and ID fusion."""
import json
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
from scipy.spatial.transform import Rotation
import yaml
from .geometry import matrix


class Detector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.set_parameters([Parameter('use_sim_time', value=True)])
        self.declare_parameter('config', '')
        self.declare_parameter('output', '')
        self.config = yaml.safe_load(Path(self.get_parameter('config').value).read_text())
        self.output = Path(self.get_parameter('output').value)
        self.output.mkdir(parents=True, exist_ok=True)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, self.config['marker_dictionary']))
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(dictionary, params)
        half = self.config['marker_size_m']/2
        self.object_points = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]], np.float32)
        self.info = None
        self.depth = deque(maxlen=10)
        self.observations = {}
        self.hints = {}
        self.last_processed = -1.0
        self.last_debug = -10.0
        self.rejected, self.detected = 0, 0
        self.targets = self.create_publisher(String, '/autonomy/targets',
                                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.hint_publisher = self.create_publisher(String, '/autonomy/observation_hints',
                                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/depth_image', lambda msg: self.depth.append(msg), qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/image', self.on_image, qos_profile_sensor_data)
        self.create_timer(1.0, self.publish)

    def on_info(self, msg):
        self.info = msg

    @staticmethod
    def seconds(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9

    def on_image(self, image):
        stamp = self.seconds(image)
        if self.info is None or stamp-self.last_processed < 0.18:
            return
        self.last_processed = stamp
        if image.encoding != 'rgb8':
            return
        rgb = np.ndarray((image.height, image.width, 3), np.uint8,
                         buffer=image.data, strides=(image.step, 3, 1)).copy()
        if stamp-self.last_debug > 2:
            cv2.imwrite(str(self.output/'camera_latest.png'), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            cv2.imwrite(str(self.output/f'camera_{int(stamp):04d}.png'), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            self.last_debug = stamp
        corners, ids, _ = self.detector.detectMarkers(cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY))
        if ids is None:
            return
        self.detected += len(ids)
        cv2.aruco.drawDetectedMarkers(rgb, corners, ids)
        # OpenCV pixels and solvePnP use optical axes, regardless of the Gazebo
        # scoped frame carried in Image.header. Do not use that scoped body frame.
        try:
            transform = self.tf.lookup_transform('map', self.config['optical_frame'], Time.from_msg(image.header.stamp))
        except TransformException:
            return
        map_camera = matrix(transform.transform)
        camera_matrix = np.array(self.info.k).reshape(3, 3)
        distortion = np.array(self.info.d)
        for points, marker in zip(corners, ids.flatten()):
            marker = int(marker)
            if marker not in self.config['normal_marker_ids']:
                continue
            points = points.reshape(4, 2)
            min_side = min(np.linalg.norm(points[(i+1)%4]-points[i]) for i in range(4))
            if min_side < 10:
                continue
            ok, rvec, tvec = cv2.solvePnP(self.object_points, points, camera_matrix, distortion,
                                        flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok or tvec[2, 0] <= 0:
                continue
            projected, _ = cv2.projectPoints(self.object_points, rvec, tvec, camera_matrix, distortion)
            error = float(np.sqrt(np.mean((projected.reshape(4, 2)-points)**2)))
            if error > self.config['max_reprojection_error_px']:
                self.rejected += 1
                continue
            # Independent RGB-D check, only when a matching depth frame exists.
            matching = min(self.depth, key=lambda msg: abs(self.seconds(msg)-stamp), default=None)
            depth_error = None
            if matching is not None and abs(self.seconds(matching)-stamp) < 0.12 and matching.encoding == '32FC1':
                values = np.ndarray((matching.height, matching.width),
                                    dtype=np.dtype('>f4' if matching.is_bigendian else '<f4'),
                                    buffer=matching.data, strides=(matching.step, 4))
                u, v = np.round(points.mean(axis=0)).astype(int)
                patch = values[max(0,v-2):v+3, max(0,u-2):u+3]
                valid = patch[np.isfinite(patch) & (patch > 0)]
                if valid.size:
                    depth_error = float(abs(np.median(valid)-tvec[2,0]))
                    if depth_error > max(0.10, 0.08*tvec[2,0]):
                        self.rejected += 1
                        continue
            camera_marker = np.eye(4)
            camera_marker[:3, :3] = cv2.Rodrigues(rvec)[0]
            camera_marker[:3, 3] = tvec.flatten()
            map_marker = map_camera @ camera_marker
            position, normal = map_marker[:3, 3], map_marker[:3, 2]
            # A front-facing square's outward normal must point toward its observer.
            if np.dot(normal, map_camera[:3, 3]-position) <= 0 or abs(normal[2]) > 0.4:
                self.rejected += 1
                continue
            self.hints[marker] = {'id': marker, 'position': position.tolist(), 'normal': normal.tolist(),
                                  'last_seen_sim_time': stamp, 'min_side_px': float(min_side),
                                  'confirmed': False}
            # Small/oblique detections may guide another observation, but never
            # enter the confirmed target history or trigger rescue handover.
            if min_side < 22:
                continue
            history = self.observations.setdefault(marker, deque(maxlen=30))
            if len(history) >= 3 and np.linalg.norm(position-np.median([h['position'] for h in history], axis=0)) > 0.25:
                self.rejected += 1
                continue
            history.append({'position': position.tolist(), 'normal': normal.tolist(),
                            'quaternion': Rotation.from_matrix(map_marker[:3, :3]).as_quat().tolist(),
                            'stamp': stamp, 'reprojection_error_px': error, 'depth_error_m': depth_error})
            cv2.imwrite(str(self.output/f'marker_{marker}.png'), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        self.publish()

    def publish(self):
        targets = []
        for marker, history in self.observations.items():
            if len(history) < self.config['min_observations']:
                continue
            position = np.median([h['position'] for h in history], axis=0)
            normal = np.mean([h['normal'] for h in history], axis=0)
            normal /= np.linalg.norm(normal)
            targets.append({'id': marker, 'type': 'normal', 'frame_id': 'map',
                            'position': position.tolist(), 'normal': normal.tolist(),
                            'quaternion_xyzw': history[-1]['quaternion'], 'observations': len(history),
                            'position_std_m': np.std([h['position'] for h in history], axis=0).tolist(),
                            'last_seen_sim_time': history[-1]['stamp']})
        self.targets.publish(String(data=json.dumps(targets)))
        self.hint_publisher.publish(String(data=json.dumps(list(self.hints.values()))))
        (self.output/'perception.json').write_text(json.dumps({'targets': targets, 'raw_detections': self.detected,
                                                            'rejected': self.rejected,
                                                            'observation_hints': list(self.hints.values())}, indent=2)+'\n')


def main():
    # Leave CPU capacity for physics, SLAM and control on shared workstations.
    cv2.setNumThreads(2)
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    node = Detector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
