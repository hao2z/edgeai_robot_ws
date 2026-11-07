"""Synthetic image-to-map tests: no simulator ground truth is fed to the node."""
from pathlib import Path
import sys
import tempfile
import unittest
import json
from types import SimpleNamespace
import cv2
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'src/edgeai_perception/scripts'))
from aruco_geometry import detector, estimate, transform_point, rotation_matrix, rotation_quaternion, fuse_quaternions


def panel():
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker = cv2.aruco.drawMarker(d, 3, 120) if not hasattr(cv2.aruco, 'generateImageMarker') else cv2.aruco.generateImageMarker(d, 3, 120)
    image = np.full((480, 640), 255, dtype=np.uint8)
    image[180:300, 260:380] = marker
    return image


class GeometryTests(unittest.TestCase):
    def test_detect_and_metric_scale(self):
        corners, ids, _ = detector('DICT_4X4_50')(panel())
        self.assertEqual(ids.flatten().tolist(), [3])
        k = np.array([[600., 0, 320], [0, 600, 240], [0, 0, 1]])
        result = estimate(corners[0], .20, k, np.zeros(5), 12, 2.5, .15, 5)
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result[2][2], 1.0, delta=.03)
        self.assertLess(result[0], .5)
        self.assertIsNone(estimate(corners[0], .2, k, np.zeros(5), 150, 2.5, .15, 5))

    def test_white_panel_on_dark_wall_nested_contours(self):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        marker = (cv2.aruco.drawMarker(dictionary, 3, 120) if not hasattr(cv2.aruco, 'generateImageMarker')
                  else cv2.aruco.generateImageMarker(dictionary, 3, 120))
        image = np.zeros((480,640), np.uint8)
        image[88:232,188:332] = 110
        image[100:220,200:320] = np.minimum(marker,110)
        _, before, _ = detector('DICT_4X4_50', .05)(image)
        _, after, _ = detector('DICT_4X4_50', .02)(image)
        self.assertIsNone(before)
        self.assertIn(3, after.flatten().tolist())

    def test_orientation_composition_and_antipodal_fusion(self):
        # A vertical wall's outward normal is -Y; map correction rotates it to +X.
        r = np.array([[1.,0.,0.],[0.,0.,-1.],[0.,1.,0.]])
        q = rotation_quaternion(r)
        np.testing.assert_allclose(rotation_matrix(q), r, atol=1e-8)
        fused = fuse_quaternions([q, -q, q])
        np.testing.assert_allclose(rotation_matrix(fused), r, atol=1e-8)
        map_r = rotation_matrix([0,0,2**-.5,2**-.5]) @ r
        np.testing.assert_allclose(map_r[:,2], [1.,0.,0.], atol=1e-8)

    def test_transform_optical_point(self):
        tf = SimpleNamespace(rotation=SimpleNamespace(x=0,y=0,z=2**-.5,w=2**-.5),
                             translation=SimpleNamespace(x=1,y=2,z=3))
        np.testing.assert_allclose(transform_point(tf, [1, 0, 0]), [1, 3, 3], atol=1e-8)

    def test_ros_confirmation_and_map_output(self):
        import rclpy
        from rclpy.time import Time
        from sensor_msgs.msg import CameraInfo
        from geometry_msgs.msg import TransformStamped
        from aruco_targets import Targets
        rclpy.init()
        node = Targets()
        try:
            info = CameraInfo(width=640, height=480)
            info.k = [600.,0.,320.,0.,600.,240.,0.,0.,1.]
            info.d = [0.]*5
            node.camera(info)
            static = TransformStamped()
            static.header.frame_id = 'odom'; static.child_frame_id = 'camera_optical_frame'
            static.transform.rotation.w = 1.
            static.transform.translation.x = 2.
            node.buffer.set_transform_static(static, 'test')
            with tempfile.TemporaryDirectory() as output:
                node.output = Path(output)
                for _ in range(3):
                    msg = node.bridge.cv2_to_imgmsg(panel(), 'mono8')
                    msg.header.frame_id = 'camera_optical_frame'
                    msg.header.stamp = node.get_clock().now().to_msg()
                    node.image(msg); node.process()
                self.assertIn(3, node.confirmed)
                dynamic = TransformStamped()
                dynamic.header.frame_id = 'map'; dynamic.child_frame_id = 'odom'
                dynamic.header.stamp = node.get_clock().now().to_msg()
                dynamic.transform.rotation.w = 1.; dynamic.transform.translation.y = 4.
                node.buffer.set_transform_static(dynamic, 'test')
                node.publish()
                result = json.loads((Path(output)/'targets.json').read_text())['targets'][0]
                self.assertEqual(result['state'], 'confirmed')
                self.assertTrue(result['confirmed'])
                self.assertEqual(len(result['orientation']), 4)
                self.assertEqual(result['type'], 'unknown')
                np.testing.assert_allclose(result['position'], [2.,4.,1.], atol=.03)
                self.assertEqual(len((Path(output)/'target-observations.jsonl').read_text().splitlines()), 3)
                # Wrong image frame must never contribute another observation.
                msg.header.frame_id = 'wrong_frame';msg.header.stamp=node.get_clock().now().to_msg()
                node.image(msg); node.process()
                self.assertEqual(len(node.records[3]),3)
        finally:
            node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
