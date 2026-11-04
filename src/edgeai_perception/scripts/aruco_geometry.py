"""OpenCV compatibility and metric square geometry, independent of ROS."""
import cv2
import numpy as np


def detector(dictionary_name):
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
    if hasattr(cv2.aruco, 'ArucoDetector'):
        return cv2.aruco.ArucoDetector(dictionary).detectMarkers
    parameters = cv2.aruco.DetectorParameters_create()
    return lambda image: cv2.aruco.detectMarkers(image, dictionary, parameters=parameters)


def estimate(corners, size, k, distortion, min_side, max_error, min_distance, max_distance):
    c = np.asarray(corners, dtype=np.float64).reshape(4, 2)
    if np.min(np.linalg.norm(c-np.roll(c, -1, axis=0), axis=1)) < min_side:
        return None
    s = size/2
    points = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=float)
    ok, rotations, translations, _ = cv2.solvePnPGeneric(
        points, c, k, distortion, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    candidates = []
    for r, t in zip(rotations, translations):
        camera_points = (cv2.Rodrigues(r)[0] @ points.T).T + t.reshape(3)
        distance = float(np.linalg.norm(t))
        if np.min(camera_points[:, 2]) <= 0 or not min_distance <= distance <= max_distance:
            continue
        projected = cv2.projectPoints(points, r, t, k, distortion)[0].reshape(4, 2)
        error = float(np.sqrt(np.mean(np.sum((projected-c)**2, axis=1))))
        if np.isfinite(error) and error <= max_error:
            candidates.append((error, r.reshape(3), t.reshape(3)))
    return min(candidates, key=lambda x: x[0]) if candidates else None


def transform_point(transform, point):
    q = transform.rotation
    v = np.array([q.x, q.y, q.z], dtype=float)
    w = q.w
    norm = float(v@v+w*w)
    if norm < 1e-12:
        raise ValueError('Invalid TF quaternion')
    v, w = v/np.sqrt(norm), w/np.sqrt(norm)
    p = np.asarray(point, dtype=float)
    rotated = p+2*np.cross(v, np.cross(v, p)+w*p)
    t = transform.translation
    return rotated+np.array([t.x, t.y, t.z])
