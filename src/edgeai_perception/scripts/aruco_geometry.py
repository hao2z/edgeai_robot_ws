"""OpenCV compatibility and metric square geometry, independent of ROS."""
import cv2
import numpy as np


def rotation_matrix(q):
    """Rotation of a normalized ROS quaternion or an xyzw sequence."""
    if hasattr(q, 'w'):
        q = [q.x, q.y, q.z, q.w]
    q = np.asarray(q, dtype=float)
    if not np.all(np.isfinite(q)) or np.linalg.norm(q) < 1e-12:
        raise ValueError('Invalid quaternion')
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def rotation_quaternion(matrix):
    """Stable matrix to xyzw conversion including rotations near pi."""
    r = np.asarray(matrix)
    values, vectors = np.linalg.eigh(np.array([
        [r[0,0]-r[1,1]-r[2,2], r[0,1]+r[1,0], r[0,2]+r[2,0], r[2,1]-r[1,2]],
        [r[0,1]+r[1,0], r[1,1]-r[0,0]-r[2,2], r[1,2]+r[2,1], r[0,2]-r[2,0]],
        [r[0,2]+r[2,0], r[1,2]+r[2,1], r[2,2]-r[0,0]-r[1,1], r[1,0]-r[0,1]],
        [r[2,1]-r[1,2], r[0,2]-r[2,0], r[1,0]-r[0,1], np.trace(r)]]))
    return vectors[:, np.argmax(values)]


def fuse_quaternions(quaternions):
    if not quaternions:
        raise ValueError('No orientation observations')
    q = np.asarray(quaternions)
    _, vectors = np.linalg.eigh(q.T @ q)
    return vectors[:, -1]


def detector(dictionary_name, min_marker_distance_rate=.02):
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
    # OpenCV 4.6 exposes a bare DetectorParameters constructor that can segfault.
    # Keep its factory API paired with legacy detectMarkers.
    parameters = (cv2.aruco.DetectorParameters() if hasattr(cv2.aruco, 'ArucoDetector')
                  else cv2.aruco.DetectorParameters_create())
    parameters.minMarkerDistanceRate = float(min_marker_distance_rate)
    if hasattr(cv2.aruco, 'ArucoDetector'):
        return cv2.aruco.ArucoDetector(dictionary, parameters).detectMarkers
    return lambda image: cv2.aruco.detectMarkers(image, dictionary, parameters=parameters)


def estimate(corners, size, k, distortion, min_side, max_error, min_distance, max_distance, diagnostics=None):
    def reject(reason):
        if diagnostics is not None:
            diagnostics[reason] = diagnostics.get(reason, 0)+1

    c = np.asarray(corners, dtype=np.float64).reshape(4, 2)
    if np.min(np.linalg.norm(c-np.roll(c, -1, axis=0), axis=1)) < min_side:
        reject('marker_too_small')
        return None
    s = size/2
    points = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=float)
    ok, rotations, translations, _ = cv2.solvePnPGeneric(
        points, c, k, distortion, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        reject('pnp_failed')
        return None
    candidates = []
    for r, t in zip(rotations, translations):
        camera_points = (cv2.Rodrigues(r)[0] @ points.T).T + t.reshape(3)
        distance = float(np.linalg.norm(t))
        if np.min(camera_points[:, 2]) <= 0:
            reject('pnp_candidate_behind_camera')
            continue
        if not min_distance <= distance <= max_distance:
            reject('pnp_candidate_out_of_range')
            continue
        projected = cv2.projectPoints(points, r, t, k, distortion)[0].reshape(4, 2)
        error = float(np.sqrt(np.mean(np.sum((projected-c)**2, axis=1))))
        if np.isfinite(error) and error <= max_error:
            candidates.append((error, r.reshape(3), t.reshape(3)))
        else:
            reject('pnp_candidate_reprojection_error')
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
