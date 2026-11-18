"""Mask estimated peer-body returns without inventing free space."""
import copy
from functools import lru_cache
import math

import numpy as np


@lru_cache(maxsize=16)
def ray_directions(count, angle_min, angle_increment):
    angles = angle_min + np.arange(count) * angle_increment
    return np.cos(angles), np.sin(angles)


def mask_scan(scan, laser_in_other, bounds, margin):
    """Pass through untouched scans and mask only returns inside the peer body."""
    x0, y0, yaw = laser_in_other
    xmin, xmax, ymin, ymax = bounds
    xmin, xmax, ymin, ymax = xmin-margin, xmax+margin, ymin-margin, ymax+margin
    nearest = math.hypot(max(xmin-x0, 0., x0-xmax), max(ymin-y0, 0., y0-ymax))
    farthest = math.hypot(max(abs(xmin-x0), abs(xmax-x0)),
                         max(abs(ymin-y0), abs(ymax-y0)))
    epsilon = 1e-9
    near = max(scan.range_min, nearest-epsilon)
    far = min(scan.range_max, farthest+epsilon)
    if near > far:
        return scan, 0
    ranges = np.asarray(scan.ranges)
    candidates = np.flatnonzero(np.isfinite(ranges) & (ranges >= near) & (ranges <= far))
    if candidates.size == 0:
        return scan, 0
    cosines, sines = ray_directions(len(ranges), scan.angle_min, scan.angle_increment)
    x = ranges[candidates] * cosines[candidates]
    y = ranges[candidates] * sines[candidates]
    c, s = math.cos(yaw), math.sin(yaw)
    peer_x, peer_y = x0+c*x-s*y, y0+s*x+c*y
    hits = candidates[(xmin <= peer_x) & (peer_x <= xmax) &
                      (ymin <= peer_y) & (peer_y <= ymax)]
    if hits.size == 0:
        return scan, 0
    result = copy.deepcopy(scan)
    for index in hits:
        result.ranges[index] = float('nan')
    return result, int(hits.size)
