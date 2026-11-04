"""Geometry gate for full-body rotation, independent of ROS."""
import math
import numpy as np


def open_disk(data, width, height, resolution, origin, position, radius):
    """Require every cell intersecting the disk to be known and non-lethal.

    Occupancy costmap uses 100 for lethal obstacles, 99 for inscribed inflation.
    Inflation remains a soft planning cost here; Spin checks actual footprint.
    """
    if resolution <= 0 or radius <= 0:
        return False
    x, y = position
    ox, oy = origin
    if x-radius < ox or y-radius < oy or x+radius > ox+width*resolution or y+radius > oy+height*resolution:
        return False
    a = np.asarray(data).reshape(height, width)
    lo_x = max(0, int(math.floor((x-radius-ox)/resolution)))
    hi_x = min(width-1, int(math.floor((x+radius-ox)/resolution)))
    lo_y = max(0, int(math.floor((y-radius-oy)/resolution)))
    hi_y = min(height-1, int(math.floor((y+radius-oy)/resolution)))
    for j in range(lo_y, hi_y+1):
        for i in range(lo_x, hi_x+1):
            # Exact shortest distance to a closed square, including cell edges.
            dx = max(ox+i*resolution-x, 0, x-(ox+(i+1)*resolution))
            dy = max(oy+j*resolution-y, 0, y-(oy+(j+1)*resolution))
            if dx*dx+dy*dy <= radius*radius and (a[j, i] < 0 or a[j, i] >= 100):
                return False
    return True


def new_view(position, history, min_distance):
    return all(math.hypot(position[0]-p[0], position[1]-p[1]) >= min_distance for p in history)
