"""Offline coverage for the development arena; truth is never sent to the robot."""
import math
from pathlib import Path
import sys

import cv2
import numpy as np
import yaml

path = Path(sys.argv[1])
config = yaml.safe_load(path.read_text(encoding='utf-8'))
image = cv2.imread(str(path.parent / config['image']), cv2.IMREAD_GRAYSCALE)
if image is None:
    raise RuntimeError('Saved occupancy map image is missing')
resolution = float(config['resolution'])
origin_x, origin_y, origin_yaw = config['origin']
if abs(origin_yaw) > 1e-6:
    raise ValueError('Evaluator expects an axis-aligned map')
rows, cols = np.indices(image.shape)
# Single-robot launch starts at world (-1.5, -1.5), heading zero; this
# registration is valid only for this development setup and drifts with SLAM.
x = origin_x + (cols + .5)*resolution - 1.5
y = origin_y + (image.shape[0] - rows - .5)*resolution - 1.5
interior = (abs(x) < 2.5) & (abs(y) < 2.5)
obstacle1 = (abs(x + .8) <= .4) & (abs(y - .9) <= .15)
obstacle2 = (abs(x - 1.1) <= .15) & (abs(y - .5) <= .5)
c, s = math.cos(.523599), math.sin(.523599)
dx, dy = x - .8, y + 1.2
obstacle3 = (abs(c*dx+s*dy) <= .4) & (abs(-s*dx+c*dy) <= .15)
truth_free = interior & ~(obstacle1 | obstacle2 | obstacle3)
if not truth_free.any():
    raise RuntimeError('Map does not overlap the expected single-robot arena')
unknown = image == 205
observed = truth_free & ~unknown
estimated_free = truth_free & (image >= 250)
# The denominator covers the entire arena, including space outside the saved
# map rectangle. The three non-overlapping rectangular obstacles total .78 m2.
truth_free_area = 25.0 - .78
observed_area = np.count_nonzero(observed)*resolution**2
estimated_free_area = np.count_nonzero(estimated_free)*resolution**2
print(f'COVERAGE truth_free_m2={truth_free_area:.2f} '
      f'observed_pct={100*observed_area/truth_free_area:.2f} '
      f'estimated_free_pct={100*estimated_free_area/truth_free_area:.2f} '
      f'unobserved_pct={100*(truth_free_area-observed_area)/truth_free_area:.2f} '
      '(development ground truth, fixed start alignment; SLAM drift affects this estimate)', flush=True)
