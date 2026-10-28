"""Inspect a recorded SLAM pose against a saved occupancy map."""
from pathlib import Path
import sys
import cv2
import yaml
import numpy as np
p = Path(sys.argv[1])
cfg = yaml.safe_load(p.read_text())
im = cv2.imread(str(p.parent / cfg['image']), cv2.IMREAD_GRAYSCALE)
x,y = map(float, sys.argv[2:4])
r = cfg['resolution']
ix = int((x-cfg['origin'][0])/r)
iy = im.shape[0]-1-int((y-cfg['origin'][1])/r)
distance = cv2.distanceTransform((im != 0).astype(np.uint8), cv2.DIST_L2, 5)*r
print(f'map_pose=({x},{y}), pixel={int(im[iy,ix])}, distance_to_occupied={distance[iy,ix]:.3f}m')
