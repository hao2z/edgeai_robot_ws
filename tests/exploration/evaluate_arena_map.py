"""Offline fixed-start evaluation; SDF geometry is never supplied to exploration."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import cv2
import numpy as np
import yaml

config_path, world_path = map(Path, sys.argv[1:3])
config = yaml.safe_load(config_path.read_text())
image = cv2.imread(str(config_path.parent / config['image']), cv2.IMREAD_GRAYSCALE)
step = .025
ys, xs = np.mgrid[-2.5+step/2:2.5:step, -2.5+step/2:2.5:step]
free = np.ones(xs.shape, dtype=bool)
for model in ET.parse(world_path).getroot().findall('world/model'):
    pose = list(map(float, (model.findtext('pose') or '0 0 0 0 0 0').split()))
    for collision in model.findall('link/collision'):
        size = collision.findtext('geometry/box/size')
        if not size:
            continue
        sx, sy, _ = map(float, size.split())
        dx, dy = xs-pose[0], ys-pose[1]
        c, s = np.cos(pose[5]), np.sin(pose[5])
        free &= ~((np.abs(c*dx+s*dy) <= sx/2) & (np.abs(-s*dx+c*dy) <= sy/2))
safe = cv2.erode(free.astype(np.uint8), cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (21, 21)),
                 borderType=cv2.BORDER_CONSTANT, borderValue=0)
count, labels = cv2.connectedComponents(safe)
start = (int((-1.5+2.5)/step), int((-1.5+2.5)/step))
reachable = labels == labels[start]
print(f'GEOMETRY safe_components={count-1} start_component_cells={np.count_nonzero(reachable)}')
resolution = config['resolution']
ox, oy, yaw = config['origin']
if abs(yaw) > 1e-6:
    raise ValueError('Expected axis-aligned saved map')
ix = np.floor((xs+1.5-ox)/resolution).astype(int)
iy = image.shape[0]-1-np.floor((ys+1.5-oy)/resolution).astype(int)
inside = (ix >= 0) & (ix < image.shape[1]) & (iy >= 0) & (iy < image.shape[0])
pixels = np.full(xs.shape, 205, dtype=np.uint8)
pixels[inside] = image[iy[inside], ix[inside]]
regions = {'all': np.ones_like(free), 'upper_left': (xs < -1)&(ys > .55),
           'upper_middle': (xs >= -1)&(xs <= 1.05)&(ys > .55),
           'upper_right': (xs > 1.05)&(ys > .55),
           'lower_right': (xs > 1.2)&(ys < -.15),
           'main_area': (xs < 1.1)&(ys < .45)}
for name, region in regions.items():
    mask = free & region
    total = np.count_nonzero(mask)
    print(f'COVERAGE {name}: free_m2={total*step**2:.3f} '
          f'observed_pct={100*np.count_nonzero(mask & (pixels != 205))/total:.2f} '
          f'estimated_free_pct={100*np.count_nonzero(mask & (pixels >= 250))/total:.2f}')
preview = cv2.resize(image, (808, 808), interpolation=cv2.INTER_NEAREST)
cv2.imwrite(str(config_path.with_suffix('.png')), preview)
