"""Render SDF box geometry as a development arena floor plan."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import cv2
import numpy as np

world_path, output = map(Path, sys.argv[1:3])
image = np.full((860, 860, 3), 245, dtype=np.uint8)
def pixel(x, y):
    return int(430+x*150), int(430-y*150)
for model in ET.parse(world_path).getroot().findall('world/model'):
    pose = list(map(float, (model.findtext('pose') or '0 0 0 0 0 0').split()))
    for collision in model.findall('link/collision'):
        size = collision.findtext('geometry/box/size')
        if not size:
            continue
        sx, sy, _ = map(float, size.split())
        points = []
        for dx, dy in ((-sx/2,-sy/2),(-sx/2,sy/2),(sx/2,sy/2),(sx/2,-sy/2)):
            c, s = np.cos(pose[5]), np.sin(pose[5])
            points.append(pixel(pose[0]+c*dx-s*dy, pose[1]+s*dx+c*dy))
        cv2.fillPoly(image, [np.asarray(points)], (75, 75, 75))
cv2.circle(image, pixel(-1.5,-1.5), 18, (65,160,40), -1)
for text,x,y in [('START',-1.8,-1.85),('T1',-1.75,2.15),('TC1',0,2.15),('T2',1.75,2.15),('T3',1.75,-1.2)]:
    cv2.putText(image,text,pixel(x,y),cv2.FONT_HERSHEY_SIMPLEX,.65,(20,70,170),2)
cv2.putText(image,'PPT-inspired development arena: 5 m x 5 m',(95,25),cv2.FONT_HERSHEY_SIMPLEX,.65,(30,30,30),2)
cv2.imwrite(str(output), image)
