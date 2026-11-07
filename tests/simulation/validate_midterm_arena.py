"""Check Figure 1 geometry, actual composed markers and collision-free starts."""
import importlib.util
import math
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import yaml

root = Path(__file__).resolve().parents[2]
share = root / 'src/edgeai_simulation'
spec = importlib.util.spec_from_file_location('simulation_support', share / 'launch/simulation_support.py')
support = importlib.util.module_from_spec(spec)
spec.loader.exec_module(support)
world = ET.parse(share / 'worlds/midterm.sdf').getroot().find('world')

expected = {
    'south_wall': (2.5, 0, 5.1, .1), 'north_wall': (2.5, 5, 5.1, .1),
    'west_wall': (0, 2.5, .1, 5), 'east_wall': (5, 2.5, .1, 5),
    'W1': (1.1, 3.1, 2.2, .1), 'W1b': (2.2, 3.35, .1, .5),
    'W2': (3.4, 1.4, .1, 2.8), 'O2': (2.2, 2, .6, 1), 'O3': (4.4, 3.6, .8, .6),
}
bounds = []
for name, expected_box in expected.items():
    model = world.find(f"model[@name='{name}']")
    x, y, _, _, _, yaw = map(float, model.findtext('pose').split())
    sx, sy, _ = map(float, model.findtext('link/collision/geometry/box/size').split())
    assert np.allclose((x, y, sx, sy), expected_box) and yaw == 0, name
    bounds.append((x-sx/2, x+sx/2, y-sy/2, y+sy/2))
targets = support.load_yaml(share / 'config/targets_midterm.yaml')
assert targets['dictionary'] == 'DICT_4X4_50' and targets['marker_size_m'] == .16
assert [(t['id'], t['name'], t['type'], t['pose'][:3], t['rescue_radius_m'], t['dwell_s'])
        for t in targets['targets']] == [
            (1, 'S1', 'normal', [1, 4.1, .15], .5, 10),
            (2, 'C1', 'cooperative', [4.1, 1, .15], .7, 10)]
for target, sign in zip(targets['targets'], (-1, 1)):
    assert math.isclose(math.sin(target['pose'][5]), sign)
    assert abs(math.cos(target['pose'][5])) < 1e-9
    for part in ('base', 'pole'):
        assert world.find(f"model[@name='{target['name']}_stand_{part}']/link/collision") is not None
    panel = cv2.imread(str(share / targets['assets_directory'] / f"aruco_{target['id']}/marker.png"), 0)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    _, ids, _ = cv2.aruco.detectMarkers(panel, dictionary)
    assert ids.flatten().tolist() == [target['id']]

footprint = yaml.safe_load((root / 'src/edgeai_navigation/config/nav2_exploration_rectangle.yaml').read_text())
corners = yaml.safe_load(footprint['local_costmap']['local_costmap']['ros__parameters']['footprint'])
robot_bounds = []
for robot in support.load_yaml(share / 'config/robots_midterm.yaml')['robots']:
    x, y, _, _, _, yaw = robot['pose']
    points = [(x+math.cos(yaw)*a-math.sin(yaw)*b, y+math.sin(yaw)*a+math.cos(yaw)*b)
              for a, b in corners]
    assert all(.05 < a < 1.4 and .05 < b < 1 for a, b in points)
    rb = (min(a for a, b in points), max(a for a, b in points),
          min(b for a, b in points), max(b for a, b in points))
    for ob in bounds + robot_bounds:
        assert rb[1] < ob[0] or rb[0] > ob[1] or rb[3] < ob[2] or rb[2] > ob[3]
    robot_bounds.append(rb)
with tempfile.TemporaryDirectory(prefix='edgeai-midterm-check-') as temp:
    output = Path(temp) / 'world.sdf'
    support.make_world(share, share / 'config/targets_midterm.yaml', output, share / 'worlds/midterm.sdf')
    composed = ET.parse(output).getroot().find('world')
    assert len(composed.findall('include')) == 2
    for target in targets['targets']:
        include = composed.find(f"include[name='{target['name']}']")
        assert list(map(float, include.findtext('pose').split())) == target['pose']
        disk = composed.find(f"model[@name='zone_{target['name']}_0']")
        assert list(map(float, disk.findtext('pose').split()))[:2] == target['pose'][:2]
        assert float(disk.findtext('link/visual/geometry/cylinder/radius')) == target['rescue_radius_m']
        assert disk.find('link/collision') is None
print('PASS: midterm walls, targets, marker decoding, centred rescue radii and both start footprints')
