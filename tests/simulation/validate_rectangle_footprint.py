"""Check that the configured rectangle encloses the model's wheel/caster sweep."""
from pathlib import Path
import json
import math
import xml.etree.ElementTree as ET
import yaml
import xacro

root = Path(__file__).resolve().parents[2]
share = root/'src/edgeai_robot_description'
urdf = ET.parse(share/'urdf/edgeai_robot_real.urdf.xacro').getroot()
props = {p.get('name'): float(p.get('value')) for p in urdf
         if p.tag.endswith('property') and p.get('name') in (
             'plate_length','plate_width','wheel_x','wheel_radius','wheel_separation',
             'wheel_width','caster_x','caster_y','caster_offset','caster_wheel_radius','caster_wheel_width')}
config = yaml.safe_load((share/'config/nav2_exploration_rectangle.yaml').read_text())
local = config['local_costmap']['local_costmap']['ros__parameters']
global_map = config['global_costmap']['global_costmap']['ros__parameters']
assert local['footprint'] == global_map['footprint']
assert 'robot_radius' not in local and 'robot_radius' not in global_map
corners = json.loads(local['footprint'])
xmin,xmax = min(p[0] for p in corners),max(p[0] for p in corners)
ymin,ymax = min(p[1] for p in corners),max(p[1] for p in corners)
points = [(sx*props['plate_length']/2,sy*props['plate_width']/2) for sx in (-1,1) for sy in (-1,1)]
points += [(props['wheel_x']+sx*props['wheel_radius'],
            sy*(props['wheel_separation']/2+props['wheel_width']/2))
           for sx in (-1,1) for sy in (-1,1)]
for i in range(360):
    angle=math.radians(i)
    for side in (-1,1):
        for sx in (-1,1):
            for sy in (-1,1):
                dx=-props['caster_offset']+sx*props['caster_wheel_radius']
                dy=sy*props['caster_wheel_width']/2
                points.append((props['caster_x']+math.cos(angle)*dx-math.sin(angle)*dy,
                               side*props['caster_y']+math.sin(angle)*dx+math.cos(angle)*dy))
# Geometry above is measured in chassis coordinates. Navigation rotates about
# the drive axle, so check the expanded TF and translate every envelope point.
expanded = ET.fromstring(xacro.process_file(str(share/'urdf/edgeai_robot_real.urdf.xacro')).toxml())
translation = list(map(float, expanded.find("joint[@name='base_footprint_joint']/origin").get('xyz').split()))
assert math.isclose(translation[0], -props['wheel_x'], abs_tol=1e-9)
for name in ('left_wheel_joint', 'right_wheel_joint'):
    wheel_pose = list(map(float, expanded.find(f"joint[@name='{name}']/origin").get('xyz').split()))
    assert math.isclose(wheel_pose[0]+translation[0], 0, abs_tol=1e-9)
points = [(x+translation[0],y+translation[1]) for x,y in points]
assert all(xmin <= x <= xmax and ymin <= y <= ymax for x,y in points)
pad=local['footprint_padding']
length,width=xmax-xmin+2*pad,ymax-ymin+2*pad
assert width < .40 < (length+width)/math.sqrt(2)
assert 'ObstacleFootprint' in config['controller_server']['ros__parameters']['FollowPath']['critics']
assert config['planner_server']['ros__parameters']['GridBased']['plugin'].endswith('SmacPlannerLattice')
primitive_path = share/'config'/config['planner_server']['ros__parameters']['GridBased']['lattice_filepath']
lattice = json.loads(primitive_path.read_text())
metadata = lattice['lattice_metadata']
assert metadata['motion_model'] == 'diff'
assert metadata['grid_resolution'] == local['resolution'] == global_map['resolution'] == .05
assert metadata['turning_radius'] == .20
turns = [p for p in lattice['primitives'] if all(abs(v) < 1e-8 for v in p['poses'][-1][:2])
         and p['start_angle_index'] != p['end_angle_index']]
assert {p['start_angle_index'] for p in turns} == set(range(metadata['num_of_headings']))
print(f'PASS: {len(points)} chassis/wheel/caster samples enclosed; padded rectangle {length:.3f} x {width:.3f} m')
print('PASS: 0.40 m straight gap admits the rectangle at zero yaw but rejects its 45-degree orientation')
print(f'PASS: wheel axle matches base_footprint; padded conservative spin diameter {2*max(math.hypot(x+math.copysign(pad,x),y+math.copysign(pad,y)) for x,y in corners):.3f} m')
print(f'PASS: lattice matches 5 cm grid and 0.20 m curve setting; {len(turns)} in-place turns cover every heading')
