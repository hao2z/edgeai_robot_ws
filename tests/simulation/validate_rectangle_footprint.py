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
config = yaml.safe_load((root/'src/edgeai_navigation/config/nav2_exploration_rectangle.yaml').read_text())
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
follow = config['controller_server']['ros__parameters']['FollowPath']
assert follow['plugin'] == 'nav2_rotation_shim_controller::RotationShimController'
assert follow['primary_controller'] == 'dwb_core::DWBLocalPlanner'
assert not follow['use_path_orientations'] and follow['closed_loop']
assert follow['rotate_to_heading_angular_vel'] <= follow['max_vel_theta']
assert follow['max_angular_accel'] <= follow['acc_lim_theta']
frontier = yaml.safe_load((root/'src/edgeai_exploration/config/frontier_exploration.yaml').read_text())['frontier_explorer']['ros__parameters']
assert frontier['goal_preemption_enabled'] and not frontier['goal_skip_on_blocked_goal']
# Outer exploration timeout is tunable; this geometry check does not prescribe policy.
assert frontier['frontier_suppression_no_progress_timeout_s'] >= 0.0
assert frontier['goal_preemption_complete_if_within_m'] == 0.0
assert frontier['goal_preemption_complete_if_within_m'] <= config['controller_server']['ros__parameters']['general_goal_checker']['xy_goal_tolerance']
assert config['planner_server']['ros__parameters']['GridBased']['plugin'] == 'nav2_navfn_planner::NavfnPlanner'
assert local['resolution'] == global_map['resolution'] == .05
spin_radius = max(math.hypot(x+math.copysign(pad,x),y+math.copysign(pad,y)) for x,y in corners)
assert local['inflation_layer']['inflation_radius'] == global_map['inflation_layer']['inflation_radius']
assert local['inflation_layer']['inflation_radius'] > 0
# Local avoidance and global routing may use different soft-cost decay.
for costmap in (local, global_map):
    assert costmap['inflation_layer']['cost_scaling_factor'] > 0
assert follow['min_vel_x'] == config['velocity_smoother']['ros__parameters']['min_velocity'][0]
assert follow['min_vel_x'] <= 0
assert config['planner_server']['ros__parameters']['GridBased']['use_final_approach_orientation']
# Measure actual collision boxes, not the arena generator's historical comment.
world = ET.parse(root/'src/edgeai_simulation/worlds/competition_rooms.sdf').getroot()
def bounds(name):
    model = world.find(f"world/model[@name='{name}']")
    x, y, _, _, _, yaw = map(float, model.findtext('pose').split())
    assert abs(yaw) < 1e-9
    sx, sy, _ = map(float, model.findtext('link/collision/geometry/box/size').split())
    return x-sx/2, x+sx/2, y-sy/2, y+sy/2

gaps = {
    'upper_left_door': bounds('upper_left_partition')[2]-bounds('left_room_bottom')[3],
    'upper_right_door': bounds('upper_right_partition')[2]-bounds('right_room_bottom')[3],
    'lower_right_door': bounds('lower_right_top')[2]-bounds('lower_right_side')[3],
    'right_horizontal_corridor': bounds('right_room_bottom')[2]-bounds('lower_right_top')[3],
}
for name, gap in gaps.items():
    # Reserve one whole cell at each side for a conservative rasterization check.
    available = gap-width-2*local['resolution']
    assert available > 0, (name, gap, width)
    soft_gap = gap-2*local['inflation_layer']['inflation_radius']
    print(f'PASS: {name}: opening {gap:.3f} m; padded straight-body clearance {gap-width:.3f} m; '
          f'after two grid cells {available:.3f} m; outside-inflation strip {max(0,soft_gap):.3f} m')
print(f'PASS: {len(points)} chassis/wheel/caster samples enclosed; padded rectangle {length:.3f} x {width:.3f} m')
print('PASS: 0.40 m straight gap admits the rectangle at zero yaw but rejects its 45-degree orientation')
print(f'PASS: wheel axle matches base_footprint; padded conservative spin diameter {2*max(math.hypot(x+math.copysign(pad,x),y+math.copysign(pad,y)) for x,y in corners):.3f} m')
print('PASS: NavFn active; both costmaps use 5 cm resolution')
