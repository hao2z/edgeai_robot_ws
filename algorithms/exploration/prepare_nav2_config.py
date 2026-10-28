"""Create a robot-specific config from the installed Nav2 bringup defaults."""
from copy import deepcopy
from pathlib import Path
import json
import math
import sys
import xml.etree.ElementTree as ET

import yaml


class NoAliasesDumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True


source, output = map(Path, sys.argv[1:3])
model = ET.parse(output.parent.parent/'urdf/edgeai_robot_real.urdf.xacro').getroot()
wheel_x = next(float(p.get('value')) for p in model if p.tag.endswith('property') and p.get('name') == 'wheel_x')
footprint = [[round(x-wheel_x, 6), y] for x,y in
             ((.20,.165),(-.225,.165),(-.225,-.165),(.20,-.165))]
circle_radius = math.ceil(max(math.hypot(x,y) for x,y in footprint)*100)/100
config = yaml.safe_load(source.read_text(encoding='utf-8'))
for section in ('bt_navigator', 'behavior_server'):
    config[section]['ros__parameters']['robot_base_frame'] = 'base_footprint'
for section in ('local_costmap', 'global_costmap'):
    p = config[section][section]['ros__parameters']
    p['robot_base_frame'] = 'base_footprint'
    p['robot_radius'] = circle_radius
    p['inflation_layer']['inflation_radius'] = 0.45
    p.update(use_sim_time=True, update_frequency=20.0, publish_frequency=10.0,
             transform_tolerance=0.2)
local = config['local_costmap']['local_costmap']['ros__parameters']
local['plugins'] = ['obstacle_layer', 'inflation_layer']
local['obstacle_layer'] = config['global_costmap']['global_costmap']['ros__parameters']['obstacle_layer'].copy()
control = config['controller_server']['ros__parameters']
control['general_goal_checker'].update(xy_goal_tolerance=0.15, yaw_goal_tolerance=0.25)
control['progress_checker']['required_movement_radius'] = 0.15
control['progress_checker']['movement_time_allowance'] = 25.0
control['FollowPath'].update(vx_max=0.20, vx_min=-0.08, wz_max=0.65,
                             ax_max=0.4, ax_min=-0.4, az_max=0.8,
                             vx_std=0.10, wz_std=0.25, batch_size=800)
config['behavior_server']['ros__parameters'].update(max_rotational_vel=0.65, min_rotational_vel=0.25,
                                                     rotational_acc_lim=0.8)
smooth = config['velocity_smoother']['ros__parameters']
smooth.update(max_velocity=[0.20, 0.0, 0.65], min_velocity=[-0.08, 0.0, -0.65],
              max_accel=[0.4, 0.0, 0.8], max_decel=[-0.4, 0.0, -0.8])
monitor = config['collision_monitor']['ros__parameters']
monitor['FootprintApproach']['min_points'] = 3
monitor['scan']['min_height'] = -0.1
monitor['scan']['max_height'] = 0.5
for section in ('controller_server', 'behavior_server', 'velocity_smoother', 'collision_monitor'):
    config[section]['ros__parameters']['enable_stamped_cmd_vel'] = False
output.write_text('# Derived from nav2_bringup nav2_params.yaml (Apache-2.0).\n' +
                  yaml.dump(config, Dumper=NoAliasesDumper, sort_keys=False), encoding='utf-8')

# DWB uses Nav2's documented differential-drive example with the same
# speed, acceleration, footprint and costmaps as the MPPI trial.
dwb = deepcopy(config)
dwb['controller_server']['ros__parameters']['FollowPath'] = {
    'plugin': 'dwb_core::DWBLocalPlanner',
    'min_vel_x': 0.0, 'min_vel_y': 0.0, 'max_vel_x': 0.20, 'max_vel_y': 0.0,
    'max_vel_theta': 0.65, 'min_speed_xy': 0.0, 'max_speed_xy': 0.20,
    'min_speed_theta': 0.0, 'acc_lim_x': 0.4, 'acc_lim_y': 0.0,
    'acc_lim_theta': 0.8, 'decel_lim_x': -0.4, 'decel_lim_y': 0.0,
    'decel_lim_theta': -0.8, 'vx_samples': 20, 'vy_samples': 5,
    'vtheta_samples': 20, 'sim_time': 1.7, 'linear_granularity': 0.05,
    'angular_granularity': 0.025, 'transform_tolerance': 0.3,
    'xy_goal_tolerance': 0.15, 'trans_stopped_velocity': 0.05,
    'short_circuit_trajectory_evaluation': True, 'stateful': True,
    'critics': ['RotateToGoal', 'Oscillation', 'BaseObstacle', 'GoalAlign',
                'PathAlign', 'PathDist', 'GoalDist'],
    'BaseObstacle.scale': 0.02, 'PathAlign.scale': 32.0, 'GoalAlign.scale': 24.0,
    'PathAlign.forward_point_distance': 0.1, 'GoalAlign.forward_point_distance': 0.1,
    'PathDist.scale': 32.0, 'GoalDist.scale': 24.0,
    'RotateToGoal.scale': 32.0, 'RotateToGoal.slowing_factor': 5.0,
    'RotateToGoal.lookahead_time': -1.0,
}
dwb_output = output.with_name(output.stem + '_dwb.yaml')
dwb_output.write_text('# Nav2 DWB controller example with robot-specific limits.\n' +
                      yaml.dump(dwb, Dumper=NoAliasesDumper, sort_keys=False), encoding='utf-8')
safe = deepcopy(dwb)
for section in ('local_costmap', 'global_costmap'):
    p = safe[section][section]['ros__parameters']
    p['robot_radius'] = max(0.40, circle_radius)
    p['inflation_layer']['inflation_radius'] = 0.50
safe['collision_monitor']['ros__parameters']['FootprintApproach']['min_points'] = 1
output.with_name(output.stem + '_dwb_safe.yaml').write_text(
    '# Conservative footprint for doorways; paired with inflated-costmap frontier selection.\n' +
    yaml.dump(safe, Dumper=NoAliasesDumper, sort_keys=False), encoding='utf-8')

# Rectangular envelope includes drive wheels and the rear caster swivel sweep.
# Coordinates are relative to base_footprint; safety padding is separate.
rectangle = deepcopy(dwb)
footprint = json.dumps(footprint)
for section in ('local_costmap', 'global_costmap'):
    p = rectangle[section][section]['ros__parameters']
    p.pop('robot_radius', None)
    p.update(footprint=footprint, footprint_padding=0.01)
follow = rectangle['controller_server']['ros__parameters']['FollowPath']
follow['critics'] = [v.replace('BaseObstacle', 'ObstacleFootprint') for v in follow['critics']]
follow.pop('BaseObstacle.scale', None)
follow['ObstacleFootprint.scale'] = 0.02
follow.update(plugin='nav2_rotation_shim_controller::RotationShimController',
              primary_controller='dwb_core::DWBLocalPlanner',
              angular_dist_threshold=0.785, angular_disengage_threshold=0.25,
              forward_sampling_distance=0.35, rotate_to_heading_angular_vel=0.5,
              max_angular_accel=0.8, simulate_ahead_time=1.5,
              rotate_to_heading_once=True, rotate_to_goal_heading=False,
              use_path_orientations=True, closed_loop=True)
rectangle['planner_server']['ros__parameters']['GridBased'] = {
    'plugin': 'nav2_smac_planner::SmacPlannerLattice',
    'tolerance': 0.15, 'allow_unknown': True, 'max_iterations': 500000,
    'max_on_approach_iterations': 1000, 'max_planning_time': 3.0,
    'lattice_filepath': 'lattice/diff_5cm_r020.json',
    'allow_reverse_expansion': False, 'smooth_path': True,
    'lookup_table_size': 5.0, 'rotation_penalty': 5.0,
}
rectangle['collision_monitor']['ros__parameters']['FootprintApproach']['min_points'] = 1
output.with_name(output.stem + '_rectangle.yaml').write_text(
    '# Axle-centred rectangle: front .1125 m, rear .3125 m, width .33 m; padding .01 m.\n' +
    yaml.dump(rectangle, Dumper=NoAliasesDumper, sort_keys=False), encoding='utf-8')

# Task 2 integration candidate, not a measured precision guarantee or rescue judge.
rescue = deepcopy(rectangle)
rescue_control = rescue['controller_server']['ros__parameters']
rescue_control['general_goal_checker'].update(
    plugin='nav2_controller::StoppedGoalChecker', xy_goal_tolerance=0.05,
    yaw_goal_tolerance=math.pi, trans_stopped_velocity=0.02, rot_stopped_velocity=0.05)
rescue_control['FollowPath'].update(xy_goal_tolerance=0.05, trans_stopped_velocity=0.02)
rescue['planner_server']['ros__parameters']['GridBased']['tolerance'] = 0.05
output.with_name('nav2_rescue_rectangle.yaml').write_text(
    '# Candidate Task 2 goal tolerances; requires visual refinement and dwell verification.\n' +
    yaml.dump(rescue, Dumper=NoAliasesDumper, sort_keys=False), encoding='utf-8')
