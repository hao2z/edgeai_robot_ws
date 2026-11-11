#!/usr/bin/env python3
# Shared runtime configuration for exploration and standalone rescue.
from pathlib import Path
import sys, yaml, os
source, output = map(Path, sys.argv[1:])
# Include both compatibility scripts and the source package before the workspace
# overlay has been sourced (the standalone Part2 preparation stage).
sys.path[:0]=[str(source.parent.parent/'scripts'),str(source.parent.parent)]
from navigation_config import load_navigation
profile=os.environ.get('EDGEAI_NAV_PROFILE','rectangle')
depth_switch=os.environ.get('EDGEAI_DEPTH_OBSTACLES','config').lower()
if depth_switch not in ('config','true','false'):
    raise ValueError('EDGEAI_DEPTH_OBSTACLES must be config, true or false')
config=load_navigation(source.parent,os.environ.get('EDGEAI_CONTROLLER','dwb'),profile,
                       None if depth_switch=='config' else depth_switch=='true')
planner_profile=os.environ.get('EDGEAI_GLOBAL_PLANNER','lattice' if profile=='rectangle' else 'navfn')
if planner_profile=='lattice':
    from ament_index_python.packages import get_package_share_directory
    from planner_profile import configure_lattice
    configure_lattice(config,source.parent,get_package_share_directory('nav2_smac_planner'))
elif planner_profile!='navfn':
    raise ValueError('EDGEAI_GLOBAL_PLANNER must be lattice or navfn')
for section in ('local_costmap', 'global_costmap'):
    costmap = config[section][section]['ros__parameters']
    for dimension in ('width', 'height'):
        value = costmap.get(dimension)
        if value is not None and (type(value) is not int or value <= 0):
            raise ValueError(f'{section}.{dimension} must be a positive integer in Nav2 Jazzy; '
                             'set fractional_window.width_m/height_m for fractional local dimensions')
if os.environ.get('EDGEAI_MOTION_ABLATION') == 'goal_only':
    assert os.environ.get('EDGEAI_TURN_TEST') == 'true'
    follow = config['controller_server']['ros__parameters']['FollowPath']
    assert follow['plugin'] == 'nav2_mppi_controller::MPPIController'
    follow['critics'] = ['ConstraintCritic', 'GoalCritic', 'GoalAngleCritic', 'PreferForwardCritic']
    follow['GoalCritic']['threshold_to_consider'] = 10.0
    print('TEST ABLATION: goal-only MPPI; collision monitor remains enabled. Not a deployment profile.', flush=True)
planner = config['planner_server']['ros__parameters']['GridBased']
if 'lattice_filepath' in planner:
    planner['lattice_filepath'] = str((source.parent / planner['lattice_filepath']).resolve())
# A selectable stopped checker tightens rescue acceptance without changing exploration.
rescue_config = yaml.safe_load((Path(os.environ.get('EDGEAI_RESCUE_CONFIG', str(source.parents[0]/'normal_rescue.yaml')))).read_text())
control = config['controller_server']['ros__parameters']
control['general_goal_checker']['plugin'] = 'nav2_controller::PositionGoalChecker'
control['general_goal_checker'].pop('yaw_goal_tolerance', None)
if os.environ.get('EDGEAI_POINT_TEST_PARAMS'):
    assert os.environ.get('EDGEAI_TURN_TEST')=='true' and os.environ.get('EDGEAI_MOTION_TEST')=='points'
    overrides=yaml.safe_load(Path(os.environ['EDGEAI_POINT_TEST_PARAMS']).read_text()) or {}
    control['progress_checker'].update(overrides.pop('progress_checker',{}))
    control['FollowPath'].update(overrides)
is_navfn=planner['plugin']=='nav2_navfn_planner::NavfnPlanner'
if is_navfn:planner['use_final_approach_orientation'] = True
# Per-action policies use fixed plugin IDs; never mutate a live planner parameter.
from copy import deepcopy
planners = config['planner_server']['ros__parameters']
if is_navfn:
    planners['GridBasedHeading'] = deepcopy(planner)
    planners['GridBasedHeading']['use_final_approach_orientation'] = False
planners['planner_plugins'] = list(dict.fromkeys(planners['planner_plugins']+['GridBasedHeading']))
control['goal_checker_plugins'] = list(dict.fromkeys(control['goal_checker_plugins']+['heading_goal_checker','rescue_goal_checker']))
control['heading_goal_checker'] = dict(plugin='nav2_controller::SimpleGoalChecker', stateful=True,
    xy_goal_tolerance=control['general_goal_checker']['xy_goal_tolerance'],
    yaw_goal_tolerance=float(rescue_config['yaw_tolerance_rad']))
control['rescue_goal_checker'] = dict(plugin='nav2_controller::StoppedGoalChecker', stateful=True,
    xy_goal_tolerance=float(rescue_config['position_tolerance_m']),
    yaw_goal_tolerance=float(rescue_config['yaw_tolerance_rad']),
    trans_stopped_velocity=float(rescue_config['stopped_linear_mps']),
    rot_stopped_velocity=float(rescue_config['stopped_angular_rps']))
sys.path.insert(0,str(source.parent.parent/'scripts'))
from goal_policy import configure_controllers
configure_controllers(config)
from motion_limits import configure_motion_limits
import json
motion_report=configure_motion_limits(config)
output.with_name('motion-limits.json').write_text(json.dumps(motion_report,indent=2))
if motion_report['controllers']:
    print('MOTION_LIMITS: RPP预测前速度已联动公共限幅；详见 motion-limits.json',flush=True)
for warning in motion_report['warnings']:print('MOTION_LIMITS_WARNING: '+warning,flush=True)
bt = config['bt_navigator']['ros__parameters']
local_geometry=config['local_costmap']['local_costmap']['ros__parameters']
escape=config['behavior_server']['ros__parameters']['escape_occupied']
escape['actual_footprint']=local_geometry.get('footprint','[]')
escape['footprint_padding']=float(local_geometry.get('footprint_padding',0.))
bt['plugin_lib_names']=list(dict.fromkeys(bt.get('plugin_lib_names',[])+['edgeai_navigation_bt_nodes']))
bt['navigators'] = ['navigate_to_pose']
bt['default_nav_to_pose_bt_xml'] = str((source.parent/'behavior_trees/explore_recovery.xml').resolve())
if os.environ.get('EDGEAI_COOPERATIVE_RESCUE') == 'true':
    from fleet_config import configure_peer_costmaps
    coop=yaml.safe_load((source.parent/'cooperative_rescue.yaml').read_text())
    configure_peer_costmaps(config,'',coop.get('peer_costmap',{}))
output.write_text(yaml.safe_dump(config, sort_keys=False))
