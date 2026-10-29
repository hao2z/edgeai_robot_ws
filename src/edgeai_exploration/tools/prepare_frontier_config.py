"""Adapt upstream explorer parameters to the current robot and Nav2 stack."""
from pathlib import Path
import sys
import yaml

source, target = map(Path, sys.argv[1:3])
config = yaml.safe_load(source.read_text())
p = config['frontier_explorer']['ros__parameters']
p.update(repeat_guard_enabled=True, repeat_guard_match_distance_m=.35,
         repeat_guard_min_gain_m2=.03, repeat_guard_base_cooldown_s=30.0,
         repeat_guard_max_cooldown_s=240.0, use_sim_time=True, control_service_enabled=True,
         min_frontier_size_cells=5, dilation_kernel_radius_cells=1,
         post_goal_settle_enabled=True, post_goal_min_settle=1.5,
         navigate_to_pose_action_name='/exploration/navigate_to_pose',
         frontier_suppression_enabled=True, all_frontiers_suppressed_behavior='stay',
         max_linear_speed_vmax=.60, max_angular_speed_wmax=.90,
         frontier_selection_min_distance=.60, frontier_candidate_min_goal_distance_m=.50,
         goal_preemption_complete_if_within_m=0.0,
         goal_preemption_enabled=True,
         goal_skip_on_blocked_goal=False,
         goal_preemption_min_interval_s=3.0,
         frontier_suppression_attempt_threshold=2,
         frontier_suppression_timeout_s=10.0,
         frontier_suppression_base_size_m=.30,
         frontier_suppression_expansion_size_m=.10,
         frontier_suppression_no_progress_timeout_s=0.0,
         frontier_suppression_progress_epsilon_m=0.10)
target.write_text('# Adapted from frontier_exploration_ros2 1.6.1 (Apache-2.0).\n' +
                  yaml.safe_dump(config, sort_keys=False), encoding='utf-8')
rviz_path = target.parent.parent / 'rviz/realistic_robot.rviz'
rviz = yaml.safe_load(rviz_path.read_text())
manager = rviz['Visualization Manager']
manager['Global Options']['Fixed Frame'] = 'map'
for display in manager['Displays']:
    if display.get('Class') == 'rviz_default_plugins/Map':
        display.update(Enabled=True, Value=True)
        display['Topic']['Durability Policy'] = 'Transient Local'
manager['Displays'].append({
    'Class': 'rviz_default_plugins/MarkerArray', 'Enabled': True,
    'Name': 'Exploration frontiers', 'Value': True,
    'Marker Topic': {'Value': '/explore/frontiers', 'Depth': 10,
                     'Reliability Policy': 'Reliable', 'Durability Policy': 'Volatile'}})
manager['Displays'].append({
    'Class': 'rviz_default_plugins/Polygon', 'Enabled': True, 'Name': 'Robot footprint',
    'Value': True, 'Alpha': 1.0, 'Color': '0; 255; 0',
    'Topic': {'Value': '/local_costmap/published_footprint', 'Depth': 5,
              'Reliability Policy': 'Reliable', 'Durability Policy': 'Volatile'}})
manager['Views']['Current'].update(Pitch=1.5707, Yaw=0.0, Distance=9.0,
                                  **{'Target Frame': 'map', 'Focal Point': {'X': 1.5, 'Y': 1.5, 'Z': 0.0}})
(rviz_path.parent / 'exploration.rviz').write_text(yaml.safe_dump(rviz, sort_keys=False), encoding='utf-8')
from prepare_rviz_diagnostics import configure
configure(rviz_path.parent / 'exploration.rviz')
