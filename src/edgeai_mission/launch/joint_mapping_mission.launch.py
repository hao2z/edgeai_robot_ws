"""Compose mapping, robot navigation, passive explorers with native robot namespaces."""
from pathlib import Path
import json
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from edgeai_nav.fleet_config import namespaced_config, configure_peer_costmaps, frontier_endpoint_geometry


def setup(context):
    directory=Path(LaunchConfiguration('directory').perform(context))
    nav=Path(get_package_share_directory('edgeai_navigation'))
    mapping=Path(get_package_share_directory('edgeai_mapping'))
    mission=Path(get_package_share_directory('edgeai_mission'))
    exploration=Path(get_package_share_directory('edgeai_exploration'))
    source=yaml.safe_load((directory/'nav2-runtime.yaml').read_text())
    secondary=namespaced_config(source,'robot2',source_namespace='robot1')
    # Both cameras serve target detection. Keep robot2 navigation laser-based.
    local=secondary['local_costmap']['local_costmap']['ros__parameters']
    local['plugins']=[p for p in local['plugins'] if p!='depth_layer']
    cooperative=yaml.safe_load((nav/'config/cooperative_rescue.yaml').read_text())
    configure_peer_costmaps(secondary,'robot2',cooperative.get('peer_costmap',{}))
    second=directory/'robot2';second.mkdir(exist_ok=True)
    (second/'nav2-runtime.yaml').write_text(yaml.safe_dump(secondary,sort_keys=False))
    footprint=json.loads(source['global_costmap']['global_costmap']['ros__parameters']['footprint'])
    xs,ys=zip(*footprint);bounds=[min(xs),max(xs),min(ys),max(ys)]
    runtime_file = directory/'mission-runtime.json'
    mapping_args = {'directory': str(directory), 'body_bounds': json.dumps(bounds)}
    if runtime_file.is_file():
        prior = json.loads(runtime_file.read_text()).get('initial_relative_pose')
        if prior is not None:
            mapping_args['initial_relative_pose'] = json.dumps(prior)
    actions=[
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(mapping/'launch/joint_mapping.launch.py')),
            launch_arguments=mapping_args.items()),
        ExecuteProcess(cmd=['python3','-u',str(nav/'scripts/joint_peer_poses.py'),
            '--ros-args','-p','use_sim_time:=true'],output='screen'),
        ExecuteProcess(cmd=['python3','-u',str(mission/'scripts/joint_frontier_monitor.py'),
            '--ros-args','-p','use_sim_time:=true'],output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav/'launch/managed_navigation.launch.py')),
            launch_arguments={'namespace':'robot2','params_file':str(second/'nav2-runtime.yaml')}.items()),
        ExecuteProcess(cmd=['python3','-u',str(nav/'scripts/navigation_authority.py'),'--directory',str(second),
            '--mode','STOPPED','--tree',str(nav/'config/behavior_trees/explore_recovery.xml'),
            '--ros-args','-r','__ns:=/robot2','-p','use_sim_time:=true'],output='screen'),
    ]
    frontier=yaml.safe_load((exploration/'config/frontier_exploration.yaml').read_text())
    p=frontier['frontier_explorer']['ros__parameters']
    p.update(frontier_endpoint_geometry(source))
    first=directory/'robot1';first.mkdir(exist_ok=True)
    primary=dict(p,autostart=False,robot_base_frame='robot1/base_footprint',costmap_topic='/robot1/global_costmap/costmap',local_costmap_topic='/robot1/local_costmap/costmap',navigate_to_pose_action_name='/robot1/navigation/assigned_frontier_to_pose',
        completion_event_topic='/robot1/exploration_complete',frontier_marker_topic='/robot1/explore/frontiers',
        selected_frontier_topic='/robot1/explore/selected_frontier',optimized_map_topic='/robot1/explore/optimized_map')
    (first/'frontier-runtime.yaml').write_text(yaml.safe_dump({'/**':{'ros__parameters':primary}},sort_keys=False))
    p.update(frontier_endpoint_geometry(secondary))
    p.update(robot_base_frame='robot2/base_footprint',map_topic='/map',autostart=False,
        costmap_topic='/robot2/global_costmap/costmap',local_costmap_topic='/robot2/local_costmap/costmap',
        navigate_to_pose_action_name='/robot2/navigation/assigned_frontier_to_pose',
        frontier_marker_topic='/robot2/explore/frontiers',selected_frontier_topic='/robot2/explore/selected_frontier',
        optimized_map_topic='/robot2/explore/optimized_map',completion_event_topic='/robot2/exploration_complete')
    config_file=second/'frontier-runtime.yaml';config_file.write_text(yaml.safe_dump({'/**':{'ros__parameters':p}},sort_keys=False))
    actions.append(Node(package='frontier_exploration_ros2',executable='frontier_explorer',namespace='robot2',
        name='frontier_explorer',parameters=[str(config_file)],
        remappings=[('/explore/available_frontiers','/robot2/explore/available_frontiers')],output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('directory'),OpaqueFunction(function=setup)])
