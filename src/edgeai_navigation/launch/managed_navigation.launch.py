"""Keep Nav2 actions behind the cooperative navigation authority."""
from pathlib import Path
import json
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import SetRemap, PushRosNamespace


def setup(context):
    namespace=LaunchConfiguration("namespace").perform(context).strip("/")
    backend="/"+namespace+"/navigation_backend/" if namespace else "/navigation_backend/"
    names = ('navigate_to_pose', 'navigate_through_poses', 'follow_waypoints',
             'follow_path', 'compute_path_to_pose', 'compute_path_through_poses',
             'spin', 'backup', 'drive_on_heading', 'escape_occupied', 'wait', 'assisted_teleop',
             'dock_robot', 'undock_robot', 'goal_pose')
    navigation = Path(get_package_share_directory('nav2_bringup'))/'launch/navigation_launch.py'
    params_file = LaunchConfiguration('params_file').perform(context)
    params = yaml.safe_load(Path(params_file).read_text())
    local = params['local_costmap']['local_costmap']['ros__parameters']
    depth_actions = []
    if 'depth_layer' in local['plugins'] and local['depth_layer'].get('enabled', True):
        script = Path(get_package_share_directory('edgeai_navigation'))/'scripts/depth_obstacle_filter.py'
        # Pass the selected section explicitly: an unqualified YAML node key
        # would not match /robot2/depth_obstacle_filter in a fleet launch.
        args = ['python3', '-u', str(script), '--ros-args', '-r', '__ns:=/'+namespace]
        for name, value in params['depth_obstacle_filter']['ros__parameters'].items():
            args.extend(['-p', name+':='+json.dumps(value)])
        depth_actions.append(ExecuteProcess(cmd=args, output='screen'))
    return [
        *depth_actions,
        GroupAction([
            PushRosNamespace(namespace),
            SetRemap(src="/tf", dst="/tf"),
            SetRemap(src="/tf_static", dst="/tf_static"),
            SetRemap(src='controller_server:cmd_vel', dst='/cmd_vel_mppi_raw',
                     condition=IfCondition(LaunchConfiguration('mppi_invert_linear'))),
            # MPPI's predicted optimal path uses the existing local-plan display
            # and diagnostic subscribers. DWB already publishes local_plan.
            SetRemap(src='optimal_trajectory', dst='local_plan'),
            *[SetRemap(src=name, dst=backend+name) for name in names],
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(navigation)),
                launch_arguments={'namespace': namespace, 'params_file': LaunchConfiguration('params_file'),
                                  'use_sim_time': LaunchConfiguration('use_sim_time'),
                                  'autostart': 'true', 'use_composition': 'False'}.items()),
        ]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('params_file'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('mppi_invert_linear', default_value='false'),
        OpaqueFunction(function=setup),
    ])
