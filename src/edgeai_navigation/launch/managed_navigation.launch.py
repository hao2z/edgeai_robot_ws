"""Keep Nav2 actions behind the cooperative navigation authority."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import SetRemap


def generate_launch_description():
    names = ('navigate_to_pose', 'navigate_through_poses', 'follow_waypoints',
             'follow_path', 'compute_path_to_pose', 'compute_path_through_poses',
             'spin', 'backup', 'drive_on_heading', 'wait', 'assisted_teleop',
             'dock_robot', 'undock_robot', 'goal_pose')
    navigation = Path(get_package_share_directory('nav2_bringup'))/'launch/navigation_launch.py'
    return LaunchDescription([
        DeclareLaunchArgument('params_file'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('mppi_invert_linear', default_value='false'),
        GroupAction([
            SetRemap(src='controller_server:cmd_vel', dst='/cmd_vel_mppi_raw',
                     condition=IfCondition(LaunchConfiguration('mppi_invert_linear'))),
            # MPPI's predicted optimal path uses the existing local-plan display
            # and diagnostic subscribers. DWB already publishes local_plan.
            SetRemap(src='optimal_trajectory', dst='local_plan'),
            *[SetRemap(src=name, dst='/navigation_backend/'+name) for name in names],
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(navigation)),
                launch_arguments={'params_file': LaunchConfiguration('params_file'),
                                  'use_sim_time': LaunchConfiguration('use_sim_time'),
                                  'autostart': 'true', 'use_composition': 'False'}.items()),
        ]),
    ])
