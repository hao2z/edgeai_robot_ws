"""Midterm world, two freestanding targets and one or two robots in the start area."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def setup(context):
    share = Path(get_package_share_directory('edgeai_simulation'))
    count = int(LaunchConfiguration('robot_count').perform(context))
    if count not in (1, 2):
        raise ValueError('Midterm start area supports robot_count:=1 or 2')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch/simulation_real.launch.py')),
        launch_arguments={
            'world_file': str(share / 'worlds/midterm.sdf'),
            'targets_file': str(share / 'config/targets_midterm.yaml'),
            'robots_file': str(share / 'config/robots_midterm.yaml'),
            'use_robot_pose': 'true',
            'multi_robot': 'true' if count == 2 else 'false',
            'robot_count': str(count),
            'gui': LaunchConfiguration('gui').perform(context),
            'rviz': LaunchConfiguration('rviz').perform(context),
            'slam': LaunchConfiguration('slam').perform(context),
        }.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_count', default_value='1'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('slam', default_value='false'),
        OpaqueFunction(function=setup),
    ])
