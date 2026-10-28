"""Two robots by default; overrides pass through to the common launch."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory('edgeai_robot_description'))
    return LaunchDescription([
        DeclareLaunchArgument('robot_count', default_value='2'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch/simulation_real.launch.py')),
            launch_arguments={'multi_robot': 'true', 'robot_count': LaunchConfiguration('robot_count')}.items()),
    ])
