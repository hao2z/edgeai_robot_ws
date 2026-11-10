"""Minimal Nav2 stack: SLAM supplies map->odom; all motion goes through guard."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from edgeai_autonomy.drive_odometry import AXLE_OFFSET_X


def generate_launch_description():
    params = os.path.join(get_package_share_directory('edgeai_autonomy'), 'config', 'nav2.yaml')
    nodes = [('nav2_controller', 'controller_server'), ('nav2_planner', 'planner_server'),
             ('nav2_behaviors', 'behavior_server'), ('nav2_bt_navigator', 'bt_navigator')]
    return LaunchDescription([
        Node(package=package, executable=name, name=name, output='screen',
             parameters=[params, {'use_sim_time': True}],
             remappings=[('cmd_vel', '/cmd_vel_nav')]) for package, name in nodes
    ] + [Node(package='tf2_ros', executable='static_transform_publisher',
              name='drive_axle_tf', parameters=[{'use_sim_time': True}],
              arguments=['--x', str(AXLE_OFFSET_X), '--frame-id', 'base_footprint',
                         '--child-frame-id', 'drive_axle']),
         Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
              name='lifecycle_manager_navigation', output='screen',
              parameters=[{'use_sim_time': True, 'autostart': True,
                           'node_names': [name for _, name in nodes]}])])
