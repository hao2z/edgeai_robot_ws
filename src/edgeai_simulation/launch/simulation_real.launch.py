import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import xacro


def generate_launch_description():

    # Package paths
    # package_name = 'edgeai_robot_description'
    # pkg_share = get_package_share_directory(package_name)
    description_share = get_package_share_directory('edgeai_robot_description')
    simulation_share = get_package_share_directory('edgeai_simulation')

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    # Launch arguments
    use_slam_arg = DeclareLaunchArgument(
        'use_slam',
        default_value='true',
        description='Automatically launch SLAM Toolbox online_async_launch.py'
    )

    # New realistic robot model
    xacro_file = os.path.join(
        description_share,
        'urdf',
        'edgeai_robot_real.urdf.xacro'
    )

    # Reuse the existing Gazebo world
    world_file = os.path.join(
        simulation_share,
        'worlds',
        'competition_rooms.sdf'
    )

    # Convert Xacro to robot_description
    robot_description_config = xacro.process_file(xacro_file)

    robot_description = {
        'robot_description': robot_description_config.toxml()
    }

    # Start Gazebo Harmonic
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_ros_gz_sim,
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}'
        }.items()
    )

    # Publish robot TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Spawn the new robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'edgeai_robot_real',
            '-topic', 'robot_description',
            '-x', '-1.5',
            '-y', '-1.5',
            '-z', '0.01'
        ],
        output='screen'
    )

    # ROS 2 <-> Gazebo bridge for DiffDrive
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',

            # LiDAR
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',

            # RGB-D camera
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',

            # Clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',

        ]
    )

    # Static TF for Gazebo RGB-D camera sensor frame
    camera_sensor_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_sensor_tf',
        output='screen',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'camera_link',
            '--child-frame-id',
            'edgeai_robot_real/base_footprint/rgbd_camera',
        ]
    )

    # SLAM Toolbox (delayed by 3.0s so /clock and /tf are active before activation)
    slam_params_file = os.path.join(simulation_share, 'config', 'slam_toolbox.yaml')
    slam_toolbox_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        pkg_slam_toolbox,
                        'launch',
                        'online_async_launch.py'
                    )
                ),
                launch_arguments={
                    'slam_params_file': slam_params_file,
                    'use_sim_time': 'true',
                }.items(),
                condition=IfCondition(LaunchConfiguration('use_slam'))
            )
        ]
    )
    
    return LaunchDescription([
        use_slam_arg,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
        camera_sensor_tf,
        slam_toolbox_launch,
    ])