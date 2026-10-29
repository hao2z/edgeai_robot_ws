"""Single-robot compatible entry point, with optional 1..4 namespaced robots."""
import importlib.util
import math
import os
from pathlib import Path
import re
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def setup(context):
    share = Path(get_package_share_directory('edgeai_simulation'))
    description_share = Path(get_package_share_directory('edgeai_robot_description'))
    spec = importlib.util.spec_from_file_location('edgeai_simulation_support', share / 'launch/simulation_support.py')
    support = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(support)
    count = int(LaunchConfiguration('robot_count').perform(context))
    multi = LaunchConfiguration('multi_robot').perform(context).lower() == 'true'
    if not 1 <= count <= 4 or (not multi and count != 1):
        raise ValueError('Use multi_robot:=true for 2..4 robots; robot_count must be 1..4')
    fleet = support.load_yaml(LaunchConfiguration('robots_file').perform(context))['robots'][:count]
    if len(fleet) != count:
        raise ValueError('robots_file does not contain enough robot entries')
    names = [robot['name'] for robot in fleet]
    if len(set(names)) != count or any(not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', name) for name in names):
        raise ValueError('Robot names must be unique simple ROS namespace identifiers')
    for robot in fleet:
        if len(robot['pose']) != 6 or not all(math.isfinite(v) for v in robot['pose']):
            raise ValueError('Robot poses must contain six finite values')
    imu = support.load_yaml(LaunchConfiguration('imu_file').perform(context))
    world_dir = Path(tempfile.mkdtemp(prefix='edgeai_world_'))
    world_path = world_dir / 'arena.sdf'
    support.make_world(share, LaunchConfiguration('targets_file').perform(context), world_path,
                       LaunchConfiguration('world_file').perform(context))
    gz_share = Path(get_package_share_directory('ros_gz_sim'))
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gz_share / 'launch/gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {"" if gui else "-s "}{world_path}'}.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
             parameters=[{'use_sim_time': True}])]
    raw_xml = xacro.process_file(str(description_share / 'urdf/edgeai_robot_real.urdf.xacro')).toxml()
    # Diagnostic switches are environment-only so the normal competition
    # configuration remains unchanged.  Removing the sensor from the model
    # also avoids creating its rendering/update thread in Gazebo.
    disabled_sensors = {
        name for name, variable in (
            ('rgbd_camera', 'EDGEAI_DISABLE_RGBD_CAMERA'),
            ('lidar_sensor', 'EDGEAI_DISABLE_GPU_LIDAR'),
            ('imu_sensor', 'EDGEAI_DISABLE_IMU'),
        ) if os.environ.get(variable, '').lower() in ('1', 'true', 'yes')
    }
    if disabled_sensors:
        root = ET.fromstring(raw_xml)
        for gazebo in root.findall('gazebo'):
            for sensor in list(gazebo.findall('sensor')):
                if sensor.get('name') in disabled_sensors:
                    gazebo.remove(sensor)
        raw_xml = ET.tostring(root, encoding='unicode')
    for robot in fleet:
        ns = robot['name'] if multi else ''
        prefix = ns + '/' if ns else ''
        entity = ns or 'edgeai_robot_real'
        topic = lambda name: '/' + prefix + name
        description = support.prepare_robot(raw_xml, ns, imu)
        pose = robot['pose'] if multi else [-1.5, -1.5, 0.01, 0, 0, 0]
        actions.append(Node(package='robot_state_publisher', executable='robot_state_publisher',
                            namespace=ns, name='robot_state_publisher',
                            parameters=[{'robot_description': description, 'use_sim_time': True}]))
        actions.append(Node(package='ros_gz_sim', executable='create', namespace=ns,
                            name='spawn_robot', arguments=['-name', entity, '-topic', topic('robot_description'),
                            '-x', str(pose[0]), '-y', str(pose[1]), '-z', str(pose[2]),
                            '-R', str(pose[3]), '-P', str(pose[4]), '-Y', str(pose[5])], output='screen'))
        bridge_types = {
            'odom': ('nav_msgs/msg/Odometry', 'gz.msgs.Odometry'),
            'tf': ('tf2_msgs/msg/TFMessage', 'gz.msgs.Pose_V'),
            'joint_states': ('sensor_msgs/msg/JointState', 'gz.msgs.Model'),
            'scan': ('sensor_msgs/msg/LaserScan', 'gz.msgs.LaserScan'),
            'imu/data_raw': ('sensor_msgs/msg/Imu', 'gz.msgs.IMU'),
            'camera/image': ('sensor_msgs/msg/Image', 'gz.msgs.Image'),
            'camera/depth_image': ('sensor_msgs/msg/Image', 'gz.msgs.Image'),
            'camera/camera_info': ('sensor_msgs/msg/CameraInfo', 'gz.msgs.CameraInfo'),
            'camera/points': ('sensor_msgs/msg/PointCloud2', 'gz.msgs.PointCloudPacked'),
        }
        if 'rgbd_camera' in disabled_sensors:
            bridge_types = {name: value for name, value in bridge_types.items()
                            if not name.startswith('camera/')}
        if 'lidar_sensor' in disabled_sensors:
            bridge_types.pop('scan', None)
        if 'imu_sensor' in disabled_sensors:
            bridge_types.pop('imu/data_raw', None)
        arguments = [f'{topic(name)}@{ros}[{gz}' for name, (ros, gz) in bridge_types.items()]
        arguments.append(f'{topic("cmd_vel")}@geometry_msgs/msg/Twist]gz.msgs.Twist')
        remaps = [(topic('tf'), '/tf')] if multi else []
        actions.append(Node(package='ros_gz_bridge', executable='parameter_bridge', namespace=ns,
                            name='ros_gz_bridge', arguments=arguments, remappings=remaps,
                            parameters=[{'use_sim_time': True}], output='screen'))
        if LaunchConfiguration('slam').perform(context).lower() == 'true':
            params = support.load_yaml(share / 'config/slam_toolbox.yaml')['slam_toolbox']['ros__parameters']
            params.update(use_sim_time=True, map_frame=prefix + 'map', odom_frame=prefix + 'odom',
                          base_frame=prefix + 'base_footprint', scan_topic=topic('scan'))
            actions.append(Node(package='slam_toolbox', executable='async_slam_toolbox_node',
                                namespace=ns, name='slam_toolbox', parameters=[params],
                                remappings=[('/map', topic('map')), ('/map_metadata', topic('map_metadata'))],
                                output='screen'))
            actions.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                                namespace=ns, name='slam_lifecycle_manager',
                                parameters=[{'use_sim_time': True, 'autostart': True,
                                             'node_names': ['slam_toolbox'], 'bond_timeout': 0.0}]))
        if LaunchConfiguration('rviz').perform(context).lower() == 'true':
            rviz_text = (description_share / 'rviz/realistic_robot.rviz').read_text()
            rviz_text = rviz_text.replace('  X: 1920', '  X: 40').replace('  Width: 1920', '  Width: 1280').replace('  Height: 1043', '  Height: 720')
            fixed = prefix + ('map' if LaunchConfiguration('slam').perform(context).lower() == 'true' else 'odom')
            rviz_text = rviz_text.replace('Fixed Frame: odom', 'Fixed Frame: ' + fixed)
            if multi:
                rviz_text = rviz_text.replace('Target Frame: base_link', 'Target Frame: ' + prefix + 'base_link')
                for name in sorted(['robot_description', 'map', 'map_updates', 'initialpose', 'goal_pose', 'clicked_point'] + list(bridge_types), key=len, reverse=True):
                    if name != 'tf':
                        rviz_text = rviz_text.replace('Value: /' + name + '\n', 'Value: ' + topic(name) + '\n')
            rviz_path = world_dir / (entity + '.rviz')
            rviz_path.write_text(rviz_text)
            actions.append(Node(package='rviz2', executable='rviz2', namespace=ns,
                                name='rviz', arguments=['-d', str(rviz_path)],
                                parameters=[{'use_sim_time': True}]))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('edgeai_simulation'))
    return LaunchDescription([
        DeclareLaunchArgument('multi_robot', default_value='false'),
        DeclareLaunchArgument('robot_count', default_value='1'),
        DeclareLaunchArgument('robots_file', default_value=str(share / 'config/robots.yaml')),
        DeclareLaunchArgument('targets_file', default_value=str(share / 'config/targets.yaml')),
        DeclareLaunchArgument('world_file', default_value=str(share / 'worlds/empty.sdf')),
        DeclareLaunchArgument('imu_file', default_value=str(share / 'config/imu.yaml')),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('slam', default_value='false'),
        OpaqueFunction(function=setup),
    ])
