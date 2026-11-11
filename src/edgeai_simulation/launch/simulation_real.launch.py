"""Single-robot compatible entry point, with optional 1..4 namespaced robots."""
import importlib.util
import math
import os
from pathlib import Path
import re
import tempfile
import xml.etree.ElementTree as ET
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
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
    truth = LaunchConfiguration('truth').perform(context).lower() == 'true'
    navigation_test = LaunchConfiguration('navigation_test').perform(context).lower() == 'true'
    if navigation_test and (not truth or LaunchConfiguration('slam').perform(context).lower()=='true'):
        raise ValueError('Truth navigation requires truth:=true and slam:=false')
    clock_rate = float(LaunchConfiguration('clock_rate').perform(context))
    if not math.isfinite(clock_rate) or (clock_rate != 0 and not 20 <= clock_rate <= 1000):
        raise ValueError('clock_rate must be 0 (unthrottled) or 20..1000 wall Hz')
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gz_share / 'launch/gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {"" if gui else "-s "}{world_path}'}.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
             remappings=[('/clock', '/simulation/clock_raw')] if clock_rate else [],
             parameters=[{'use_sim_time': False}])]
    if clock_rate:
        # One C++ relay bounds clock fan-out to every Python/Nav2 node. Forward
        # original timestamps; wall-clock throttling must not depend on its own output.
        actions.append(Node(package='topic_tools', executable='throttle', name='simulation_clock',
            arguments=['messages', '/simulation/clock_raw', str(clock_rate), '/clock'],
            parameters=[{'use_sim_time': False, 'use_wall_clock': True, 'lazy': False}]))
    if navigation_test:
        map_file=LaunchConfiguration('map_file').perform(context)
        if not Path(map_file).is_file():raise ValueError('Truth navigation map_file missing')
        actions.extend([
            Node(package='nav2_map_server',executable='map_server',name='map_server',
                 parameters=[{'use_sim_time':True,'yaml_filename':map_file}]),
            Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='test_map_lifecycle_manager',
                 parameters=[{'use_sim_time':True,'autostart':True,'node_names':['map_server'],'bond_timeout':0.}]),
            ExecuteProcess(cmd=['python3','-u',str(share/'tools/navigation_truth_localization.py'),
                                '--ros-args','-p','use_sim_time:=true'],output='screen')])
    raw_xml = xacro.process_file(str(description_share / 'urdf/edgeai_robot_real.urdf.xacro')).toxml()
    camera_rate = os.environ.get('EDGEAI_CAMERA_RATE', '30')
    try:
        camera_rate = str(max(5.0, min(30.0, float(camera_rate))))
    except ValueError:
        camera_rate = '30'
    raw_xml = re.sub(r'(<sensor name="rgbd_camera"[^>]*>.*?<update_rate>)[^<]+(</update_rate>)',
                     rf'\g<1>{camera_rate}\g<2>', raw_xml, count=1, flags=re.S)
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
        primary_unscoped = LaunchConfiguration('primary_unscoped').perform(context).lower() == 'true'
        ns = robot['name'] if multi and not (primary_unscoped and robot is fleet[0]) else ''
        prefix = ns + '/' if ns else ''
        entity = ns or 'edgeai_robot_real'
        topic = lambda name: '/' + prefix + name
        robot_xml=raw_xml
        robot_disabled=set(disabled_sensors)
        if primary_unscoped and robot is not fleet[0]:
            # The waiting/localizing follower needs lidar, not a second RGB-D renderer.
            robot_disabled.add('rgbd_camera')
            root=ET.fromstring(robot_xml)
            for gazebo in root.findall('gazebo'):
                for sensor in list(gazebo.findall('sensor')):
                    if sensor.get('name')=='rgbd_camera':gazebo.remove(sensor)
            robot_xml=ET.tostring(root,encoding='unicode')
        description = support.prepare_robot(robot_xml, ns, imu)
        if truth:
            root = ET.fromstring(description)
            plugin = ET.SubElement(ET.SubElement(root, 'gazebo'), 'plugin',
                filename='gz-sim-pose-publisher-system', name='gz::sim::systems::PosePublisher')
            for key, value in dict(publish_model_pose='true', publish_link_pose='false',
                                   publish_nested_model_pose='false', publish_visual_pose='false',
                                   publish_collision_pose='false', publish_sensor_pose='false',
                                   use_pose_vector_msg='true', update_frequency='20' if navigation_test else '2').items():
                ET.SubElement(plugin, key).text = value
            description = ET.tostring(root, encoding='unicode')
        configured_pose = LaunchConfiguration('use_robot_pose').perform(context).lower() == 'true'
        pose = robot['pose'] if multi or configured_pose else [-1.5, -1.5, 0.01, 0, 0, 0]
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
        if 'rgbd_camera' in robot_disabled:
            bridge_types = {name: value for name, value in bridge_types.items()
                            if not name.startswith('camera/')}
        if 'lidar_sensor' in disabled_sensors:
            bridge_types.pop('scan', None)
        if 'imu_sensor' in disabled_sensors:
            bridge_types.pop('imu/data_raw', None)
        bridge_parameters = {'use_sim_time': True}
        if os.environ.get('EDGEAI_EAGER_POINT_CLOUD', 'false').lower() != 'true' and 'camera/points' in bridge_types:
            # The depth filter normally projects sampled depth pixels itself.
            # Subscribe to the costly Gazebo cloud only while a ROS client needs it.
            ros_type, gz_type = bridge_types.pop('camera/points')
            cloud_topic = topic('camera/points')
            cloud_config = world_dir / (entity + '-cloud-bridge.yaml')
            cloud_config.write_text(yaml.safe_dump([dict(
                ros_topic_name=cloud_topic, gz_topic_name=cloud_topic,
                ros_type_name=ros_type, gz_type_name=gz_type,
                direction='GZ_TO_ROS', lazy=True)]))
            bridge_parameters['config_file'] = str(cloud_config)
        arguments = [f'{topic(name)}@{ros}[{gz}' for name, (ros, gz) in bridge_types.items()]
        arguments.append(f'{topic("cmd_vel")}@geometry_msgs/msg/Twist]gz.msgs.Twist')
        remaps = [(topic('tf'), '/tf')] if multi else []
        if truth:
            gz_pose = f'/model/{entity}/pose'
            arguments.append(f'{gz_pose}@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V')
            remaps.append((gz_pose, topic('truth/poses')))
        actions.append(Node(package='ros_gz_bridge', executable='parameter_bridge', namespace=ns,
                            name='ros_gz_bridge', arguments=arguments, remappings=remaps,
                            parameters=[bridge_parameters], output='screen'))
        if truth:
            actions.append(ExecuteProcess(cmd=[
                'python3', '-u', str(share / 'tools/truth_diagnostics.py'), '--ros-args',
                '-r', '__ns:=/'+ns, '-p', 'use_sim_time:=false',
                '-p', 'world_file:='+str(world_path),
                '-p', 'targets_file:='+LaunchConfiguration('targets_file').perform(context),
                '-p', 'entity:='+entity, '-p', 'frame_prefix:='+(prefix or "''"),
                '-p', 'shared_map:='+str(primary_unscoped).lower(),
                '-p', 'reference_frame:='+('map' if navigation_test or LaunchConfiguration('slam').perform(context).lower() == 'true' else 'odom'),
                '-p', 'log_directory:='+(os.environ.get('EDGEAI_TRUTH_DIRECTORY') or "''")], output='screen'))
        if (LaunchConfiguration('slam').perform(context).lower() == 'true' and
                (not primary_unscoped or robot is fleet[0])):
            params = support.load_yaml(share / 'config/slam_toolbox.yaml')['slam_toolbox']['ros__parameters']
            params.update(use_sim_time=True, map_frame=prefix + 'map', odom_frame=prefix + 'odom',
                          base_frame=prefix + 'base_footprint', scan_topic=topic('scan'))
            if primary_unscoped and multi and count==2:
                nav_share=Path(get_package_share_directory('edgeai_navigation'))
                filtered=support.load_yaml(nav_share/'config/fleet_scan_filter.yaml')
                filtered.pop('localization',None)
                # One source for the initial relative pose used by filter and localization.
                filtered['relative_pose']=support.load_yaml(nav_share/'config/cooperative_rescue.yaml')['initial_relative_pose']
                args=['python3','-u',str(nav_share/'scripts/fleet_scan_filter.py'),'--ros-args','-p','use_sim_time:=true']
                args.extend(['-r','robot2/odom:=/'+fleet[1]['name']+'/odom'])
                filter_file=world_dir/'fleet-filter.yaml'
                filter_file.write_text(yaml.safe_dump({'fleet_scan_filter':{'ros__parameters':filtered}}))
                args.extend(['--params-file',str(filter_file)])
                actions.append(ExecuteProcess(cmd=args,output='screen'))
                params['scan_topic']=topic('scan_slam')
            actions.append(Node(package='edgeai_navigation', executable='anchored_slam_toolbox',
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
                rviz_text = rviz_text.replace('Value: /truth/markers', 'Value: '+topic('truth/markers'))
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
        DeclareLaunchArgument('primary_unscoped', default_value='false'),
        DeclareLaunchArgument('robot_count', default_value='1'),
        DeclareLaunchArgument('robots_file', default_value=str(share / 'config/robots_midterm.yaml')),
        DeclareLaunchArgument('use_robot_pose', default_value='true'),
        DeclareLaunchArgument('targets_file', default_value=str(share / 'config/targets_midterm.yaml')),
        DeclareLaunchArgument('world_file', default_value=str(share / 'worlds/midterm.sdf')),
        DeclareLaunchArgument('imu_file', default_value=str(share / 'config/imu.yaml')),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('slam', default_value='false'),
        DeclareLaunchArgument('truth', default_value='true'),
        DeclareLaunchArgument('navigation_test', default_value='false'),
        DeclareLaunchArgument('map_file', default_value=''),
        DeclareLaunchArgument('clock_rate', default_value=os.environ.get('EDGEAI_CLOCK_RATE', '100')),
        OpaqueFunction(function=setup),
    ])
