"""Static-map localization for both robots and managed navigation for robot2."""
from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def setup(context):
    directory=Path(LaunchConfiguration('directory').perform(context))
    reuse_robot2=LaunchConfiguration('reuse_robot2_navigation',default='false').perform(context).lower()=='true'
    autostart=LaunchConfiguration('localization_autostart',default='true').perform(context).lower()=='true'
    robot2=LaunchConfiguration('enable_robot2',default='true').perform(context).lower()=='true'
    share=Path(get_package_share_directory('edgeai_navigation'))
    filter_config=yaml.safe_load((share/'config/fleet_scan_filter.yaml').read_text())
    localization_filter=dict(filter_config.get('localization',{}))
    filter_enabled=localization_filter.pop('enabled',True)
    actions=[Node(package='nav2_map_server',executable='map_server',name='map_server',
                  parameters=[{'use_sim_time':True,'yaml_filename':str(directory/'cooperative-map.yaml')}])]
    for ns in (('robot1','robot2') if robot2 else ('robot1',)):
        folder=directory/'robot2' if ns=='robot2' else directory
        config=yaml.safe_load((folder/'nav2-runtime.yaml').read_text())
        params=dict(config['amcl']['ros__parameters'])
        params.update(use_sim_time=True,map_topic='/map',set_initial_pose=False,
                      update_min_d=.05,update_min_a=.05,transform_tolerance=.2,save_pose_rate=-1.)
        if filter_enabled and robot2:
            other='robot2/' if ns=='robot1' else 'robot1/'
            filtering=dict(localization_filter,use_sim_time=True,bounds=filter_config['bounds'],
                own_odom_frame=params['odom_frame_id'],other_odom_frame=other+'odom',
                other_base_frame=other+'base_footprint',other_pose_topic='/'+other+'amcl_pose')
            filter_file=folder/'localization-scan-filter.yaml'
            filter_file.write_text(yaml.safe_dump({'/**':{'ros__parameters':filtering}}))
            actions.append(ExecuteProcess(cmd=['python3','-u',str(share/'scripts/fleet_scan_filter.py'),
                '--localization','--ros-args','--params-file',str(filter_file),
                '-r','__ns:=/'+ns],output='screen'))
            params['scan_topic']='scan_localization'
        # Persist the effective AMCL settings separately from generic Nav2/obstacle input.
        (folder/'amcl-localization.yaml').write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}))
        actions.append(Node(package='nav2_amcl',executable='amcl',namespace=ns,name='amcl',parameters=[params]))
    # Nav2 bonds use relative "bond" topics and the node's basename as ID.
    # Each namespace therefore needs its own manager; a root manager cannot
    # monitor robot2/amcl as if it published ID "robot2/amcl" on /bond.
    # The mission still owns the complete launch and starts map/root first.
    actions.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='shared_localization_manager',
        parameters=[{'use_sim_time':True,'autostart':autostart,'node_names':['map_server']}]))
    for ns in (('robot1','robot2') if robot2 else ('robot1',)):
        actions.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',
            namespace=ns,name='localization_manager',
            parameters=[{'use_sim_time':True,'autostart':autostart,'node_names':['amcl']}]))
    if robot2 and not reuse_robot2:
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/managed_navigation.launch.py')),
            launch_arguments={'namespace':'robot2','params_file':str(directory/'robot2/nav2-runtime.yaml')}.items()))
        # The authority inherits the same namespace; all of its ROS endpoints are relative.
        actions.append(ExecuteProcess(cmd=['python3','-u',str(share/'scripts/navigation_authority.py'),
            '--directory',str(directory/'robot2'),'--mode','MANUAL','--tree',
            config['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'],
            '--ros-args','-r','__ns:=/robot2','-p','use_sim_time:=true'],output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('directory'),
        DeclareLaunchArgument('reuse_robot2_navigation',default_value='false'),
        DeclareLaunchArgument('enable_robot2',default_value='true'),
        DeclareLaunchArgument('localization_autostart',default_value='true'),OpaqueFunction(function=setup)])
