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
    share=Path(get_package_share_directory('edgeai_navigation'))
    actions=[Node(package='nav2_map_server',executable='map_server',name='map_server',
                  parameters=[{'use_sim_time':True,'yaml_filename':str(directory/'cooperative-map.yaml')}]),
             Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='shared_map_manager',
                  parameters=[{'use_sim_time':True,'autostart':True,'node_names':['map_server']}])]
    for ns in ('','robot2'):
        folder=directory/'robot2' if ns else directory
        config=yaml.safe_load((folder/'nav2-runtime.yaml').read_text())
        params=dict(config['amcl']['ros__parameters'])
        params.update(use_sim_time=True,map_topic='/map',set_initial_pose=False,
                      update_min_d=.05,update_min_a=.05,transform_tolerance=.2,save_pose_rate=-1.)
        actions.extend([
            Node(package='nav2_amcl',executable='amcl',namespace=ns,name='amcl',parameters=[params]),
            Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',namespace=ns,
                 name='localization_manager',parameters=[{'use_sim_time':True,'autostart':True,'node_names':['amcl']}])])
    actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/managed_navigation.launch.py')),
        launch_arguments={'namespace':'robot2','params_file':str(directory/'robot2/nav2-runtime.yaml')}.items()))
    # The authority inherits the same namespace; all of its ROS endpoints are relative.
    actions.append(ExecuteProcess(cmd=['python3','-u',str(share/'scripts/navigation_authority.py'),
        '--directory',str(directory/'robot2'),'--mode','MANUAL','--tree',
        config['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'],
        '--ros-args','-r','__ns:=/robot2','-p','use_sim_time:=true','-p','allow_auto:=false'],output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('directory'),OpaqueFunction(function=setup)])
