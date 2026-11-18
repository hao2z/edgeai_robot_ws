"""Public SLAM only; robot navigation/exploration are composed by the mission."""
import json
import math
import os
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    directory=Path(LaunchConfiguration('directory').perform(context))
    directory.mkdir(parents=True,exist_ok=True)
    share=Path(get_package_share_directory('edgeai_mapping'))
    config=yaml.safe_load((share/'config/joint_mapping.yaml').read_text())
    bounds=json.loads(LaunchConfiguration('body_bounds').perform(context))
    if len(bounds)!=4:
        raise ValueError('Expected xmin,xmax,ymin,ymax')
    prior = LaunchConfiguration('initial_relative_pose', default='').perform(context)
    if prior:
        pose = json.loads(prior)
        if not isinstance(pose, list) or len(pose) != 3 or not all(
                isinstance(v, (int, float)) and math.isfinite(v) for v in pose):
            raise ValueError('Expected finite initial_relative_pose [x, y, yaw]')
        config['initialization']['initial_relative_pose'] = [float(v) for v in pose]
    (directory/'joint-mapping-config.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
    slam=yaml.safe_load((share/'config/slam_toolbox.yaml').read_text())['slam_toolbox']['ros__parameters']
    slam.update(config.get('slam',{}))
    slam.update(use_sim_time=True,map_frame='map',odom_frame='robot1/odom',base_frame='robot1/base_footprint',
                scan_topic='/joint_mapping/unused_scan',transform_publish_period=0.,enable_interactive_mode=False)
    for robot in ('robot1','robot2'):
        slam.update({robot+'_base_frame':robot+'/base_footprint',robot+'_sensor_frame':robot+'/lidar_link',
                     robot+'_odom_frame':robot+'/odom',robot+'_scan_topic':'/'+robot+'/scan_slam'})
    initializer=dict(config['initialization'],use_sim_time=True,output_directory=str(directory))
    runtime=dict(config['runtime'],use_sim_time=True,directory=str(directory),bounds=bounds)
    for name,params in [('initializer',initializer),('runtime',runtime)]:
        (directory/('joint-'+name+'.yaml')).write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}))
    diagnostics=[]
    if os.environ.get('EDGEAI_RUNTIME_PROFILE','debug') != 'performance':
        diagnostics=[
        ExecuteProcess(cmd=['python3','-u',str(share/'scripts/mapping_diagnostics.py'),
            '--ros-args','-p','use_sim_time:=true','-p','directory:='+str(directory),
            *sum((['-p',k+':='+str(v)] for k,v in config.get('diagnostics',{}).items()),[])],output='screen')]
    return diagnostics+[
        Node(package='edgeai_mapping',executable='joint_slam_toolbox',name='slam_toolbox',parameters=[slam],output='screen'),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='joint_slam_manager',
             parameters=[dict(use_sim_time=True,autostart=True,node_names=['slam_toolbox'],bond_timeout=0.)]),
        *[ExecuteProcess(cmd=['python3','-u',str(share/('scripts/joint_mapping_'+name+'.py')),
            '--ros-args','--params-file',str(directory/('joint-'+name+'.yaml'))],output='screen')
          for name in ('initializer','runtime')]]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('directory'),
        DeclareLaunchArgument('initial_relative_pose', default_value=''),
        DeclareLaunchArgument('body_bounds',default_value='[-0.3125,0.1125,-0.165,0.165]'),
        OpaqueFunction(function=setup)])
