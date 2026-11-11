#!/usr/bin/env python3
"""Standalone dual-robot C1 test: static truth map/target, estimated AMCL poses."""
import argparse
import copy
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import yaml

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'src/edgeai_simulation/launch'),str(ROOT/'src/edgeai_simulation/tools')]
from simulation_support import make_world
from generate_navigation_truth_map import generate


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory',type=Path,required=True)
    parser.add_argument('--params',type=Path,required=True)
    parser.add_argument('--duration',type=float,default=600.)
    parser.add_argument('--domain',type=int,default=86)
    parser.add_argument('--filter-check',action='store_true',help='Compare raw and filtered SLAM in the initial robot arrangement')
    args=parser.parse_args();directory=args.directory.resolve();directory.mkdir(parents=True,exist_ok=True)
    nav=yaml.safe_load(args.params.read_text())
    # Use current shared costmap layers when replaying an older controller snapshot.
    shared=yaml.safe_load((ROOT/'src/edgeai_navigation/config/nav2_exploration_rectangle.yaml').read_text())
    global_params=nav['global_costmap']['global_costmap']['ros__parameters']
    global_params['plugins']=shared['global_costmap']['global_costmap']['ros__parameters']['plugins']
    global_params['obstacle_layer']=shared['global_costmap']['global_costmap']['ros__parameters']['obstacle_layer']
    # Include the recent map/odom recovery-frame fix in older runtime snapshots.
    nav['behavior_server']['ros__parameters']['local_frame']=nav['local_costmap']['local_costmap']['ros__parameters']['global_frame']
    nav['behavior_server']['ros__parameters']['global_frame']=nav['global_costmap']['global_costmap']['ros__parameters']['global_frame']
    if not args.filter_check:
        navigation=ROOT/'src/edgeai_navigation'
        sys.path.insert(0,str(navigation/'scripts'))
        from fleet_config import configure_peer_costmaps
        coop=yaml.safe_load((navigation/'config/cooperative_rescue.yaml').read_text())
        configure_peer_costmaps(nav,'',coop.get('peer_costmap',{}))
    (directory/'nav2-runtime.yaml').write_text(yaml.safe_dump(nav,sort_keys=False))
    share=ROOT/'src/edgeai_simulation';targets=share/'config/targets_midterm.yaml'
    world=directory/'truth-world.sdf';make_world(share,targets,world,share/'worlds/midterm.sdf')
    generate(world,directory/'cooperative-map')
    data=yaml.safe_load(targets.read_text())
    (directory/'cooperative-targets.json').write_text(json.dumps(dict(frame_id="map",targets=[
        dict(id=t['id'],type=t['type'],x=t['pose'][0],y=t['pose'][1],yaw=t['pose'][5]) for t in data['targets']]),indent=2))
    robots=yaml.safe_load((share/'config/robots_midterm.yaml').read_text())
    # First robot starts beside S1, as after successful single rescue.
    if not args.filter_check:robots['robots'][0]['pose']=[1.,3.65,.01,0.,0.,1.5707963267948966]
    (directory/'robots.yaml').write_text(yaml.safe_dump(robots))
    (directory/'cooperative-test-initial.json').write_text(json.dumps([
        [r['pose'][0],r['pose'][1],r['pose'][5]] for r in robots['robots']]))
    rviz=yaml.safe_load((ROOT/'src/edgeai_exploration/rviz/exploration.rviz').read_text())
    displays=rviz['Visualization Manager']['Displays']
    for display in list(displays):
        if display.get('Class')=='rviz_default_plugins/RobotModel':
            second=copy.deepcopy(display);second['Name']='Robot2';second['Description Topic']['Value']='/robot2/robot_description';displays.append(second)
        if display.get('Class')=='rviz_default_plugins/Map' and display.get('Name')=='SLAM map (occupancy)':
            display['Name']='Truth map (world coordinates)'
    displays.append({'Class':'rviz_default_plugins/Path','Name':'Robot2 global path','Enabled':True,
                     'Topic':{'Value':'/robot2/plan','Depth':5,'History Policy':'Keep Last','Reliability Policy':'Reliable','Durability Policy':'Volatile'},
                     'Color':'255; 128; 0','Line Style':'Lines','Line Width':.03,'Value':True})
    (directory/'cooperative.rviz').write_text(yaml.safe_dump(rviz,sort_keys=False))
    os.environ.update(ROS_DOMAIN_ID=str(args.domain),GZ_PARTITION=f'edgeai_cooperative_{args.domain}_{os.getpid()}',
        EDGEAI_TRUTH_DIRECTORY=str(directory),PROJECT_ROOT=str(ROOT),
        CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>')
    record=dict(pid=os.getpid(),start_ticks=Path('/proc/self/stat').read_text().split()[21],
                directory=str(directory),domain=args.domain,partition=os.environ['GZ_PARTITION'],profile='cooperative-truth-test',params_source=str(args.params))
    registration=ROOT/f'runtime/exploration-{args.domain}.json';registration.write_text(json.dumps(record,indent=2))
    (directory/'session.json').write_text(json.dumps(record,indent=2));(ROOT/'runtime/latest-session.txt').write_text(str(directory)+'\n')
    processes=[];logs=[]
    def launch(name,command):
        log=(directory/(name+'.log')).open('w');logs.append(log)
        child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);processes.append(child);return child
    signal.signal(signal.SIGTERM,lambda *_:(_ for _ in ()).throw(KeyboardInterrupt()))
    result=1
    try:
        launch('sim',['ros2','launch','edgeai_simulation','simulation_real.launch.py','multi_robot:=true','robot_count:=2',
            'primary_unscoped:=true','gui:=true','rviz:=false','slam:='+str(args.filter_check).lower(),'navigation_test:=false',
            'robots_file:='+str(directory/'robots.yaml'),'world_file:='+str(share/'worlds/midterm.sdf'),'targets_file:='+str(targets),'truth:=true'])
        if args.filter_check:
            launch('nav',['ros2','launch','edgeai_navigation','managed_navigation.launch.py','params_file:='+str(directory/'nav2-runtime.yaml')])
            raw_params=yaml.safe_load((share/'config/slam_toolbox.yaml').read_text())['slam_toolbox']['ros__parameters']
            raw_params.update(use_sim_time=True,map_frame='map',odom_frame='odom',base_frame='base_footprint',
                              scan_topic='/scan',transform_publish_period=0.)
            path=directory/'raw-slam.yaml';path.write_text(yaml.safe_dump({'raw_slam':{'ros__parameters':raw_params}}))
            launch('raw-slam',['ros2','run','slam_toolbox','async_slam_toolbox_node','--ros-args','-r','__node:=raw_slam',
                '-r','/map:=/map_raw','-r','/map_metadata:=/map_raw_metadata','--params-file',str(path)])
            launch('raw-manager',['ros2','run','nav2_lifecycle_manager','lifecycle_manager','--ros-args','-r','__node:=raw_slam_manager',
                '-p','use_sim_time:=true','-p','autostart:=true','-p','node_names:=[raw_slam]','-p','bond_timeout:=0.0'])
            launch('rviz',['rviz2','-d',str(directory/'cooperative.rviz'),'--ros-args','-p','use_sim_time:=true'])
            verifier=launch('filter-check',['python3','-u',str(ROOT/'tests/simulation/check_fleet_filter_live.py'),
                '--directory',str(directory),'--ros-args','-p','use_sim_time:=true'])
            # Allow actual GUI initialization before asking the Windows observer.
            time.sleep(8.);(directory/'ready').touch()
            result=verifier.wait(timeout=args.duration)
            print((directory/'filter-result.json').read_text(),flush=True)
            return result
        launch('nav',['ros2','launch','edgeai_navigation','managed_navigation.launch.py','params_file:='+str(directory/'nav2-runtime.yaml')])
        launch('authority',['python3','-u',str(ROOT/'src/edgeai_navigation/scripts/navigation_authority.py'),'--mode','MANUAL',
            '--directory',str(directory),'--tree',nav['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'],
            '--ros-args','-p','use_sim_time:=true','-p','allow_auto:=false'])
        launch('rviz',['rviz2','-d',str(directory/'cooperative.rviz'),'--ros-args','-p','use_sim_time:=true'])
        # Exercise the same task Action and behavior tree as the default mission.
        from ament_index_python.packages import get_package_prefix
        import rclpy
        from rclpy.action import ActionClient
        from edgeai_mission.action import ExecuteTask
        task_tree=directory/'c1-task.xml'
        task_tree.write_text((ROOT/'src/edgeai_mission/config/tasks/c1.xml').read_text())
        settings=dict(directory=str(directory),project_root=str(ROOT),cooperative_static_test=True,
            wall_timeout_s=args.duration,task_trees=dict(c1=str(task_tree)),
            unit_worker_script=str(ROOT/'src/edgeai_mission/scripts/task_units.py'))
        config=directory/'mission-runtime.json';config.write_text(json.dumps(settings,indent=2))
        binary=Path(get_package_prefix('edgeai_mission'))/'lib/edgeai_mission/task_executor'
        launch('c1-task',[str(binary),'--ros-args','-p','config:='+str(config),'-p','task_name:=c1'])
        time.sleep(8.);(directory/'ready').touch();print('COOPERATIVE TRUTH TEST '+str(directory),flush=True)
        rclpy.init();peer=rclpy.create_node('cooperative_test_client')
        try:
            client=ActionClient(peer,ExecuteTask,'/mission/tasks/c1/execute')
            if not client.wait_for_server(timeout_sec=10.):raise RuntimeError('C1 task server unavailable')
            end=time.monotonic()+args.duration
            def wait(future):
                while not future.done() and time.monotonic()<end:rclpy.spin_once(peer,timeout_sec=.05)
                if not future.done():raise subprocess.TimeoutExpired('C1 task Action',args.duration)
                return future.result()
            handle=wait(client.send_goal_async(ExecuteTask.Goal(task_id=directory.name+':c1')))
            if not handle.accepted:raise RuntimeError('C1 task rejected')
            terminal=wait(handle.get_result_async())
            result=0 if terminal.status==4 and terminal.result.success else 1
            (directory/'task-action-result.json').write_text(json.dumps(dict(success=result==0,reason=terminal.result.reason,result_json=terminal.result.result_json)))
            print((directory/'task-action-result.json').read_text(),flush=True)
        finally:peer.destroy_node();rclpy.shutdown()
    except subprocess.TimeoutExpired:
        print('Test wall timeout',flush=True)
    except KeyboardInterrupt:
        print('Test interrupted',flush=True)
    finally:
        (directory/'final-capture').touch()
        capture_end=time.monotonic()+(12. if os.environ.get('EDGEAI_CAPTURE_ACK')=='true' else 0.)
        while not (directory/'capture-finished').exists() and time.monotonic()<capture_end:time.sleep(.2)
        for child in reversed(processes):
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=12.)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid,signal.SIGTERM)
                    try:child.wait(timeout=5.)
                    except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
        for log in logs:log.close()
        registration.unlink(missing_ok=True);(directory/'exit-code.txt').write_text(str(result))
    return result


if __name__=='__main__':raise SystemExit(main())
