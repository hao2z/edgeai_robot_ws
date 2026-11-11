#!/usr/bin/env python3
"""Standalone dual-robot C1 test: static truth map/target, estimated AMCL poses."""
import argparse
import json
import os
from pathlib import Path
import signal
import shutil
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
    parser.add_argument('--midterm-part2',action='store_true',help='Both robots start in the start area; coordinator executes S1 then C1')
    parser.add_argument('--prepared-map',type=Path,help='Prebuilt map in arena/world coordinates, used only in Part2')
    parser.add_argument('--verification-seconds',type=float,default=0.,help='Keep the stopped Part2 scene open after success')
    args=parser.parse_args();directory=args.directory.resolve();directory.mkdir(parents=True,exist_ok=True)
    if args.midterm_part2 and args.filter_check:parser.error('Part2 cannot run the filter diagnostic')
    if args.duration<=0 or args.verification_seconds<0:parser.error('Invalid time budget')
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
    if args.prepared_map:
        metadata=yaml.safe_load(args.prepared_map.read_text())
        image=args.prepared_map.parent/metadata['image']
        destination=directory/('cooperative-map'+image.suffix)
        shutil.copyfile(image,destination);metadata['image']=destination.name
        (directory/'cooperative-map.yaml').write_text(yaml.safe_dump(metadata))
    else:generate(world,directory/'cooperative-map')
    data=yaml.safe_load(targets.read_text())
    (directory/'cooperative-targets.json').write_text(json.dumps(dict(frame_id="map",targets=[
        dict(id=t['id'],type=t['type'],x=t['pose'][0],y=t['pose'][1],yaw=t['pose'][5],
             confirmed=True,spread_m=0.,observations=1,source='specified arena coordinates')
        for t in data['targets']]),indent=2))
    if args.midterm_part2:
        shutil.copyfile(directory/'cooperative-targets.json',directory/'targets.json')
    robots=yaml.safe_load((share/'config/robots_midterm.yaml').read_text())
    # First robot starts beside S1, as after successful single rescue.
    if not args.filter_check and not args.midterm_part2:robots['robots'][0]['pose']=[1.,3.65,.01,0.,0.,1.5707963267948966]
    (directory/'robots.yaml').write_text(yaml.safe_dump(robots))
    (directory/'cooperative-test-initial.json').write_text(json.dumps([
        [r['pose'][0],r['pose'][1],r['pose'][5]] for r in robots['robots']]))
    rviz_env=dict(os.environ,EDGEAI_COOPERATIVE_RESCUE='true')
    controller='rpp' if 'RegulatedPurePursuitController' in json.dumps(nav) else 'dwb'
    subprocess.run([sys.executable,str(ROOT/'scripts/ros/prepare_session_rviz.py'),
        str(ROOT/'src/edgeai_exploration/rviz/exploration.rviz'),str(directory/'cooperative.rviz'),
        str(not args.filter_check).lower(),controller,str(directory/'nav2-runtime.yaml')],env=rviz_env,check=True)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain),GZ_PARTITION=f'edgeai_cooperative_{args.domain}_{os.getpid()}',
        EDGEAI_TRUTH_DIRECTORY=str(directory),PROJECT_ROOT=str(ROOT),
        CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>')
    record=dict(pid=os.getpid(),start_ticks=Path('/proc/self/stat').read_text().split()[21],
                directory=str(directory),domain=args.domain,partition=os.environ['GZ_PARTITION'],profile='cooperative-truth-test',params_source=str(args.params))
    registration=ROOT/f'runtime/exploration-{args.domain}.json';registration.write_text(json.dumps(record,indent=2))
    (directory/'session.json').write_text(json.dumps(record,indent=2));(ROOT/'runtime/latest-session.txt').write_text(str(directory)+'\n')
    processes=[];logs=[];stream_process=None
    def launch(name,command,*,console=False):
        log=(directory/(name+'.log')).open('a');logs.append(log)
        if console:
            command=['python3','-u',str(ROOT/'scripts/ros/stream_session_logs.py'),str(directory)]
            child=subprocess.Popen(command,stderr=log,start_new_session=True)
            processes.append(child);return child
        child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);processes.append(child);return child
    signal.signal(signal.SIGTERM,lambda *_:(_ for _ in ()).throw(KeyboardInterrupt()))
    result=1;bootstrap=None;ros_started=False;startup_phase='launch'
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
        # Keep the same live log reducer, diagnostics and ROS monitor as the
        # normal mission entry. The test-specific files remain raw evidence.
        for name in ('rosout','alignment','cooperative','cooperative-nav','mission','target-search','rescue','nav','authority','rviz'):
            (directory/(name+'.log')).touch(exist_ok=True)
        launch('rosout',['python3','-u',str(ROOT/'scripts/ros/monitor_ros.py')])
        launch('alignment',['python3','-u',str(ROOT/'scripts/ros/navigation_diagnostics.py'),
            '--ros-args','-p','use_sim_time:=true','-p','static_map:=true',
            '-p','cooperative:=true','-p','log_directory:='+str(directory)])
        launch('rviz',['rviz2','-d',str(directory/'cooperative.rviz'),'--ros-args','-p','use_sim_time:=true'])
        stream_process=launch('stream-errors',[],console=True)
        if args.midterm_part2:
            # Infrastructure only: all rescue scheduling/checks remain in the
            # coordinator and existing task trees. No simulator pose localization.
            import rclpy
            from ament_index_python.packages import get_package_prefix
            from edgeai_tasks.cooperative_rescue import CooperativeRescue
            sys.path.insert(0,str(ROOT/'src/edgeai_mission/scripts'))
            from prepare_mission import prepare
            settings=prepare(ROOT,directory,ROOT/'src/edgeai_navigation/config/midterm_single_rescue.yaml',
                directory/'map',args.duration,True,False,'frontier')
            settings.update(tree=str(ROOT/'src/edgeai_mission/config/midterm_part2.xml'),
                initial_pose_seed=False,cooperative_static_test=True,external_navigation=True,
                wall_timeout_s=args.duration,demonstration_part='part2')
            (directory/'mission-runtime.json').write_text(json.dumps(settings,indent=2))
            rclpy.init(args=['--ros-args','-p','use_sim_time:=true']);ros_started=True
            coop=dict(coop,wall_timeout_s=min(coop['wall_timeout_s'],args.duration))
            bootstrap=CooperativeRescue(directory,coop,node_name='midterm_navigation_support')
            bootstrap.static_test=True;bootstrap.seed_enabled=False
            startup_phase='AMCL activation and coarse initial poses'
            with (directory/'cooperative.log').open('a') as bootstrap_log:
                # ROS logs use stderr; forward bootstrap events through the same
                # reducer as task-local cooperative events without duplicate output.
                previous_stderr=os.dup(2)
                try:
                    os.dup2(bootstrap_log.fileno(),2)
                    bootstrap.handover()
                    startup_phase='navigation lifecycle readiness'
                    bootstrap.wait_navigation_active()
                    startup_phase='dual AMCL convergence'
                    bootstrap.localize()
                finally:
                    os.dup2(previous_stderr,2);os.close(previous_stderr)
            (directory/'ready').touch()
            # Reuse coordinator process supervision without changing task logic.
            owned={str(i):dict(pid=child.pid,start_ticks=Path(f'/proc/{child.pid}/stat').read_text().split()[21])
                   for i,child in enumerate(processes+[bootstrap.process])}
            (directory/'mission-processes.json').write_text(json.dumps(owned))
            binary=Path(get_package_prefix('edgeai_mission'))/'lib/edgeai_mission/mission_coordinator'
            coordinator=launch('mission',[str(binary),'--ros-args','-p','use_sim_time:=true',
                '-p','config:='+str(directory/'mission-runtime.json')])
            startup_phase='S1/C1 task trees'
            end=time.monotonic()+args.duration
            print('PART2_READY: 两车起始区定位完成；任务树开始 S1 → C1',flush=True)
            while coordinator.poll() is None:
                if time.monotonic()>end:raise subprocess.TimeoutExpired('Part2 mission',args.duration)
                if bootstrap.process.poll() is not None:raise RuntimeError('Part2 localization/navigation exited')
                rclpy.spin_once(bootstrap,timeout_sec=.05)
            saved=directory/'mission-result.json'
            result=0 if coordinator.returncode==0 and saved.exists() and json.loads(saved.read_text()).get('success') else 1
            print('PART2_RESULT: '+(saved.read_text() if saved.exists() else 'No confirmed result'),flush=True)
            if result==0 and args.verification_seconds>0:
                print(f'核验阶段：保持停车画面 {args.verification_seconds:.0f} 秒；结果位于 {directory}',flush=True)
                until=time.monotonic()+args.verification_seconds
                while time.monotonic()<until:rclpy.spin_once(bootstrap,timeout_sec=.1)
            return result
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
    except RuntimeError as error:
        print(f'Test failed: {error}',flush=True)
        failure=dict(success=False,phase=startup_phase,reason=str(error))
        if bootstrap is not None:failure['localization']=bootstrap.localization_diagnostics()
        (directory/'startup-failure.json').write_text(json.dumps(failure,indent=2))
    finally:
        # Capture while both robots, shared map and diagnostic displays are alive.
        (directory/'final-capture').touch()
        capture_end=time.monotonic()+(12. if os.environ.get('EDGEAI_CAPTURE_ACK')=='true' else 0.)
        while not (directory/'capture-finished').exists() and time.monotonic()<capture_end:time.sleep(.2)
        (directory/'stopping').touch()
        if bootstrap is not None:
            # A localization cleanup error must not leave the simulator alive.
            try:bootstrap.close()
            except Exception as error:
                print(f'Navigation cleanup: {error}',flush=True)
                if bootstrap.process is not None:processes.append(bootstrap.process)
            bootstrap.destroy_node()
        if ros_started:
            import rclpy
            if rclpy.ok():rclpy.shutdown()
        for child in reversed(processes):
            if child is stream_process:continue
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=12.)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid,signal.SIGTERM)
                    try:child.wait(timeout=5.)
                    except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
        if stream_process is not None:
            (directory/'stream.stop').touch()
            try:stream_process.wait(timeout=5.)
            except subprocess.TimeoutExpired:stream_process.terminate()
        for log in logs:log.close()
        registration.unlink(missing_ok=True);(directory/'exit-code.txt').write_text(str(result))
    return result


if __name__=='__main__':raise SystemExit(main())
