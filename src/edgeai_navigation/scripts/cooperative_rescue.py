#!/usr/bin/env python3
"""S1 handover, laser localization and concurrent C1 arrival with joint dwell."""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger, Empty
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import State, Transition
from nav2_msgs.action import NavigateToPose, ComputePathToPose, Spin
from tf2_ros import Buffer, TransformListener, TransformException, TransformBroadcaster
import yaml

from cooperative_policy import compose, relative, cell, parking_pairs, JointDwell
from fleet_config import namespaced_config


def pose_value(p):
    q=p.orientation
    return [p.position.x,p.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]


class CooperativeRescue(Node):
    def __init__(self,directory,config):
        super().__init__('cooperative_rescue')
        self.directory,self.c=Path(directory),config
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.seed_broadcaster=TransformBroadcaster(self);self.seed_enabled=True
        self.mapping_start_pose=None
        self.odom={};self.initial_odom={};self.scan={};self.localization={};self.states={}
        self.scan_history={'':deque(maxlen=10),'robot2':deque(maxlen=10)};self.scan_matches={}
        self.map=None;self.process=None;self.log=None;self.handles=[];self.sequences={}
        self.state_received={};self.nomotion={};self.closing=False
        self.deadline=float('inf');self.initialized=False;self.last_report=0.
        self.measurement_after=0.
        self.state_pub=self.create_publisher(String,'/cooperative_rescue/state',
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.initial_pubs={};self.nav={};self.plan={};self.spin={};self.manual={};self.stop={}
        for ns in ('','robot2'):
            topic=lambda name:'/'+(ns+'/' if ns else '')+name
            self.create_subscription(Odometry,topic('odom'),lambda m,n=ns:self.receive_odom(n,m),qos_profile_sensor_data)
            self.create_subscription(LaserScan,topic('scan'),lambda m,n=ns:self.receive_scan(n,m),qos_profile_sensor_data)
            self.create_subscription(PoseWithCovarianceStamped,topic('amcl_pose'),
                lambda m,n=ns:self.localization.update({n:m}),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_subscription(String,topic('navigation_authority/state'),
                lambda m,n=ns:self.receive_state(n,m),
                QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.initial_pubs[ns]=self.create_publisher(PoseWithCovarianceStamped,topic('initialpose'),1)
            self.nav[ns]=ActionClient(self,NavigateToPose,topic('navigation/rescue_to_pose'))
            self.plan[ns]=ActionClient(self,ComputePathToPose,topic('navigation_backend/compute_path_to_pose'))
            self.spin[ns]=ActionClient(self,Spin,topic('navigation_backend/spin'))
            self.manual[ns]=self.create_client(SetBool,topic('navigation_authority/set_manual'))
            self.stop[ns]=self.create_client(Trigger,topic('navigation_authority/stop'))
            self.nomotion[ns]=self.create_client(Empty,topic('request_nomotion_update'))
        self.create_subscription(OccupancyGrid,'/map',lambda m:setattr(self,'map',m),
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(.25,self.update_at_rest)
        self.create_timer(.2,self.publish_initial_estimate)

    def publish_initial_estimate(self):
        if not self.seed_enabled or len(self.initial_odom)!=2:return
        if self.mapping_start_pose is None:
            try:self.mapping_start_pose=compose(self.tf_pose('odom'),self.initial_odom[''])
            except TransformException:return
            self.event('INITIAL_REFERENCE',robot1_start_map=self.mapping_start_pose,
                       relative_prior=self.c['initial_relative_pose'])
        root_start=self.mapping_start_pose
        map_odom2=compose(compose(root_start,self.c['initial_relative_pose']),relative(self.initial_odom['robot2'],[0.,0.,0.]))
        msg=TransformStamped();msg.header.frame_id='map';msg.child_frame_id='robot2/odom'
        msg.header.stamp=self.get_clock().now().to_msg()
        msg.transform.translation.x,msg.transform.translation.y=map_odom2[:2]
        msg.transform.rotation.z=math.sin(map_odom2[2]/2);msg.transform.rotation.w=math.cos(map_odom2[2]/2)
        self.seed_broadcaster.sendTransform(msg)

    def update_at_rest(self):
        if self.process is None or self.closing:return
        for client in self.nomotion.values():
            if client.service_is_ready():client.call_async(Empty.Request())

    def receive_state(self,ns,msg):
        self.states[ns]=json.loads(msg.data);self.state_received[ns]=time.monotonic()

    def receive_odom(self,ns,msg):
        self.odom[ns]=msg;self.initial_odom.setdefault(ns,pose_value(msg.pose.pose))

    def receive_scan(self,ns,msg):
        self.scan[ns]=msg;self.scan_history[ns].append(msg)

    def event(self,phase,**data):
        record=dict(phase=phase,sim_time=self.now(),**data)
        self.state_pub.publish(String(data=json.dumps(record,ensure_ascii=False)))
        with (self.directory/'cooperative-events.jsonl').open('a',encoding='utf-8') as f:
            f.write(json.dumps(record,ensure_ascii=False)+'\n')
        self.get_logger().info('COOPERATIVE_'+phase+': '+json.dumps(data,ensure_ascii=False))

    def now(self):return self.get_clock().now().nanoseconds/1e9

    def check_owner(self):
        for ns,seq in self.sequences.items():
            state=self.states.get(ns,{})
            age=time.monotonic()-self.state_received.get(ns,0)
            if age>2.:
                raise RuntimeError(f'{ns or "robot1"}: authority state stale ({age:.2f}s)')
            if state.get('command_sequence')!=seq or state.get('mode')!='MANUAL':
                raise RuntimeError(f'{ns or "robot1"}: navigation ownership changed '
                    f'(expected sequence={seq}, observed={state.get("command_sequence")}, mode={state.get("mode")})')

    def acquire_manual(self,ns):
        # A service may become ready before its first transient state arrives.
        # Wait for that baseline; never accept an old sequence after the request.
        self.wait(lambda:self.manual[ns].service_is_ready() and ns in self.states and
                  time.monotonic()-self.state_received.get(ns,0)<2.,40.)
        expected=self.states[ns]['command_sequence']+1
        self.service(self.manual[ns],SetBool.Request(data=True))
        def acquired():
            state=self.states.get(ns,{})
            if state.get('command_sequence',-1)>expected:
                raise RuntimeError(f'{ns or "robot1"}: navigation ownership changed during handover')
            return (state.get('command_sequence')==expected and state.get('mode')=='MANUAL' and
                    state.get('phase')=='IDLE' and time.monotonic()-self.state_received.get(ns,0)<2.)
        self.wait(acquired,15.)
        self.sequences[ns]=expected
        self.event('AUTHORITY_ACQUIRED',robot=ns or 'robot1',sequence=expected)

    def wait(self,predicate,timeout=15.,ownership=False):
        end=min(self.deadline,time.monotonic()+timeout)
        while rclpy.ok() and time.monotonic()<end:
            rclpy.spin_once(self,timeout_sec=.05)
            if ownership:self.check_owner()
            if not self.closing and self.process is not None and self.process.poll() is not None:
                raise RuntimeError('Cooperative navigation launch exited')
            if predicate():return
        raise RuntimeError('Cooperative phase timeout')

    def service(self,client,request,timeout=15.):
        self.wait(client.service_is_ready,timeout)
        future=client.call_async(request);self.wait(future.done,timeout)
        response=future.result()
        if hasattr(response,'success') and not response.success:
            transition=getattr(getattr(request,'transition',None),'id',None)
            raise RuntimeError(f'Service failed: {client.srv_name}; transition={transition}; '+getattr(response,'message',''))
        return response

    def stop_slam(self,name='/slam_toolbox'):
        state_client=self.create_client(GetState,name+'/get_state')
        change_client=self.create_client(ChangeState,name+'/change_state')
        transitions={State.PRIMARY_STATE_ACTIVE:(Transition.TRANSITION_DEACTIVATE,State.PRIMARY_STATE_INACTIVE),
            State.PRIMARY_STATE_INACTIVE:(Transition.TRANSITION_CLEANUP,State.PRIMARY_STATE_UNCONFIGURED),
            State.PRIMARY_STATE_UNCONFIGURED:(Transition.TRANSITION_UNCONFIGURED_SHUTDOWN,State.PRIMARY_STATE_FINALIZED)}
        try:
            for _ in range(4):
                state=self.service(state_client,GetState.Request()).current_state
                if state.id==State.PRIMARY_STATE_FINALIZED:
                    self.event('SLAM_STOPPED',node=name,state=state.label)
                    return
                if state.id not in transitions:
                    raise RuntimeError(f'{name}: unexpected lifecycle state {state.id} ({state.label})')
                transition,expected=transitions[state.id]
                self.event('SLAM_TRANSITION',node=name,from_state=state.label,transition=transition)
                request=ChangeState.Request();request.transition.id=transition
                self.service(change_client,request)
                after=self.service(state_client,GetState.Request()).current_state
                if after.id!=expected:
                    raise RuntimeError(f'{name}: transition {transition} ended in {after.label}, expected state {expected}')
            raise RuntimeError(f'{name}: lifecycle shutdown did not finish')
        finally:
            self.destroy_client(state_client);self.destroy_client(change_client)

    def tf_pose(self,frame,stamp=None):
        tf=self.buffer.lookup_transform('map',frame,stamp or Time())
        return pose_value(type('Pose',(),dict(position=tf.transform.translation,orientation=tf.transform.rotation))())

    def laser_match(self,ns):
        if self.map is None:return 0.
        scan=None
        # Scan and TF arrive independently. Select a fresh transformable sample,
        # rather than calling an otherwise healthy localization invalid each tick.
        for candidate in reversed(self.scan_history[ns]):
            stamp=Time.from_msg(candidate.header.stamp)
            if not 0<=self.now()-stamp.nanoseconds/1e9<=self.c['max_data_age_s']:continue
            key=(stamp.nanoseconds,id(self.map))
            cached=self.scan_matches.get(ns)
            if cached is not None and cached[0]==key:return cached[1]
            try:p=self.tf_pose(candidate.header.frame_id,stamp)
            except TransformException:continue
            scan=candidate;break
        if scan is None:return 0.
        matched=total=0;step=max(1,len(scan.ranges)//60)
        resolution=self.map.info.resolution
        reach=math.ceil(self.c['scan_match_distance_m']/resolution)
        offsets=[(i*resolution,j*resolution) for i in range(-reach,reach+1) for j in range(-reach,reach+1)
                 if math.hypot(i*resolution,j*resolution)<=self.c['scan_match_distance_m']]
        for i in range(0,len(scan.ranges),step):
            distance=scan.ranges[i]
            if not math.isfinite(distance) or not scan.range_min<distance<scan.range_max:continue
            angle=p[2]+scan.angle_min+i*scan.angle_increment
            x,y=p[0]+distance*math.cos(angle),p[1]+distance*math.sin(angle)
            total+=1
            matched+=any(cell(self.map,x+dx,y+dy)>=65 for dx,dy in offsets)
        ratio=matched/total if total>=15 else 0.
        self.scan_matches[ns]=(key,ratio)
        return ratio

    def localized(self,ns):
        sample=self.localization.get(ns)
        if sample is None:return False
        stamp=Time.from_msg(sample.header.stamp).nanoseconds/1e9
        cov=sample.pose.covariance
        if not (0<=self.now()-stamp<=self.c['max_data_age_s'] and
                all(0<=cov[i]<=self.c['max_std_xy_m']**2 for i in (0,7)) and
                0<=cov[35]<=self.c['max_std_yaw_rad']**2):return False
        return self.laser_match(ns)>=self.c['min_scan_match_ratio']

    def action(self,client,request,timeout):
        self.wait(client.server_is_ready,15.,True)
        sent=client.send_goal_async(request);self.wait(sent.done,10.,True)
        handle=sent.result()
        if not handle.accepted:raise RuntimeError('Cooperative action rejected')
        self.handles.append(handle)
        result=handle.get_result_async()
        try:self.wait(result.done,timeout,True)
        except BaseException:
            canceled=handle.cancel_goal_async()
            old_deadline=self.deadline;self.deadline=max(self.deadline,time.monotonic()+8.)
            try:self.wait(lambda:canceled.done() and result.done(),8.)
            finally:self.deadline=old_deadline
            raise
        self.handles.remove(handle)
        return result.result()

    def goal_pose(self,pose):
        msg=PoseStamped();msg.header.frame_id='map';msg.header.stamp=self.get_clock().now().to_msg()
        msg.pose.position.x,msg.pose.position.y=map(float,pose[:2])
        msg.pose.orientation.z=math.sin(pose[2]/2);msg.pose.orientation.w=math.cos(pose[2]/2)
        return msg

    def navigate(self,ns,pose):
        for attempt in range(self.c['navigation_retries']+1):
            self.wait(lambda:self.states.get(ns,{}).get('phase')=='IDLE',10.,True)
            self.event('NAVIGATING',robot=ns or 'robot1',goal=pose,attempt=attempt+1)
            terminal=self.action(self.nav[ns],NavigateToPose.Goal(pose=self.goal_pose(pose)),self.c['navigation_timeout_s'])
            if terminal.status==4 and terminal.result.error_code==0:
                self.measurement_after=self.now()
                return
            if terminal.status!=6:raise RuntimeError('Cooperative navigation canceled')
        raise RuntimeError(f'{ns or "robot1"}: navigation retries exhausted')

    def navigate_pair(self,pair):
        """Dispatch both goals before waiting; retry each robot independently."""
        pending={};attempts={ns:0 for ns in ('','robot2')}
        def dispatch(ns,pose):
            self.wait(lambda:self.states.get(ns,{}).get('phase')=='IDLE' and self.nav[ns].server_is_ready(),15.,True)
            attempts[ns]+=1
            self.event('NAVIGATING',robot=ns or 'robot1',goal=pose,attempt=attempts[ns],concurrent=True)
            sent=self.nav[ns].send_goal_async(NavigateToPose.Goal(pose=self.goal_pose(pose)))
            self.wait(sent.done,10.,True)
            handle=sent.result()
            if not handle.accepted:raise RuntimeError(f'{ns or "robot1"}: cooperative goal rejected')
            self.handles.append(handle)
            pending[ns]=(pose,handle,handle.get_result_async(),time.monotonic()+self.c['navigation_timeout_s'])
        for ns,pose in zip(('','robot2'),pair):dispatch(ns,pose)
        while pending:
            self.wait(lambda:True,1.,True)
            for ns,(pose,handle,future,end) in list(pending.items()):
                if not future.done():
                    if time.monotonic()>end:raise RuntimeError(f'{ns or "robot1"}: cooperative navigation timeout')
                    continue
                terminal=future.result();self.handles.remove(handle);del pending[ns]
                self.event('NAV_RESULT',robot=ns or 'robot1',status=terminal.status,error=terminal.result.error_code)
                if terminal.status==4 and terminal.result.error_code==0:continue
                if terminal.status!=6:raise RuntimeError(f'{ns or "robot1"}: cooperative navigation canceled')
                if attempts[ns]>self.c['navigation_retries']:
                    raise RuntimeError(f'{ns or "robot1"}: navigation retries exhausted')
                dispatch(ns,pose)
        self.measurement_after=self.now()

    def handover(self):
        if getattr(self,'static_test',False):
            self.deadline=time.monotonic()+self.c['wall_timeout_s']
            self.targets=json.loads((self.directory/'cooperative-targets.json').read_text())
            poses=json.loads((self.directory/'cooperative-test-initial.json').read_text())
            self.event('TEST_SETUP',map='static truth map',targets='known world coordinates',localization='AMCL',initial_poses=poses)
            self.start_localization(*poses)
            return
        self.event('WAITING_S1')
        path=self.directory/'rescue-result.json'
        self.wait(lambda:path.is_file(),86400.)
        result=json.loads(path.read_text())
        if not result.get('success'):raise RuntimeError('S1 failed; cooperative rescue not started')
        self.deadline=time.monotonic()+self.c['wall_timeout_s']
        self.wait(lambda:len(self.odom)==2 and self.map is not None,15.)
        root_pose=self.tf_pose('base_footprint')
        # Keep robot1's odometry drift out of the stationary follower's prior.
        if self.mapping_start_pose is None:
            raise RuntimeError('Mapping-start reference missing for robot2 initialization')
        root_start=self.mapping_start_pose
        second=compose(compose(root_start,self.c['initial_relative_pose']),
                       relative(self.initial_odom['robot2'],pose_value(self.odom['robot2'].pose.pose)))
        self.event('HANDOVER',initial_poses=[root_pose,second])
        save=subprocess.run(['ros2','run','nav2_map_server','map_saver_cli','-f',str(self.directory/'cooperative-map'),
            '--ros-args','-p','use_sim_time:=true'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=30.)
        (self.directory/'cooperative-map-save.log').write_text(save.stdout)
        if save.returncode:raise RuntimeError('Final map save failed')
        # Target positions must belong to this final map, not the pre-S1 snapshot.
        self.targets=json.loads((self.directory/'targets.json').read_text())
        (self.directory/'cooperative-targets.json').write_text(json.dumps(self.targets,indent=2))
        self.stop_slam()
        self.seed_enabled=False
        self.start_localization(root_pose,second)

    def start_localization(self,root_pose,second):
        folder=self.directory/'robot2';folder.mkdir(exist_ok=True)
        nav=yaml.safe_load((self.directory/'nav2-runtime.yaml').read_text())
        (folder/'nav2-runtime.yaml').write_text(yaml.safe_dump(namespaced_config(nav,'robot2'),sort_keys=False))
        self.log=(self.directory/'cooperative-nav.log').open('w')
        os.environ.setdefault('CYCLONEDDS_URI','<CycloneDDS><Domain><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>')
        self.process=subprocess.Popen(['ros2','launch','edgeai_navigation','cooperative_navigation.launch.py',
            'directory:='+str(self.directory)],stdout=self.log,stderr=subprocess.STDOUT,start_new_session=True)
        for ns,pose in zip(('','robot2'),(root_pose,second)):
            state_client=self.create_client(GetState,'/'+(ns+'/' if ns else '')+'amcl/get_state')
            active_by=time.monotonic()+40.
            while self.service(state_client,GetState.Request()).current_state.id!=3:
                if time.monotonic()>active_by:raise RuntimeError('AMCL activation timeout')
                next_poll=time.monotonic()+.2
                self.wait(lambda:time.monotonic()>=next_poll,1.)
            self.wait(lambda n=ns:self.initial_pubs[n].get_subscription_count()>0,30.)
            msg=PoseWithCovarianceStamped();msg.header.frame_id='map';msg.header.stamp=self.get_clock().now().to_msg()
            msg.pose.pose=self.goal_pose(pose).pose
            msg.pose.covariance[0]=msg.pose.covariance[7]=self.c['initial_std_xy_m']**2
            msg.pose.covariance[35]=self.c['initial_std_yaw_rad']**2
            self.initial_pubs[ns].publish(msg)
            self.acquire_manual(ns)
        self.initialized=True

    def localize(self):
        self.event('LOCALIZING')
        stable=None;last=None;last_poses=None;begin=time.monotonic();next_motion=begin+5.
        while time.monotonic()-begin<self.c['localization_timeout_s']:
            self.wait(lambda:True,1.,True)
            now=self.now()
            # Ask AMCL to consume scans at rest as well; this does not move the robot.
            if now-self.last_report>=.5:
                self.last_report=now
                good=all(self.localized(ns) for ns in ('','robot2'))
                poses=[pose_value(self.localization[n].pose.pose) for n in ('','robot2')] if good else None
                moved=poses is not None and last_poses is not None and any(
                    math.dist(a[:2],b[:2])>.05 or abs(relative(a,b)[2])>.08 for a,b in zip(last_poses,poses))
                if not good or moved or (last is not None and now<last):stable=None
                elif stable is None:stable=now
                last=now;last_poses=poses
                if stable is not None and now-stable>=self.c['localization_stable_s']:
                    self.event('LOCALIZED',poses=poses,scan_match=[self.laser_match(n) for n in ('','robot2')]);return
            if time.monotonic()>=next_motion and not self.localized('robot2'):
                next_motion=time.monotonic()+10.
                # Native Spin retains full swept-footprint collision checking.
                request=Spin.Goal(target_yaw=.35);request.time_allowance.sec=6
                terminal=self.action(self.spin['robot2'],request,12.)
                self.event('LOCALIZATION_MOTION',status=terminal.status,error=terminal.result.error_code)
        raise RuntimeError('Laser localization did not converge')

    def refresh_target(self,force=False):
        """Use the live map-frame estimate; handover files remain audit snapshots."""
        self.target_ready=getattr(self,'target_ready',True)
        if getattr(self,'static_test',False):return False
        wall=time.monotonic()
        if not force and wall<getattr(self,'target_poll_after',0.):return False
        self.target_poll_after=wall+.5
        path=self.directory/'targets.json'
        if not path.exists():return False
        try:
            registry=json.loads(path.read_text())
            if registry.get('frame_id')!='map':raise ValueError('Target registry frame is not map')
            target=next((t for t in registry['targets'] if t['id']==self.c['target_id']),None)
            if (target is None or not target.get('confirmed',False) or
                    any(not isinstance(target.get(k),(int,float)) or not math.isfinite(target[k])
                        for k in ('x','y','yaw'))):
                raise ValueError('C1 estimate missing, unconfirmed or invalid')
        except (OSError,ValueError,KeyError,TypeError) as error:
            if self.target_ready:self.event('TARGET_UNAVAILABLE',reason=str(error))
            self.target_ready=False
            return False
        previous=self.target
        # A new registry stamp is not a new visual observation. Old odom-based
        # detections are republished through AMCL's changing map->odom transform;
        # fixed-map landmarks must not follow that localization correction.
        observed=target.get('last_observed_ns',0)
        minimum=max(previous.get('last_observed_ns',0),getattr(self,'target_observation_floor',0))
        if observed<=minimum:
            if not getattr(self,'ignored_target_reprojection',False):
                self.event('TARGET_REPROJECTION_IGNORED',target_id=target['id'],
                           last_observed_ns=observed,minimum_observation_ns=minimum,
                           reason='No new visual observation; retain fixed-map target')
                self.ignored_target_reprojection=True
            self.target_ready=True
            return False
        self.ignored_target_reprojection=False
        shift=math.hypot(target['x']-previous['x'],target['y']-previous['y'])
        angle=abs(math.atan2(math.sin(target['yaw']-previous['yaw']),math.cos(target['yaw']-previous['yaw'])))
        changed=shift>.02 or angle>.05
        if changed or not self.target_ready:
            self.event('TARGET_UPDATED',target_id=target['id'],shift_m=shift,
                       position=[target['x'],target['y']],stamp_ns=registry.get('stamp_ns'))
        self.target=target;self.target_ready=True
        return changed

    def select_parking_pair(self):
        pairs=parking_pairs(self.map,self.target,self.footprint,self.c)
        # Reachability is checked through each robot's native planner before dispatch.
        pairs.sort(key=lambda pair:sum(math.dist(self.tf_pose('robot2/base_footprint' if ns else 'base_footprint')[:2],p[:2])
                   for ns,p in zip(('','robot2'),pair)))
        selected=None
        for pair in pairs[:12]:
            okay=True
            for ns,p in zip(('','robot2'),pair):
                plan=self.action(self.plan[ns],ComputePathToPose.Goal(goal=self.goal_pose(p),planner_id='GridBasedHeading'),15.)
                if plan.status!=4 or plan.result.error_code!=0:
                    self.event('PLAN_REJECTED',robot=ns or 'robot1',start=self.tf_pose('robot2/base_footprint' if ns else 'base_footprint'),
                               goal=p,error=plan.result.error_code,message=getattr(plan.result,'error_msg',''))
                    okay=False;break
            if okay:selected=pair;break
        if selected is None:raise RuntimeError('No jointly reachable C1 parking pair')
        self.event('PARKING_SELECTED',target_id=self.target['id'],goals=selected)
        return selected

    def execute(self):
        self.handover();self.localize()
        target=next((t for t in self.targets['targets'] if t['id']==self.c['target_id'] and t.get('yaw') is not None),None)
        if target is None:raise RuntimeError('C1 not localized during mapping; target search required')
        self.target=target
        self.target_observation_floor=self.targets.get('stamp_ns',0)
        self.refresh_target()
        if not self.target_ready:raise RuntimeError('C1 estimate unavailable before navigation')
        nav=yaml.safe_load((self.directory/'nav2-runtime.yaml').read_text())
        self.footprint=json.loads(nav['local_costmap']['local_costmap']['ros__parameters']['footprint'])
        selected=self.select_parking_pair()
        self.navigate_pair(selected)
        self.refresh_target(force=True)
        self.event('JOINT_DWELL',target_id=target['id'])
        dwell=JointDwell(self.c['radius_m'],self.c['dwell_s']);last_event=-100.;last_sample=None;repairs=0
        while time.monotonic()<self.deadline:
            self.wait(lambda:True,1.,True);now=self.now();distances=[];valid=True
            if self.refresh_target():dwell=JointDwell(self.c['radius_m'],self.c['dwell_s'])
            target=self.target
            # Compare at one common, fresh odometry timestamp.
            stamp=min(Time.from_msg(m.header.stamp).nanoseconds for m in self.odom.values())
            try:
                for frame in ('base_footprint','robot2/base_footprint'):
                    stamp=min(stamp,Time.from_msg(self.buffer.lookup_transform('map',frame,Time()).header.stamp).nanoseconds)
            except TransformException as error:
                dwell.update(now,[],False)
                if now-last_event>=1.:
                    last_event=now;self.event('JOINT_STATUS',valid=False,reason='TF unavailable: '+str(error),dwell_s=0.)
                continue
            if not 0<=now-stamp/1e9<=self.c['max_data_age_s']:
                dwell.update(stamp/1e9,[],False)
                if now-last_event>=1.:
                    last_event=now;self.event('JOINT_STATUS',valid=False,reason='Pose sample stale or from future',dwell_s=0.)
                continue
            if last_sample==stamp:continue
            last_sample=stamp
            if stamp/1e9<self.measurement_after:
                dwell.update(stamp/1e9,[],False)
                continue
            reasons=[]
            try:
                for ns in ('','robot2'):
                    p=self.tf_pose('robot2/base_footprint' if ns else 'base_footprint',Time(nanoseconds=stamp))
                    distances.append(math.dist(p[:2],[target['x'],target['y']]))
                    v=self.odom[ns].twist.twist
                    if math.hypot(v.linear.x,v.linear.y)>=.02 or abs(v.angular.z)>=.05:
                        reasons.append((ns or 'robot1')+' moving')
                    if not self.localized(ns):reasons.append((ns or 'robot1')+' localization invalid')
                if not self.target_ready:reasons.append('C1 estimate unavailable')
                valid=not reasons
            except TransformException:
                valid=False;reasons.append('TF unavailable at common sample')
            if any(d>self.c['radius_m'] for d in distances):reasons.append('Outside rescue circle')
            done=dwell.update(stamp/1e9,distances,valid)
            if now-last_event>=1.:
                last_event=now;self.event('JOINT_STATUS',distances_m=distances,valid=valid,
                    radius_m=self.c['radius_m'],required_dwell_s=self.c['dwell_s'],
                    in_range=[d<=self.c['radius_m'] for d in distances],target_ready=self.target_ready,
                    reasons=reasons,
                    dwell_s=stamp/1e9-dwell.start if dwell.start is not None else 0.)
            if done:return dict(success=True,target_id=target['id'],distances_m=distances,dwell_s=self.c['dwell_s'],goals=selected)
            if valid and len(distances)==2 and any(d>self.c['radius_m'] for d in distances):
                if repairs>=self.c.get('rescue_repair_rounds',3):
                    raise RuntimeError('C1 remains outside rescue radius after correction rounds')
                repairs+=1
                self.event('RESCUE_REPAIR',attempt=repairs,distances_m=distances,
                           radius_m=self.c['radius_m'],reason='Outside rescue circle; reselect parking pair')
                selected=self.select_parking_pair()
                self.navigate_pair(selected)
                self.refresh_target(force=True)
                dwell=JointDwell(self.c['radius_m'],self.c['dwell_s'])
        raise RuntimeError('Joint rescue completion timeout')

    def close(self):
        self.closing=True;self.deadline=time.monotonic()+35.
        for handle in self.handles:
            try:
                future=handle.cancel_goal_async();self.wait(future.done,5.)
            except Exception:pass
        for ns,seq in self.sequences.items():
            if self.states.get(ns,{}).get('command_sequence')==seq:
                try:
                    self.service(self.stop[ns],Trigger.Request(),5.)
                    self.wait(lambda n=ns:self.states.get(n,{}).get('phase')=='IDLE' and
                              self.states[n].get('mode')=='STOPPED',5.)
                except Exception:pass
        if self.process is not None and self.process.poll() is None:
            os.killpg(self.process.pid,signal.SIGINT)
            try:self.process.wait(timeout=12.)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid,signal.SIGTERM);self.process.wait(timeout=8.)
        if self.log:self.log.close()


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--directory',required=True);parser.add_argument('--config',required=True)
    parser.add_argument('--static-test',action='store_true',help='Use prepared static map and target coordinates; skip S1/SLAM handover')
    args,ros_args=parser.parse_known_args();rclpy.init(args=ros_args)
    node=CooperativeRescue(args.directory,yaml.safe_load(Path(args.config).read_text()))
    node.static_test=args.static_test
    if args.static_test:node.seed_enabled=False
    signal.signal(signal.SIGTERM,lambda *_:(_ for _ in ()).throw(KeyboardInterrupt()))
    result=dict(success=False)
    try:result=node.execute()
    except (Exception,KeyboardInterrupt) as exc:result=dict(success=False,reason=str(exc) or 'Interrupted')
    finally:
        try:node.close()
        except Exception as exc:result.update(success=False,reason='Cleanup failed: '+str(exc))
        node.event('RESULT',**result)
        path=Path(args.directory)/'cooperative-result.json';temporary=path.with_suffix('.tmp')
        temporary.write_text(json.dumps(result,indent=2,ensure_ascii=False));temporary.replace(path)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return 0 if result['success'] else 1


if __name__=='__main__':raise SystemExit(main())
