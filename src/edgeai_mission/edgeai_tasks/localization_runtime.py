#!/usr/bin/env python3
"""Shared navigation data and localization lifecycle, owned by the mission."""
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
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger, Empty
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import State, Transition
from nav2_msgs.action import NavigateToPose, ComputePathToPose, Spin, DriveOnHeading, BackUp
from nav2_msgs.srv import ManageLifecycleNodes
from tf2_ros import Buffer, TransformListener, TransformException
import yaml

from edgeai_tasks.cooperative_policy import relative, cell
from edgeai_nav.fleet_config import namespaced_config, configure_peer_costmaps



def validate_preplan_recovery(settings):
    if not settings.get('enabled',False):return
    for key in ('speed_mps','time_allowance_s','action_timeout_s','min_progress_m','min_progress_rad'):
        if not math.isfinite(float(settings.get(key,0.))) or settings.get(key,0.)<=0:
            raise ValueError('Invalid preplan_recovery.'+key)
    motions=settings.get('motions',[])
    if not motions:raise ValueError('preplan_recovery.motions is empty')
    for motion in motions:
        kind=motion.get('kind')
        if kind not in ('spin','backup','forward','escape'):raise ValueError('Unknown preplan recovery motion')
        value=float(motion.get('angle_rad' if kind=='spin' else 'distance_m',0.))
        if not math.isfinite(value) or value==0 or (kind=='backup' and value>0) or (kind=='forward' and value<0):
            raise ValueError('Invalid preplan recovery motion direction/distance')


def pose_value(p):
    q=p.orientation
    return [p.position.x,p.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]


class LocalizationRuntime(Node):
    def __init__(self,directory,config,*,node_name='localization_runtime'):
        super().__init__(node_name)
        self.directory,self.c=Path(directory),config
        self.localization_namespaces=tuple(config.get('localization_namespaces',('robot1','robot2')))
        self.joint_mapping=bool(config.get('joint_mapping',False)) or os.environ.get('EDGEAI_JOINT_MAPPING')=='true'
        # Task adapters pass rescue tuning separately from the coordinator config.
        mission_config=self.directory/'mission-runtime.json'
        if mission_config.is_file():
            self.joint_mapping=self.joint_mapping or bool(json.loads(mission_config.read_text()).get('joint_mapping',False))
        margin=float(config.get('settle_margin_s',0.))
        if not math.isfinite(margin) or margin<0:raise ValueError('Invalid settle_margin_s')
        validate_preplan_recovery(config.get('preplan_recovery',{}))
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.odom={};self.scan={};self.localization={};self.states={}
        self.scan_history={ns:deque(maxlen=10) for ns in self.localization_namespaces};self.scan_matches={}
        self.map=None;self.process=None;self.log=None;self.handles=[];self.sequences={}
        self.state_received={};self.nomotion={};self.nomotion_pending={};self.closing=False
        self.localizing=False;self.initialization_subscriptions=[]
        self.deadline=float('inf');self.initialized=False;self.last_report=0.
        self.measurement_after=0.
        self.state_pub=self.create_publisher(String,'/cooperative_rescue/state',
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.initial_pubs={};self.nav={};self.plan={};self.spin={};self.manual={};self.stop={}
        self.escape={};self.backup={};self.preplan_blocked={};self.preplan_recovery_history={}
        self.peer_poses_enabled=False;self.peer_pubs={}
        for ns in self.localization_namespaces:
            topic=lambda name:'/'+(ns+'/' if ns else '')+name
            self.initialization_subscriptions.append(self.create_subscription(Odometry,topic('odom'),
                lambda m,n=ns:self.receive_odom(n,m),qos_profile_sensor_data))
            self.initialization_subscriptions.append(self.create_subscription(LaserScan,topic('scan'),
                lambda m,n=ns:self.receive_scan(n,m),qos_profile_sensor_data))
            self.create_subscription(PoseWithCovarianceStamped,topic('amcl_pose'),
                lambda m,n=ns:self.localization.update({n:m}),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.initialization_subscriptions.append(self.create_subscription(String,topic('navigation_authority/state'),
                lambda m,n=ns:self.receive_state(n,m),
                QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)))
            self.initial_pubs[ns]=self.create_publisher(PoseWithCovarianceStamped,topic('initialpose'),1)
            if len(self.localization_namespaces)>1 and self.c.get('peer_costmap',{}).get('enabled',True):
                self.peer_pubs[ns]=self.create_publisher(PoseStamped,'/cooperative_rescue/peer_pose/'+(ns or 'robot1'),1)
            self.nav[ns]=ActionClient(self,NavigateToPose,topic('navigation/cooperative_to_pose'))
            self.plan[ns]=ActionClient(self,ComputePathToPose,topic('navigation_backend/compute_path_to_pose'))
            self.spin[ns]=ActionClient(self,Spin,topic('navigation_backend/spin'))
            self.escape[ns]=ActionClient(self,DriveOnHeading,topic('navigation_backend/escape_occupied'))
            self.backup[ns]=ActionClient(self,BackUp,topic('navigation_backend/backup'))
            if not hasattr(self,'advance'):self.advance={}
            self.advance[ns]=ActionClient(self,DriveOnHeading,topic('navigation_backend/drive_on_heading'))
            self.manual[ns]=self.create_client(SetBool,topic('navigation_authority/set_manual'))
            self.stop[ns]=self.create_client(Trigger,topic('navigation_authority/stop'))
            self.nomotion[ns]=self.create_client(Empty,topic('request_nomotion_update'))
        self.initialization_subscriptions.append(self.create_subscription(OccupancyGrid,'/map',lambda m:setattr(self,'map',m),
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)))
        interval=float(config.get('nomotion_update_interval_s',.25))
        if not math.isfinite(interval) or interval<=0:raise ValueError('Invalid nomotion_update_interval_s')
        self.nomotion_timer=self.create_timer(interval,self.update_at_rest)
        if self.peer_pubs:self.create_timer(.10,self.publish_peer_poses)


    def publish_peer_poses(self):
        if not self.peer_poses_enabled or self.closing:return
        now=self.get_clock().now()
        max_age=self.c.get('peer_costmap',{}).get('max_age_s',.5)
        for ns,pub in self.peer_pubs.items():
            sample=self.localization.get(ns)
            if sample is None:continue
            cov=sample.pose.covariance
            if not (all(0<=cov[i]<=self.c['max_std_xy_m']**2 for i in (0,7)) and
                    0<=cov[35]<=self.c['max_std_yaw_rad']**2):continue
            try:tf=self.buffer.lookup_transform('map',(ns+'/' if ns else '')+'base_footprint',Time())
            except TransformException:continue
            stamp=Time.from_msg(tf.header.stamp)
            if not -.25<=(now-stamp).nanoseconds/1e9<=max_age or not stamp.nanoseconds:continue
            msg=PoseStamped();msg.header.frame_id='map';msg.header.stamp=min(stamp,now).to_msg()
            t=tf.transform.translation
            msg.pose.position.x,msg.pose.position.y,msg.pose.position.z=t.x,t.y,t.z
            msg.pose.orientation=tf.transform.rotation;pub.publish(msg)


    def update_at_rest(self):
        # Forced scan updates are for initial convergence, not continuous tracking.
        if not self.localizing or self.closing:return
        for ns,client in self.nomotion.items():
            pending=self.nomotion_pending.get(ns)
            if pending is not None and not pending.done():continue
            odom=self.odom.get(ns)
            if odom is None:continue
            age=self.now()-Time.from_msg(odom.header.stamp).nanoseconds/1e9
            velocity=odom.twist.twist
            if not 0<=age<=self.c['max_data_age_s']:continue
            if math.hypot(velocity.linear.x,velocity.linear.y)>=.02 or abs(velocity.angular.z)>=.05:continue
            if client.service_is_ready():self.nomotion_pending[ns]=client.call_async(Empty.Request())


    def finish_initialization(self):
        """Mission owner keeps TF/AMCL peer poses and launch supervision only."""
        self.localizing=False;self.nomotion_timer.cancel()
        for subscription in self.initialization_subscriptions:self.destroy_subscription(subscription)
        self.initialization_subscriptions.clear()
        for group in (self.nav,self.plan,self.spin,self.escape,self.backup,self.advance):
            for client in group.values():client.destroy()
            group.clear()
        for group in (self.manual,self.stop,self.nomotion):
            for client in group.values():self.destroy_client(client)
            group.clear()
        self.nomotion_pending.clear();self.scan.clear();self.scan_matches.clear()
        self.odom.clear();self.states.clear();self.state_received.clear();self.map=None
        for history in self.scan_history.values():history.clear()



    def receive_state(self,ns,msg):
        self.states[ns]=json.loads(msg.data);self.state_received[ns]=time.monotonic()


    def receive_odom(self,ns,msg):
        self.odom[ns]=msg


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
            if getattr(self,'stop_requested',False) and not self.closing:raise KeyboardInterrupt()
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


    def stop_slam(self,name=None):
        name=name or '/slam_toolbox'
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


    def laser_match(self,ns,*,require_data=False):
        if self.map is None:
            if require_data:raise ValueError(ns+' localization map unavailable')
            return 0.
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
        if scan is None:
            if require_data:raise ValueError(ns+' fresh scan/TF pair unavailable')
            return 0.
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


    def localized(self,ns,*,require_data=False):
        def unavailable(reason):
            if require_data:raise ValueError(ns+' '+reason)
            return False
        sample=self.localization.get(ns)
        if sample is None:return unavailable('AMCL pose unavailable')
        stamp=Time.from_msg(sample.header.stamp).nanoseconds/1e9
        cov=sample.pose.covariance
        # AMCL need not republish amcl_pose while stationary. Validate the
        # live TF and laser instead of forcing particle updates to refresh it.
        age=self.now()-stamp
        if age<0 or (self.localizing and age>self.c['max_data_age_s']):
            return unavailable('AMCL sample time unavailable')
        if not (all(0<=cov[i]<=self.c['max_std_xy_m']**2 for i in (0,7)) and
                0<=cov[35]<=self.c['max_std_yaw_rad']**2):return False
        try:tf=self.buffer.lookup_transform('map',(ns+'/' if ns else '')+'base_footprint',Time())
        except TransformException:return unavailable('localization TF unavailable')
        age=self.now()-Time.from_msg(tf.header.stamp).nanoseconds/1e9
        if not 0<=age<=self.c['max_data_age_s']:return unavailable('localization TF time unavailable')
        return self.laser_match(ns,require_data=require_data)>=self.c['min_scan_match_ratio']


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


    def finalize_targets(self):
        client=self.create_client(Trigger,'/targets/finalize_map')
        try:
            result=self.service(client,Trigger.Request(),15.)
            binding=json.loads(result.message)
            if not binding.get('finalized') or binding.get('mode')!='fixed_map':
                raise RuntimeError('Target map finalization not confirmed')
            self.event('TARGET_MAP_FINALIZED',**binding)
        finally:self.destroy_client(client)


    def start_localization(self,root_pose,second):
        folder=self.directory/'robot2';folder.mkdir(exist_ok=True)
        nav=yaml.safe_load((self.directory/'nav2-runtime.yaml').read_text())
        single=len(self.localization_namespaces)==1
        if self.joint_mapping:
            if not (folder/'nav2-runtime.yaml').is_file():
                raise RuntimeError('Joint mapping requires the running robot2 nav2-runtime.yaml')
        elif not single:
            nav2=namespaced_config(nav,'robot2',source_namespace='robot1')
            configure_peer_costmaps(nav2,'robot2',self.c.get('peer_costmap',{}))
            (folder/'nav2-runtime.yaml').write_text(yaml.safe_dump(nav2,sort_keys=False))
        self.log=(self.directory/'cooperative-nav.log').open('w')
        os.environ.setdefault('CYCLONEDDS_URI','<CycloneDDS><Domain><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>')
        launch_args=['directory:='+str(self.directory)]
        if self.c.get('reuse_robot2_navigation',self.joint_mapping) or single:
            launch_args.append('reuse_robot2_navigation:=true')
        if single:launch_args.append('enable_robot2:=false')
        if getattr(self,'defer_localization_initialization',False):launch_args.append('localization_autostart:=false')
        self.process=subprocess.Popen(['ros2','launch','edgeai_navigation','cooperative_navigation.launch.py',
            *launch_args],stdout=self.log,stderr=subprocess.STDOUT,start_new_session=True)
        if getattr(self,'mission_localization',False):
            # Let the parent coordinator clean up this owned launch group even
            # if the transitional Python worker is forcefully terminated.
            owned=dict(pid=self.process.pid,start_ticks=Path(f'/proc/{self.process.pid}/stat').read_text().split()[21])
            path=self.directory/'mission-owned-navigation.json';tmp=path.with_suffix('.tmp')
            tmp.write_text(json.dumps(owned));tmp.replace(path)
        self.localization_initial_poses=(root_pose,second)
        if getattr(self,'defer_localization_initialization',False):
            for ns in self.localization_namespaces:self.acquire_manual(ns)
            return
        self.initialize_localization()


    def initialize_localization(self):
        if self.initialized:return
        for ns,pose in zip(self.localization_namespaces,self.localization_initial_poses):
            state_client=self.create_client(GetState,'/'+(ns+'/' if ns else '')+'amcl/get_state')
            try:self.wait_amcl_active(state_client)
            finally:self.destroy_client(state_client)
            self.wait(lambda n=ns:self.initial_pubs[n].get_subscription_count()>0,30.)
            self.wait(lambda:self.now()>0 and ns in self.odom and self.map is not None,20.)
            msg=PoseWithCovarianceStamped();msg.header.frame_id='map';msg.header.stamp=self.get_clock().now().to_msg()
            msg.pose.pose=self.goal_pose(pose).pose
            msg.pose.covariance[0]=msg.pose.covariance[7]=self.c['initial_std_xy_m']**2
            msg.pose.covariance[35]=self.c['initial_std_yaw_rad']**2
            self.initial_pubs[ns].publish(msg)
            self.event('INITIAL_POSE_SENT',robot=ns or 'robot1',pose=pose,
                       std_xy_m=self.c['initial_std_xy_m'],std_yaw_rad=self.c['initial_std_yaw_rad'])
            if ns not in self.sequences:self.acquire_manual(ns)
        self.initialized=True


    def localization_transition(self,command):
        managers=['/shared_localization_manager/manage_nodes']
        managers+=['/'+ns+'/localization_manager/manage_nodes'
                   for ns in self.localization_namespaces if ns]
        # Start the shared map before followers; stop/reset followers first.
        if command in (ManageLifecycleNodes.Request.RESET,ManageLifecycleNodes.Request.SHUTDOWN):
            managers.reverse()
        for manager in managers:
            client=self.create_client(ManageLifecycleNodes,manager)
            try:
                self.event('LOCALIZATION_TRANSITION',command=int(command),manager=manager)
                self.service(client,ManageLifecycleNodes.Request(command=command),
                             self.c.get('localization_startup_timeout_s',40.))
            finally:self.destroy_client(client)


    def activate_localization(self):
        if self.initialized:return
        if not getattr(self,'defer_localization_initialization',False):return
        self.localization_transition(ManageLifecycleNodes.Request.STARTUP)
        self.initialize_localization()


    def reset_localization(self):
        if getattr(self,'external_navigation',False):raise RuntimeError('External localization is not owned by this task')
        self.localization_transition(ManageLifecycleNodes.Request.RESET)
        self.initialized=False;self.localization.clear();self.scan_matches.clear()


    def wait_amcl_active(self,client):
        # The first GetState response may be lost while AMCL is being configured.
        # Retry read-only discovery within one budget, before publishing initialpose.
        active_by=min(self.deadline,time.monotonic()+40.)
        last='state unavailable'
        while time.monotonic()<active_by:
            future=None
            try:
                self.wait(client.service_is_ready,min(2.,active_by-time.monotonic()))
                future=client.call_async(GetState.Request())
                self.wait(future.done,min(2.,max(0.,active_by-time.monotonic())))
                state=future.result().current_state
                if state.id==State.PRIMARY_STATE_ACTIVE:
                    self.event('AMCL_ACTIVE',service=client.srv_name)
                    return
                last=state.label
            except RuntimeError as error:
                if self.process is not None and self.process.poll() is not None:raise
                last=str(error)
                if future is not None and not future.done():
                    client.remove_pending_request(future)
            pause=min(.2,max(0.,active_by-time.monotonic()))
            next_poll=time.monotonic()+pause
            if pause:self.wait(lambda:time.monotonic()>=next_poll,pause+.1)
        raise RuntimeError(f'AMCL activation timeout: {client.srv_name}; last={last}')


    def wait_navigation_active(self,timeout=45.):
        """Action discovery and authority IDLE do not imply lifecycle activation."""
        names=['/map_server']+[
            '/'+(ns+'/' if ns else '')+name
            for ns in self.localization_namespaces
            for name in ('amcl','controller_server','planner_server','behavior_server',
                         'velocity_smoother','collision_monitor','bt_navigator')]
        clients={name:self.create_client(GetState,name+'/get_state') for name in names}
        deadline=min(self.deadline,time.monotonic()+timeout);waiting=None
        def budget():
            remaining=deadline-time.monotonic()
            if remaining<=0:raise RuntimeError('Cooperative navigation activation timeout')
            return remaining
        try:
            while True:
                self.wait(lambda:all(c.service_is_ready() for c in clients.values()),budget(),True)
                futures={name:c.call_async(GetState.Request()) for name,c in clients.items()}
                self.wait(lambda:all(f.done() for f in futures.values()),budget(),True)
                inactive={name:f.result().current_state.label for name,f in futures.items()
                          if f.result().current_state.id!=State.PRIMARY_STATE_ACTIVE}
                if not inactive:
                    self.event('NAVIGATION_READY',nodes=names);return
                if inactive!=waiting:
                    self.event('NAVIGATION_WAITING',inactive=inactive);waiting=inactive
                next_poll=time.monotonic()+.2
                self.wait(lambda:time.monotonic()>=next_poll,budget(),True)
        finally:
            for client in clients.values():self.destroy_client(client)


    def localize(self):
        self.localizing=True
        try:return self._localize()
        finally:self.localizing=False


    def _localize(self):
        self.peer_poses_enabled=False
        self.event('LOCALIZING')
        stable=None;last=None;last_poses=None;begin=time.monotonic();next_motion=begin+5.
        while time.monotonic()-begin<self.c['localization_timeout_s']:
            self.wait(lambda:True,1.,True)
            now=self.now()
            # Ask AMCL to consume scans at rest as well; this does not move the robot.
            if now-self.last_report>=.5:
                self.last_report=now
                good=all(self.localized(ns) for ns in self.localization_namespaces)
                poses=[pose_value(self.localization[n].pose.pose) for n in self.localization_namespaces] if good else None
                moved=poses is not None and last_poses is not None and any(
                    math.dist(a[:2],b[:2])>.05 or abs(relative(a,b)[2])>.08 for a,b in zip(last_poses,poses))
                if not good or moved or (last is not None and now<last):stable=None
                elif stable is None:stable=now
                last=now;last_poses=poses
                if stable is not None and now-stable>=self.c['localization_stable_s']:
                    self.peer_poses_enabled=True;self.publish_peer_poses()
                    self.event('LOCALIZED',poses=poses,scan_match=[self.laser_match(n) for n in self.localization_namespaces]);return
            motion_ns='robot2' if 'robot2' in self.localization_namespaces else 'robot1'
            if time.monotonic()>=next_motion and not self.localized(motion_ns):
                next_motion=time.monotonic()+10.
                # Native Spin retains full swept-footprint collision checking.
                request=Spin.Goal(target_yaw=.35);request.time_allowance.sec=6
                terminal=self.action(self.spin[motion_ns],request,12.)
                self.event('LOCALIZATION_MOTION',status=terminal.status,error=terminal.result.error_code)
        diagnostics=self.localization_diagnostics()
        self.event('LOCALIZATION_FAILED',robots=diagnostics)
        raise RuntimeError('Laser localization did not converge: '+json.dumps(diagnostics))


    def localization_diagnostics(self):
        result={}
        for ns in self.localization_namespaces:
            sample=self.localization.get(ns);scan=self.scan.get(ns)
            row=dict(amcl_pose=sample is not None,scan=scan is not None)
            if sample is not None:
                row.update(pose=pose_value(sample.pose.pose),
                    age_s=self.now()-Time.from_msg(sample.header.stamp).nanoseconds/1e9,
                    std_xy_m=[math.sqrt(max(0.,sample.pose.covariance[i])) for i in (0,7)],
                    std_yaw_rad=math.sqrt(max(0.,sample.pose.covariance[35])))
            try:row['scan_match']=self.laser_match(ns) if scan is not None else None
            except (TransformException,ValueError,RuntimeError) as error:row['scan_match_error']=str(error)
            result[ns or 'robot1']=row
        return result


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
