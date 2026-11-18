#!/usr/bin/env python3
"""Concurrent C1 navigation and rescue checks using mission-owned localization."""
import argparse
import json
import math
from pathlib import Path
import signal
import time

import rclpy
from rclpy.time import Time
from nav2_msgs.action import NavigateToPose, ComputePathToPose, Spin, DriveOnHeading, BackUp
from nav2_msgs.srv import GetCostmap
from tf2_ros import TransformException
import yaml

from edgeai_tasks.cooperative_policy import relative, parking_pairs, JointDwell, rescue_yaw_error, path_clear_of_peer
from edgeai_nav.fleet_config import parking_geometry


class PreplanStartOccupied(RuntimeError):
    category='start_occupied'


from edgeai_tasks.localization_runtime import LocalizationRuntime,pose_value,validate_preplan_recovery


class CooperativeRescue(LocalizationRuntime):
    def __init__(self,directory,config,*,node_name='cooperative_rescue'):
        super().__init__(directory,config,node_name=node_name)
        # C1 consumes prepared localization; it never starts or stops AMCL.
        self.peer_pubs={}

    def handover(self):
        if getattr(self,'mission_localization_required',False):
            state=json.loads((self.directory/'localization-state.json').read_text())
            if not state.get('ready'):raise RuntimeError('Mission localization is not ready')
        self.deadline=time.monotonic()+self.c['wall_timeout_s']
        self.targets=json.loads((self.directory/'cooperative-targets.json').read_text())
        for ns in ('robot1','robot2'):self.acquire_manual(ns)
        self.initialized=True
        self.event('HANDOVER_REQUESTED',source='prepared localization')

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
        pending={};attempts={ns:0 for ns in ('robot1','robot2')}
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
        for ns,pose in zip(('robot1','robot2'),pair):dispatch(ns,pose)
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
            expected=getattr(self,'targets',{}).get('map_binding',{})
            if expected.get('finalized'):
                binding=registry.get('map_binding',{})
                if (binding.get('epoch')!=expected.get('epoch') or binding.get('mode')!='fixed_map'
                    or not binding.get('finalized')):raise ValueError('Target registry belongs to another or unfinished map')
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


    def record_occupied_costmap(self,ns,pose,error):
        """One bounded crop per robot/5 sim seconds; no extra subscriptions."""
        recorded=getattr(self,'occupied_recorded',{})
        if self.now()-recorded.get(ns,-100.)<5.:return
        self.occupied_recorded=recorded;recorded[ns]=self.now()
        client=self.create_client(GetCostmap,('/'+ns if ns else '')+'/global_costmap/get_costmap')
        try:
            if not client.service_is_ready():return
            grid=self.service(client,GetCostmap.Request(),2.).map
            if grid.header.frame_id!='map':return
            info=grid.metadata;origin=pose_value(info.origin)
            local=relative(origin,pose)
            cx,cy=math.floor(local[0]/info.resolution),math.floor(local[1]/info.resolution)
            reach=math.ceil(.6/info.resolution)
            x0,x1=max(0,cx-reach),min(info.size_x,cx+reach+1)
            y0,y1=max(0,cy-reach),min(info.size_y,cy+reach+1)
            from edgeai_tasks.cooperative_policy import footprint_clear,separated
            peer=self.tf_pose(('robot2' if ns=='robot1' else 'robot1')+'/base_footprint')
            record=dict(sim_time=self.now(),robot=ns or 'robot1',error=error,pose=pose,
                        peer_pose=peer,physical_bodies_separated=separated(pose,peer,self.footprint,0.),
                        padded_bodies_separated=separated(pose,peer,self.footprint,self.c.get('footprint_padding_m',.01)),
                        static_start_clear=footprint_clear(self.map,pose,self.footprint,self.c.get('footprint_padding_m',.01)),
                        footprint_padding_m=self.c.get('footprint_padding_m'),
                        parking_peer_clearance_m=self.c.get('parking_peer_clearance_m'),
                        resolution=info.resolution,origin=origin,crop_origin_cell=[x0,y0],
                        size=[max(0,x1-x0),max(0,y1-y0)],
                        costs=[list(grid.data[y*info.size_x+x0:y*info.size_x+x1]) for y in range(y0,y1)])
            path=self.directory/('occupied-costmap-'+(ns or 'robot1')+'.json')
            path.write_text(json.dumps(record))
            self.event('OCCUPIED_DIAGNOSTIC',robot=ns or 'robot1',static_start_clear=record['static_start_clear'],file=path.name)
        except (RuntimeError,ValueError,OSError,TransformException) as error:
            self.event('OCCUPIED_DIAGNOSTIC_UNAVAILABLE',robot=ns or 'robot1',reason=str(error))
        finally:self.destroy_client(client)


    def recover_preplan_start(self):
        settings=self.c.get('preplan_recovery',{})
        if not settings.get('enabled',False) or not self.preplan_blocked:
            raise PreplanStartOccupied('Preplan start recovery disabled or no blocked robot')
        for ns in self.preplan_blocked:
            history=self.preplan_recovery_history.setdefault(ns,[])
            moved=False
            for index,motion in enumerate(settings['motions']):
                if index in history:continue
                history.append(index)
                before=self.tf_pose((ns+'/' if ns else '')+'base_footprint')
                kind=motion['kind']
                if kind=='spin':
                    request=Spin.Goal();request.target_yaw=float(motion['angle_rad']);client=self.spin[ns]
                else:
                    request=BackUp.Goal() if kind=='backup' else DriveOnHeading.Goal()
                    request.target.x=float(motion['distance_m']);request.speed=float(settings['speed_mps'])
                    client=self.backup[ns] if kind=='backup' else self.escape[ns] if kind=='escape' else self.advance[ns]
                request.time_allowance=rclpy.duration.Duration(seconds=float(settings['time_allowance_s'])).to_msg()
                self.event('PREPLAN_RECOVERY_STARTED',robot=ns or 'robot1',motion=motion,attempt=len(history))
                try:terminal=self.action(client,request,float(settings['action_timeout_s']))
                except RuntimeError as error:
                    raise PreplanStartOccupied('Preplan recovery did not complete: '+str(error)) from error
                if terminal.status==5:
                    error=RuntimeError('Preplan recovery canceled externally');error.category='fatal';raise error
                after=self.tf_pose((ns+'/' if ns else '')+'base_footprint')
                distance=math.dist(before[:2],after[:2]);angle=abs(math.remainder(after[2]-before[2],2*math.pi))
                moved=(terminal.status==4 and terminal.result.error_code==0 and
                       (distance>=settings['min_progress_m'] or angle>=settings['min_progress_rad']))
                self.event('PREPLAN_RECOVERY_RESULT',robot=ns or 'robot1',status=terminal.status,
                           error=terminal.result.error_code,distance_m=distance,angle_rad=angle,progress=moved)
                if moved:break
            if not moved:raise PreplanStartOccupied(f'{ns or "robot1"}: no remaining safe preplan recovery with progress')
        self.preplan_blocked={}


    def select_parking_pair(self,locked=None,excluded=None,settled=()):
        self.preplan_blocked={}
        self.parking_route_failures=[]
        pairs=parking_pairs(self.map,self.target,self.footprint,self.c)
        robots=('robot1','robot2')
        poses={ns:self.tf_pose((ns+'/' if ns else '')+'base_footprint') for ns in robots}
        settled=set(settled)
        # Candidate generation is unordered. Test both robot assignments.
        pairs=[p for a,b in pairs for p in ((a,b),(b,a))]
        if excluded:
            pairs=[pair for pair in pairs if not any(
                math.dist(pair[i][:2],old[:2])<.04
                for i,ns in enumerate(('robot1','robot2')) for old in excluded.get(ns,[]))]
        if locked:
            from edgeai_tasks.cooperative_policy import footprint_clear,separated
            constrained=[]
            for pair in pairs:
                pair=[list(p) for p in pair]
                if any(math.dist(pair[i][:2],pose[:2])>.10 for i,ns in enumerate(('robot1','robot2'))
                       for key,pose in locked.items() if ns==key):continue
                for i,ns in enumerate(('robot1','robot2')):
                    if ns in locked:pair[i]=list(locked[ns])
                if (all(footprint_clear(self.map,p,self.footprint,self.c.get('footprint_padding_m',.01)) for p in pair) and
                    math.dist(pair[0][:2],pair[1][:2])>=self.c['min_separation_m'] and
                    separated(*pair,self.footprint,self.c.get('parking_peer_clearance_m',.08)/2)):
                    constrained.append(pair)
            pairs=constrained
        # Reachability is checked through each robot's native planner before dispatch.
        pairs.sort(key=lambda pair:sum(math.dist(poses[ns][:2],p[:2]) for ns,p in zip(robots,pair)))
        selected=None
        margin=float(self.c.get('parking_peer_clearance_m',self.c.get('parked_peer_clearance_m',.05)))
        if not math.isfinite(margin) or margin<0:raise ValueError('Invalid C1 parked-peer clearance')
        plans={}
        for pair in list({tuple(map(tuple,pair)):pair for pair in pairs}.values())[:12]:
            okay=True
            for i,(ns,p) in enumerate(zip(robots,pair)):
                if ns in settled:continue
                peer=robots[1-i];parked=poses[peer] if peer in settled else pair[1-i]
                key=(ns,tuple(p))
                if key not in plans:
                    plan=self.action(self.plan[ns],ComputePathToPose.Goal(goal=self.goal_pose(p),planner_id='GridBasedCooperative'),15.)
                    result=plan.result
                    if plan.status!=4 or result.error_code!=0 or not result.path.poses or result.path.header.frame_id!='map':
                        self.parking_route_failures.append(dict(robot=ns,goal=list(p)))
                        self.event('PLAN_REJECTED',robot=ns or 'robot1',start=poses[ns],goal=p,error=result.error_code,
                                   message=getattr(result,'error_msg','') or 'No usable path in map frame')
                        if result.error_code in (205,206):self.record_occupied_costmap(ns,poses[ns],result.error_code)
                        if result.error_code==ComputePathToPose.Result.START_OCCUPIED:
                            self.preplan_blocked={ns:dict(start=poses[ns],goal=p)}
                            raise PreplanStartOccupied(f'{ns or "robot1"}: preplan start occupied; recovery required before selecting another goal')
                        plans[key]=None
                    else:plans[key]=[pose_value(s.pose) for s in result.path.poses]
                path=plans[key]
                if path is None:
                    okay=False;break
                if not path_clear_of_peer([poses[ns],*path,p],parked,self.footprint,margin):
                    self.parking_route_failures.append(dict(robot=ns,goal=list(p)))
                    self.event('PLAN_BLOCKED_BY_PARKED_PEER',robot=ns or 'robot1',goal=p,
                               peer=peer or 'robot1',peer_pose=parked,clearance_m=margin)
                    okay=False;break
            if okay:selected=pair;break
        if selected is None:raise RuntimeError('No verified C1 parking pair with peer occupancy')
        self.event('PARKING_SELECTED',target_id=self.target['id'],goals=selected)
        return selected


    def execute(self):
        self.handover();self.wait_navigation_active()
        target=next((t for t in self.targets['targets'] if t['id']==self.c['target_id'] and t.get('yaw') is not None),None)
        if target is None:raise RuntimeError('C1 not localized during mapping; target search required')
        self.target=target
        self.target_observation_floor=self.targets.get('stamp_ns',0)
        self.refresh_target()
        if not self.target_ready:raise RuntimeError('C1 estimate unavailable before navigation')
        nav=yaml.safe_load((self.directory/'nav2-runtime.yaml').read_text())
        self.footprint=parking_geometry(nav,self.c)
        selected=self.select_parking_pair()
        self.navigate_pair(selected)
        self.refresh_target(force=True)
        self.event('JOINT_DWELL',target_id=target['id'])
        dwell=JointDwell(self.c['radius_m'],self.c['dwell_s']+self.c.get('settle_margin_s',0.),self.c.get('dwell_data_grace_s',2.));last_event=-100.;last_sample=None;repairs=0
        while time.monotonic()<self.deadline:
            self.wait(lambda:True,1.,True);now=self.now();distances=[];valid=True
            # Preserve valid dwell across new observations, as in the task BT.
            self.refresh_target()
            target=self.target
            # Compare at one common, fresh odometry timestamp.
            stamp=min(Time.from_msg(m.header.stamp).nanoseconds for m in self.odom.values())
            try:
                for frame in ('robot1/base_footprint','robot2/base_footprint'):
                    stamp=min(stamp,Time.from_msg(self.buffer.lookup_transform('map',frame,Time()).header.stamp).nanoseconds)
            except TransformException as error:
                retained=dwell.awaiting_data(now)
                if now-last_event>=1.:
                    last_event=now;self.event('JOINT_STATUS',valid=False,reason='TF unavailable: '+str(error),
                        dwell_retained=retained,dwell_s=dwell.last-dwell.start if dwell.start is not None else 0.)
                continue
            if not 0<=now-stamp/1e9<=self.c['max_data_age_s']:
                retained=dwell.awaiting_data(now)
                if now-last_event>=1.:
                    last_event=now;self.event('JOINT_STATUS',valid=False,reason='Pose sample stale or from future',
                        dwell_retained=retained,dwell_s=dwell.last-dwell.start if dwell.start is not None else 0.)
                continue
            if last_sample==stamp:continue
            if stamp/1e9<self.measurement_after:
                dwell.update(stamp/1e9,[],False)
                continue
            reasons=[];waiting=[];yaw_errors=[];heading_ok=[]
            try:
                for ns in ('robot1','robot2'):
                    p=self.tf_pose(ns+'/base_footprint',Time(nanoseconds=stamp))
                    distances.append(math.dist(p[:2],[target['x'],target['y']]))
                    yaw_errors.append(rescue_yaw_error(p,target))
                    heading_ok.append(abs(yaw_errors[-1])<=self.c['yaw_tolerance_rad'])
                    v=self.odom[ns].twist.twist
                    if math.hypot(v.linear.x,v.linear.y)>=.02 or abs(v.angular.z)>=.05:
                        reasons.append((ns or 'robot1')+' moving')
                    try:
                        if not self.localized(ns,require_data=True):reasons.append(ns+' localization quality invalid')
                    except ValueError as error:waiting.append(str(error))
                if not self.target_ready:waiting.append('C1 estimate unavailable')
                if waiting and not reasons and all(heading_ok) and all(d<=self.c['radius_m'] for d in distances):
                    retained=dwell.awaiting_data(now)
                    if now-last_event>=1.:
                        last_event=now;self.event('JOINT_STATUS',valid=False,reason='; '.join(waiting),
                            dwell_retained=retained,dwell_s=dwell.last-dwell.start if dwell.start is not None else 0.)
                    continue
                last_sample=stamp
                valid=not reasons and not waiting
            except TransformException:
                dwell.awaiting_data(now)
                continue
            if any(d>self.c['radius_m'] for d in distances):reasons.append('Outside rescue circle')
            if not all(heading_ok):reasons.append('Rescue heading outside tolerance')
            reasons.extend(waiting)
            done=dwell.update(stamp/1e9,distances,valid and len(heading_ok)==2 and all(heading_ok))
            if now-last_event>=1.:
                last_event=now;self.event('JOINT_STATUS',distances_m=distances,yaw_errors_rad=yaw_errors,heading_ok=heading_ok,valid=valid,
                    radius_m=self.c['radius_m'],required_dwell_s=self.c['dwell_s'],
                    in_range=[d<=self.c['radius_m'] for d in distances],target_ready=self.target_ready,
                    reasons=reasons,
                    dwell_s=stamp/1e9-dwell.start if dwell.start is not None else 0.)
            if done:return dict(success=True,target_id=target['id'],distances_m=distances,yaw_errors_rad=yaw_errors,dwell_s=self.c['dwell_s'],goals=selected)
            if valid and len(distances)==2 and (any(d>self.c['radius_m'] for d in distances) or not all(heading_ok)):
                if repairs>=self.c.get('rescue_repair_rounds',3):
                    raise RuntimeError('C1 rescue distance or heading remains invalid after correction rounds')
                repairs+=1
                self.event('RESCUE_REPAIR',attempt=repairs,distances_m=distances,
                           radius_m=self.c['radius_m'],yaw_errors_rad=yaw_errors,reason='Rescue distance or heading invalid; reselect parking pair')
                selected=self.select_parking_pair()
                self.navigate_pair(selected)
                self.refresh_target(force=True)
                dwell=JointDwell(self.c['radius_m'],self.c['dwell_s']+self.c.get('settle_margin_s',0.),self.c.get('dwell_data_grace_s',2.))
        raise RuntimeError('Joint rescue completion timeout')




def main():
    parser=argparse.ArgumentParser();parser.add_argument('--directory',required=True);parser.add_argument('--config',required=True)
    parser.add_argument('--static-test',action='store_true',help='Use prepared static map and target coordinates; skip S1/SLAM handover')
    args,ros_args=parser.parse_known_args();rclpy.init(args=ros_args)
    node=CooperativeRescue(args.directory,yaml.safe_load(Path(args.config).read_text()))
    node.static_test=args.static_test
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
