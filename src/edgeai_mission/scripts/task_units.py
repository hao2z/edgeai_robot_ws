"""Task-local unit adapters. No task sequencing or navigation retry loops here.

Short Trigger endpoints are local to one task executor. Long work delegates to
Nav2 Actions. Canceling the owning Task Action terminates this backend and
confirms legacy Action cleanup before releasing the task slot.
"""
import argparse
import json
import math
from pathlib import Path
import signal
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from rclpy.time import Time
from tf2_ros import TransformException
import yaml


class UnitFailure(RuntimeError):
    def __init__(self,reason,category='failure'):super().__init__(reason);self.category=category


class Units:
    def __init__(self,task,c):
        self.task,self.c=task,c;self.directory=Path(c['directory']);root=Path(c['project_root'])
        self.pending={};self.arrived=set();self.selected=None;self.success=False;self.last=None
        self.failure_kind='initial';self.navigation_after=0.;self.last_sample=None;self.last_event=-100.
        self.alignment_started=None;self.last_target_poll=0.;self.last_nav_goal=None
        self.navigation_deadlines={};self.condition_complete=False;self.fatal_reason=None
        self.failed_parking={}
        self.has_control=False
        if task!='c1':
            from edgeai_tasks.normal_rescue import Rescue
            from edgeai_tasks.rescue_policy import validate_config,Dwell
            settings=yaml.safe_load((self.directory/'rescue-config.yaml').read_text());validate_config(settings)
            self.node=Rescue(settings,self.directory);self.dwell=Dwell(settings);self.goal=None
            self.target_index=0;self.required_targets=c.get('required_targets',[])
        else:
            from edgeai_tasks.cooperative_rescue import CooperativeRescue
            from edgeai_tasks.cooperative_policy import JointDwell
            settings=yaml.safe_load((root/'src/edgeai_navigation/config/cooperative_rescue.yaml').read_text())
            self.node=CooperativeRescue(self.directory,settings)
            self.node.static_test=bool(c.get('cooperative_static_test',False))
            self.node.seed_enabled=False;self.node.handover_reference=str(self.directory/'mission-reference.json')
            self.dwell=JointDwell(settings['radius_m'],settings['dwell_s']+settings.get('settle_margin_s',0.))

    def reset_dwell(self):
        self.condition_complete=False
        if self.task!='c1':
            from edgeai_tasks.rescue_policy import Dwell
            self.dwell=Dwell(self.node.c)
        else:
            from edgeai_tasks.cooperative_policy import JointDwell
            self.dwell=JointDwell(self.node.c['radius_m'],self.node.c['dwell_s']+self.node.c.get('settle_margin_s',0.))
        self.last_sample=None

    def joint_target(self, registry):
        """A missing marker is an explicit condition, never a bare StopIteration."""
        target=next((t for t in registry.get('targets',[]) if t.get('id')==self.node.c['target_id']
                     and t.get('type')=='cooperative' and t.get('yaw') is not None),None)
        if registry.get('frame_id')!='map' or target is None:
            raise UnitFailure('C1 target not localized; camera search required')
        return target

    def cancel_navigation(self,all_robots=False,shutting_down=False,robots=None):
        n=self.node
        for ns,entry in list(self.pending.items()):
            if robots is not None and ns not in robots:continue
            handle,future,goal=entry
            if future.done() and future.result().status==5 and not shutting_down:
                raise UnitFailure('Navigation canceled externally','fatal')
            if self.task=='c1' and not all_robots and robots is None and not future.done():continue
            if not future.done():
                canceled=handle.cancel_goal_async()
                n.wait(lambda:canceled.done() and future.done(),8.)
            if self.task=='c1' and handle in n.handles:n.handles.remove(handle)
            if self.task!='c1':n.goal_handle=None
            del self.pending[ns]
            self.navigation_deadlines.pop(ns,None)
        # A canceled task may have been inside a correction/search adapter.
        # Those actions use the same authority slot but are not navigation entries.
        extra=getattr(n,'goal_handle',None) if self.task!='c1' else None
        if extra is not None and extra.accepted:
            terminal=extra.get_result_async();canceled=extra.cancel_goal_async()
            n.wait(lambda:canceled.done() and terminal.done(),8.)
            n.goal_handle=None
        if all_robots:self.arrived.clear()
        return dict(complete=False)

    def dispatch(self):
        n=self.node
        if self.task!='c1':
            self.cancel_navigation(True);self.goal.update(n.latest_goal(self.goal));goals={'':self.goal}
        else:goals=dict(zip(('','robot2'),self.selected))
        for ns,goal in goals.items():
            if ns in self.pending or ns in self.arrived:continue
            self.check_owner()
            client=n.rescue_client if self.task!='c1' else n.nav[ns]
            def ready():
                self.check_owner()
                state=n.state if self.task!='c1' else n.states.get(ns,{})
                return client.server_is_ready() and state.get('phase')=='IDLE'
            n.wait(ready,10.)
            if self.task!='c1':
                from geometry_msgs.msg import PoseStamped
                msg=PoseStamped();msg.header.frame_id='map';msg.header.stamp=n.get_clock().now().to_msg()
                msg.pose.position.x,msg.pose.position.y=goal['x'],goal['y']
                msg.pose.orientation.z=math.sin(goal['yaw']/2);msg.pose.orientation.w=math.cos(goal['yaw']/2)
            else:msg=n.goal_pose(goal)
            sent=client.send_goal_async(NavigateToPose.Goal(pose=msg));n.wait(sent.done,10.)
            handle=sent.result()
            if not handle.accepted:raise UnitFailure('Navigation goal rejected')
            self.pending[ns]=(handle,handle.get_result_async(),dict(goal) if self.task!='c1' else list(goal))
            self.navigation_deadlines.setdefault(ns,time.monotonic()+n.c['navigation_timeout_s'])
            if self.task!='c1':
                n.goal_handle=handle;self.last_nav_goal=dict(goal)
                n.navigation_attempts.append(dict(attempt=len(n.navigation_attempts)+1,goal=dict(goal),arrival_policy='rescue',accepted=True))
            else:
                n.handles.append(handle);n.event('NAVIGATING',robot=ns or 'robot1',goal=goal,concurrent=True,source='task_tree')
        return dict(complete=False)

    def poll_navigation(self):
        n=self.node;self.check_owner()
        if self.task!='c1':n.check_navigation_budget(n.navigation_started)
        if self.task!='c1' and time.monotonic()>=self.last_target_poll:
            self.last_target_poll=time.monotonic()+n.c.get('target_update_interval_s',1.)
            updated=n.latest_goal(self.goal)
            if n.goal_changed(self.goal,updated):
                self.goal.update(updated);self.failure_kind='target_updated'
                raise UnitFailure('Target changed; cancel and rebuild approach','target_updated')
        failures=[]
        for ns,(handle,future,goal) in list(self.pending.items()):
            if not future.done():
                if time.monotonic()>=self.navigation_deadlines[ns]:
                    canceled=handle.cancel_goal_async()
                    n.wait(lambda:canceled.done() and future.done(),8.)
                    if self.task!='c1':n.goal_handle=None
                    elif handle in n.handles:n.handles.remove(handle)
                    del self.pending[ns]
                    self.navigation_deadlines.pop(ns,None)
                    if self.task=='c1':self.failed_parking.setdefault(ns,[]).append(goal)
                    failures.append(f'{ns or "robot1"}: navigation timeout')
                continue
            terminal=future.result();okay=terminal.status==4 and terminal.result.error_code==0
            if terminal.status==5:raise UnitFailure('Navigation canceled externally','fatal')
            if self.task!='c1':
                n.goal_handle=None;n.navigation_attempts[-1].update(status=terminal.status,error_code=terminal.result.error_code)
            else:
                if handle in n.handles:n.handles.remove(handle)
                n.event('NAV_RESULT',robot=ns or 'robot1',status=terminal.status,error=terminal.result.error_code)
            del self.pending[ns]
            self.navigation_deadlines.pop(ns,None)
            if okay:self.arrived.add(ns)
            else:
                if self.task=='c1' and terminal.result.error_code!=ComputePathToPose.Result.START_OCCUPIED:
                    self.failed_parking.setdefault(ns,[]).append(goal)
                failures.append(f'{ns or "robot1"}: status={terminal.status}, error={terminal.result.error_code}')
        if failures:
            self.failure_kind='failure';raise UnitFailure('; '.join(failures))
        done=len(self.arrived)==(1 if self.task!='c1' else 2)
        if done:
            self.navigation_after=n.get_clock().now().nanoseconds/1e9;self.reset_dwell()
        return dict(complete=done)

    def check_single(self):
        n=self.node;self.check_owner();now=n.get_clock().now().nanoseconds/1e9
        if now-n.navigation_started>n.c['navigation_timeout_s']+n.c['dwell_s']+15:raise UnitFailure('Single rescue time budget exhausted')
        try:
            updated=n.latest_goal(self.goal)
            if n.goal_changed(self.goal,updated):self.reset_dwell()
            self.goal.update(updated);stamp,last=n.fresh_measurement(now,self.goal);self.last=last
            self.last['sample_time_s']=stamp
            if stamp<self.navigation_after:return dict(complete=False,needs_correction=False)
            from edgeai_tasks.rescue_alignment import correction
            correction_needed=n.c.get('alignment_enabled',False) and correction(n.c,self.goal,last) is not None
            done=self.dwell.update(stamp,last['rescue_distance_m'],last['yaw_error_rad'],last['linear_mps'],last['angular_rps'],position_limit=last['rescue_position_limit_m'])
            self.condition_complete=done
            return dict(complete=done,needs_correction=bool(correction_needed),
                distance_m=last['rescue_distance_m'],radius_m=last['rescue_position_limit_m'],
                yaw_error_rad=last['yaw_error_rad'],dwell_s=stamp-self.dwell.since if self.dwell.since is not None else 0.)
        except (TransformException,ValueError) as error:
            self.reset_dwell();return dict(complete=False,needs_correction=False,state='WAITING_DATA',reason=str(error))

    def wait_joint_data(self,now,reason,**details):
        retained=self.dwell.awaiting_data(now,self.node.c['max_data_age_s'])
        self.condition_complete=False
        if not retained:self.last_sample=None
        elapsed=self.dwell.last-self.dwell.start if self.dwell.start is not None else 0.
        return dict(complete=False,needs_correction=False,state='WAITING_DATA',reason=reason,
                    dwell_retained=retained,dwell_s=elapsed,**details)

    def check_joint(self):
        from edgeai_tasks.cooperative_policy import rescue_yaw_error
        n=self.node;self.check_owner();now=n.now()
        if n.refresh_target():self.reset_dwell()
        try:
            stamp=min(Time.from_msg(m.header.stamp).nanoseconds for m in n.odom.values())
            for frame in ('base_footprint','robot2/base_footprint'):
                stamp=min(stamp,Time.from_msg(n.buffer.lookup_transform('map',frame,Time()).header.stamp).nanoseconds)
            age=now-stamp/1e9
            if not 0<=age<=n.c['max_data_age_s'] or stamp/1e9<self.navigation_after:
                return self.wait_joint_data(now,'Pose sample ahead of clock' if age<0 else 'Pose sample stale or before navigation completion',
                                            sample_time_s=stamp/1e9,sample_age_s=age)
            if stamp==self.last_sample:return dict(complete=False,needs_correction=False)
            distances=[];yaw_errors=[];poses=[];valid=n.target_ready
            for ns in ('','robot2'):
                p=n.tf_pose('robot2/base_footprint' if ns else 'base_footprint',Time(nanoseconds=stamp))
                poses.append(list(p))
                distances.append(math.hypot(p[0]-n.target['x'],p[1]-n.target['y']))
                yaw_errors.append(rescue_yaw_error(p,n.target))
                v=n.odom[ns].twist.twist
                valid=valid and math.hypot(v.linear.x,v.linear.y)<.02 and abs(v.angular.z)<.05 and n.localized(ns)
            heading_ok=[abs(e)<=n.c['yaw_tolerance_rad'] for e in yaw_errors]
            self.last_sample=stamp
            done=self.dwell.update(stamp/1e9,distances,valid and all(heading_ok))
            self.last=dict(distances_m=distances,yaw_errors_rad=yaw_errors,dwell_s=n.c['dwell_s'],
                           poses=poses,sample_time_s=stamp/1e9)
            self.condition_complete=done
            if now-self.last_event>=1.:
                self.last_event=now;n.event('JOINT_STATUS',distances_m=distances,yaw_errors_rad=yaw_errors,heading_ok=heading_ok,valid=valid,in_range=[d<=n.c['radius_m'] for d in distances],dwell_s=stamp/1e9-self.dwell.start if self.dwell.start is not None else 0.)
            if valid and (any(d>n.c['radius_m'] for d in distances) or not all(heading_ok)):
                if n.c.get('alignment_enabled',True):
                    return dict(complete=False,needs_correction=True,distances_m=distances,
                                yaw_errors_rad=yaw_errors,heading_ok=heading_ok)
                self.arrived={ns for ns,d,h in zip(('','robot2'),distances,heading_ok) if d<=n.c['radius_m'] and h}
                for ns,pose,distance in zip(('','robot2'),self.selected,distances):
                    if distance>n.c['radius_m']:self.failed_parking.setdefault(ns,[]).append(pose)
                reason='Outside rescue circle; reselect parking' if any(d>n.c['radius_m'] for d in distances) else 'Rescue heading outside tolerance; retry parking alignment'
                raise UnitFailure(reason,'outside' if all(heading_ok) else 'heading')
            return dict(complete=done,needs_correction=False,distances_m=distances,yaw_errors_rad=yaw_errors,heading_ok=heading_ok,radius_m=n.c['radius_m'],valid=valid,
                        dwell_s=stamp/1e9-self.dwell.start if self.dwell.start is not None else 0.)
        except (TransformException,ValueError) as error:
            return self.wait_joint_data(now,str(error))

    def check_owner(self):
        try:self.node.check_owner()
        except RuntimeError as error:raise UnitFailure(str(error),'fatal') from error

    def correct_joint(self):
        from edgeai_tasks.rescue_alignment import adjust_joint
        return adjust_joint(self)

    def invoke(self,op):
        if self.fatal_reason:raise UnitFailure(self.fatal_reason,'fatal')
        try:
            if self.has_control:self.check_owner()
            return self._invoke(op)
        except Exception as error:
            # Legacy algorithm waits can report ownership loss as RuntimeError.
            # Recheck before classifying it as a retryable calculation failure.
            if self.has_control:
                try:self.check_owner()
                except UnitFailure as lost:error=lost
            self.failure_kind=getattr(error,'category','failure')
            if self.failure_kind=='fatal':self.fatal_reason=str(error)
            raise error

    def _invoke(self,op):
        n=self.node
        if time.monotonic()>=n.deadline:raise UnitFailure('Task wall budget exhausted','fatal')
        if op=='acquire':
            n.acquire_control();self.has_control=True;n.navigation_started=n.get_clock().now().nanoseconds/1e9
        elif op=='select_target':
            if self.task=='find_targets':
                target=self.required_targets[self.target_index]
                n.c.update(target_id=target['id'],target_type=target['type'],goal_source='detected_target')
                n.event('REQUIRED_TARGET',json.dumps(target))
            self.goal=n.select_current_goal();n.rescue_goal=self.goal
        elif op=='search_target':self.goal=n.find_target();n.rescue_goal=self.goal
        elif op=='next_target':
            self.target_index+=1
            return dict(complete=self.target_index>=len(self.required_targets))
        elif op=='check_targets':
            for target in self.required_targets:
                n.c.update(target_id=target['id'],target_type=target['type'],goal_source='detected_target')
                n.select_current_goal()
            self.condition_complete=True
        elif op=='select_approach':
            if not n.navigation_attempts:n.navigation_started=n.get_clock().now().nanoseconds/1e9
            self.arrived.clear();self.goal.update(n.latest_goal(self.goal))
            if n.c.get('approach_selection',{}).get('enabled'):self.goal.update(n.choose_approach(self.goal,exclude_current=self.failure_kind=='failure',started=n.navigation_started))
        elif op=='cancel_navigation':return self.cancel_navigation()
        elif op=='cancel_all_navigation':return self.cancel_navigation(True)
        elif op=='dispatch_navigation':return self.dispatch()
        elif op=='poll_navigation':return self.poll_navigation()
        elif op=='inside_rescue_region':
            self.check_owner()
            if self.failure_kind=='target_updated':raise UnitFailure('Rebuild approach for updated target','target_updated')
            self.cancel_navigation(True)
            try:
                _,m=n.fresh_measurement(n.get_clock().now().nanoseconds/1e9,self.goal)
                if m['rescue_distance_m']>m['rescue_position_limit_m']:raise ValueError('Outside task region')
            except (TransformException,ValueError) as error:raise UnitFailure(str(error),self.failure_kind) from error
        elif op=='check_rescue':return self.check_single() if self.task!='c1' else self.check_joint()
        elif op=='correct':
            if self.task=='c1':return self.correct_joint()
            from edgeai_tasks.rescue_alignment import adjust
            now=n.get_clock().now().nanoseconds/1e9
            if self.alignment_started is None:self.alignment_started=now
            if now-self.alignment_started>n.c.get('alignment_timeout_s',60.) or len(n.alignment_attempts)>=n.c.get('alignment_max_actions',20):raise UnitFailure('Correction budget exhausted')
            self.reset_dwell();adjust(n,self.goal,self.last);self.navigation_after=n.get_clock().now().nanoseconds/1e9
        elif op=='handover':n.handover();self.has_control=True
        elif op=='wait_navigation_ready':n.wait_navigation_active()
        elif op=='localize':n.localize()
        elif op=='prepare_target':
            n.target=self.joint_target(n.targets)
            n.target_observation_floor=n.targets.get('stamp_ns',0);n.refresh_target()
            if not n.target_ready:raise UnitFailure('Target estimate unavailable')
            from edgeai_nav.fleet_config import parking_geometry
            nav=yaml.safe_load((self.directory/'nav2-runtime.yaml').read_text());n.footprint=parking_geometry(nav,n.c)
        elif op=='recover_preplan_start':
            if self.failure_kind!='start_occupied':raise UnitFailure('Not a preplan start occupancy failure',self.failure_kind)
            self.cancel_navigation(robots=set(n.preplan_blocked))
            n.recover_preplan_start()
        elif op in ('select_parking','select_parking_locked'):
            locked={ns:self.selected[i] for i,ns in enumerate(('','robot2')) if self.selected and (ns in self.pending or ns in self.arrived)} if op.endswith('locked') else {}
            self.selected=n.select_parking_pair(locked=locked,excluded=self.failed_parking,
                                               settled=self.arrived.intersection(locked))
            self.alignment_started=None
        elif op=='record_success':self.record_success()
        else:raise UnitFailure('Unregistered unit: '+op)
        return {}

    def record_success(self):
        n=self.node
        self.check_owner()
        if not self.condition_complete:raise UnitFailure('Rescue conditions have not passed','fatal')
        if self.task=='find_targets':
            result=dict(success=True,required_targets=self.required_targets,search_attempts=n.search_attempts)
        elif self.task=='s1':
            result=dict(success=True,goal=self.goal,measurement=self.last,dwell_s=n.c['dwell_s'],target_id=self.goal['target_id'],task_state='completed',judge='estimated map pose',navigation_attempts=n.navigation_attempts,alignment_attempts=n.alignment_attempts)
        else:result=dict(success=True,target_id=n.target['id'],distances_m=self.last['distances_m'],yaw_errors_rad=self.last['yaw_errors_rad'],dwell_s=n.c['dwell_s'],goals=self.selected,alignment_attempts=getattr(self,'joint_corrections',[]))
        if self.task in ('s1','c1'):
            from rescue_success_log import success_evidence
            if self.task=='s1':
                m=self.last;g=self.goal
                offset=g.get('approach_distance_m',n.c['normal_offset_m'])
                target_pose=([g['x']+offset*math.cos(g['yaw']),g['y']+offset*math.sin(g['yaw']),g['yaw']]
                             if n.c.get('goal_source')=='detected_target' else [g['x'],g['y'],g['yaw']])
                poses=[[m['x'],m['y'],m['yaw']]];distances=[m['rescue_distance_m']];errors=[m['yaw_error_rad']]
                radius=m['rescue_position_limit_m'];stamp=m['sample_time_s']
            else:
                target_pose=[n.target['x'],n.target['y'],n.target['yaw']]
                poses=self.last['poses'];distances=self.last['distances_m'];errors=self.last['yaw_errors_rad']
                radius=n.c['radius_m'];stamp=self.last['sample_time_s']
            evidence,messages=success_evidence(self.directory,self.task,result['target_id'],target_pose,poses,distances,errors,
                                              radius,n.c['yaw_tolerance_rad'],n.c['dwell_s'],stamp,
                                              n.c.get('stopped_linear_mps',.02),n.c.get('stopped_angular_rps',.05),wait_s=3.)
            result['success_evidence']=evidence
            for message in messages:
                if self.task=='s1':n.event('SUCCESS_POSE',message)
                else:n.event('SUCCESS_POSE',summary=message)
        path=self.directory/('target-search-result.json' if self.task=='find_targets' else 'rescue-result.json' if self.task=='s1' else 'cooperative-result.json');tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(result,ensure_ascii=False,indent=2));tmp.replace(path)
        self.success=True

    def close(self):
        self.node.closing=True
        self.node.deadline=time.monotonic()+35.
        errors=[]
        try:
            try:self.cancel_navigation(True,shutting_down=True)
            except Exception as error:errors.append(str(error))
            try:
                if self.task!='c1':self.node.finish()
                else:self.node.close()
            except Exception as error:errors.append(str(error))
        finally:self.node.destroy_node()
        if errors:raise RuntimeError('; '.join(errors))


def main():
    p=argparse.ArgumentParser();p.add_argument('--task',choices=('find_targets','s1','c1'),required=True);p.add_argument('--config',type=Path,required=True)
    a=p.parse_args();c=json.loads(a.config.read_text());rclpy.init(args=['--ros-args','-p','use_sim_time:=true'])
    units=Units(a.task,c);server=Node(a.task+'_unit_backend');result=dict(success=False)
    stopping=[False]
    def request_stop(*_):
        # Never raise inside an arbitrary rclpy C-extension call or destructor.
        # The main loop and algorithm waits stop at their next Python boundary.
        stopping[0]=True;units.node.stop_requested=True
    signal.signal(signal.SIGTERM,request_stop)
    common=['cancel_navigation','cancel_all_navigation','dispatch_navigation','poll_navigation','check_rescue','record_success']
    operations=common+(['acquire','select_target','search_target','next_target','check_targets'] if a.task=='find_targets' else ['acquire','select_target','search_target','select_approach','inside_rescue_region','correct'] if a.task=='s1' else ['handover','wait_navigation_ready','localize','prepare_target','select_parking','select_parking_locked','recover_preplan_start','correct'])
    def handle(op,response):
        try:data=dict(success=True,**units.invoke(op))
        except Exception as error:data=dict(success=False,reason=str(error),category=getattr(error,'category','failure'))
        response.success=data['success'];response.message=json.dumps(data)
        with (units.directory/'task-unit-events.jsonl').open('a') as f:
            f.write(json.dumps(dict(task=a.task,operation=op,wall_time=time.time(),
                sim_time=units.node.get_clock().now().nanoseconds/1e9,**data))+'\n')
        return response
    services=[server.create_service(Trigger,'/mission/tasks/'+a.task+'/units/'+op,lambda req,res,op=op:handle(op,res)) for op in operations]
    # Legacy algorithms spin their own data node while awaiting ROS replies.
    # The service callback must therefore use a separate executor, never the
    # global executor used by those waits (nested spin is rejected by rclpy).
    service_executor=SingleThreadedExecutor();service_executor.add_node(server)
    try:
        while rclpy.ok() and not stopping[0]:
            # Drain data callbacks before the next service. One spin per 20 ms
            # cannot keep up with TF + clock + odometry + authority streams.
            for _ in range(32):rclpy.spin_once(units.node,timeout_sec=0.)
            service_executor.spin_once(timeout_sec=.01)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        try:units.close();result['success']=units.success
        except Exception as error:result.update(success=False,reason='Cleanup failed: '+str(error))
        saved=units.directory/('target-search-result.json' if a.task=='find_targets' else 'rescue-result.json' if a.task=='s1' else 'cooperative-result.json')
        if result['success'] and saved.exists():result['task_result']=json.loads(saved.read_text())
        path=units.directory/('task-'+a.task+'-worker-result.json');path.write_text(json.dumps(result))
        service_executor.shutdown();server.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return 0 if result['success'] else 1


if __name__=='__main__':raise SystemExit(main())
