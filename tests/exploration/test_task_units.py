"""Unit adapter contracts: independent navigation, deadlines and terminal checks."""
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace as NS
import sys
import math
import time
import unittest
from unittest.mock import patch
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/edgeai_mission/scripts'))
sys.path.insert(0,str(ROOT/'src/edgeai_navigation/scripts'))
from task_units import Units,UnitFailure


def ready(value):
    f=Future();f.set_result(value);return f


class Handle:
    accepted=True
    def __init__(self):self.future=Future();self.cancellations=0
    def get_result_async(self):return self.future
    def cancel_goal_async(self):
        self.cancellations+=1
        if not self.future.done():self.finish(5,0)
        return ready(NS())
    def finish(self,status=4,error=0):self.future.set_result(NS(status=status,result=NS(error_code=error)))


class Client:
    def __init__(self):self.goals=[]
    def server_is_ready(self):return True
    def send_goal_async(self,goal):
        h=Handle();self.goals.append(h);return ready(h)


class TaskUnits(unittest.TestCase):
    def timed_single(self):
        from edgeai_tasks.rescue_policy import Dwell
        u=Units.__new__(Units)
        u.check_owner=lambda:None
        u.goal=dict(x=1.,y=0.,yaw=0.,target_id=1)
        u.navigation_after=0.
        clock=[1.]
        n=NS(c=dict(alignment_enabled=False,navigation_timeout_s=180.,dwell_s=1.,
                    max_data_age_s=.5,position_tolerance_m=.1,yaw_tolerance_rad=.25,
                    stopped_linear_mps=.02,stopped_angular_rps=.05),navigation_started=0.)
        n.get_clock=lambda:NS(now=lambda:NS(nanoseconds=round(clock[0]*1e9)))
        n.latest_goal=lambda goal:dict(goal,x=1.+(.11 if round(clock[0]*10)%2 else 0.))
        n.goal_changed=lambda a,b:abs(a['x']-b['x'])>=.1
        n.fresh_measurement=lambda now,g:(now,dict(rescue_distance_m=.35,
            rescue_position_limit_m=.5,yaw_error_rad=.05,linear_mps=0.,angular_rps=0.))
        u.node=n;u.dwell=Dwell(n.c)
        return u,n,clock

    def test_s1_target_jitter_within_task_limits_does_not_restart_dwell(self):
        u,n,clock=self.timed_single()
        self.assertFalse(u.check_single()['complete'])
        for i in range(1,12):
            clock[0]=1.+i*.1
            result=u.check_single()
        self.assertTrue(result['complete'])
        self.assertEqual(u.dwell.since,1.)
        # Real violation still resets the dwell, even with the same target ID.
        n.fresh_measurement=lambda now,g:(now,dict(rescue_distance_m=.55,
            rescue_position_limit_m=.5,yaw_error_rad=.05,linear_mps=0.,angular_rps=0.))
        clock[0]=2.2
        self.assertFalse(u.check_single()['complete'])
        self.assertIsNone(u.dwell.since)

    def test_s1_callback_skew_preserves_dwell_but_long_outage_resets(self):
        u,n,clock=self.timed_single()
        measure=n.fresh_measurement
        def missing(*args):raise ValueError('No fresh odometry/TF pair')
        self.assertFalse(u.check_single()['complete'])
        for i in range(1,12):
            clock[0]=1.+i*.1
            n.fresh_measurement=missing
            pending=u.check_single()
            self.assertTrue(pending['dwell_retained'])
            self.assertFalse(pending['complete'])
            self.assertFalse(u.condition_complete)
            self.assertEqual(u.dwell.since,1.)
            n.fresh_measurement=measure
            done=u.check_single()
        self.assertTrue(done['complete'])
        # Missing data cannot itself confirm success, even after a completed sample.
        clock[0]=2.2;n.fresh_measurement=missing
        self.assertFalse(u.check_single()['complete'])
        self.assertFalse(u.condition_complete)
        clock[0]=2.7
        self.assertFalse(u.check_single()['dwell_retained'])
        self.assertIsNone(u.dwell.since)
        n.fresh_measurement=measure
        self.assertFalse(u.check_single()['complete'])
        self.assertEqual(u.dwell.since,2.7)

    def test_s1_pairs_odometry_at_common_tf_time_without_extrapolation(self):
        from collections import deque
        from rclpy.time import Time
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import TransformStamped
        from edgeai_tasks.normal_rescue import Rescue
        n=Rescue.__new__(Rescue)
        n.c=dict(max_data_age_s=.5,goal_source='map_pose',position_tolerance_m=.1)
        n.base_frame='robot1/base_footprint';n.odom_received=time.monotonic()
        n.odom_history=deque()
        for stamp in (1.90,1.92,1.94):
            odom=Odometry();odom.header.stamp=Time(nanoseconds=round(stamp*1e9)).to_msg()
            n.odom_history.append(odom)
        tf=TransformStamped();tf.header.stamp=Time(nanoseconds=1_920_000_000).to_msg()
        tf.transform.rotation.w=1.;tf.transform.translation.x=.03
        calls=[]
        def lookup(parent,child,stamp):
            calls.append(stamp.nanoseconds)
            if stamp.nanoseconds>1_920_000_000:raise AssertionError('Queried ahead of common TF time')
            return tf
        n.buffer=NS(lookup_transform=lookup)
        stamp,measurement=n.fresh_measurement(1.96,dict(x=0.,y=0.,yaw=0.))
        self.assertAlmostEqual(stamp,1.92)
        self.assertAlmostEqual(measurement['rescue_distance_m'],.03)
        self.assertEqual(calls,[0,1_920_000_000])
        with self.assertRaisesRegex(ValueError,'No fresh odometry/TF pair'):
            n.fresh_measurement(2.5,dict(x=0.,y=0.,yaw=0.))

    def timed_joint(self):
        from rclpy.time import Time
        from edgeai_tasks.cooperative_policy import JointDwell
        u=self.joint();n=u.node;clock=[1.]
        n.c.update(max_data_age_s=.5,yaw_tolerance_rad=.25)
        n.target=dict(x=0.,y=0.,yaw=0.);n.target_ready=True
        n.now=lambda:clock[0];n.refresh_target=lambda:False;n.localized=lambda ns,**kwargs:True
        n.tf_pose=lambda *a:[.5,0.,math.pi]
        u.dwell=JointDwell(.7,1.);u.navigation_after=0.;u.last_sample=None;u.last_event=-100.
        def sample(now,stamp=None):
            clock[0]=now
            msg=Time(nanoseconds=round((now if stamp is None else stamp)*1e9)).to_msg()
            n.odom={ns:NS(header=NS(stamp=msg),twist=NS(twist=NS(linear=NS(x=0.,y=0.),angular=NS(z=0.)))) for ns in ('robot1','robot2')}
            n.buffer=NS(lookup_transform=lambda *a:NS(header=NS(stamp=msg)))
            return u.check_joint()
        return u,n,sample,clock

    def test_c1_new_target_estimates_preserve_dwell_until_conditions_fail(self):
        u,n,sample,_=self.timed_joint()
        n.c['dwell_s']=1.;n.c['settle_margin_s']=0.
        # The real refresh contract updates geometry and returns True for a
        # significant estimate shift. Reproduce repeated 2 cm visual changes.
        updates=[0]
        def refresh():
            updates[0]+=1
            n.target['x']=.021 if updates[0]%2 else 0.
            return True
        n.refresh_target=refresh
        n.tf_pose=lambda *a:[.5,0.,math.pi]
        self.assertFalse(sample(1.)['complete'])
        for i in range(1,12):result=sample(1.+i*.1)
        self.assertTrue(result['complete'])
        self.assertEqual(u.dwell.start,1.)
        # A genuinely out-of-range latest estimate must still reset the dwell.
        n.tf_pose=lambda *a:[.9,0.,math.pi]
        self.assertFalse(sample(2.2)['complete'])
        self.assertIsNone(u.dwell.start)
        n.tf_pose=lambda *a:[.5,0.,math.pi]
        self.assertFalse(sample(2.3)['complete'])
        self.assertAlmostEqual(u.dwell.start,2.3)

    def test_c1_callback_skew_does_not_restart_confirmed_dwell(self):
        u,n,sample,_=self.timed_joint()
        self.assertFalse(sample(1.)['complete'])
        for i in range(1,11):
            now=1.+i*.1
            pending=sample(now,now+.02) # Odometry callback precedes /clock.
            self.assertTrue(pending['dwell_retained'])
            self.assertEqual(u.dwell.start,1.)
            done=sample(now)
        self.assertTrue(done['complete'])

    def test_c1_localization_callback_skew_preserves_dwell_but_quality_failure_resets(self):
        u,n,sample,_=self.timed_joint()
        sample(1.)
        for i in range(1,12):
            now=1.+i*.1
            def missing(ns,**kwargs):raise ValueError(ns+' localization TF time unavailable')
            n.localized=missing
            pending=sample(now)
            self.assertTrue(pending['dwell_retained'])
            self.assertEqual(u.dwell.start,1.)
            n.localized=lambda ns,**kwargs:True
            result=sample(now)
        self.assertTrue(result['complete'])
        n.localized=lambda ns,**kwargs:False
        self.assertFalse(sample(2.2)['complete'])
        self.assertIsNone(u.dwell.start)

    def test_c1_localization_outage_is_bounded(self):
        u,n,sample,_=self.timed_joint()
        sample(1.)
        def missing(ns,**kwargs):raise ValueError(ns+' fresh scan/TF pair unavailable')
        n.localized=missing
        self.assertTrue(sample(1.2)['dwell_retained'])
        self.assertFalse(sample(1.6)['dwell_retained'])
        self.assertIsNone(u.dwell.start)

    def test_c1_motion_still_resets_dwell_during_localization_wait(self):
        u,n,sample,_=self.timed_joint()
        sample(1.)
        def missing(ns,**kwargs):
            raise ValueError(ns+' fresh scan/TF pair unavailable')
        n.localized=missing
        original=n.tf_pose
        def moving(*args):
            for msg in n.odom.values():msg.twist.twist.linear.x=.1
            return original(*args)
        n.tf_pose=moving
        result=sample(1.1)
        self.assertFalse(result['valid'])
        self.assertIsNone(u.dwell.start)

    def test_c1_data_grace_retains_dwell_without_confirming_on_missing_data(self):
        from edgeai_tasks.cooperative_policy import JointDwell
        u,n,sample,_=self.timed_joint()
        u.dwell=JointDwell(.7,1.,2.)
        sample(1.)
        def missing(ns,**kwargs):raise ValueError(ns+' localization TF time unavailable')
        n.localized=missing
        pending=sample(2.5)
        self.assertTrue(pending['dwell_retained'])
        self.assertFalse(pending['complete'])
        n.localized=lambda ns,**kwargs:True
        self.assertTrue(sample(2.6)['complete'])

    def test_c1_tf_wait_retries_same_sample_and_long_outage_resets(self):
        from tf2_ros import TransformException
        u,n,sample,clock=self.timed_joint()
        sample(1.);sample(1.1)
        pose=n.tf_pose
        def missing(*a):raise TransformException('TF callback pending')
        n.tf_pose=missing
        pending=sample(1.2)
        self.assertTrue(pending['dwell_retained'])
        self.assertAlmostEqual(u.last_sample/1e9,1.1)
        n.tf_pose=pose
        self.assertFalse(sample(1.2)['complete'])
        self.assertAlmostEqual(u.last_sample/1e9,1.2)
        clock[0]=1.8
        self.assertFalse(u.wait_joint_data(1.8,'no new data')['dwell_retained'])
        self.assertIsNone(u.dwell.start)
        self.assertFalse(sample(1.8)['complete'])
        self.assertEqual(u.dwell.start,1.8)

    def test_c1_heading_failure_resets_dwell_and_keeps_healthy_robot(self):
        from rclpy.time import Time
        from edgeai_tasks.cooperative_policy import JointDwell
        u=self.joint();n=u.node
        n.c.update(max_data_age_s=.5,yaw_tolerance_rad=.25,alignment_enabled=False)
        n.target=dict(x=0.,y=0.,yaw=0.);n.target_ready=True
        n.now=lambda:2.;n.refresh_target=lambda:False;n.localized=lambda ns,**kwargs:True
        stamp=Time(nanoseconds=2_000_000_000).to_msg()
        n.odom={ns:NS(header=NS(stamp=stamp),twist=NS(twist=NS(linear=NS(x=0.,y=0.),angular=NS(z=0.)))) for ns in ('robot1','robot2')}
        n.buffer=NS(lookup_transform=lambda *a:NS(header=NS(stamp=stamp)))
        poses={'robot1/base_footprint':[.5,0.,math.pi],'robot2/base_footprint':[0.,.5,0.]}
        n.tf_pose=lambda frame,*a:poses[frame]
        u.dwell=JointDwell(.7,1.);u.dwell.start=1.
        u.navigation_after=0.;u.last_sample=None;u.last_event=-100.;u.arrived={'robot1','robot2'}
        with self.assertRaisesRegex(UnitFailure,'heading outside tolerance'):u.check_joint()
        self.assertEqual(u.arrived,{'robot1'});self.assertFalse(u.failed_parking)
        self.assertIsNone(u.dwell.start);self.assertFalse(u.condition_complete)
        self.assertAlmostEqual(abs(u.last['yaw_errors_rad'][1]),math.pi/2)
        poses['robot2/base_footprint'][2]=-math.pi/2;u.last_sample=None
        self.assertFalse(u.check_joint()['complete']);self.assertEqual(u.dwell.start,2.)

    def test_c1_heading_error_requests_local_correction(self):
        u,n,sample,_=self.timed_joint()
        n.tf_pose=lambda *a:[.5,0.,0.]
        result=sample(1.)
        self.assertTrue(result['needs_correction'])
        self.assertFalse(result['complete'])
        self.assertIsNone(u.dwell.start)

    def test_c1_corrects_one_robot_with_shared_s1_geometry(self):
        u,n,sample,_=self.timed_joint()
        u.alignment_started=None
        n.tf_pose=lambda *a:[.5,0.,0.]
        sample(1.)
        n.states={ns:dict(phase='IDLE') for ns in ('robot1','robot2')}
        n.spin={ns:object() for ns in ('robot1','robot2')}
        n.wait=lambda predicate,*a: self.assertTrue(predicate())
        actions=[]
        n.action=lambda client,request,*a:(actions.append(request) or NS(status=4,result=NS(error_code=0)))
        result=u.correct_joint()
        self.assertEqual(result['kind'],'spin')
        self.assertEqual(len(actions),1)
        self.assertAlmostEqual(abs(actions[0].target_yaw),.6)
        self.assertIsNone(u.dwell.start)

    def test_c1_outside_circle_uses_bounded_forward_step(self):
        u,n,sample,_=self.timed_joint();u.alignment_started=None
        n.tf_pose=lambda *a:[.75,0.,math.pi]
        sample(1.)
        n.states={ns:dict(phase='IDLE') for ns in ('robot1','robot2')}
        n.advance={ns:object() for ns in ('robot1','robot2')}
        n.wait=lambda predicate,*a:self.assertTrue(predicate())
        actions=[]
        n.action=lambda client,request,*a:(actions.append(request) or NS(status=4,result=NS(error_code=0)))
        result=u.correct_joint()
        self.assertEqual(result['kind'],'forward')
        self.assertAlmostEqual(actions[0].target.x,.03)
        self.assertAlmostEqual(actions[0].speed,.03)

    def test_stop_request_is_handled_before_entering_ros_spin(self):
        import rclpy
        from edgeai_tasks.cooperative_rescue import CooperativeRescue
        from edgeai_tasks.normal_rescue import Rescue
        for algorithm in (CooperativeRescue,Rescue):
            node=algorithm.__new__(algorithm)
            node.deadline=float('inf');node.stop_requested=True;node.closing=False
            with patch.object(rclpy,'ok',return_value=True),patch.object(rclpy,'spin_once') as spin:
                with self.assertRaises(KeyboardInterrupt):node.wait(lambda:False)
                spin.assert_not_called()

    def joint(self):
        u=Units.__new__(Units);u.task='c1';u.pending={};u.arrived=set();u.selected=[[0.,0.,0.],[1.,0.,0.]]
        u.navigation_deadlines={};u.condition_complete=False;u.failure_kind='initial';u.fatal_reason=None
        u.failed_parking={}
        u.has_control=True
        u.node=NS(c=dict(navigation_timeout_s=180.,radius_m=.7,dwell_s=10.),nav={'robot1':Client(),'robot2':Client()},
            handles=[],check_owner=lambda:None,goal_pose=lambda p:PoseStamped(),event=lambda *a,**kw:None,
            states={'robot1':dict(phase='IDLE'),'robot2':dict(phase='IDLE')},
            get_clock=lambda:NS(now=lambda:NS(nanoseconds=2_000_000_000)))
        def wait(predicate,timeout):
            if not predicate():raise RuntimeError('transition not ready')
        u.node.wait=wait
        return u

    def test_first_task_checks_both_types_and_reports_missing_c1(self):
        from edgeai_tasks.rescue_policy import select_goal
        u=self.joint();u.task='find_targets';u.target_index=0
        u.required_targets=[dict(id=1,type='normal'),dict(id=2,type='cooperative')]
        u.node.deadline=time.monotonic()+30
        u.node.c.update(normal_offset_m=.35,goal_source='detected_target')
        rows=[dict(id=1,type='normal',x=1.,y=2.,yaw=0.,spread_m=.01)]
        u.node.select_current_goal=lambda:select_goal(u.node.c,dict(frame_id='map',targets=rows))
        u.invoke('select_target');self.assertEqual(u.goal['target_id'],1)
        self.assertFalse(u.invoke('next_target')['complete'])
        with self.assertRaisesRegex(ValueError,'cooperative'):u.invoke('select_target')
        rows.append(dict(id=2,type='cooperative',x=3.,y=2.,yaw=0.,spread_m=.01))
        u.invoke('select_target');self.assertEqual(u.goal['target_id'],2)
        self.assertTrue(u.invoke('next_target')['complete'])
        u.invoke('check_targets');self.assertTrue(u.condition_complete)

    def test_c1_missing_target_has_explicit_reason(self):
        u=self.joint();u.node.c['target_id']=2
        with self.assertRaisesRegex(UnitFailure,'C1 target not localized'):
            u.joint_target(dict(frame_id='map',targets=[]))

    def test_dispatches_both_without_waiting_for_first_result(self):
        u=self.joint();u.dispatch()
        self.assertEqual(set(u.pending),{'robot1','robot2'})
        self.assertTrue(all(not entry[1].done() for entry in u.pending.values()))

    def test_failure_preserves_other_active_action_and_does_not_resend(self):
        u=self.joint();u.dispatch();healthy=u.pending['robot2'][0]
        u.pending['robot1'][0].finish(6,208)
        with self.assertRaises(UnitFailure):u.poll_navigation()
        u.cancel_navigation();u.dispatch()
        self.assertIs(u.pending['robot2'][0],healthy)
        self.assertEqual(healthy.cancellations,0)
        self.assertEqual(len(u.node.nav['robot2'].goals),1)
        self.assertEqual(len(u.node.nav['robot1'].goals),2)
        self.assertEqual(u.failed_parking['robot1'],[[0.,0.,0.]])

    def test_no_result_during_nav2_recovery_does_not_trigger_task_retry(self):
        u=self.joint();u.dispatch()
        for _ in range(5):self.assertFalse(u.poll_navigation()['complete'])
        self.assertEqual([h[0].cancellations for h in u.pending.values()],[0,0])

    def test_start_occupied_failure_does_not_blacklist_parking_goal(self):
        u=self.joint();u.dispatch();u.pending['robot1'][0].finish(6,ComputePathToPose.Result.START_OCCUPIED)
        with self.assertRaises(UnitFailure):u.poll_navigation()
        self.assertFalse(u.failed_parking)

    def test_arrived_robot_waits_while_other_navigates(self):
        u=self.joint();u.dispatch();u.pending['robot1'][0].finish()
        self.assertFalse(u.poll_navigation()['complete'])
        u.cancel_navigation();u.dispatch()
        self.assertEqual(len(u.node.nav['robot1'].goals),1)
        u.pending['robot2'][0].finish()
        self.assertTrue(u.poll_navigation()['complete'])
        self.assertFalse(u.condition_complete,'Navigation completion is not rescue success')

    def test_timeout_cancels_and_confirms_only_failed_robot(self):
        u=self.joint();u.dispatch();late=u.pending['robot1'][0];healthy=u.pending['robot2'][0]
        u.navigation_deadlines['robot1']=time.monotonic()-1
        with self.assertRaisesRegex(UnitFailure,'timeout'):u.poll_navigation()
        self.assertTrue(late.future.done());self.assertEqual(late.cancellations,1)
        self.assertEqual(healthy.cancellations,0)

    def test_external_cancellation_is_fatal_and_sticky(self):
        u=self.joint();u.node.deadline=time.monotonic()+30;u.dispatch();u.pending['robot1'][0].finish(5)
        with self.assertRaises(UnitFailure) as first:u.invoke('poll_navigation')
        self.assertEqual(first.exception.category,'fatal')
        with self.assertRaises(UnitFailure) as second:u.invoke('dispatch_navigation')
        self.assertEqual(second.exception.category,'fatal')

    def test_external_cancel_between_retry_units_cannot_be_discarded(self):
        u=self.joint();u.dispatch();u.pending['robot2'][0].finish(5)
        with self.assertRaises(UnitFailure) as error:u.cancel_navigation()
        self.assertEqual(error.exception.category,'fatal')
        u.cancel_navigation(True,shutting_down=True)
        self.assertFalse(u.pending)

    def test_ownership_loss_cannot_be_masked_by_region_fallback(self):
        u=self.joint();u.node.deadline=time.monotonic()+30
        def lost():raise RuntimeError('operator takeover')
        u.node.check_owner=lost
        with self.assertRaises(UnitFailure):u.invoke('dispatch_navigation')
        with self.assertRaises(UnitFailure) as error:u.invoke('inside_rescue_region')
        self.assertEqual(error.exception.category,'fatal')

    def test_ownership_loss_inside_calculation_is_not_retryable(self):
        u=self.joint();u.node.deadline=time.monotonic()+30;owned=[True]
        def check():
            if not owned[0]:raise RuntimeError('operator takeover')
        def calculation(**kw):
            owned[0]=False;raise RuntimeError('legacy wait failed')
        u.node.check_owner=check;u.node.select_parking_pair=calculation
        with self.assertRaises(UnitFailure) as error:u.invoke('select_parking')
        self.assertEqual(error.exception.category,'fatal')
        self.assertEqual(u.fatal_reason,'operator takeover')

    def test_success_requires_rescue_condition_confirmation(self):
        u=self.joint()
        with self.assertRaisesRegex(UnitFailure,'conditions'):u.record_success()

    def test_cancel_failure_still_attempts_authority_and_process_cleanup(self):
        u=self.joint();closed=[]
        def fail(*a,**kw):raise RuntimeError('cancel confirmation timeout')
        u.cancel_navigation=fail;u.node.close=lambda:closed.append('shutdown')
        u.node.destroy_node=lambda:closed.append('destroy')
        with self.assertRaisesRegex(RuntimeError,'cancel confirmation'):u.close()
        self.assertEqual(closed,['shutdown','destroy'])

    def test_start_occupied_without_path_cannot_pass_peer_check(self):
        from edgeai_tasks.cooperative_rescue import CooperativeRescue
        n=CooperativeRescue.__new__(CooperativeRescue)
        n.map=None;n.target={'id':2};n.footprint=[];n.c={};n.plan={'robot1':'root','robot2':'second'}
        n.tf_pose=lambda *a:[0.,0.,0.];n.goal_pose=lambda p:PoseStamped()
        events=[];n.event=lambda name,**kw:events.append(name)
        n.action=lambda *a:NS(status=6,result=NS(error_code=ComputePathToPose.Result.START_OCCUPIED))
        n.record_occupied_costmap=lambda *a:None
        pair=([.5,0.,0.],[0.,.5,0.])
        with patch('edgeai_tasks.cooperative_rescue.parking_pairs',return_value=[pair]):
            with self.assertRaisesRegex(RuntimeError,'preplan start occupied'):n.select_parking_pair()
        self.assertEqual(events.count('PLAN_REJECTED'),1)
        self.assertEqual(set(n.preplan_blocked),{'robot1'})

    def test_c1_extra_settle_time_survives_dwell_reset(self):
        u=self.joint();u.node.c['settle_margin_s']=1.
        u.reset_dwell()
        self.assertEqual(u.dwell.duration,u.node.c['dwell_s']+1.)
        self.assertFalse(u.dwell.update(0.,[.5,.5]))
        for i in range(1,int(u.dwell.duration*10)):
            self.assertFalse(u.dwell.update(i/10,[.5,.5]))
        self.assertTrue(u.dwell.update(u.dwell.duration,[.5,.5]))

    def preplan_recovery_node(self):
        import yaml
        from edgeai_tasks.cooperative_rescue import CooperativeRescue
        n=CooperativeRescue.__new__(CooperativeRescue)
        n.c=yaml.safe_load((ROOT/'src/edgeai_navigation/config/cooperative_rescue.yaml').read_text())
        n.preplan_blocked={'robot2':{}};n.preplan_recovery_history={}
        n.spin={'robot2':'spin'};n.backup={'robot2':'backup'}
        n.escape={'robot2':'escape'};n.advance={'robot2':'forward'}
        n.event=lambda *a,**kw:None
        return n

    def test_preplan_recovery_turns_and_retains_tried_motion_history(self):
        n=self.preplan_recovery_node();pose=[0.,0.,0.];calls=[]
        n.tf_pose=lambda *a:list(pose)
        def action(client,request,timeout):
            calls.append((client,request.target_yaw));pose[2]+=request.target_yaw
            return NS(status=4,result=NS(error_code=0))
        n.action=action;n.recover_preplan_start()
        self.assertFalse(n.preplan_blocked)
        n.preplan_blocked={'robot2':{}};n.recover_preplan_start()
        self.assertEqual(calls,[('spin',.35),('spin',-.35)])
        self.assertEqual(n.preplan_recovery_history['robot2'],[0,1])

    def test_preplan_recovery_success_without_motion_is_not_accepted(self):
        n=self.preplan_recovery_node();n.tf_pose=lambda *a:[0.,0.,0.]
        n.action=lambda *a:NS(status=4,result=NS(error_code=0))
        from edgeai_tasks.cooperative_rescue import PreplanStartOccupied
        with self.assertRaises(PreplanStartOccupied):n.recover_preplan_start()
        self.assertEqual(len(n.preplan_recovery_history['robot2']),len(n.c['preplan_recovery']['motions']))

    def test_preplan_recovery_rejects_wrong_direction_configuration(self):
        from edgeai_tasks.cooperative_rescue import validate_preplan_recovery
        settings=self.preplan_recovery_node().c['preplan_recovery']
        validate_preplan_recovery(settings)
        settings['motions']=[dict(kind='backup',distance_m=.05)]
        with self.assertRaises(ValueError):validate_preplan_recovery(settings)

    def test_reselect_parking_excludes_failed_goal_history(self):
        from edgeai_tasks.cooperative_rescue import CooperativeRescue
        n=CooperativeRescue.__new__(CooperativeRescue)
        n.map=None;n.target={'id':2};n.footprint=[];n.c={};n.plan={'robot1':'root','robot2':'second'}
        n.tf_pose=lambda *a:[0.,0.,0.];n.goal_pose=lambda p:PoseStamped();n.event=lambda *a,**kw:None
        n.action=lambda *a:NS(status=4,result=NS(error_code=0,path=NS(header=NS(frame_id='map'),poses=[PoseStamped()])))
        failed=([.5,0.,0.],[0.,.5,0.]);next_pair=([.6,0.,0.],[0.,.5,0.])
        with patch('edgeai_tasks.cooperative_rescue.parking_pairs',return_value=[failed,next_pair]),patch('edgeai_tasks.cooperative_rescue.path_clear_of_peer',return_value=True):
            self.assertNotEqual(n.select_parking_pair(excluded={'robot1':[failed[0]]})[0],failed[0])

    def test_rejects_native_path_that_crosses_parked_peer(self):
        from edgeai_tasks.cooperative_rescue import CooperativeRescue
        n=CooperativeRescue.__new__(CooperativeRescue)
        n.map=None;n.target={'id':2};n.c={};n.plan={'robot1':'root','robot2':'second'}
        n.footprint=[(.15,.15),(-.15,.15),(-.15,-.15),(.15,-.15)]
        n.tf_pose=lambda frame:[-2.,0.,0.] if frame=='base_footprint' else [0.,2.,0.]
        def pose(p):
            msg=PoseStamped();msg.pose.position.x,msg.pose.position.y=p[:2];msg.pose.orientation.w=1.
            return msg
        n.goal_pose=pose
        events=[];n.event=lambda name,**kw:events.append((name,kw))
        def plan(client,request,timeout):
            waypoints=[request.goal]
            if client=='root' and request.goal.pose.position.x==-1.:
                waypoints=[pose((0.,0.)),request.goal]
            return NS(status=4,result=NS(error_code=0,path=NS(header=NS(frame_id='map'),poses=waypoints)))
        n.action=plan
        blocked=([-1.,0.,0.],[0.,0.,0.]);clear=([-1.,1.,0.],[0.,0.,0.])
        with patch('edgeai_tasks.cooperative_rescue.parking_pairs',return_value=[blocked,clear]):
            self.assertCountEqual(n.select_parking_pair(),clear)
        self.assertTrue(any(name=='PLAN_BLOCKED_BY_PARKED_PEER' and row['robot']=='robot1' for name,row in events))


if __name__=='__main__':unittest.main()
