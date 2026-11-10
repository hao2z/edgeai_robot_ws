"""Exercise real ROS action/service handover and independent dwell, no Gazebo truth."""
from pathlib import Path
import sys
import json
import math
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
import yaml
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer, CancelResponse
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import CameraInfo
from nav2_msgs.action import NavigateToPose, ComputePathToPose, Spin, DriveOnHeading
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'src/edgeai_navigation/scripts'))
from normal_rescue import Rescue


class FakeNavigation(Node):
    def __init__(self):
        super().__init__('rescue_test_backend')
        self.mode, self.sequence = 'AUTO', 0
        self.reject = False
        self.errors = []
        self.calls = []
        self.plan_errors=[];self.plan_calls=[];self.follow_goal=False
        self.follow_goal_after=1000000;self.spin_errors=[]
        self.arrival_policies = []
        self.pause_tf_after_abort_s = 0.
        self.tf_pause_until = 0.
        self.takeover_after_abort = False
        self.busy_after_abort = False
        self.phase = 'IDLE'
        self.update_file = None
        self.canceled_update = False
        self.search_file = None
        self.search_discover_at = 2
        self.spin_calls = []
        self.discover_on_spin = False
        self.alignment_motion=False
        self.reject_alignment=False
        self.hold_alignment=False
        self.advance_calls=[]
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(String, '/navigation_authority/state', qos)
        self.odometry = self.create_publisher(Odometry, '/odom', 10)
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', qos)
        self.map = OccupancyGrid()
        self.map.header.frame_id='map'
        self.map.info.width=self.map.info.height=70
        self.map.info.resolution=.1
        self.map.info.origin.position.x=self.map.info.origin.position.y=-1.
        self.map.info.origin.orientation.w=1.
        self.map.data=[0]*4900
        self.create_service(SetBool, '/navigation_authority/set_manual', self.manual)
        self.create_service(Trigger, '/navigation_authority/stop', self.stop)
        self.action = ActionServer(self, NavigateToPose, '/navigate_to_pose',
            lambda handle:self.navigate(handle,'position'),
            callback_group=ReentrantCallbackGroup(), cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.rescue_action = ActionServer(self, NavigateToPose, '/navigation/rescue_to_pose',
            lambda handle:self.navigate(handle,'rescue'),
            callback_group=ReentrantCallbackGroup(), cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.heading_action = ActionServer(self, NavigateToPose, '/navigation/heading_to_pose',
            lambda handle:self.navigate(handle,'heading'),
            callback_group=ReentrantCallbackGroup(), cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.spin_action = ActionServer(self, Spin, '/navigation_backend/spin', self.scan,
            callback_group=ReentrantCallbackGroup(), cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.advance_action = ActionServer(self, DriveOnHeading, '/navigation_backend/drive_on_heading', self.advance,
            callback_group=ReentrantCallbackGroup(), cancel_callback=lambda _:CancelResponse.ACCEPT)
        self.plan_action=ActionServer(self,ComputePathToPose,'/navigation_backend/compute_path_to_pose',self.plan,
            callback_group=ReentrantCallbackGroup(),cancel_callback=lambda _:CancelResponse.ACCEPT)
        self.tf = TransformBroadcaster(self)
        t = TransformStamped(); t.header.frame_id='map'; t.child_frame_id='base_footprint'
        t.header.stamp=self.get_clock().now().to_msg()
        t.transform.rotation.w=1.; t.transform.translation.x=1.; t.transform.translation.y=2.
        self.transform = t
        self.create_timer(.05, self.publish)

    def plan(self,handle):
        self.plan_calls.append(handle.request.goal)
        result=ComputePathToPose.Result()
        code=self.plan_errors.pop(0) if self.plan_errors else 0
        result.error_code=code
        if code:
            result.error_msg='test planning failure';handle.abort()
        else:
            result.path.header.frame_id='map';result.path.poses=[handle.request.goal];handle.succeed()
        return result

    def manual(self, request, response):
        self.mode='MANUAL'; self.sequence+=1
        response.success=True
        return response

    def stop(self, request, response):
        self.mode='STOPPED'; self.sequence+=1
        response.success=True
        return response

    def navigate(self, handle, arrival):
        self.arrival_policies.append(arrival)
        self.calls.append((time.monotonic(), handle.request.pose))
        result = NavigateToPose.Result()
        if self.search_file:
            self.transform.transform.translation.x=handle.request.pose.pose.position.x
            self.transform.transform.translation.y=handle.request.pose.pose.position.y
            self.transform.transform.rotation=handle.request.pose.pose.orientation
            if len(self.calls) == self.search_discover_at:
                Path(self.search_file).write_text(json.dumps(dict(frame_id='map',targets=[
                    *getattr(self,'retained_search_targets',[]),
                    dict(id=getattr(self,'search_target_id',1),type=getattr(self,'search_target_type','normal'),x=handle.request.pose.pose.position.x+.7,
                         y=handle.request.pose.pose.position.y,yaw=3.141592653589793,spread_m=.01)])))
                deadline=time.monotonic()+5
                while not handle.is_cancel_requested and time.monotonic()<deadline:
                    time.sleep(.01)
                if handle.is_cancel_requested:
                    self.canceled_update=True
                    handle.canceled()
                    return result
        if self.update_file and len(self.calls) == 1:
            registry = dict(frame_id='map', targets=[
                dict(id=1, type='normal', x=2.7, y=2., yaw=3.141592653589793, spread_m=.01),
                dict(id=3, type='normal', x=9., y=9., yaw=0., spread_m=.001, observations=100)])
            Path(self.update_file).write_text(json.dumps(registry))
            deadline = time.monotonic()+5
            while not handle.is_cancel_requested and time.monotonic() < deadline:
                time.sleep(.01)
            if handle.is_cancel_requested:
                self.canceled_update = True
                handle.canceled()
                return result
        if self.update_file and len(self.calls) > 1:
            self.transform.transform.translation.x=2.
        error = self.errors.pop(0) if self.errors else 0
        if self.reject or error:
            handle.abort()
            result.error_code = error
            self.tf_pause_until = time.monotonic()+self.pause_tf_after_abort_s
            if self.takeover_after_abort:
                self.mode='AUTO'; self.sequence+=1
            if self.busy_after_abort:
                self.phase='ACTIVE'
        else:
            if self.follow_goal or len(self.calls)>=self.follow_goal_after:
                self.transform.transform.translation.x=handle.request.pose.pose.position.x
                self.transform.transform.translation.y=handle.request.pose.pose.position.y
                self.transform.transform.rotation=handle.request.pose.pose.orientation
            handle.succeed()
        return result

    def scan(self, handle):
        self.spin_calls.append(handle.request.target_yaw)
        result = Spin.Result()
        error=self.spin_errors.pop(0) if self.spin_errors else 0
        if error:
            handle.abort();result.error_code=error
            return result
        if self.discover_on_spin:
            Path(self.search_file).write_text(json.dumps(dict(frame_id='map',targets=[
                dict(id=1,type='normal',x=self.transform.transform.translation.x+.7,
                     y=self.transform.transform.translation.y,yaw=math.pi,spread_m=.01)])))
            deadline = time.monotonic()+5.
            while not handle.is_cancel_requested and time.monotonic()<deadline:
                time.sleep(.01)
            if handle.is_cancel_requested:
                handle.canceled()
                return result
        if self.alignment_motion:
            q=self.transform.transform.rotation
            yaw=2*math.atan2(q.z,q.w)+handle.request.target_yaw
            q.z=math.sin(yaw/2);q.w=math.cos(yaw/2)
            if self.hold_alignment:
                until=time.monotonic()+.4
                while time.monotonic()<until and not handle.is_cancel_requested:time.sleep(.01)
                if handle.is_cancel_requested:
                    handle.canceled();return result
        handle.succeed()
        return result

    def advance(self,handle):
        self.advance_calls.append(handle.request.target.x)
        result=DriveOnHeading.Result()
        if self.reject_alignment:
            handle.abort();result.error_code=DriveOnHeading.Result.COLLISION_AHEAD
            return result
        q=self.transform.transform.rotation;yaw=2*math.atan2(q.z,q.w)
        self.transform.transform.translation.x+=handle.request.target.x*math.cos(yaw)
        self.transform.transform.translation.y+=handle.request.target.x*math.sin(yaw)
        handle.succeed()
        return result

    def publish(self):
        self.map.header.stamp=self.get_clock().now().to_msg()
        self.map_publisher.publish(self.map)
        # Keep asynchronous pairing, with headroom below the 150 ms freshness
        # limit so executor scheduling is not the subject of these action tests.
        self.transform.header.stamp = rclpy.time.Time(nanoseconds=self.get_clock().now().nanoseconds-50_000_000).to_msg()
        if time.monotonic() >= self.tf_pause_until:
            self.tf.sendTransform(self.transform)
        self.publisher.publish(String(data=json.dumps(dict(mode=self.mode, phase=self.phase,
            command_sequence=self.sequence, explorer_paused=self.mode!='AUTO'))))
        msg=Odometry();msg.header.stamp=self.get_clock().now().to_msg()
        msg.header.frame_id='odom';msg.child_frame_id='base_footprint'
        msg.pose.pose.orientation.w=1.
        self.odometry.publish(msg)


class RescueROS(unittest.TestCase):
    def test_task_circle_does_not_chase_offset_point_and_collision_can_retry(self):
        rclpy.init();fake=FakeNavigation();fake.alignment_motion=True
        executor=MultiThreadedExecutor(num_threads=4);executor.add_node(fake)
        thread=threading.Thread(target=executor.spin);thread.start()
        c=yaml.safe_load((Path(__file__).resolve().parents[2]/'src/edgeai_navigation/config/normal_rescue.yaml').read_text())
        c.update(target_id=1,normal_offset_m=.35,target_radius_m=.5,alignment_enabled=True,
                 dwell_s=.15,wall_timeout_s=20.,navigation_retry_delay_s=.1,rescue_max_retries=1)
        try:
            with tempfile.TemporaryDirectory() as tmp:
                (Path(tmp)/'targets.json').write_text(json.dumps(dict(frame_id='map',targets=[
                    dict(id=1,type='normal',x=1.457,y=2.,yaw=math.pi,spread_m=.01)])))
                for mode in ('inside_circle','collision_retry','collision_exhausted','navigation_failed_inside'):
                    fake.calls=[];fake.spin_calls=[];fake.spin_errors=[];fake.reject=False;fake.follow_goal_after=1000000
                    fake.transform.transform.translation.x=1.;fake.transform.transform.translation.y=2.
                    q=fake.transform.transform.rotation;q.z=0.;q.w=1.
                    if mode.startswith('collision'):
                        q.z=math.sin(math.pi/4);q.w=math.cos(math.pi/4)
                        fake.spin_errors=[703]*5
                        if mode=='collision_retry':fake.follow_goal_after=2
                    if mode=='navigation_failed_inside':fake.reject=True
                    node=Rescue(c,tmp)
                    try:
                        if mode=='collision_exhausted':
                            with self.assertRaisesRegex(RuntimeError,'Rescue retries exhausted'):
                                node.execute()
                            self.assertEqual(len(fake.calls),2)
                        else:
                            result=node.execute();self.assertTrue(result['success'])
                            self.assertLess(result['measurement']['rescue_distance_m'],.5)
                            if mode=='inside_circle':
                                self.assertGreater(result['measurement']['distance_m'],.1)
                                self.assertFalse(fake.spin_calls);self.assertFalse(fake.advance_calls)
                            elif mode=='collision_retry':
                                self.assertEqual(len(fake.calls),2)
                                self.assertEqual(node.alignment_attempts[0]['error_code'],703)
                                self.assertEqual(node.rescue_attempts[0]['phase'],'correction')
                            else:
                                self.assertEqual(len(fake.calls),3)
                                self.assertFalse(node.rescue_attempts[-1]['navigate'])
                    finally:node.finish();node.destroy_node()
        finally:
            executor.shutdown();thread.join(timeout=5.);fake.destroy_node();rclpy.shutdown()

    def test_goal_failure_selects_another_checked_approach_for_same_target(self):
        rclpy.init();fake=FakeNavigation();fake.follow_goal=True
        fake.plan_errors=[206,0,0];fake.errors=[208,0]
        executor=MultiThreadedExecutor(num_threads=4);executor.add_node(fake)
        thread=threading.Thread(target=executor.spin);thread.start()
        c=yaml.safe_load((Path(__file__).resolve().parents[2]/'src/edgeai_navigation/config/normal_rescue.yaml').read_text())
        c.update(goal_source='detected_target',target_id=1,dwell_s=.15,wall_timeout_s=25.,navigation_retry_delay_s=.1,
                 approach_selection=dict(enabled=True,angles_deg=[0,30,-30],max_candidates=6))
        try:
            with tempfile.TemporaryDirectory() as tmp:
                root=Path(__file__).resolve().parents[2]
                (Path(tmp)/'nav2-runtime.yaml').write_text((root/'src/edgeai_navigation/config/nav2_exploration_rectangle.yaml').read_text())
                (Path(tmp)/'targets.json').write_text(json.dumps(dict(frame_id='map',targets=[
                    dict(id=1,type='normal',x=1.7,y=2.,yaw=math.pi,spread_m=.01,confirmed=True)])))
                node=Rescue(c,tmp)
                try:
                    result=node.execute()
                    self.assertTrue(result['success']);self.assertEqual(result['target_id'],1)
                    self.assertEqual(len(fake.calls),2);self.assertEqual(len(fake.plan_calls),3)
                    self.assertEqual(node.approach_attempts[0]['reason'],'goal_occupied')
                    self.assertEqual(node.navigation_attempts[0]['failure_reason'],'no_valid_path')
                    self.assertNotEqual(fake.calls[0][1].pose.position.y,fake.calls[1][1].pose.position.y)
                    self.assertEqual(fake.arrival_policies,['rescue','rescue'])
                finally:node.finish();node.destroy_node()
        finally:
            executor.shutdown();thread.join(timeout=5.);fake.destroy_node();rclpy.shutdown()

    def test_shim_rescue_then_fallback_rotation_rechecks_and_cancels_spin(self):
        rclpy.init();fake=FakeNavigation();fake.alignment_motion=True;fake.hold_alignment=True
        q=fake.transform.transform.rotation;q.z=math.sin(math.pi/4);q.w=math.cos(math.pi/4)
        executor=MultiThreadedExecutor(num_threads=4);executor.add_node(fake)
        thread=threading.Thread(target=executor.spin);thread.start()
        c=yaml.safe_load((Path(__file__).resolve().parents[2]/'src/edgeai_navigation/config/normal_rescue.yaml').read_text())
        c.update(alignment_enabled=True,dwell_s=.15,wall_timeout_s=20.)
        try:
            with tempfile.TemporaryDirectory() as tmp:
                node=Rescue(c,tmp)
                try:
                    result=node.execute(dict(target_id=1,x=1.,y=2.,yaw=0.,frame_id='map'))
                    self.assertTrue(result['success'])
                    self.assertEqual(fake.arrival_policies,['rescue'])
                    self.assertFalse(fake.advance_calls)
                    self.assertTrue(any(a.get('cancel_reason')=='conditions_changed' and a['status']==5
                                        for a in node.alignment_attempts))
                finally:node.finish();node.destroy_node()
        finally:
            executor.shutdown();thread.join(timeout=5);fake.destroy_node();rclpy.shutdown()

    def test_final_correction_turns_steps_and_checks_dwell_with_limits(self):
        rclpy.init();fake=FakeNavigation();fake.alignment_motion=True
        executor=MultiThreadedExecutor(num_threads=4);executor.add_node(fake)
        thread=threading.Thread(target=executor.spin);thread.start()
        c=yaml.safe_load((Path(__file__).resolve().parents[2]/'src/edgeai_navigation/config/normal_rescue.yaml').read_text())
        c.update(alignment_enabled=True,dwell_s=.15,wall_timeout_s=20.)
        try:
            with tempfile.TemporaryDirectory() as tmp:
                for mode in ('success','rejected','budget'):
                    fake.transform.transform.translation.x=1.;fake.transform.transform.translation.y=2.
                    q=fake.transform.transform.rotation;q.z=math.sin(math.pi/4);q.w=math.cos(math.pi/4)
                    fake.reject_alignment=mode=='rejected';fake.advance_calls=[];fake.spin_calls=[];fake.arrival_policies=[]
                    node=Rescue(dict(c,alignment_max_actions=1 if mode=='budget' else 20),tmp)
                    try:
                        goal=dict(target_id=1,x=1.106,y=2.,yaw=math.pi/2,frame_id='map')
                        if mode=='success':
                            result=node.execute(goal)
                            self.assertTrue(result['success'])
                            self.assertEqual(fake.arrival_policies,['rescue'])
                            self.assertEqual(node.navigation_attempts[0]['arrival_policy'],'rescue')
                            self.assertLess(result['measurement']['distance_m'],c['position_tolerance_m'])
                            self.assertLess(abs(result['measurement']['yaw_error_rad']),c['yaw_tolerance_rad'])
                            self.assertTrue(fake.advance_calls)
                            self.assertTrue(all(0<d<=.03 for d in fake.advance_calls))
                            self.assertLess(fake.spin_calls[0],0.)
                            self.assertGreater(fake.spin_calls[-1],0.)
                        else:
                            with self.assertRaisesRegex(RuntimeError,'correction failed' if mode=='rejected' else 'budget exhausted'):
                                node.execute(goal)
                    finally:node.finish();node.destroy_node()
        finally:
            executor.shutdown();thread.join(timeout=5);fake.destroy_node();rclpy.shutdown()

    def test_suspect_round_has_priority_and_confirms_target(self):
        rclpy.init()
        fake=FakeNavigation()
        info=CameraInfo();info.header.frame_id='camera_optical_frame'
        info.width=640;info.height=480;info.k=[400.,0.,320.,0.,400.,240.,0.,0.,1.]
        info_pub=fake.create_publisher(CameraInfo,'/camera/camera_info',10)
        fake.create_timer(.05,lambda:info_pub.publish(info))
        mounting=TransformStamped();mounting.header.frame_id='base_footprint'
        mounting.child_frame_id='camera_optical_frame';mounting.transform.translation.z=.35
        mounting.transform.rotation.x=-.5;mounting.transform.rotation.y=.5
        mounting.transform.rotation.z=-.5;mounting.transform.rotation.w=.5
        static=StaticTransformBroadcaster(fake);static.sendTransform(mounting)
        pub=fake.create_publisher(String,'/targets/suspected',
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        def hints():
            now=fake.get_clock().now().nanoseconds/1e9
            pub.publish(String(data=json.dumps(dict(frame_id='map',stamp=now,candidates=[
                dict(id=0,position=[3.,2.,.25],normal=[1.,0.],confidence=.8,last_seen=now)]))))
        fake.create_timer(.05,hints)
        executor=MultiThreadedExecutor(num_threads=4);executor.add_node(fake)
        thread=threading.Thread(target=executor.spin);thread.start()
        c=yaml.safe_load((Path(__file__).resolve().parents[2]/'src/edgeai_navigation/config/normal_rescue.yaml').read_text())
        c.update(dwell_s=.2,wall_timeout_s=30.,search_suspects=dict(enabled=True,max_positions=2))
        try:
            with tempfile.TemporaryDirectory() as tmp:
                fake.search_file=Path(tmp)/'targets.json'
                fake.search_file.write_text(json.dumps(dict(frame_id='map',targets=[])))
                node=Rescue(c,tmp)
                try:
                    node.wait(lambda:node.search_suspects is not None,timeout=5.)
                    result=node.execute()
                    self.assertTrue(result['success'])
                    self.assertEqual([a['candidate']['strategy'] for a in node.search_attempts],
                                     ['suspected_panel','suspected_panel'])
                    self.assertEqual(fake.arrival_policies,['heading','heading','rescue'])
                    self.assertFalse(node.search_active)
                finally:
                    node.finish();node.destroy_node()
        finally:
            executor.shutdown();thread.join(timeout=5);fake.destroy_node();rclpy.shutdown()

    def test_frustum_falls_back_without_calibration(self):
        from search_candidates import make_candidates
        rclpy.init()
        c=yaml.safe_load((Path(__file__).resolve().parents[2]/'src/edgeai_navigation/config/normal_rescue.yaml').read_text())
        c['search_frustum']=dict(enabled=True)
        try:
            with tempfile.TemporaryDirectory() as tmp:
                node=Rescue(c,tmp)
                try:
                    grid=OccupancyGrid()
                    grid.info.width=grid.info.height=40
                    grid.info.resolution=.1
                    grid.data=[0]*1600
                    candidates=make_candidates(node,grid,dict(x=1.,y=1.,yaw=.7))
                    view=candidates.next_view((1.,1.,.7))
                    self.assertEqual(view['strategy'],'spatial')
                    self.assertEqual(view['yaw'],.7)
                finally:
                    node.destroy_node()
        finally:
            rclpy.shutdown()

    def test_frustum_candidates_feed_existing_search_and_rescue(self):
        rclpy.init()
        fake=FakeNavigation()
        planner=Node('planner_server')
        planner.declare_parameter('GridBased.use_final_approach_orientation',True)
        info=CameraInfo()
        info.header.frame_id='camera_optical_frame'
        info.width=640;info.height=480
        info.k=[400.,0.,320.,0.,400.,240.,0.,0.,1.]
        publisher=fake.create_publisher(CameraInfo,'/camera/camera_info',10)
        fake.create_timer(.05,lambda:publisher.publish(info))
        mounting=TransformStamped()
        mounting.header.frame_id='base_footprint'
        mounting.child_frame_id='camera_optical_frame'
        mounting.transform.translation.z=.35
        q=mounting.transform.rotation
        q.x,q.y,q.z,q.w=-.5,.5,-.5,.5
        mounting.header.stamp=fake.get_clock().now().to_msg()
        static=StaticTransformBroadcaster(fake);static.sendTransform(mounting)
        executor=MultiThreadedExecutor(num_threads=4)
        executor.add_node(fake);executor.add_node(planner)
        thread=threading.Thread(target=executor.spin);thread.start()
        c=yaml.safe_load((Path(__file__).resolve().parents[2]/'src/edgeai_navigation/config/normal_rescue.yaml').read_text())
        c.update(dwell_s=.2,wall_timeout_s=30.,search_frustum=dict(enabled=True,candidate_pool_size=6))
        try:
            with tempfile.TemporaryDirectory() as tmp:
                fake.search_file=Path(tmp)/'targets.json'
                fake.search_file.write_text(json.dumps(dict(frame_id='map',targets=[])))
                node=Rescue(c,tmp)
                try:
                    result=node.execute()
                    self.assertTrue(result['success'])
                    self.assertEqual(len(node.search_attempts),2)
                    first,second=[r['candidate'] for r in node.search_attempts]
                    self.assertEqual(first['strategy'],'frustum')
                    self.assertGreater(first['gain_m2'],0)
                    self.assertGreater(second['coverage_keys'],0)
                    self.assertEqual(len(fake.spin_calls),1)
                    self.assertEqual(fake.arrival_policies,['position','position','rescue'])
                finally:
                    node.finish()
                    self.assertTrue(planner.get_parameter('GridBased.use_final_approach_orientation').value)
                    node.destroy_node()
        finally:
            executor.shutdown();thread.join(timeout=5)
            fake.destroy_node();planner.destroy_node();rclpy.shutdown()

    def test_target_registry_tolerates_file_replacement_and_prefers_live_topic(self):
        rclpy.init()
        c=yaml.safe_load((Path(__file__).resolve().parents[2]/'src/edgeai_navigation/config/normal_rescue.yaml').read_text())
        try:
            with tempfile.TemporaryDirectory() as tmp:
                node=Rescue(c,tmp)
                try:
                    old=dict(target_id=1,x=1.,y=2.,yaw=0.,frame_id='map')
                    with patch.object(Path,'read_text',side_effect=FileNotFoundError('replacement window')):
                        with self.assertRaisesRegex(ValueError,'No eligible'):
                            node.select_current_goal()
                        self.assertEqual(node.latest_goal(old),old)
                    (Path(tmp)/'targets.json').write_text('{')
                    with self.assertRaisesRegex(ValueError,'No eligible'):
                        node.select_current_goal()
                    node.targets_received(String(data=json.dumps(dict(frame_id='map',targets=[
                        dict(id=1,type='normal',x=2.,y=2.,yaw=0.,spread_m=.01)]))))
                    with patch.object(Path,'read_text',side_effect=FileNotFoundError('replacement window')):
                        self.assertAlmostEqual(node.select_current_goal()['x'],2.+c['normal_offset_m'])
                        self.assertAlmostEqual(node.latest_goal(old)['x'],2.+c['normal_offset_m'])
                finally:
                    node.finish();node.destroy_node()
        finally:
            rclpy.shutdown()

    def test_search_moves_views_then_finds_target_and_rescues(self):
        rclpy.init()
        fake=FakeNavigation()
        planner=Node('planner_server')
        planner.declare_parameter('GridBased.use_final_approach_orientation',True)
        executor=MultiThreadedExecutor(num_threads=4)
        executor.add_node(fake); executor.add_node(planner)
        thread=threading.Thread(target=executor.spin); thread.start()
        c=yaml.safe_load((Path(__file__).resolve().parents[2]/'src/edgeai_navigation/config/normal_rescue.yaml').read_text())
        c.update(dwell_s=.2,wall_timeout_s=25.)
        try:
            with tempfile.TemporaryDirectory() as tmp:
                fake.search_file=Path(tmp)/'targets.json'
                # A cooperative target must not satisfy the normal rescue search.
                fake.search_file.write_text(json.dumps(dict(frame_id='map',targets=[
                    dict(id=2,type='cooperative',x=1.,y=2.,yaw=0.,spread_m=.01)])))
                node=Rescue(c,tmp)
                try:
                    result=node.execute()
                    self.assertTrue(result['success'])
                    self.assertEqual(result['target_id'],1)
                    self.assertEqual(len(node.search_attempts),2)
                    self.assertEqual(len(fake.spin_calls),1)
                    self.assertAlmostEqual(fake.spin_calls[0],2*math.pi,places=5)
                    self.assertEqual(len(node.navigation_attempts),1)
                    self.assertTrue(fake.canceled_update)
                    self.assertEqual(node.search_attempts[-1]['status'],5)
                    self.assertNotEqual(node.search_attempts[0]['x'],node.search_attempts[1]['x'])
                finally:
                    node.finish(); node.destroy_node()
                # A failed observation approach retries the same point/yaw
                # after fresh data, rather than silently consuming another point.
                fake.calls=[]; fake.errors=[102,0]; fake.search_discover_at=100
                fake.pause_tf_after_abort_s=.4
                fake.search_file.write_text(json.dumps(dict(frame_id='map',targets=[])))
                node=Rescue(dict(c,search_max_positions=1,
                                 navigation_retry_delay_s=.1),tmp)
                try:
                    with self.assertRaisesRegex(RuntimeError,'exhausted observation positions'):
                        node.execute()
                    self.assertEqual(len(node.search_attempts),2)
                    self.assertEqual([r['error_code'] for r in node.search_attempts[:2]],[102,0])
                    self.assertEqual(fake.calls[0][1].pose,fake.calls[1][1].pose)
                    self.assertGreaterEqual(fake.calls[1][0]-fake.calls[0][0],.35)
                finally:
                    node.finish();node.destroy_node()
                    fake.pause_tf_after_abort_s=0.
                # No detection: visits and total search time remain bounded.
                fake.calls=[]; fake.search_discover_at=100
                fake.search_file.write_text(json.dumps(dict(frame_id='map',targets=[])))
                node=Rescue(dict(c,search_max_positions=1),tmp)
                try:
                    with self.assertRaisesRegex(RuntimeError,'exhausted observation positions'):
                        node.execute()
                    self.assertEqual(len(node.search_attempts),1)
                    self.assertEqual(node.search_attempts[0]['spin']['status'],4)
                    self.assertEqual(len(node.navigation_attempts),0)
                finally:
                    node.finish(); node.destroy_node()
                # Detection during a sweep cancels Spin before rescue navigation.
                fake.calls=[]; fake.discover_on_spin=True
                fake.search_file.write_text(json.dumps(dict(frame_id='map',targets=[])))
                node=Rescue(dict(c,search_max_positions=1),tmp)
                try:
                    self.assertTrue(node.execute()['success'])
                    self.assertEqual(len(node.search_attempts),1)
                    self.assertEqual(node.search_attempts[0]['spin']['status'],5)
                    self.assertEqual(node.search_attempts[0]['spin']['cancel_reason'],'target_found')
                finally:
                    node.finish(); node.destroy_node()
                    fake.discover_on_spin=False
                fake.calls=[]
                fake.search_file.write_text(json.dumps(dict(frame_id='map',targets=[])))
                node=Rescue(dict(c,search_timeout_s=.05),tmp)
                try:
                    with self.assertRaisesRegex(RuntimeError,'budget exhausted'):
                        node.execute()
                    self.assertEqual(len(node.navigation_attempts),0)
                finally:
                    node.finish(); node.destroy_node()
                fake.calls=[]; fake.errors=[102]; fake.takeover_after_abort=True
                node=Rescue(c,tmp)
                try:
                    with self.assertRaisesRegex(RuntimeError,'authority changed'):
                        node.execute()
                    self.assertEqual(len(fake.calls),1)
                finally:
                    node.finish(); node.destroy_node()
                self.assertEqual(fake.mode,'AUTO')
        finally:
            executor.shutdown(); thread.join(timeout=5)
            fake.destroy_node(); planner.destroy_node(); rclpy.shutdown()

    def test_latest_target_replaces_active_goal_after_terminal(self):
        rclpy.init()
        fake=FakeNavigation()
        planner=Node('planner_server')
        planner.declare_parameter('GridBased.use_final_approach_orientation', True)
        executor=MultiThreadedExecutor(num_threads=4)
        executor.add_node(fake); executor.add_node(planner)
        thread=threading.Thread(target=executor.spin); thread.start()
        c=yaml.safe_load((Path(__file__).resolve().parents[2]/'src/edgeai_navigation/config/normal_rescue.yaml').read_text())
        c.update(dwell_s=.2, wall_timeout_s=20., navigation_retry_delay_s=.2,
                 target_update_interval_s=.1)
        try:
            with tempfile.TemporaryDirectory() as tmp:
                node=Rescue(c,tmp)
                fake.update_file=Path(tmp)/'targets.json'
                # Frozen goal is superseded even before the first dispatch.
                fake.update_file.write_text(json.dumps(dict(frame_id='map', targets=[
                    dict(id=1,type='normal',x=1.7,y=2.,yaw=3.141592653589793,spread_m=.01)])))
                goal=dict(target_id=1,x=9.,y=9.,yaw=0.,frame_id='map')
                try:
                    result=node.execute(goal)
                    self.assertTrue(result['success'])
                    self.assertTrue(fake.canceled_update)
                    self.assertEqual(len(fake.calls),2)
                    self.assertAlmostEqual(fake.calls[0][1].pose.position.x,1.)
                    self.assertAlmostEqual(fake.calls[1][1].pose.position.x,2.)
                    self.assertEqual(goal['target_id'],1)
                    self.assertEqual(node.navigation_attempts[0]['status'],5)
                    self.assertTrue(node.navigation_attempts[0]['target_updated'])
                    self.assertAlmostEqual(result['goal']['x'],2.)
                finally:
                    node.finish(); node.destroy_node()
        finally:
            executor.shutdown(); thread.join(timeout=5)
            fake.destroy_node(); planner.destroy_node(); rclpy.shutdown()

    def test_retry_recovery_limits_and_ownership(self):
        rclpy.init()
        fake=FakeNavigation()
        planner=Node('planner_server')
        planner.declare_parameter('GridBased.use_final_approach_orientation', True)
        executor=SingleThreadedExecutor(); executor.add_node(fake); executor.add_node(planner)
        thread=threading.Thread(target=executor.spin); thread.start()
        config=Path(__file__).resolve().parents[2]/'src/edgeai_navigation/config/normal_rescue.yaml'
        c=yaml.safe_load(config.read_text())
        c.update(dwell_s=.2, wall_timeout_s=20., navigation_retry_delay_s=.25,
                 navigation_retry_wait_s=1.5)
        goal=dict(target_id=1,x=1.,y=2.,yaw=0.,frame_id='map')
        try:
            with tempfile.TemporaryDirectory() as tmp:
                # TF_ERROR followed by fresh TF and success keeps the same goal.
                fake.errors=[102,0]; fake.pause_tf_after_abort_s=.7
                node=Rescue(c,tmp)
                try:
                    result=node.execute(goal)
                    self.assertTrue(result['success'])
                    self.assertEqual(len(fake.calls),2)
                    self.assertGreaterEqual(fake.calls[1][0]-fake.calls[0][0], .6)
                    self.assertEqual([r['error_code'] for r in node.navigation_attempts],[102,0])
                    self.assertEqual(fake.calls[0][1].pose, fake.calls[1][1].pose)
                finally:
                    node.finish(); node.destroy_node()

                # Persistent failure stops after initial attempt + 2 retries.
                fake.calls=[]; fake.errors=[]; fake.reject=True; fake.pause_tf_after_abort_s=0.
                fake.transform.transform.translation.x=0.
                node=Rescue(dict(c,rescue_max_retries=0),tmp)
                try:
                    with self.assertRaisesRegex(RuntimeError,'Navigation failed after 3'):
                        node.execute(goal)
                    self.assertEqual(len(fake.calls),3)
                finally:
                    node.finish(); node.destroy_node()
                    fake.transform.transform.translation.x=1.

                # No fresh TF / still-busy authority must never admit another goal.
                for pause, busy in ((10.,False),(0.,True)):
                    fake.calls=[]; fake.errors=[102]; fake.reject=False
                    fake.pause_tf_after_abort_s=pause; fake.busy_after_abort=busy; fake.phase='IDLE'
                    node=Rescue(c,tmp)
                    try:
                        with self.assertRaisesRegex(RuntimeError,'retry readiness timeout'):
                            node.execute(goal)
                        self.assertEqual(len(fake.calls),1)
                    finally:
                        node.finish(); node.destroy_node()
                        fake.tf_pause_until=0.; fake.phase='IDLE'

                # An operator takeover cancels our right to retry or stop AUTO.
                fake.calls=[]; fake.errors=[102]; fake.busy_after_abort=False
                fake.pause_tf_after_abort_s=0.; fake.takeover_after_abort=True
                node=Rescue(c,tmp)
                try:
                    with self.assertRaisesRegex(RuntimeError,'authority changed'):
                        node.execute(goal)
                    self.assertEqual(len(fake.calls),1)
                finally:
                    node.finish(); node.destroy_node()
                self.assertEqual(fake.mode,'AUTO')

                # All attempts share the original simulation-time budget.
                fake.calls=[]; fake.errors=[102]; fake.takeover_after_abort=False
                node=Rescue(dict(c,navigation_timeout_s=.1),tmp)
                try:
                    with self.assertRaisesRegex(RuntimeError,'simulation timeout'):
                        node.execute(goal)
                    self.assertEqual(len(fake.calls),1)
                finally:
                    node.finish(); node.destroy_node()
        finally:
            executor.shutdown(); thread.join(timeout=5)
            fake.destroy_node(); planner.destroy_node(); rclpy.shutdown()

    def test_managed_action_and_dwell_and_failure(self):
        rclpy.init()
        fake=FakeNavigation()
        planner=Node('planner_server')
        planner.declare_parameter('GridBased.use_final_approach_orientation', True)
        executor=SingleThreadedExecutor();executor.add_node(fake);executor.add_node(planner)
        thread=threading.Thread(target=executor.spin);thread.start()
        c=yaml.safe_load((Path(__file__).resolve().parents[2]/'src/edgeai_navigation/config/normal_rescue.yaml').read_text())
        c.update(dwell_s=.2, wall_timeout_s=20.)
        try:
            with tempfile.TemporaryDirectory() as tmp:
                node=Rescue(c,tmp)
                try:
                    result=node.execute(dict(target_id=1,x=1.,y=2.,yaw=0.,frame_id='map'))
                    self.assertTrue(result['success'])
                    self.assertEqual(result['task_state'], 'completed')
                    self.assertLess(result['measurement']['distance_m'], .001)
                finally:
                    node.finish();node.destroy_node()
                fake.reject=True
                fake.transform.transform.translation.x=0.
                node=Rescue(dict(c,rescue_max_retries=0),tmp)
                try:
                    with self.assertRaisesRegex(RuntimeError, 'Navigation failed'):
                        node.execute(dict(target_id=1,x=1.,y=2.,yaw=0.,frame_id='map'))
                finally:
                    node.finish();node.destroy_node()
        finally:
            executor.shutdown();thread.join(timeout=5)
            fake.destroy_node();planner.destroy_node();rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
