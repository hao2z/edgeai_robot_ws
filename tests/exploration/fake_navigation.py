"""Shared fake native navigation backend for current task-tree ROS tests."""
import json
import math
import time
import yaml
from pathlib import Path
import rclpy
from rclpy.node import Node
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


class FakeNavigation(Node):
    def __init__(self, robot='robot1'):
        endpoints = ['navigation/assigned_frontier_to_pose','navigation_authority/state','odom','navigation_authority/set_manual',
            'navigation_authority/stop','navigate_to_pose','navigation/rescue_to_pose',
            'navigation/heading_to_pose','navigation_backend/spin',
            'navigation_backend/drive_on_heading','navigation_backend/compute_path_to_pose']
        remaps = [arg for topic in endpoints for arg in ('-r','/'+topic+':=/'+robot+'/'+topic)]
        super().__init__('rescue_test_backend_'+robot, cli_args=['--ros-args']+remaps)
        self.mode, self.sequence = 'STOPPED', 0
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
        self.assigned_action = ActionServer(self, NavigateToPose, '/navigation/assigned_frontier_to_pose',
            lambda handle:self.navigate(handle,'position'), callback_group=ReentrantCallbackGroup(),
            cancel_callback=lambda _:CancelResponse.ACCEPT)
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
        t = TransformStamped(); t.header.frame_id='map'; t.child_frame_id=robot+'/base_footprint'
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
                self.mode='STOPPED'; self.sequence+=1
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
            command_sequence=self.sequence, explorer_paused=True))))
        msg=Odometry();msg.header.stamp=self.get_clock().now().to_msg()
        msg.header.frame_id='odom';msg.child_frame_id='base_footprint'
        msg.pose.pose.orientation.w=1.
        self.odometry.publish(msg)
