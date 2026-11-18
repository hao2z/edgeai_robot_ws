#!/usr/bin/env python3
"""Shared navigation recovery: ask a confirmed idle peer to release a route.

NavigateToPose is the transport for /[robot2/]navigation/yield_peer. Its pose
is obstruction context (the REQUESTER destination), never a requested rescue
parking point. Success means measured peer movement released that route.
"""
import json
import math
from pathlib import Path
import threading
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from ament_index_python.packages import get_package_share_directory
import yaml

from edgeai_nav.peer_yield import PeerYieldUnit


def topic(robot,name):return '/'+(robot+'/' if robot else '')+name


class PeerYieldServer(Node):
    def __init__(self, settings=None):
        super().__init__('peer_yield')
        self.c=settings or yaml.safe_load((Path(get_package_share_directory('edgeai_navigation'))/'config/peer_yield.yaml').read_text())
        self.group=ReentrantCallbackGroup()
        self.lock=threading.Lock();self.busy=False;self.fault=None
        self.requesters={};self.current=None;self.sequences={};self.epoch=None
        self.states={};self.received={};self.odom={};self.map=None
        self.handles=[];self.pending={};self.plan={};self.yielding_peer=None
        self.lightweight = self.c.get('lightweight', False)
        self.buffer = self.listener = None
        self.odom_subscriptions = []
        self.unit=PeerYieldUnit(self,self.pending)
        self.events=self.create_publisher(String,'/navigation/peer_yield/state',
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.servers=[]
        for robot in ('robot1','robot2'):
            self.create_subscription(String,topic(robot,'navigation_authority/state'),
                lambda msg,r=robot:self.receive_state(r,msg),
                QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.plan[robot]=ActionClient(self,ComputePathToPose,
                topic(robot,'navigation_backend/compute_path_to_pose'),callback_group=self.group)
            self.servers.append(ActionServer(self,NavigateToPose,topic(robot,'navigation/yield_peer'),
                execute_callback=self.execute,goal_callback=lambda request,r=robot:self.admit(r,request),
                cancel_callback=self.cancel_request,callback_group=self.group))
        self.create_subscription(OccupancyGrid,'/map',lambda msg:setattr(self,'map',msg),
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))

        if not self.lightweight:
            self.start_motion_inputs()

    def start_motion_inputs(self):
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        odom = self.odom = {}
        for robot in ('robot1', 'robot2'):
            subscription = self.create_subscription(
                Odometry, topic(robot, 'odom'),
                lambda msg, r=robot: odom.update({r: msg}), qos_profile_sensor_data)
            self.odom_subscriptions.append(subscription)

    def stop_motion_inputs(self):
        for subscription in self.odom_subscriptions:
            self.destroy_subscription(subscription)
        self.odom_subscriptions.clear()
        if self.listener is not None:
            self.listener.unregister()
        self.listener = self.buffer = None
        self.odom = {}

    def motion_inputs_ready(self):
        if self.map is None or len(self.odom) != 2:
            return False
        for robot, odom in self.odom.items():
            stamp = Time.from_msg(odom.header.stamp).nanoseconds / 1e9
            if not 0 <= self.now() - stamp <= self.c['max_data_age_s']:
                return False
            try:
                self.tf_pose(robot+'/base_footprint')
            except (RuntimeError, TransformException):
                return False
        return True

    def receive_state(self,robot,msg):
        try:state=json.loads(msg.data)
        except (TypeError,ValueError):return
        if isinstance(state,dict):self.states[robot]=state;self.received[robot]=time.monotonic()

    def now(self):return self.get_clock().now().nanoseconds/1e9

    def event(self,phase,**data):
        self.events.publish(String(data=json.dumps(dict(phase=phase,sim_time=self.now(),**data))))
        self.get_logger().info('PEER_'+phase+': '+json.dumps(data))

    def admit(self,robot,request):
        p=request.pose.pose.position
        if (request.pose.header.frame_id!='map' or request.behavior_tree or
                not all(math.isfinite(v) for v in (p.x,p.y))):return GoalResponse.REJECT
        with self.lock:
            # Includes recursive calls from the yielding peer's own navigation tree.
            if self.busy or self.fault:return GoalResponse.REJECT
            self.busy=True;self.requesters[id(request)]=robot
        return GoalResponse.ACCEPT

    def check_owner(self):
        if self.yielding_peer is None and self.current is not None and self.current.is_cancel_requested:
            raise RuntimeError('Peer yielding canceled by requester')
        for robot,sequence in self.sequences.items():
            # Committed peer navigation is owned by that robot. The requester's
            # odometry, task handover and goal changes cannot revoke its motion.
            if self.yielding_peer is not None and robot!=self.yielding_peer:continue
            state=self.states.get(robot,{})
            if (time.monotonic()-self.received.get(robot,0.)>2. or
                    state.get('command_sequence')!=sequence or
                    state.get('mode') != 'MANUAL'):
                raise RuntimeError('Yield ownership changed or state stale: '+(robot or 'robot1'))

    def cancel_request(self,handle):
        with self.lock:
            return CancelResponse.ACCEPT if self.yielding_peer is None else CancelResponse.REJECT

    def begin_yield(self,peer):
        with self.lock:
            if self.current.is_cancel_requested:
                raise RuntimeError('Peer yielding canceled before dispatch')
            self.yielding_peer=peer

    def tf_pose(self,frame):
        tf=self.buffer.lookup_transform('map',frame,Time())
        age=self.now()-Time.from_msg(tf.header.stamp).nanoseconds/1e9
        if not 0<=age<=self.c['max_data_age_s']:raise RuntimeError('Yield TF stale: '+frame)
        t,q=tf.transform.translation,tf.transform.rotation
        return [t.x,t.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]

    def goal_pose(self,pose):
        msg=PoseStamped();msg.header.frame_id='map';msg.header.stamp=self.get_clock().now().to_msg()
        msg.pose.position.x,msg.pose.position.y=map(float,pose[:2])
        msg.pose.orientation.z=math.sin(pose[2]/2);msg.pose.orientation.w=math.cos(pose[2]/2)
        return msg

    def wait(self,predicate,timeout=15.,ownership=False):
        end=time.monotonic()+timeout
        while rclpy.ok():
            if ownership:self.check_owner()
            if predicate():return
            if time.monotonic()>=end:raise RuntimeError('Peer yield wait timed out')
            time.sleep(.025)  # Sensor/action callbacks run on the other executor thread.
        raise RuntimeError('ROS shutting down')

    def action(self,client,goal,timeout):
        self.wait(client.server_is_ready,2.,True)
        sent=client.send_goal_async(goal);self.wait(sent.done,timeout,True)
        handle=sent.result()
        if not handle.accepted:raise RuntimeError('Native planner rejected yielding request')
        result=handle.get_result_async()
        try:self.wait(result.done,timeout,True)
        except Exception:
            handle.cancel_goal_async();raise
        return result.result()

    def cleanup_motion(self):
        for peer,(handle,result,_) in list(self.pending.items()):
            if not result.done():
                canceled=handle.cancel_goal_async()
                self.wait(lambda:canceled.done() and result.done(),8.)
            if result.result().status not in (4,5,6):raise RuntimeError('Yield terminal status unknown')
            self.pending.pop(peer)
            if handle in self.handles:self.handles.remove(handle)

    def execute(self,handle):
        robot=self.requesters.pop(id(handle.request));peer='robot2' if robot=='robot1' else 'robot1'
        self.current=handle;self.requester=robot
        result=NavigateToPose.Result();reason='';fatal=False
        try:
            self.sequences={r:self.states.get(r,{}).get('command_sequence') for r in (robot,peer)}
            self.check_owner()
            if self.lightweight:
                self.start_motion_inputs()
                self.wait(self.motion_inputs_ready, self.c.get('input_timeout_s', 2.0), True)
            if self.states.get(peer,{}).get('mode')!='MANUAL':
                raise RuntimeError('Peer is not task-controlled and idle')
            epoch=tuple(sorted(self.sequences.items()))
            if epoch!=self.epoch:self.unit.attempts.clear();self.epoch=epoch
            p=handle.request.pose.pose.position
            self.unit.diagnose([dict(robot=robot,goal=[p.x,p.y,0.])])
            self.unit.select();self.unit.dispatch()
            while not self.unit.poll()['complete']:
                time.sleep(.05)
        except Exception as error:
            reason=str(error);fatal=getattr(error,'category','')=='fatal'
        try:self.cleanup_motion()
        except Exception as error:reason=str(error);fatal=True
        if self.lightweight:
            self.stop_motion_inputs()
        self.unit.context=None
        if fatal:self.fault=reason
        result.error_code=902 if fatal else 901 if reason else 0
        result.error_msg=reason
        self.event('YIELD_FINISHED',requester=robot or 'robot1',peer=peer or 'robot1',
                   success=not reason,reason=reason,fatal=fatal)
        if handle.is_cancel_requested:handle.canceled()
        elif reason:handle.abort()
        else:handle.succeed()
        self.current=None;self.sequences={}
        with self.lock:self.yielding_peer=None;self.busy=False
        return result


def main():
    rclpy.init();node=PeerYieldServer();executor=MultiThreadedExecutor(num_threads=2);executor.add_node(node)
    try:executor.spin()
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        executor.shutdown();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
