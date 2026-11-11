"""Read-only motion/recovery evidence; never dispatch, retry, cancel or release a slot."""
from dataclasses import asdict
import math
from action_msgs.msg import GoalStatus,GoalStatusArray
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from authority_policy import RecoveryEvidence


class NavigationTelemetry:
    def __init__(self,node):
        self.node=node;self.pose=None;self.pose_time=0.;self.odom_frame=None
        self.following=False;self.recovery=None;self.recoveries={};self.seen={}
        self.subscriptions=[node.create_subscription(Odometry,'odom',self.odometry,qos_profile_sensor_data)]
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for kind in ('backup','drive_on_heading','escape_occupied','spin','wait','follow_path'):
            self.subscriptions.append(node.create_subscription(GoalStatusArray,
                'navigation_backend/'+kind+'/_action/status',lambda msg,kind=kind:self.action_status(kind,msg),qos))

    def reset(self):
        self.following=False;self.recovery=None;self.recoveries.clear()

    def snapshot(self):return asdict(self.recovery) if self.recovery else None

    def note_navigation(self,succeeded=False):
        if self.recovery and self.recovery.note_navigation(
                self.node.distance,self.following,self.fresh_pose(),succeeded):
            self.node.event('NAVIGATION_RESTORED',
                '恢复动作后原任务已完成' if succeeded else '恢复动作有实际位移，且原任务重新行驶并接近目标',
                evidence=self.snapshot())

    def odometry(self,message):
        frame=(message.header.frame_id,message.child_frame_id)
        if self.odom_frame is not None and frame!=self.odom_frame and self.recovery:self.recovery.start=None
        self.odom_frame=frame;p,q=message.pose.pose.position,message.pose.pose.orientation
        self.pose=(p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        self.pose_time=message.header.stamp.sec+message.header.stamp.nanosec/1e9

    def fresh_pose(self):
        age=self.node.get_clock().now().nanoseconds/1e9-self.pose_time
        return self.pose if 0<=age<=.5 else None

    def action_status(self,kind,message):
        if kind=='follow_path':
            self.following=any(item.status==GoalStatus.STATUS_EXECUTING for item in message.status_list);return
        for item in message.status_list:
            key=(kind,bytes(item.goal_info.goal_id.uuid).hex());previous=self.seen.get(key)
            self.seen[key]=item.status
            if previous==item.status or not self.node.current:continue
            if item.status==GoalStatus.STATUS_EXECUTING:
                if kind=='wait':
                    self.node.event('RECOVERY_WAIT','Nav2等待环境更新；等待完成不等于脱困');continue
                evidence=RecoveryEvidence(kind,self.fresh_pose(),self.node.distance)
                self.recoveries[key]=evidence;self.recovery=evidence
                self.node.event('RECOVERY_STARTED','执行恢复动作，原任务保持',kind=kind)
            elif item.status in (4,5,6) and key in self.recoveries:
                evidence=self.recoveries.pop(key);evidence.finish(item.status==4,self.fresh_pose())
                self.node.event('RECOVERY_ACTION_RESULT','恢复动作结束；需分别核对动作、位移和导航恢复',
                    kind=kind,status=item.status,evidence=asdict(evidence))
        if len(self.seen)>256:self.seen=dict(list(self.seen.items())[-128:])
