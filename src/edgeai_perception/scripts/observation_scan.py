#!/usr/bin/env python3
"""Schedule safe observation turns only between natural navigation tasks."""
import json
import math
from pathlib import Path
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from nav_msgs.msg import OccupancyGrid, Odometry
from tf2_ros import Buffer, TransformListener, TransformException
from scan_policy import open_disk, new_view


class ObservationScan(Node):
    def __init__(self):
        super().__init__('observation_scan')
        defaults = dict(enabled=True, check_hz=2.0, rotation_radius_m=.38, rotation_margin_m=.08,
                        min_open_radius_m=.50, min_scan_distance_m=1.2, min_scan_interval_s=30.0,
                        max_scans=8, minimum_completed_goals=1, stationary_time_s=.4,
                        max_linear_speed_mps=.03, max_angular_speed_rps=.05,
                        max_data_age_s=.5, output_directory='')
        for k,v in defaults.items():
            self.declare_parameter(k,v)
        self.p = {k:self.get_parameter(k).value for k in defaults}
        if self.p['check_hz'] <= 0 or self.p['rotation_radius_m'] <= 0 or self.p['rotation_margin_m'] < 0:
            raise ValueError('Invalid scan frequency or geometry')
        self.gate_reason = ''; self.gate_log_at = -100.
        self.grid = self.odom = None
        self.state = {}; self.state_at = 0.
        self.stationary_since = None
        self.history = []; self.last_scan = -1e20; self.retry_after = 0.
        self.child = None; self.child_log = None; self.holding = None
        self.held_sequence = None; self.hold_wall = None
        self.hold_point = None
        self.buffer = Buffer(); self.listener = TransformListener(self.buffer,self)
        self.hold = self.create_client(Trigger,'/navigation_authority/observation_hold')
        self.resume = self.create_client(SetBool,'/navigation_authority/set_manual')
        self.output = Path(self.p['output_directory']) if self.p['output_directory'] else Path('/tmp/edgeai-observation-scan')
        self.output.mkdir(parents=True,exist_ok=True)
        self.create_subscription(String,'/navigation_authority/state',self.authority,
                                 QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(OccupancyGrid,'/local_costmap/costmap',self.costmap,10)
        self.create_subscription(Odometry,'/odom',self.motion,qos_profile_sensor_data)
        self.create_timer(1/self.p['check_hz'],self.tick)
        self.get_logger().info('OBSERVATION_READY: 开阔位置途中观察已启用，仅在导航自然结束后扫描')

    def authority(self,msg):
        self.state=json.loads(msg.data);self.state_at=time.monotonic()
    def costmap(self,msg):
        self.grid=msg
    def motion(self,msg):
        self.odom=msg
        v=msg.twist.twist
        still=math.hypot(v.linear.x,v.linear.y)<=self.p['max_linear_speed_mps'] and abs(v.angular.z)<=self.p['max_angular_speed_rps']
        if still:
            if self.stationary_since is None:self.stationary_since=self.get_clock().now().nanoseconds*1e-9
        else:self.stationary_since=None

    def reject(self, reason):
        self.gate_reason = reason
        return None

    def candidate(self):
        now=self.get_clock().now();seconds=now.nanoseconds*1e-9
        if self.grid is None or self.odom is None or self.stationary_since is None:
            return self.reject('等待地图、里程计或停稳')
        if seconds-self.stationary_since<self.p['stationary_time_s']:return self.reject('停稳时间不足')
        for stamp in (self.grid.header.stamp,self.odom.header.stamp):
            age=(now-Time.from_msg(stamp)).nanoseconds*1e-9
            if not 0<=age<=self.p['max_data_age_s']:return self.reject('地图或里程计过期')
        q=self.grid.info.origin.orientation
        if abs(q.x)+abs(q.y)+abs(q.z)>1e-6:return self.reject('不支持旋转栅格原点')  # rotated grid not supported
        try:
            t=self.buffer.lookup_transform(self.grid.header.frame_id,'base_footprint',Time.from_msg(self.grid.header.stamp)).transform.translation
            m=self.buffer.lookup_transform('map','base_footprint',Time.from_msg(self.odom.header.stamp)).transform.translation
        except TransformException:return self.reject('采样时刻TF不可用')
        position=(m.x,m.y)
        if not new_view(position,self.history,self.p['min_scan_distance_m']):return self.reject('距离历史扫描位置过近')
        radius=max(self.p['min_open_radius_m'],self.p['rotation_radius_m']+self.p['rotation_margin_m'])
        if not open_disk(self.grid.data,self.grid.info.width,self.grid.info.height,self.grid.info.resolution,
                         (self.grid.info.origin.position.x,self.grid.info.origin.position.y),(t.x,t.y),radius):return self.reject('净空圆包含障碍、未知格或越出地图')
        return position

    def release_if_owned(self):
        if (self.state.get('mode')=='STOPPED' and self.state.get('command_sequence')==self.held_sequence
                and self.resume.service_is_ready()):
            self.resume.call_async(SetBool.Request(data=False))
        self.held_sequence=None;self.hold_wall=None

    def tick(self):
        now=self.get_clock().now().nanoseconds*1e-9
        if self.child is not None:
            code=self.child.poll()
            if code is None:return
            self.child=None;self.child_log.close();self.child_log=None
            self.get_logger().info(f'OBSERVATION_FINISHED: 观察进程退出码 {code}，详见对应scan结果')
            # Child owns resume after a verified terminal result. Unknown result stays held.
            if code!=0:self.p['enabled']=False
            self.held_sequence=None;self.hold_wall=None
            return
        if self.holding is not None:
            if not self.holding.done():
                if time.monotonic()-self.hold_wall>5:
                    self.get_logger().error('OBSERVATION_HOLD_UNKNOWN: 控制请求未确认，停止自动扫描')
                    self.p['enabled']=False
                return
            response=self.holding.result();self.holding=None
            if not response.success:
                self.retry_after=now+2;return
            self.held_sequence=int(response.message)
        if self.held_sequence is not None:
            if (self.state.get('mode')!='STOPPED' or self.state.get('command_sequence')!=self.held_sequence):
                if time.monotonic()-self.hold_wall>5:self.held_sequence=None
                return
            # Wait for explorer pause acknowledgement, not just the stop-service reply.
            if self.state.get('phase')!='IDLE' or not self.state.get('explorer_paused'):
                if time.monotonic()-self.hold_wall>8:self.release_if_owned()
                return
            point=self.candidate()  # recheck geometry after acquiring control
            if point is None:
                self.retry_after=now+5;self.release_if_owned();return
            index=len(self.history)+1
            self.history.append(point);self.last_scan=now
            self.child_log=(self.output/f'observation-scan-{index}.log').open('w')
            self.child=subprocess.Popen([sys.executable,'-u',str(Path(__file__).with_name('scan360.py')),
                '--ros-args','-p','use_sim_time:=true','-p',f'output_file:={self.output/f"observation-scan-{index}.json"}'],
                stdout=self.child_log,stderr=subprocess.STDOUT)
            self.get_logger().info(f'OBSERVATION_STARTED: 开阔位置{point}执行第{index}次360度观察')
            return
        if (not self.p['enabled'] or len(self.history)>=self.p['max_scans'] or now<self.retry_after
                or now-self.last_scan<self.p['min_scan_interval_s'] or time.monotonic()-self.state_at>2):return
        if (self.state.get('mode')!='AUTO' or self.state.get('phase')!='IDLE'
                or self.state.get('completed_goals',0)<self.p['minimum_completed_goals']):return
        if self.candidate() is None:
            if now-self.gate_log_at>=5:
                self.get_logger().info('OBSERVATION_DEFERRED: '+self.gate_reason)
                self.gate_log_at=now
            return
        if not self.hold.service_is_ready():return
        self.hold_point=self.candidate();self.hold_wall=time.monotonic()
        self.holding=self.hold.call_async(Trigger.Request())

    def close(self):
        if self.child is not None and self.child.poll() is None:
            self.child.terminate()
            try:self.child.wait(timeout=6)
            except subprocess.TimeoutExpired:self.child.kill();self.child.wait()
        if self.child_log:self.child_log.close()


def main():
    rclpy.init();node=ObservationScan()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        node.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
