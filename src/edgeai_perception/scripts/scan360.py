#!/usr/bin/env python3
"""Startup-only scan: authority must hold STOPPED/IDLE until Spin terminates."""
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav2_msgs.action import Spin
from std_msgs.msg import String
from std_srvs.srv import SetBool
from action_msgs.msg import GoalStatus


def main():
    rclpy.init()
    node = Node('target_scan360')
    node.declare_parameter('output_file', '')
    node.declare_parameter('time_allowance_s', 30.0)
    allowance = float(node.get_parameter('time_allowance_s').value)
    if not 5 <= allowance <= 60:
        raise ValueError('Scan time allowance must be 5..60 simulation seconds')
    state = {}
    def on_state(msg):
        state.clear(); state.update(json.loads(msg.data))
    sub = node.create_subscription(String, '/navigation_authority/state', on_state,
                                  QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    client = ActionClient(node, Spin, '/navigation_backend/spin')
    resume = node.create_client(SetBool, '/navigation_authority/set_manual')
    handle = None
    result_future = None
    traveled = [0.0]
    report = dict(success=False, terminal_verified=False, requested_angle_rad=2*math.pi)
    def wait(predicate, seconds):
        end = time.monotonic()+seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
            if predicate():
                return True
        return False
    try:
        if not wait(lambda: state.get('mode') == 'STOPPED' and state.get('phase') == 'IDLE', 10):
            raise RuntimeError('Scan requires authority STOPPED/IDLE; no spin was sent')
        if not client.wait_for_server(timeout_sec=5):
            raise RuntimeError('Nav2 Spin server unavailable')
        command_sequence = state.get('command_sequence')
        goal = Spin.Goal(target_yaw=2*math.pi)
        goal.time_allowance.sec = int(allowance)
        goal.time_allowance.nanosec = int((allowance-int(allowance))*1e9)
        def feedback(msg):
            traveled[0] = abs(float(msg.feedback.angular_distance_traveled))
        node.get_logger().info('SCAN360_STARTED: 开始360度相机观察；导航派发保持停止，碰撞检测保留')
        sent = client.send_goal_async(goal, feedback_callback=feedback)
        if not wait(sent.done, 10):
            raise RuntimeError('Spin acceptance unknown; authority remains stopped')
        handle = sent.result()
        if not handle.accepted:
            report['terminal_verified'] = True
            report['reason'] = 'goal_rejected'
        else:
            result_future = handle.get_result_async()
            deadline = time.monotonic()+max(60, allowance*4)
            while not result_future.done() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
                if state.get('mode') != 'STOPPED' or state.get('command_sequence') != command_sequence:
                    break
            if not result_future.done():
                handle.cancel_goal_async()
                if not wait(result_future.done, 10):
                    raise RuntimeError('Spin terminal result unknown; do not resume exploration')
            result = result_future.result()
            report.update(status=result.status, error_code=result.result.error_code,
                          traveled_angle_rad=traveled[0], terminal_verified=True)
            report['success'] = (result.status == GoalStatus.STATUS_SUCCEEDED and
                                 traveled[0] >= 2*math.pi-.15)
        if state.get('mode') != 'STOPPED' or state.get('command_sequence') != command_sequence:
            raise RuntimeError('Authority mode changed during scan; refusing automatic resume')
        node.get_logger().info('SCAN360_FINISHED: '+json.dumps(report))
        if not resume.wait_for_service(timeout_sec=5):
            raise RuntimeError('Authority resume service unavailable')
        response = resume.call_async(SetBool.Request(data=False))
        if not wait(response.done, 5) or not response.result().success:
            raise RuntimeError('Failed to request automatic exploration')
        if not wait(lambda: state.get('mode') == 'AUTO' and state.get('phase') == 'IDLE', 5):
            raise RuntimeError('Automatic mode not confirmed')
        if not report['success']:
            node.get_logger().warning('SCAN360_INCOMPLETE: 旋转受阻或未完成一周；已确认终态，继续探索')
    except (RuntimeError, KeyboardInterrupt) as exc:
        report['reason'] = str(exc)
        if handle is not None and handle.accepted and not report['terminal_verified']:
            handle.cancel_goal_async()
            if result_future is not None:
                report['terminal_verified'] = wait(result_future.done, 5)
        node.get_logger().error('SCAN360_ERROR: '+str(exc))
    finally:
        output = node.get_parameter('output_file').value
        if output:
            Path(output).write_text(json.dumps(report, indent=2))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if state.get('mode') == 'AUTO' and report['terminal_verified'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
