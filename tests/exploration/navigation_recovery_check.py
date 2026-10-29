"""Bounded fault injection: real Nav2 recovery, measured motion, same goal resumes."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True, type=Path)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = rclpy.create_node('navigation_recovery_check')
    client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    get = node.create_client(GetParameters, '/controller_server/get_parameters')
    set_ = node.create_client(SetParametersAtomically, '/controller_server/set_parameters_atomically')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    states, identities = [], set()

    def state(message):
        data = json.loads(message.data)
        states.append(data)
        if data.get('goal_uuid'):
            identities.add((data['generation'], data['goal_uuid']))

    subscription = node.create_subscription(String, '/navigation_authority/state', state,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def wait(future, timeout=5):
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError('RPC timeout')
        return future.result()

    names = ['progress_checker.required_movement_radius', 'progress_checker.movement_time_allowance']
    original, handle, restored = None, None, False
    result = dict(success=False, fault_injection='temporary unrealistic progress requirement')
    try:
        if not (client.wait_for_server(timeout_sec=10) and get.wait_for_service(timeout_sec=5)
                and set_.wait_for_service(timeout_sec=5)):
            raise RuntimeError('Managed navigation/parameter services unavailable')
        deadline = time.monotonic()+12
        while True:
            rclpy.spin_once(node, timeout_sec=.05)
            try:
                transform = buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform
                break
            except TransformException:
                if time.monotonic() > deadline:
                    raise RuntimeError('TF unavailable')
        q = transform.rotation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        original = wait(get.call_async(GetParameters.Request(names=names))).values
        injected = [Parameter(name, value=value).to_parameter_msg()
                    for name, value in zip(names, [1000.0, 1.5])]
        response = wait(set_.call_async(SetParametersAtomically.Request(parameters=injected)))
        if not response.result.successful:
            raise RuntimeError(response.result.reason)
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = transform.translation.x+1.20*math.cos(yaw)
        goal.pose.pose.position.y = transform.translation.y+1.20*math.sin(yaw)
        goal.pose.pose.orientation = q
        handle = wait(client.send_goal_async(goal))
        if not handle.accepted:
            raise RuntimeError('Goal rejected')
        finished = handle.get_result_async()
        deadline = time.monotonic()+65
        while not finished.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if not restored and states and (states[-1].get('recovery') or {}).get('outcome') == 'RUNNING':
                parameters = [Parameter(name=name).to_parameter_msg() for name in names]
                for parameter, value in zip(parameters, original):
                    parameter.value = value
                response = wait(set_.call_async(SetParametersAtomically.Request(parameters=parameters)))
                if not response.result.successful:
                    raise RuntimeError('Cannot restore progress parameters: '+response.result.reason)
                restored = True
                print('RECOVERY_CHECK: native recovery started; normal progress limits restored', flush=True)
        # Allow the authority's periodic state to catch up to the terminal callback.
        end = time.monotonic()+.6
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
        evidence = [s['recovery'] for s in states if s.get('recovery')]
        result.update(restored_parameters=restored, goal_identities=[list(x) for x in identities],
                      recovery_evidence=evidence[-1] if evidence else None,
                      action_status=finished.result().status if finished.done() else None,
                      success=bool(finished.done() and finished.result().status == 4 and restored
                                   and len(identities) == 1 and any(e.get('recovered') for e in evidence)))
    except Exception as error:
        result['error'] = str(error)
    finally:
        if original is not None and not restored and rclpy.ok():
            try:
                parameters = [Parameter(name=name).to_parameter_msg() for name in names]
                for parameter, value in zip(parameters, original):
                    parameter.value = value
                wait(set_.call_async(SetParametersAtomically.Request(parameters=parameters)))
            except Exception as error:
                result['restore_error'] = str(error)
        if handle is not None and rclpy.ok():
            try:
                wait(handle.cancel_goal_async(), 2)
            except Exception:
                pass
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2))
        print('RECOVERY_CHECK: '+json.dumps(result, ensure_ascii=False), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if result['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
