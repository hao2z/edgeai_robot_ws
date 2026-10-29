"""Send one short forward goal to the unmodified Nav2 TB3 demo."""
import json
import math
from pathlib import Path
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from lifecycle_msgs.srv import GetState
from tf2_ros import Buffer, TransformListener


def run(directory):
    directory = Path(directory)
    rclpy.init()
    node = rclpy.create_node("official_demo_direction_check", parameter_overrides=[
        Parameter("use_sim_time", value=True)])
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    initial_pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
    action = ActionClient(node, NavigateToPose, "/navigate_to_pose")
    state_client = node.create_client(GetState, "/bt_navigator/get_state")
    state_future = None
    active = False
    command = [0.0, 0.0]
    odom = None

    def on_command(msg):
        command[:] = [msg.linear.x, msg.angular.z]

    def on_odom(msg):
        nonlocal odom
        odom = msg.pose.pose

    node.create_subscription(Twist, "/cmd_vel", on_command, 10)
    node.create_subscription(Odometry, "/odom", on_odom, 10)
    report = {"success": False, "baseline": "official Nav2 Jazzy TB3 sandbox",
              "controller": (directory / "mppi-mode.txt").read_text().strip(), "trace": []}
    handle = None
    try:
        deadline = time.monotonic() + 75
        next_initial = 0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if state_future is not None and state_future.done():
                active = state_future.result().current_state.id == 3
                state_future = None
            if not active and state_future is None and state_client.service_is_ready():
                state_future = state_client.call_async(GetState.Request())
            if time.monotonic() >= next_initial:
                pose = PoseWithCovarianceStamped()
                pose.header.frame_id = "map"
                pose.header.stamp = node.get_clock().now().to_msg()
                pose.pose.pose.position.x = -2.0
                pose.pose.pose.position.y = -0.5
                pose.pose.pose.orientation.w = 1.0
                pose.pose.covariance[0] = 0.01
                pose.pose.covariance[7] = 0.01
                pose.pose.covariance[35] = 0.01
                initial_pub.publish(pose)
                next_initial = time.monotonic() + 1
            if active and action.server_is_ready() and odom is not None:
                try:
                    tf = buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
                    break
                except Exception:
                    pass
        else:
            raise RuntimeError("Official demo localization/action readiness timeout")
        x, y = tf.transform.translation.x, tf.transform.translation.y
        q = tf.transform.rotation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = x + 0.6*math.cos(yaw)
        goal.pose.pose.position.y = y + 0.6*math.sin(yaw)
        goal.pose.pose.orientation = q
        report.update(initial=[x, y, yaw], goal=[goal.pose.pose.position.x, goal.pose.pose.position.y])
        print("Official demo ready; sending forward 0.6 m goal", flush=True)
        (directory / "ready").touch()
        future = action.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        if not future.done() or not future.result().accepted:
            raise RuntimeError("Official demo goal was not accepted")
        handle = future.result()
        result = handle.get_result_async()
        deadline = time.monotonic() + 60
        next_sample = 0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.monotonic() >= next_sample:
                tf = buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
                pos = tf.transform.translation
                report["trace"].append({"time": node.get_clock().now().nanoseconds/1e9,
                    "pose": [pos.x, pos.y], "command": command.copy()})
                report["final"] = [pos.x, pos.y]
                report["position_error"] = math.hypot(pos.x-goal.pose.pose.position.x,
                                                      pos.y-goal.pose.pose.position.y)
                next_sample = time.monotonic() + 0.1
                if (pos.x-x)*math.cos(yaw)+(pos.y-y)*math.sin(yaw) < -0.1:
                    raise RuntimeError("Official demo moved backward by more than 0.1 m")
            if result.done():
                report["action_status"] = result.result().status
                report["success"] = report["action_status"] == 4 and report["position_error"] < 0.3
                break
        else:
            raise RuntimeError("Official demo navigation timed out")
    except Exception as exc:
        report["error"] = str(exc)
    finally:
        if handle is not None and not report["success"]:
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel, timeout_sec=3)
        (directory / "official-result.json").write_text(json.dumps(report, indent=2))
        print(json.dumps({k: v for k, v in report.items() if k != "trace"}), flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if report["success"] else 1


if __name__ == "__main__":
    raise SystemExit(run(sys.argv[1]))
