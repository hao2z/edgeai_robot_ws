"""Read a running midterm simulation and check truth + a diagnostic-only goal."""
import json
import math
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'src/edgeai_simulation/tools'))
from truth_geometry import compose, inverse
import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from rclpy.qos import QoSProfile, DurabilityPolicy


def main():
    rclpy.init()
    node = rclpy.create_node('check_truth_live')
    rows, markers = [], []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, '/truth/summary', lambda m: rows.append(json.loads(m.data)), qos)
    node.create_subscription(MarkerArray, '/truth/markers', lambda m: markers.append(m), qos)
    publisher = node.create_publisher(PoseStamped, '/navigation_authority/dispatched_goal', qos)
    deadline, sent = time.monotonic()+60, False
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if not rows or rows[-1]['status'] != 'ok':
                continue
            row = rows[-1]
            if not sent:
                goal = compose(inverse(row['world_from_reference']), (1.0, 3.75, math.pi/2))
                msg = PoseStamped()
                msg.header.frame_id = row['reference_frame']
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.pose.position.x, msg.pose.position.y = goal[:2]
                msg.pose.orientation.z, msg.pose.orientation.w = math.sin(goal[2]/2), math.cos(goal[2]/2)
                publisher.publish(msg)
                sent = True
            if 'goal_world' not in row or not markers:
                continue
            assert math.dist(row['robot_world'][:2], [.45, .55]) < .02, row
            assert math.dist(row['goal_world'][:2], [1, 3.75]) < .03, row
            assert abs(row['goal_to_targets_m']['1']-.35) < .03, row
            names = {m.ns for m in markers[-1].markers}
            assert {'truth_scene', 'truth_target', 'truth_goal', 'truth_robot'} <= names, names
            print('PASS live Gazebo truth, same-stamp TF, world goal and S1 distance, RViz markers')
            print(json.dumps(row))
            return
        raise AssertionError('Truth unavailable: '+str(rows[-1:] if rows else 'no summary'))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
