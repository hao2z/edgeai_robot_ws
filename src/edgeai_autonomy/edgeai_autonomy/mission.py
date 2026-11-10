"""SLAM-driven frontier exploration -> frozen target list -> normal rescue."""
import json
import math
from pathlib import Path
import subprocess
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose, Spin
from lifecycle_msgs.srv import GetState
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformListener, TransformException
import yaml
from .drive_odometry import axle_goal
from .geometry import frontier_goals, observation_goals, coverage_goals, clearance_at, rescue_goal, yaw


class Mission(Node):
    def __init__(self):
        super().__init__('autonomous_mission')
        self.set_parameters([Parameter('use_sim_time', value=True)])
        self.declare_parameter('config', '')
        self.declare_parameter('output', '')
        self.config = yaml.safe_load(Path(self.get_parameter('config').value).read_text())
        self.output = Path(self.get_parameter('output').value)
        self.output.mkdir(parents=True, exist_ok=True)
        self.tf, self.grid, self.targets, self.odom = Buffer(), None, [], None
        self.listener = TransformListener(self.tf, self)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', lambda msg: setattr(self, 'grid', msg), qos)
        self.create_subscription(String, '/autonomy/targets', self.on_targets, qos)
        self.hints = []
        self.create_subscription(String, '/autonomy/observation_hints',
                                 lambda msg: setattr(self, 'hints', json.loads(msg.data)), qos)
        self.create_subscription(Odometry, '/odom', lambda msg: setattr(self, 'odom', msg), qos_profile_sensor_data)
        self.heartbeat = self.create_publisher(Empty, '/autonomy/heartbeat', 10)
        self.stop = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.events = self.create_publisher(String, '/autonomy/events', 10)
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.spin_action = ActionClient(self, Spin, '/spin')
        self.goal_handle = None
        self.state = 'starting'
        self.history, self.trajectory, self.visited = [], [], []
        self.start_time = None
        self.last_sample = -1.0
        self.nav_successes = 0
        self.observation_successes = 0

    def on_targets(self, msg):
        previous = {target['id'] for target in self.targets}
        self.targets = json.loads(msg.data)
        for target in self.targets:
            if target['id'] not in previous:
                self.transition('target_confirmed', target_id=target['id'],
                                observations=target['observations'])

    def now(self):
        return self.get_clock().now().nanoseconds/1e9

    def position(self):
        try:
            sample = self.tf.lookup_transform('map', 'base_footprint', Time())
            tr, stamp = sample.transform, sample.header.stamp
            if self.now()-(stamp.sec+stamp.nanosec*1e-9) > 1.0:
                return None
            return tr.translation.x, tr.translation.y, yaw(tr.rotation)
        except TransformException:
            return None

    def tick(self):
        rclpy.spin_once(self, timeout_sec=0.025)
        self.heartbeat.publish(Empty())
        now = self.now()
        position = self.position()
        if position and now-self.last_sample > 0.1:
            self.trajectory.append([now, *position])
            self.last_sample = now
        if self.start_time is not None and now-self.start_time > self.config['mission_timeout_sim_seconds']:
            raise RuntimeError('Mission simulation time budget exceeded')

    def wait(self, predicate, wall_timeout, description):
        deadline = time.monotonic()+wall_timeout
        while time.monotonic() < deadline:
            self.tick()
            if predicate():
                return
        raise RuntimeError('Timed out: '+description)

    def transition(self, state, **data):
        self.state = state
        event = {'state': state, 'sim_time': self.now(), 'robot_id': 'robot_1',
                 'robot_pose_map': self.position(), **data}
        self.history.append(event)
        self.events.publish(String(data=json.dumps(event)))
        with open(self.output/'events.jsonl', 'a') as file:
            file.write(json.dumps(event)+'\n')
        self.get_logger().info(json.dumps(event))

    def action(self, client, goal, timeout_sim=90):
        self.wait(lambda: client.server_is_ready(), 90, 'Nav2 action server')
        future = client.send_goal_async(goal)
        self.wait(future.done, 10, 'goal acceptance')
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.goal_handle = None
            return False
        result = self.goal_handle.get_result_async()
        start, deadline = self.now(), time.monotonic()+max(120, timeout_sim*5)
        while not result.done():
            self.tick()
            if self.now()-start > timeout_sim or time.monotonic() > deadline:
                cancel = self.goal_handle.cancel_goal_async()
                self.wait(cancel.done, 10, 'goal cancellation')
                self.wait(result.done, 15, 'cancelled goal result')
                self.goal_handle = None
                self.stop.publish(Twist())
                return False
        value = result.result()
        self.goal_handle = None
        self.stop.publish(Twist())
        return value.status == 4

    def scan(self, radians=2*math.pi, required=True):
        grid = self.grid
        clearance = clearance_at(np.array(grid.data).reshape(grid.info.height, grid.info.width),
                                 grid.info.resolution,
                                 (grid.info.origin.position.x, grid.info.origin.position.y), self.position())
        if clearance < self.config['observation_clearance_m']:
            self.transition('observation_deferred', clearance_m=clearance,
                            reason='Continue mapping in transit; insufficient space for a full spin')
            return False
        goal = Spin.Goal()
        goal.target_yaw = radians
        goal.time_allowance = Duration(seconds=65).to_msg()
        self.transition('scanning', radians=radians)
        succeeded = self.action(self.spin_action, goal, timeout_sim=70)
        if not succeeded:
            self.transition('observation_incomplete', reason='Nav2 observation spin could not finish')
            if required:
                raise RuntimeError('Initial autonomous observation spin failed')
        return succeeded

    def navigate(self, x, y, heading, purpose):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = axle_goal(float(x), float(y), heading)
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(heading/2), math.cos(heading/2)
        self.transition('navigating', purpose=purpose, goal_map=[x, y, heading])
        result = self.action(self.nav, goal, timeout_sim=100)
        self.transition('navigation_result', purpose=purpose, succeeded=result)
        return result

    def wait_for_navigation(self):
        for name in ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator']:
            client = self.create_client(GetState, f'/{name}/get_state')
            self.wait(client.service_is_ready, 90, name+' lifecycle service')
            deadline = time.monotonic()+90
            while True:
                future = client.call_async(GetState.Request())
                self.wait(future.done, 10, name+' state response')
                if future.result().current_state.id == 3:
                    break
                if time.monotonic() > deadline:
                    raise RuntimeError(name+' did not activate')
                until = time.monotonic()+0.3
                while time.monotonic() < until:
                    self.tick()
            self.destroy_client(client)

    def run(self):
        result = {'status': 'failed', 'practice_parameters': self.config}
        try:
            self.wait(lambda: self.grid is not None and self.position() is not None and self.odom is not None,
                      90, 'SLAM, TF and odometry')
            self.wait_for_navigation()
            self.start_time = self.now()
            self.transition('exploring')
            self.scan()
            for _ in range(self.config['exploration_max_goals']):
                grid = self.grid
                inputs = (np.array(grid.data).reshape(grid.info.height, grid.info.width), grid.info.resolution,
                          (grid.info.origin.position.x, grid.info.origin.position.y), self.position(), self.visited)
                hints = [h for h in self.hints if self.now()-h['last_seen_sim_time'] < 120]
                options = {'clearance': self.config['exploration_clearance_m']}
                candidates = observation_goals(*inputs, hints, **options) if not self.targets and hints else []
                purpose = 'marker_observation' if candidates else 'exploration_frontier'
                if not candidates:
                    candidates = frontier_goals(*inputs, **options)
                if not candidates and not self.targets:
                    candidates = coverage_goals(*inputs, **options)
                    purpose = 'surface_observation'
                self.transition('frontiers_selected' if purpose == 'exploration_frontier' else 'observation_viewpoints_selected',
                                candidates=candidates[:5])
                if not candidates:
                    break
                candidate = candidates[0]
                self.visited.append((candidate['x'], candidate['y']))
                if self.navigate(candidate['x'], candidate['y'], candidate['yaw'], purpose):
                    if purpose == 'exploration_frontier':
                        self.nav_successes += 1
                    else:
                        self.observation_successes += 1
                    # Observe from the newly reached frontier, not a prerecorded waypoint.
                    self.scan(required=False)
                if self.targets and self.nav_successes >= self.config['exploration_min_goals']:
                    break
            if self.nav_successes < self.config['exploration_min_goals']:
                raise RuntimeError('No frontier navigation completed')
            self.wait(lambda: len(self.targets) > 0, 20, 'confirmed ArUco target')
            # Freeze Task 1 outputs; new detections cannot silently add rescue targets.
            frozen = json.loads(json.dumps(self.targets))
            self.transition('handover', targets=frozen, completed_frontier_goals=self.nav_successes)
            (self.output/'targets.json').write_text(json.dumps({'frame_id': 'map', 'targets': frozen}, indent=2)+'\n')
            saved = subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(self.output/'map'),
                                    '--ros-args', '-p', 'use_sim_time:=true', '-p', 'save_map_timeout:=15.0'],
                                   capture_output=True, text=True, timeout=30)
            (self.output/'map_saver.log').write_text(saved.stdout+saved.stderr)
            if saved.returncode or not (self.output/'map.yaml').exists():
                raise RuntimeError('Task 1 map export failed')
            target = frozen[0]
            x, y, heading = rescue_goal(target, self.config['rescue_standoff_m'])
            self.transition('rescue_assigned', target_id=target['id'], goal_map=[x, y, heading])
            if not self.navigate(x, y, heading, 'normal_rescue'):
                raise RuntimeError('Navigation to perceived rescue zone failed')
            self.transition('dwelling', target_id=target['id'])
            inside_since, started = None, self.now()
            dwell_deadline = time.monotonic()+90
            while self.now()-started < 30:
                self.tick()
                if time.monotonic() > dwell_deadline:
                    raise RuntimeError('Rescue dwell clock stalled')
                position = self.position()
                self.stop.publish(Twist())
                velocity = self.odom.twist.twist
                inside = position is not None and math.hypot(position[0]-x, position[1]-y) <= self.config['rescue_radius_m']
                odom_stamp = self.odom.header.stamp
                fresh = self.now()-(odom_stamp.sec+odom_stamp.nanosec*1e-9) < 0.7
                stationary = fresh and abs(velocity.linear.x) < 0.02 and abs(velocity.angular.z) < 0.03
                if inside and stationary:
                    if inside_since is None:
                        inside_since = self.now()
                    if self.now()-inside_since >= self.config['dwell_seconds']:
                        break
                else:
                    inside_since = None
            else:
                raise RuntimeError('Continuous rescue dwell was not satisfied')
            self.transition('completed', target_id=target['id'], dwell_started_sim_time=inside_since,
                            dwell_seconds=self.now()-inside_since)
            result.update(status='completed', target=target, rescue_goal_map=[x, y, heading],
                          completed_frontier_goals=self.nav_successes, dwell_started_sim_time=inside_since,
                          completed_sim_time=self.now())
        except (Exception, KeyboardInterrupt) as exc:
            result['error'] = str(exc) or 'Interrupted'
            self.transition('failed', reason=result['error'])
        finally:
            result['completed_frontier_goals'] = self.nav_successes
            result['completed_observation_goals'] = self.observation_successes
            if self.goal_handle:
                self.goal_handle.cancel_goal_async()
            for _ in range(10):
                self.stop.publish(Twist())
                rclpy.spin_once(self, timeout_sec=0.02)
            result['events'] = self.history
            result['trajectory_map'] = self.trajectory
            (self.output/'mission.json').write_text(json.dumps(result, indent=2)+'\n')
        return result['status'] == 'completed'


def main():
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    node = Mission()
    try:
        succeeded = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if succeeded else 1)
