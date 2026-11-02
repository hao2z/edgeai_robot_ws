#!/usr/bin/env python3
import json
import math
import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import LaserScan
from slam_toolbox.srv import Pause
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros


def normalize_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Convert quaternion to 2D yaw in radians."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def arena_to_odom(ax: float, ay: float, ayaw: float, sx: float, sy: float, syaw: float):
    """Convert Gazebo 5x5m Arena coordinates to odom/map coordinates."""
    dx = ax - sx
    dy = ay - sy
    cos_s = math.cos(-syaw)
    sin_s = math.sin(-syaw)
    ox = dx * cos_s - dy * sin_s
    oy = dx * sin_s + dy * cos_s
    oyaw = normalize_angle(ayaw - syaw)
    return ox, oy, oyaw


def odom_to_arena(ox: float, oy: float, oyaw: float, sx: float, sy: float, syaw: float):
    """Convert odom/map coordinates to Gazebo 5x5m Arena coordinates."""
    cos_s = math.cos(syaw)
    sin_s = math.sin(syaw)
    ax = sx + ox * cos_s - oy * sin_s
    ay = sy + ox * sin_s + oy * cos_s
    ayaw = normalize_angle(syaw + oyaw)
    return ax, ay, ayaw


class SingleRescueNode(Node):
    """
    Autonomous Single-Target Rescue Node (S1) for the Midterm Check:
      - Locks SLAM Toolbox (paused_new_measurements = True) and loads arena_explorer's
        saved map (maps/midterm_slam_map.yaml) so the existing map is preserved and
        never overwritten during rescue navigation.
      - Phase 1: Navigate from current position to the Start Area (0.70, 0.50).
      - Phase 2: Navigate autonomously from the Start Area to Single Rescue Target S1 (1.0, 4.1).
      - Phase 3: Hold inside S1's 0.50m rescue radius (at d ~ 0.40m) for 10 consecutive
                 seconds of simulation time, logging & saving verification records.
      - Phase 4: Navigate safely back to the Start Area (0.70, 0.50) and halt.
    """

    def __init__(self):
        super().__init__('single_rescue')

        self.declare_parameter('spawn_x', 0.7)
        self.declare_parameter('spawn_y', 0.5)
        self.declare_parameter('spawn_yaw', 0.0)
        self.declare_parameter('evidence_dir', 'evidence')
        self.declare_parameter('map_yaml_path', 'maps/midterm_slam_map.yaml')

        self.spawn_x = float(self.get_parameter('spawn_x').value)
        self.spawn_y = float(self.get_parameter('spawn_y').value)
        self.spawn_yaw = float(self.get_parameter('spawn_yaw').value)
        self.evidence_dir = str(self.get_parameter('evidence_dir').value)
        self.map_yaml_path = str(self.get_parameter('map_yaml_path').value)
        os.makedirs(self.evidence_dir, exist_ok=True)

        # Target S1 parameters per Midterm specification
        self.s1_arena_x = 1.00
        self.s1_arena_y = 4.10
        self.s1_rescue_radius = 0.50
        self.required_hold_sec = 10.0

        # Motion control parameters
        self.max_linear_speed = 0.22
        self.max_angular_speed = 0.55
        self.dist_tolerance = 0.06
        self.yaw_tolerance = math.radians(3.5)

        # Mission state
        self.waypoints = []
        self.wp_index = 0
        self.plan_initialized = False
        # States: 'ALIGN_TO_GOAL', 'DRIVE_TO_GOAL', 'ALIGN_FINAL_YAW', 'PAUSE', 'RESCUE_HOLD', 'COMPLETED'
        self.state = 'ALIGN_TO_GOAL'
        self.state_start_time = None

        # Rescue hold timer & verification tracking
        self.hold_start_sim_time = None
        self.hold_elapsed_sec = 0.0
        self.last_logged_whole_sec = -1
        self.hold_distances = []
        self.rescue_completed = False

        # Robot pose in odom/map frame
        self.curr_x = None
        self.curr_y = None
        self.curr_yaw = None
        self.pose_frame = 'odom'

        # LiDAR sector clearances
        self.front_min = 10.0
        self.front_left_min = 10.0
        self.front_right_min = 10.0

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers, Subscribers & Service Clients
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/exploration_markers', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)

        self.slam_get_param_client = self.create_client(
            GetParameters, '/slam_toolbox/get_parameters'
        )
        self.slam_pause_client = self.create_client(
            Pause, '/slam_toolbox/pause_new_measurements'
        )

        # Load arena_explorer's saved map so we use it without overwriting
        self.saved_map_msg = self.load_saved_map()
        self.slam_paused_confirmed = False

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data
        )

        self.control_timer = self.create_timer(0.05, self.control_loop)
        self.viz_timer = self.create_timer(0.25, self.publish_markers)
        self.map_repub_timer = self.create_timer(1.0, self.republish_saved_map)

        self.get_logger().info(
            'Single Target (S1) Rescue Node started! Locking SLAM map and waiting for pose...'
        )

    def load_saved_map(self):
        """Load arena_explorer's saved OccupancyGrid map from maps/midterm_slam_map.yaml."""
        yaml_path = self.map_yaml_path
        if not os.path.isfile(yaml_path):
            try:
                pkg_share = get_package_share_directory('edgeai_robot_description')
                yaml_path = os.path.join(pkg_share, 'maps', 'midterm_arena.yaml')
            except Exception:
                return None

        if not os.path.isfile(yaml_path):
            return None

        try:
            resolution = 0.05
            origin = [-0.70, -0.50, 0.0]
            image_file = 'midterm_slam_map.pgm'
            with open(yaml_path, 'r') as f:
                for line in f:
                    line = line.strip()
                    if line.startswith('image:'):
                        image_file = line.split(':', 1)[1].strip()
                    elif line.startswith('resolution:'):
                        resolution = float(line.split(':', 1)[1].strip())
                    elif line.startswith('origin:'):
                        raw = line.split(':', 1)[1].strip().strip('[]')
                        origin = [float(x.strip()) for x in raw.split(',')]

            pgm_path = os.path.join(os.path.dirname(yaml_path), image_file)
            img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
            if img is None:
                return None

            # Flip vertically back to OccupancyGrid row order (row 0 = bottom)
            img_flipped = np.flipud(img)
            height, width = img_flipped.shape
            grid = np.full((height, width), -1, dtype=np.int8)
            grid[img_flipped >= 230] = 0
            grid[img_flipped <= 100] = 100

            msg = OccupancyGrid()
            msg.header.frame_id = 'map'
            msg.info.resolution = float(resolution)
            msg.info.width = int(width)
            msg.info.height = int(height)
            msg.info.origin.position.x = float(origin[0])
            msg.info.origin.position.y = float(origin[1])
            msg.info.origin.position.z = 0.0
            msg.info.origin.orientation.z = math.sin(origin[2] / 2.0)
            msg.info.origin.orientation.w = math.cos(origin[2] / 2.0)
            msg.data = grid.flatten().tolist()

            self.get_logger().info(
                f'Loaded arena_explorer map from {yaml_path} ({width}x{height} px @ {resolution}m/px).'
            )
            return msg
        except Exception as ex:
            self.get_logger().warn(f'Could not load saved map {yaml_path}: {ex}')
            return None

    def ensure_slam_paused(self):
        """Ensure SLAM Toolbox paused_new_measurements is True so the map is not overwritten."""
        if self.slam_paused_confirmed or not self.slam_get_param_client.service_is_ready():
            return
        req = GetParameters.Request()
        req.names = ['paused_new_measurements']
        future = self.slam_get_param_client.call_async(req)

        def _on_param(fut):
            try:
                res = fut.result()
                if res.values and res.values[0].type == ParameterType.PARAMETER_BOOL:
                    curr_paused = res.values[0].bool_value
                    if not curr_paused and self.slam_pause_client.service_is_ready():
                        self.slam_pause_client.call_async(Pause.Request())
                        self.get_logger().info(
                            'Paused SLAM Toolbox new measurements to preserve arena_explorer map.'
                        )
                    self.slam_paused_confirmed = True
            except Exception:
                pass

        future.add_done_callback(_on_param)

    def republish_saved_map(self):
        """Ensure SLAM is paused and publish the clean arena_explorer map to /map."""
        self.ensure_slam_paused()
        if self.saved_map_msg is not None:
            self.saved_map_msg.header.stamp = self.get_clock().now().to_msg()
            self.map_pub.publish(self.saved_map_msg)

    def build_mission_waypoints(self, start_ax: float, start_ay: float):
        """
        Dynamically build the 4-phase mission plan based on where the robot currently is:
          Phase 1: Go to Start Area (0.70, 0.50)
          Phase 2: Go from Start Area to S1 Rescue Point (1.36, 3.92) [dist = 0.40m <= 0.50m]
          Phase 3: Hold inside S1 Rescue Zone for 10.0 consecutive simulation seconds
          Phase 4: Return to Start Area (0.70, 0.50)
        """
        raw_plan = []

        # --- Phase 1: Route from current pose to Start Area (0.70, 0.50) ---
        dist_to_start = math.hypot(start_ax - 0.70, start_ay - 0.50)
        if dist_to_start > 0.15:
            # If currently in Zone 5 (South-East / C1 room, x > 3.4)
            # Use the exact perpendicular-bisector centerline through the W2/O3 gap!
            if start_ax > 3.40 and start_ay < 3.20:
                raw_plan.extend([
                    (4.08, 2.62, 132.0, 0.4, False, 'Phase 1: Exiting C1 Room'),
                    (3.725, 3.05, 132.0, 0.4, False, 'Phase 1: Passing W2/O3 Centerline'),
                    (3.35, 3.46, 180.0, 0.4, False, 'Phase 1: Cleared W2/O3 Gap'),
                    (2.92, 3.46, -90.0, 0.4, False, 'Phase 1: Entering Central Corridor'),
                    (2.92, 0.80, 180.0, 0.4, False, 'Phase 1: South Corridor Westbound'),
                    (1.10, 0.80, -135.0, 0.4, False, 'Phase 1: Approaching Start Area'),
                ])
            # If currently in Zone 4 (North-West / S1 room, x < 2.3 and y > 3.1)
            elif start_ax < 2.30 and start_ay > 3.10:
                raw_plan.extend([
                    (2.05, 4.15, 0.0, 0.4, False, 'Phase 1: Exiting S1 Room'),
                    (2.92, 4.18, -90.0, 0.4, False, 'Phase 1: North Hub Southbound'),
                    (2.92, 0.80, 180.0, 0.4, False, 'Phase 1: South Corridor Westbound'),
                    (1.10, 0.80, -135.0, 0.4, False, 'Phase 1: Approaching Start Area'),
                ])
            # If currently in Zone 3 / North-East (x >= 2.2)
            elif start_ax >= 2.20:
                raw_plan.extend([
                    (2.92, 2.80, -90.0, 0.4, False, 'Phase 1: Central Corridor Southbound'),
                    (2.92, 0.80, 180.0, 0.4, False, 'Phase 1: South Corridor Westbound'),
                    (1.10, 0.80, -135.0, 0.4, False, 'Phase 1: Approaching Start Area'),
                ])
            # If in West Room (x < 2.2, y <= 3.1)
            elif start_ay > 1.20:
                raw_plan.append(
                    (1.10, 0.80, -135.0, 0.4, False, 'Phase 1: Approaching Start Area')
                )

        raw_plan.append(
            (0.70, 0.50, 0.0, 1.5, False, 'Phase 1: Ready in Start Area (0.70, 0.50)')
        )

        # --- Phase 2 & 3: Navigate from Start Area to S1 (1.0, 4.1) & Hold for 10s ---
        # Rescue point (1.36, 3.92) is 0.402m from S1 (1.00, 4.10) -> well inside 0.50m radius!
        raw_plan.extend([
            (1.10, 0.80, 0.0, 0.4, False, 'Phase 2: Departing Start Area'),
            (2.92, 0.80, 90.0, 0.4, False, 'Phase 2: South Corridor Eastbound'),
            (2.92, 2.80, 90.0, 0.4, False, 'Phase 2: Central Corridor Northbound'),
            (2.92, 4.18, 180.0, 0.4, False, 'Phase 2: Clearing W1b Tip'),
            (2.05, 4.15, -165.0, 0.4, False, 'Phase 2: Entering S1 Room'),
            (1.36, 3.92, 153.4, 10.0, True, 'Phase 3: S1 Single Rescue Hold (10s)'),
        ])

        # --- Phase 4: Return from S1 back to Start Area (0.70, 0.50) ---
        raw_plan.extend([
            (2.05, 4.15, 0.0, 0.4, False, 'Phase 4: Exiting S1 Room'),
            (2.92, 4.18, -90.0, 0.4, False, 'Phase 4: North Hub Southbound'),
            (2.92, 2.80, -90.0, 0.4, False, 'Phase 4: Central Corridor Southbound'),
            (2.92, 0.80, 180.0, 0.4, False, 'Phase 4: South Corridor Westbound'),
            (1.10, 0.80, -135.0, 0.4, False, 'Phase 4: Entering Start Area'),
            (0.70, 0.50, 0.0, 1.0, False, 'Phase 4: Returned to Start Area (0.70, 0.50)'),
        ])

        self.waypoints = []
        for ax, ay, ayaw_deg, pause_s, is_rescue, label in raw_plan:
            ox, oy, oyaw = arena_to_odom(
                ax, ay, math.radians(ayaw_deg),
                self.spawn_x, self.spawn_y, self.spawn_yaw
            )
            self.waypoints.append({
                'ax': ax,
                'ay': ay,
                'ox': ox,
                'oy': oy,
                'oyaw': oyaw,
                'pause': pause_s,
                'is_rescue': is_rescue,
                'label': label,
            })

    def odom_callback(self, msg: Odometry):
        if self.curr_x is None or not self.update_pose_from_tf():
            self.curr_x = msg.pose.pose.position.x
            self.curr_y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            self.curr_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
            self.pose_frame = 'odom'

    def update_pose_from_tf(self) -> bool:
        for frame in ('map', 'odom'):
            if self.tf_buffer.can_transform(frame, 'base_footprint', rclpy.time.Time()):
                try:
                    t = self.tf_buffer.lookup_transform(
                        frame, 'base_footprint', rclpy.time.Time()
                    )
                    self.curr_x = t.transform.translation.x
                    self.curr_y = t.transform.translation.y
                    q = t.transform.rotation
                    self.curr_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
                    self.pose_frame = frame
                    return True
                except tf2_ros.TransformException:
                    continue
        return False

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        if not np.any(valid):
            return

        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        front_mask = valid & (angles >= math.radians(-22.0)) & (angles <= math.radians(22.0))
        left_mask = valid & (angles >= math.radians(18.0)) & (angles <= math.radians(65.0))
        right_mask = valid & (angles >= math.radians(-65.0)) & (angles <= math.radians(-18.0))

        self.front_min = float(np.min(ranges[front_mask])) if np.any(front_mask) else 10.0
        self.front_left_min = float(np.min(ranges[left_mask])) if np.any(left_mask) else 10.0
        self.front_right_min = float(np.min(ranges[right_mask])) if np.any(right_mask) else 10.0

    def stop_robot(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    def save_rescue_verification(self, start_sim: float, end_sim: float):
        record = {
            'mission': 'Single Target Rescue (S1)',
            'target_id': 1,
            'target_name': 'S1',
            'target_center_arena_m': [self.s1_arena_x, self.s1_arena_y],
            'prescribed_radius_m': self.s1_rescue_radius,
            'required_continuous_hold_sim_sec': self.required_hold_sec,
            'hold_start_sim_time_sec': round(start_sim, 2),
            'hold_end_sim_time_sec': round(end_sim, 2),
            'continuous_hold_duration_sim_sec': round(end_sim - start_sim, 2),
            'min_distance_to_target_m': round(min(self.hold_distances), 3),
            'max_distance_to_target_m': round(max(self.hold_distances), 3),
            'mean_distance_to_target_m': round(
                sum(self.hold_distances) / len(self.hold_distances), 3
            ),
            'completed': True,
        }
        out_path = os.path.join(self.evidence_dir, 'S1_rescue_verification.json')
        try:
            with open(out_path, 'w') as f:
                json.dump(record, f, indent=2)
            self.get_logger().info(f'Saved S1 rescue verification record to {out_path}')
        except OSError:
            pass

    def control_loop(self):
        self.update_pose_from_tf()
        if self.curr_x is None:
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        ax, ay, _ = odom_to_arena(
            self.curr_x, self.curr_y, self.curr_yaw,
            self.spawn_x, self.spawn_y, self.spawn_yaw
        )

        if not self.plan_initialized:
            self.ensure_slam_paused()
            self.build_mission_waypoints(ax, ay)
            self.plan_initialized = True
            wp = self.waypoints[self.wp_index]
            self.get_logger().info(
                f'Robot currently at Arena ({ax:.2f}, {ay:.2f}). '
                f'Built {len(self.waypoints)}-step mission plan using saved map.\n'
                f'--> [1/{len(self.waypoints)}] {wp["label"]} at Arena ({wp["ax"]:.2f}, {wp["ay"]:.2f})'
            )

        if self.state == 'COMPLETED':
            self.stop_robot()
            return

        wp = self.waypoints[self.wp_index]
        dx = wp['ox'] - self.curr_x
        dy = wp['oy'] - self.curr_y
        dist = math.hypot(dx, dy)
        desired_heading = math.atan2(dy, dx)

        cmd = Twist()

        # State 1: Rotate in place toward next waypoint
        if self.state == 'ALIGN_TO_GOAL':
            if dist <= self.dist_tolerance:
                self.stop_robot()
                self.state = 'ALIGN_FINAL_YAW'
                return

            yaw_err = normalize_angle(desired_heading - self.curr_yaw)
            if abs(yaw_err) <= self.yaw_tolerance:
                self.stop_robot()
                self.state = 'DRIVE_TO_GOAL'
            else:
                speed = max(0.10, min(self.max_angular_speed, abs(yaw_err) * 1.4))
                cmd.angular.z = math.copysign(speed, yaw_err)
                self.cmd_pub.publish(cmd)

        # State 2: Drive straight to waypoint
        elif self.state == 'DRIVE_TO_GOAL':
            if dist <= self.dist_tolerance:
                self.stop_robot()
                self.state = 'ALIGN_FINAL_YAW'
                return

            yaw_err = normalize_angle(desired_heading - self.curr_yaw)
            if abs(yaw_err) > math.radians(22.0):
                self.stop_robot()
                self.state = 'ALIGN_TO_GOAL'
                return

            # Allow closer front approach when entering the S1 rescue circle (stand pole is thin)
            min_front_allowed = 0.16 if wp['is_rescue'] else 0.23
            if self.front_min < min_front_allowed:
                self.stop_robot()
                self.state = 'ALIGN_FINAL_YAW'
                return

            speed_scale = min(1.0, max(0.25, (self.front_min - 0.15) / 0.35))
            linear_v = max(0.06, min(self.max_linear_speed * speed_scale, dist * 0.9))

            lidar_steer = 0.0
            if not wp['is_rescue'] and (
                self.front_left_min < 0.33 or self.front_right_min < 0.33
            ):
                lidar_steer = 0.45 * (self.front_left_min - self.front_right_min)
                lidar_steer = max(-0.25, min(0.25, lidar_steer))

            cmd.linear.x = linear_v
            cmd.angular.z = max(
                -self.max_angular_speed,
                min(self.max_angular_speed, 1.5 * yaw_err + lidar_steer)
            )
            self.cmd_pub.publish(cmd)

        # State 3: Align to target heading at waypoint
        elif self.state == 'ALIGN_FINAL_YAW':
            yaw_err = normalize_angle(wp['oyaw'] - self.curr_yaw)
            if abs(yaw_err) <= self.yaw_tolerance:
                self.stop_robot()
                if wp['is_rescue']:
                    self.state = 'RESCUE_HOLD'
                    self.hold_start_sim_time = None
                    self.hold_elapsed_sec = 0.0
                    self.last_logged_whole_sec = -1
                    self.hold_distances = []
                else:
                    self.state = 'PAUSE'
                    self.state_start_time = now_sec
                    self.get_logger().info(
                        f'Reached {wp["label"]} at Arena ({ax:.2f}, {ay:.2f})'
                    )
            else:
                speed = max(0.10, min(self.max_angular_speed, abs(yaw_err) * 1.4))
                cmd.angular.z = math.copysign(speed, yaw_err)
                self.cmd_pub.publish(cmd)

        # State 4: Continuous 10-second simulation-time hold inside S1's 0.50m rescue zone
        elif self.state == 'RESCUE_HOLD':
            self.stop_robot()
            dist_to_s1 = math.hypot(ax - self.s1_arena_x, ay - self.s1_arena_y)

            if dist_to_s1 <= self.s1_rescue_radius:
                if self.hold_start_sim_time is None:
                    self.hold_start_sim_time = now_sec
                    self.get_logger().info(
                        f'*** ENTERED S1 RESCUE ZONE (dist={dist_to_s1:.3f}m <= {self.s1_rescue_radius:.2f}m) '
                        f'at t_sim={now_sec:.2f}s! Starting 10.0s continuous timer... ***'
                    )

                self.hold_elapsed_sec = now_sec - self.hold_start_sim_time
                self.hold_distances.append(dist_to_s1)

                whole_sec = int(self.hold_elapsed_sec)
                if whole_sec > self.last_logged_whole_sec and whole_sec <= 10:
                    self.last_logged_whole_sec = whole_sec
                    self.get_logger().info(
                        f'[S1 RESCUE HOLD] {self.hold_elapsed_sec:.1f}s / {self.required_hold_sec:.1f}s | '
                        f'Robot at Arena ({ax:.2f}, {ay:.2f}) | dist_to_S1 = {dist_to_s1:.3f}m <= 0.50m'
                    )

                if self.hold_elapsed_sec >= self.required_hold_sec:
                    self.rescue_completed = True
                    self.save_rescue_verification(self.hold_start_sim_time, now_sec)
                    self.get_logger().info(
                        f'*** S1 SINGLE RESCUE COMPLETED! Held continuously for '
                        f'{self.hold_elapsed_sec:.2f}s inside 0.50m radius! Returning to Start Area... ***'
                    )
                    self.advance_waypoint()
            else:
                if self.hold_start_sim_time is not None:
                    self.get_logger().warn(
                        f'Robot drifted outside S1 0.50m radius (dist={dist_to_s1:.3f}m); '
                        'resetting 10s hold timer and nudging closer!'
                    )
                    self.hold_start_sim_time = None
                    self.hold_elapsed_sec = 0.0
                self.state = 'DRIVE_TO_GOAL'

        # State 5: Normal brief pause at intermediate waypoints
        elif self.state == 'PAUSE':
            self.stop_robot()
            if now_sec - self.state_start_time >= wp['pause']:
                self.advance_waypoint()

    def advance_waypoint(self):
        self.wp_index += 1
        if self.wp_index >= len(self.waypoints):
            self.state = 'COMPLETED'
            self.get_logger().info(
                '=== SINGLE TARGET (S1) RESCUE & RETURN TO START AREA COMPLETED! ===\n'
                '  Robot is parked safely in the Start Area (0.70, 0.50).\n'
                f'  Verification record saved to: {self.evidence_dir}/S1_rescue_verification.json'
            )
        else:
            next_wp = self.waypoints[self.wp_index]
            self.state = 'ALIGN_TO_GOAL'
            self.get_logger().info(
                f'--> [{self.wp_index + 1}/{len(self.waypoints)}] {next_wp["label"]} '
                f'at Arena ({next_wp["ax"]:.2f}, {next_wp["ay"]:.2f})'
            )

    def publish_markers(self):
        if not self.plan_initialized:
            return

        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()
        frame = self.pose_frame

        # 1. Path line strip
        path_marker = Marker()
        path_marker.header.stamp = now
        path_marker.header.frame_id = frame
        path_marker.ns = 'exploration_path'
        path_marker.id = 0
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.03
        path_marker.color.r = 1.0
        path_marker.color.g = 0.6
        path_marker.color.b = 0.1
        path_marker.color.a = 0.7

        for i, wp in enumerate(self.waypoints):
            pt = Point()
            pt.x = wp['ox']
            pt.y = wp['oy']
            pt.z = 0.03
            path_marker.points.append(pt)

            sphere = Marker()
            sphere.header.stamp = now
            sphere.header.frame_id = frame
            sphere.ns = 'exploration_nodes'
            sphere.id = i + 1
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = wp['ox']
            sphere.pose.position.y = wp['oy']
            sphere.pose.position.z = 0.05
            sphere.scale.x = 0.12
            sphere.scale.y = 0.12
            sphere.scale.z = 0.12
            if i < self.wp_index:
                sphere.color.r = 0.1
                sphere.color.g = 0.9
                sphere.color.b = 0.2
                sphere.color.a = 0.8
            elif i == self.wp_index and self.state != 'COMPLETED':
                sphere.color.r = 1.0
                sphere.color.g = 0.9
                sphere.color.b = 0.0
                sphere.color.a = 1.0
            else:
                sphere.color.r = 0.2
                sphere.color.g = 0.5
                sphere.color.b = 1.0
                sphere.color.a = 0.6
            marker_array.markers.append(sphere)

        marker_array.markers.append(path_marker)

        # 2. S1 Rescue Zone Ring & Live 10s Timer Text in RViz
        s1_ox, s1_oy, _ = arena_to_odom(
            self.s1_arena_x, self.s1_arena_y, 0.0,
            self.spawn_x, self.spawn_y, self.spawn_yaw
        )
        zone_marker = Marker()
        zone_marker.header.stamp = now
        zone_marker.header.frame_id = frame
        zone_marker.ns = 's1_rescue_status'
        zone_marker.id = 100
        zone_marker.type = Marker.CYLINDER
        zone_marker.action = Marker.ADD
        zone_marker.pose.position.x = s1_ox
        zone_marker.pose.position.y = s1_oy
        zone_marker.pose.position.z = 0.015
        zone_marker.scale.x = self.s1_rescue_radius * 2.0
        zone_marker.scale.y = self.s1_rescue_radius * 2.0
        zone_marker.scale.z = 0.015
        if self.rescue_completed:
            zone_marker.color.r = 0.0
            zone_marker.color.g = 1.0
            zone_marker.color.b = 0.2
            zone_marker.color.a = 0.45
        elif self.state == 'RESCUE_HOLD':
            zone_marker.color.r = 1.0
            zone_marker.color.g = 0.85
            zone_marker.color.b = 0.0
            zone_marker.color.a = 0.45
        else:
            zone_marker.color.r = 1.0
            zone_marker.color.g = 0.3
            zone_marker.color.b = 0.3
            zone_marker.color.a = 0.30
        marker_array.markers.append(zone_marker)

        timer_text = Marker()
        timer_text.header.stamp = now
        timer_text.header.frame_id = frame
        timer_text.ns = 's1_rescue_status'
        timer_text.id = 101
        timer_text.type = Marker.TEXT_VIEW_FACING
        timer_text.action = Marker.ADD
        timer_text.pose.position.x = s1_ox
        timer_text.pose.position.y = s1_oy
        timer_text.pose.position.z = 0.72
        timer_text.scale.z = 0.16
        timer_text.color.r = 1.0
        timer_text.color.g = 1.0
        timer_text.color.b = 0.2
        timer_text.color.a = 1.0
        if self.rescue_completed:
            timer_text.text = 'S1 RESCUED! (10.0s / 10.0s)'
        elif self.state == 'RESCUE_HOLD':
            timer_text.text = f'S1 RESCUE HOLD: {self.hold_elapsed_sec:.1f}s / 10.0s'
        else:
            timer_text.text = 'S1 Target (r=0.5m)'
        marker_array.markers.append(timer_text)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = SingleRescueNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
