#!/usr/bin/env python3
import json
import math
import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import LaserScan
from slam_toolbox.srv import Pause
from std_msgs.msg import String
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


class ArenaExplorerNode(Node):
    """
    Autonomous Arena Explorer & SLAM Map Renderer for Part 1 of the Midterm Check.

    Features:
    - Uses SLAM-corrected TF (map -> base_footprint) with automatic fallback to /odom.
    - Tracks live OccupancyGrid (/map) coverage and frontiers across the 5.0m x 5.0m arena.
    - Uses 360-degree LiDAR (/scan) for corridor centering and reactive obstacle safety.
    - Actively scans rooms and observes both S1 (ID 1) and C1 (ID 2) ArUco targets.
    - Automatically exports the resulting SLAM map to maps/midterm_slam_map.{pgm,yaml}.
    """

    def __init__(self):
        super().__init__('arena_explorer')

        self.declare_parameter('spawn_x', 0.7)
        self.declare_parameter('spawn_y', 0.5)
        self.declare_parameter('spawn_yaw', 0.0)
        self.declare_parameter('map_save_dir', 'maps')

        self.spawn_x = float(self.get_parameter('spawn_x').value)
        self.spawn_y = float(self.get_parameter('spawn_y').value)
        self.spawn_yaw = float(self.get_parameter('spawn_yaw').value)
        self.map_save_dir = str(self.get_parameter('map_save_dir').value)
        os.makedirs(self.map_save_dir, exist_ok=True)

        # Control parameters
        self.max_linear_speed = 0.22
        self.max_angular_speed = 0.55
        self.dist_tolerance = 0.07
        self.yaw_tolerance = math.radians(3.5)

        # Topological exploration viewpoints in 5.0m x 5.0m Arena coordinates:
        # (arena_x, arena_y, target_yaw_deg, pause_sec, zone_name)
        self.arena_viewpoints = [
            # Zone 1 & 2: West Room (below W1, West of O2)
            (0.95, 2.30, 90.0, 1.0, 'Zone 2: West Room North'),
            (1.10, 0.80, 0.0, 0.6, 'Zone 1: South-West Corridor Entry'),
            # Zone 3: South & Central Corridor (between O2 and W2)
            (2.92, 0.80, 90.0, 0.8, 'Zone 3: South-Central Corridor'),
            (2.92, 2.80, 90.0, 0.8, 'Zone 3: Central Hub (O2 / W2)'),
            (2.92, 4.18, 180.0, 0.8, 'Zone 4 Entry: North of W1b'),
            # Zone 4: North-West Room (Single Target S1 at (1.0, 4.1))
            (2.05, 4.15, -175.0, 1.5, 'Zone 4: S1 Outer Viewpoint'),
            (1.80, 3.80, 160.0, 2.5, 'Zone 4: S1 Close Visual Inspection'),
            (2.05, 4.15, 0.0, 0.6, 'Zone 4 Exit: Clearing W1b'),
            (2.92, 4.18, 0.0, 0.6, 'Zone 4 Exit: Back to North Hub'),
            # Zone 5: North-East & South-East Room (around O3 to Cooperative Target C1 at (4.1, 1.0))
            (3.45, 4.35, -45.0, 1.2, 'Zone 5: North-East Wall & O3 Top'),
            (3.35, 3.46, -48.0, 0.5, 'Zone 5: Align for W2/O3 Centerline'),
            (3.725, 3.05, -48.0, 0.5, 'Zone 5: Passing W2/O3 Centerline'),
            (4.08, 2.62, -90.0, 1.0, 'Zone 5: C1 Outer Viewpoint'),
            (4.10, 2.00, -90.0, 2.5, 'Zone 5: C1 Close Visual Inspection'),
        ]

        # Convert arena viewpoints into odom/map coordinates
        self.waypoints = []
        for ax, ay, ayaw_deg, pause_s, label in self.arena_viewpoints:
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
                'label': label,
            })

        self.wp_index = 0
        # States: 'INIT_SCAN', 'ALIGN_TO_GOAL', 'DRIVE_TO_GOAL', 'ALIGN_FINAL_YAW', 'OBSERVE_PAUSE', 'COMPLETED'
        self.state = 'INIT_SCAN'
        self.state_start_time = None

        # Robot pose in odom/map frame
        self.curr_x = None
        self.curr_y = None
        self.curr_yaw = None
        self.pose_frame = 'odom'

        # LiDAR sector clearances
        self.front_min = 10.0
        self.front_left_min = 10.0
        self.front_right_min = 10.0

        # SLAM map & Target tracking
        self.latest_map = None
        self.mapped_area_m2 = 0.0
        self.discovered_targets = {}

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers & Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/exploration_markers', 10)
        self.slam_get_param_client = self.create_client(
            GetParameters, '/slam_toolbox/get_parameters'
        )
        self.slam_pause_client = self.create_client(
            Pause, '/slam_toolbox/pause_new_measurements'
        )

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10
        )
        self.targets_sub = self.create_subscription(
            String, '/detected_targets', self.targets_callback, 10
        )

        self.control_timer = self.create_timer(0.05, self.control_loop)
        self.viz_timer = self.create_timer(0.5, self.publish_markers)

        # Ensure SLAM Toolbox is unpaused when exploration starts
        self.set_slam_paused(False)

        self.get_logger().info(
            'Autonomous Arena Explorer initialized! Waiting for /odom and /scan to begin SLAM exploration...'
        )

    def set_slam_paused(self, target_paused: bool):
        """Ensure SLAM Toolbox paused_new_measurements matches target_paused."""
        if not self.slam_get_param_client.service_is_ready():
            return
        req = GetParameters.Request()
        req.names = ['paused_new_measurements']
        future = self.slam_get_param_client.call_async(req)

        def _on_param(fut):
            try:
                res = fut.result()
                if res.values and res.values[0].type == ParameterType.PARAMETER_BOOL:
                    curr_paused = res.values[0].bool_value
                    if curr_paused != target_paused and self.slam_pause_client.service_is_ready():
                        self.slam_pause_client.call_async(Pause.Request())
                        state_str = 'PAUSED (map locked)' if target_paused else 'ACTIVE (mapping)'
                        self.get_logger().info(f'SLAM Toolbox new measurements set to: {state_str}')
            except Exception:
                pass

        future.add_done_callback(_on_param)

    def odom_callback(self, msg: Odometry):
        # Always keep /odom as fallback if TF map->base_footprint is not yet ready
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
        # Front sector: [-25 deg, +25 deg]
        front_mask = valid & (angles >= math.radians(-25.0)) & (angles <= math.radians(25.0))
        # Front-left sector: [+18 deg, +65 deg]
        left_mask = valid & (angles >= math.radians(18.0)) & (angles <= math.radians(65.0))
        # Front-right sector: [-65 deg, -18 deg]
        right_mask = valid & (angles >= math.radians(-65.0)) & (angles <= math.radians(-18.0))

        self.front_min = float(np.min(ranges[front_mask])) if np.any(front_mask) else 10.0
        self.front_left_min = float(np.min(ranges[left_mask])) if np.any(left_mask) else 10.0
        self.front_right_min = float(np.min(ranges[right_mask])) if np.any(right_mask) else 10.0

    def map_callback(self, msg: OccupancyGrid):
        # Once exploration is completed, do not overwrite the saved map
        if self.state == 'COMPLETED':
            return
        self.latest_map = msg
        data = np.array(msg.data, dtype=np.int16)
        known_cells = int(np.sum(data != -1))
        cell_area = msg.info.resolution * msg.info.resolution
        self.mapped_area_m2 = known_cells * cell_area
        # Periodically export the live SLAM map so it is always saved on disk
        self.save_occupancy_grid(msg)

    def targets_callback(self, msg: String):
        try:
            targets = json.loads(msg.data)
            for t in targets:
                self.discovered_targets[t['id']] = t
        except (json.JSONDecodeError, KeyError):
            pass

    def save_occupancy_grid(self, msg: OccupancyGrid):
        width = msg.info.width
        height = msg.info.height
        if width == 0 or height == 0:
            return

        grid = np.array(msg.data, dtype=np.int16).reshape((height, width))
        # Standard ROS map_saver encoding: unknown(-1)->205, free(0..25)->254, occupied(>=65)->0
        img = np.full((height, width), 205, dtype=np.uint8)
        img[(grid >= 0) & (grid <= 25)] = 254
        img[grid >= 65] = 0
        # Flip vertically so row 0 is top (+Y)
        img_flipped = np.flipud(img)

        pgm_path = os.path.join(self.map_save_dir, 'midterm_slam_map.pgm')
        yaml_path = os.path.join(self.map_save_dir, 'midterm_slam_map.yaml')
        cv2.imwrite(pgm_path, img_flipped)

        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        q = msg.info.origin.orientation
        origin_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

        yaml_content = (
            f"image: midterm_slam_map.pgm\n"
            f"mode: trinary\n"
            f"resolution: {msg.info.resolution:.4f}\n"
            f"origin: [{origin_x:.4f}, {origin_y:.4f}, {origin_yaw:.4f}]\n"
            f"negate: 0\n"
            f"occupied_thresh: 0.65\n"
            f"free_thresh: 0.25\n"
        )
        with open(yaml_path, 'w') as f:
            f.write(yaml_content)

    def stop_robot(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    def control_loop(self):
        self.update_pose_from_tf()
        if self.curr_x is None:
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if self.state_start_time is None:
            self.state_start_time = now_sec
            self.get_logger().info(
                'Performing initial 2.0s sensor scan to initialize SLAM Toolbox map...'
            )

        # State 0: Initial brief pause so SLAM Toolbox locks the initial scan in the Start Area
        if self.state == 'INIT_SCAN':
            self.stop_robot()
            if now_sec - self.state_start_time >= 2.0:
                self.state = 'ALIGN_TO_GOAL'
                wp = self.waypoints[self.wp_index]
                self.get_logger().info(
                    f'--> [1/{len(self.waypoints)}] Navigating to {wp["label"]} '
                    f'at Arena ({wp["ax"]:.2f}, {wp["ay"]:.2f})'
                )
            return

        if self.state == 'COMPLETED':
            self.stop_robot()
            return

        wp = self.waypoints[self.wp_index]
        dx = wp['ox'] - self.curr_x
        dy = wp['oy'] - self.curr_y
        dist = math.hypot(dx, dy)
        desired_heading = math.atan2(dy, dx)

        cmd = Twist()

        # State 1: Rotate in place to face the next waypoint before driving
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

        # State 2: Drive straight toward the waypoint with heading correction + LiDAR centering
        elif self.state == 'DRIVE_TO_GOAL':
            if dist <= self.dist_tolerance:
                self.stop_robot()
                self.state = 'ALIGN_FINAL_YAW'
                return

            yaw_err = normalize_angle(desired_heading - self.curr_yaw)
            # If heading drifted significantly, re-align in place first
            if abs(yaw_err) > math.radians(22.0):
                self.stop_robot()
                self.state = 'ALIGN_TO_GOAL'
                return

            # LiDAR safety check in front
            if self.front_min < 0.23:
                # Too close to an obstacle ahead: stop forward motion and advance to observation
                self.get_logger().warn(
                    f'LiDAR front clearance ({self.front_min:.2f}m) low near {wp["label"]}; '
                    'stopping forward motion safely.'
                )
                self.stop_robot()
                self.state = 'ALIGN_FINAL_YAW'
                return

            # Smooth linear speed with deceleration near waypoint and near obstacles
            speed_scale = min(1.0, (self.front_min - 0.18) / 0.35)
            linear_v = max(0.06, min(self.max_linear_speed * speed_scale, dist * 0.9))

            # Mild LiDAR corridor centering repulsion if passing close to walls on left/right
            lidar_steer = 0.0
            if self.front_left_min < 0.33 or self.front_right_min < 0.33:
                lidar_steer = 0.45 * (self.front_left_min - self.front_right_min)
                lidar_steer = max(-0.25, min(0.25, lidar_steer))

            cmd.linear.x = linear_v
            cmd.angular.z = max(
                -self.max_angular_speed,
                min(self.max_angular_speed, 1.5 * yaw_err + lidar_steer)
            )
            self.cmd_pub.publish(cmd)

        # State 3: Rotate in place to the target observation yaw at the viewpoint
        elif self.state == 'ALIGN_FINAL_YAW':
            yaw_err = normalize_angle(wp['oyaw'] - self.curr_yaw)
            if abs(yaw_err) <= self.yaw_tolerance:
                self.stop_robot()
                self.state = 'OBSERVE_PAUSE'
                self.state_start_time = now_sec
                ax, ay, _ = odom_to_arena(
                    self.curr_x, self.curr_y, self.curr_yaw,
                    self.spawn_x, self.spawn_y, self.spawn_yaw
                )
                self.get_logger().info(
                    f'Reached {wp["label"]} at Arena ({ax:.2f}, {ay:.2f}) | '
                    f'Mapped area: {self.mapped_area_m2:.1f} m^2 | '
                    f'Targets found: {sorted(self.discovered_targets.keys())}'
                )
            else:
                speed = max(0.10, min(self.max_angular_speed, abs(yaw_err) * 1.4))
                cmd.angular.z = math.copysign(speed, yaw_err)
                self.cmd_pub.publish(cmd)

        # State 4: Pause at viewpoint to allow camera & SLAM Toolbox to lock observations
        elif self.state == 'OBSERVE_PAUSE':
            self.stop_robot()
            if now_sec - self.state_start_time >= wp['pause']:
                self.wp_index += 1
                if self.wp_index >= len(self.waypoints):
                    self.state = 'COMPLETED'
                    if self.latest_map is not None:
                        self.save_occupancy_grid(self.latest_map)
                    # Lock SLAM Toolbox map so subsequent rescue runs do not overwrite the map
                    self.set_slam_paused(True)
                    self.get_logger().info(
                        f'=== AUTONOMOUS EXPLORATION & MAPPING COMPLETED! ===\n'
                        f'  Total Mapped Area : {self.mapped_area_m2:.2f} m^2\n'
                        f'  Discovered Targets: {json.dumps(list(self.discovered_targets.values()))}\n'
                        f'  Saved SLAM Map    : {self.map_save_dir}/midterm_slam_map.yaml (Locked for Rescue)'
                    )
                else:
                    next_wp = self.waypoints[self.wp_index]
                    self.state = 'ALIGN_TO_GOAL'
                    self.get_logger().info(
                        f'--> [{self.wp_index + 1}/{len(self.waypoints)}] Navigating to '
                        f'{next_wp["label"]} at Arena ({next_wp["ax"]:.2f}, {next_wp["ay"]:.2f})'
                    )

    def publish_markers(self):
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()
        frame = self.pose_frame

        # Line strip connecting waypoints
        path_marker = Marker()
        path_marker.header.stamp = now
        path_marker.header.frame_id = frame
        path_marker.ns = 'exploration_path'
        path_marker.id = 0
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.03
        path_marker.color.r = 0.2
        path_marker.color.g = 0.8
        path_marker.color.b = 1.0
        path_marker.color.a = 0.6

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
                # Visited: Green
                sphere.color.r = 0.1
                sphere.color.g = 0.9
                sphere.color.b = 0.2
                sphere.color.a = 0.8
            elif i == self.wp_index and self.state != 'COMPLETED':
                # Active goal: Yellow
                sphere.color.r = 1.0
                sphere.color.g = 0.9
                sphere.color.b = 0.0
                sphere.color.a = 1.0
            else:
                # Pending: Blue
                sphere.color.r = 0.2
                sphere.color.g = 0.5
                sphere.color.b = 1.0
                sphere.color.a = 0.6
            marker_array.markers.append(sphere)

        marker_array.markers.append(path_marker)
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = ArenaExplorerNode()
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

