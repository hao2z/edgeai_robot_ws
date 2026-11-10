#!/usr/bin/env python3
"""Run a bounded single-robot acceptance test in a private Gazebo session."""
import argparse
import csv
import datetime as dt
import json
import hashlib
import platform
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, LaserScan, PointCloud2, CameraInfo, JointState
from tf2_ros import Buffer, TransformListener
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V
from PIL import Image as PILImage
import yaml

ROOT = Path(__file__).resolve().parents[1]


def yaw(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def pose(p):
    return (p.position.x, p.position.y, yaw(p.orientation))


class Probe(Node):
    def __init__(self):
        super().__init__('single_robot_baseline_probe')
        self.latest, self.received, self.counts = {}, {}, {}
        self.truth, self.truth_at = None, 0.0
        self.trace = []
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        topics = {'/clock': Clock, '/odom': Odometry, '/scan': LaserScan,
                  '/camera/image': Image, '/camera/depth_image': Image,
                  '/camera/camera_info': CameraInfo, '/camera/points': PointCloud2,
                  '/joint_states': JointState, '/map': OccupancyGrid, '/baseline/reloaded_map': OccupancyGrid}
        self.subscriptions_kept = []
        for topic, msg_type in topics.items():
            qos = qos_profile_sensor_data
            if topic in ('/map', '/baseline/reloaded_map'):
                qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
            self.subscriptions_kept.append(self.create_subscription(
                msg_type, topic, lambda msg, t=topic: self.receive(t, msg), qos))
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.gz = GzNode()
        self.gz.subscribe(Pose_V, '/world/empty/dynamic_pose/info', self.on_truth)

    def receive(self, topic, msg):
        self.latest[topic] = msg
        self.received[topic] = time.monotonic()
        self.counts[topic] = self.counts.get(topic, 0) + 1

    def on_truth(self, msg):
        for item in msg.pose:
            if item.name == 'edgeai_robot_real':
                self.truth = pose(item)
                self.truth_at = time.monotonic()
                break

    def tick(self):
        rclpy.spin_once(self, timeout_sec=0.02)
        if self.truth and '/odom' in self.latest and '/clock' in self.latest:
            c = self.latest['/clock'].clock
            row = [c.sec + c.nanosec * 1e-9, *self.truth, *pose(self.latest['/odom'].pose.pose)]
            if not self.trace or row[0] != self.trace[-1][0]:
                self.trace.append(row)

    def wait_for(self, predicate, timeout, description):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.tick()
            if predicate():
                return
        raise RuntimeError('Timed out waiting for ' + description)

    def pump(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.tick()

    def fresh(self, topics, age=3):
        now = time.monotonic()
        return all(now - self.received.get(t, 0) < age for t in topics)

    def sim_time(self):
        c = self.latest['/clock'].clock
        return c.sec + c.nanosec * 1e-9

    def zero(self):
        self.publisher.publish(Twist())

    def drive(self, linear, angular, duration):
        start = self.sim_time()
        deadline = time.monotonic() + 45
        try:
            while self.sim_time() - start < duration:
                self.tick()
                if time.monotonic() > deadline:
                    raise RuntimeError('Simulation stalled during motion')
                if not self.fresh(['/clock', '/scan', '/odom']) or time.monotonic() - self.truth_at > 3:
                    raise RuntimeError('Lost sensor data or Gazebo ground truth during motion')
                if linear:
                    scan = self.latest['/scan']
                    front = [r for i, r in enumerate(scan.ranges)
                             if abs(angle(scan.angle_min + i*scan.angle_increment)) < math.pi/4
                             and math.isfinite(r) and r >= scan.range_min]
                    if not front or min(front) < 0.55:
                        raise RuntimeError('Insufficient forward clearance')
                # The test stays close to its known free starting area.
                if math.hypot(self.truth[0] + 1.5, self.truth[1] + 1.5) > 0.65:
                    raise RuntimeError('Robot left the baseline test area')
                command = Twist()
                command.linear.x, command.angular.z = linear, angular
                self.publisher.publish(command)
                time.sleep(0.025)
        finally:
            # Stopping publication alone does not stop Gazebo DiffDrive.
            for _ in range(8):
                self.zero()
                self.pump(0.05)

    def snapshot(self):
        return {'ground_truth': list(self.truth), 'odom': list(pose(self.latest['/odom'].pose.pose))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gui', action='store_true', help='Open Gazebo and RViz during the test')
    parser.add_argument('--output', type=Path, help='New directory for report, map and logs')
    parser.add_argument('--world', type=Path, default=ROOT/'src/edgeai_robot_description/worlds/empty.sdf',
                        help='SDF world to test (world name must be empty)')
    args = parser.parse_args()
    world = args.world.resolve()
    try:
        world_element = ET.parse(world).getroot().find('world')
    except (OSError, ET.ParseError) as exc:
        parser.error(f'Cannot read --world: {exc}')
    if world_element is None or world_element.get('name') != 'empty':
        parser.error('--world must be an existing SDF with world name empty')
    if not os.environ.get('GZ_PARTITION', '').startswith('edgeai_baseline_') or os.environ.get('ROS_DOMAIN_ID') != '71':
        parser.error('Use scripts/run_single_robot_baseline.sh for isolated simulation discovery')
    output = (args.output or ROOT / 'artifacts' / ('baseline_' + dt.datetime.now().strftime('%Y%m%d_%H%M%S'))).resolve()
    output.mkdir(parents=True, exist_ok=False)
    children, files = [], []
    report = {'status': 'failed', 'started_at': dt.datetime.now(dt.timezone.utc).isoformat(),
              'git_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
              'git_dirty': bool(subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT, text=True).strip()),
              'platform': platform.platform(), 'python_version': platform.python_version(),
              'world_file': str(world), 'world_sha256': hashlib.sha256(world.read_bytes()).hexdigest(),
              'pixi_lock_sha256': hashlib.sha256((ROOT/'pixi.lock').read_bytes()).hexdigest(),
              'gazebo_version': subprocess.check_output(['gz', 'sim', '--version'], text=True, timeout=15).strip(),
              'ros_distro': os.environ.get('ROS_DISTRO'), 'ros_domain_id': 71, 'gz_partition': os.environ['GZ_PARTITION'], 'checks': {}}
    rclpy.init()
    probe = Probe()

    def launch(name, command):
        log = open(output / (name + '.log'), 'w')
        files.append(log)
        child = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        children.append(child)
        return child

    def check(name, valid, **details):
        report['checks'][name] = {'passed': bool(valid), **details}
        print(('PASS ' if valid else 'FAIL ') + name, details, flush=True)
        if not valid:
            raise RuntimeError('Check failed: ' + name)

    def interrupted(signum, frame):
        raise KeyboardInterrupt()

    signal.signal(signal.SIGTERM, interrupted)
    try:
        # gz_args overrides the included simulator arguments; -s keeps headless
        # server rendering sensors without a Gazebo GUI. A graphics driver is still needed.
        command = ['ros2', 'launch', 'edgeai_robot_description', 'simulation_real.launch.py']
        command.append(f'gz_args:=-r {"" if args.gui else "-s "}{world}')
        launch('simulation', command)
        required = ['/clock', '/odom', '/scan', '/camera/image', '/camera/depth_image',
                    '/camera/camera_info', '/camera/points', '/joint_states']
        probe.wait_for(lambda: probe.fresh(required) and probe.truth is not None, 90, 'all sensors and ground truth')
        t0 = probe.sim_time()
        probe.wait_for(lambda: probe.sim_time() > t0 + 0.25, 15, 'advancing simulation clock')
        check('sensor_streams', True, messages=dict(probe.counts))
        scan = probe.latest['/scan']
        check('lidar', len(scan.ranges) == 360 and scan.header.frame_id == 'lidar_link'
              and sum(math.isfinite(r) and scan.range_min <= r <= scan.range_max for r in scan.ranges) > 10,
              samples=len(scan.ranges), frame=scan.header.frame_id)
        rgb, depth = probe.latest['/camera/image'], probe.latest['/camera/depth_image']
        depth_values = np.ndarray((depth.height, depth.width), dtype=np.dtype('>f4' if depth.is_bigendian else '<f4'),
                                  buffer=depth.data, strides=(depth.step, 4))
        check('camera', rgb.width == depth.width == 640 and rgb.height == depth.height == 480
              and rgb.encoding == 'rgb8' and depth.encoding == '32FC1'
              and np.any(np.isfinite(depth_values) & (depth_values > 0))
              and probe.latest['/camera/points'].width * probe.latest['/camera/points'].height > 0,
              rgb_encoding=rgb.encoding, depth_encoding=depth.encoding)
        rgb_array = np.ndarray((rgb.height, rgb.width, 3), dtype=np.uint8,
                               buffer=rgb.data, strides=(rgb.step, 3, 1))
        PILImage.fromarray(rgb_array.copy()).save(output / 'camera.png')
        pairs = [('odom', 'base_footprint'), ('base_footprint', 'lidar_link'),
                 ('base_footprint', 'camera_link'), ('camera_link', probe.latest['/camera/points'].header.frame_id)]
        for parent, child in pairs:
            probe.wait_for(lambda p=parent, c=child: probe.tf.can_transform(p, c, Time()), 20, f'TF {parent} -> {child}')
        check('tf_chain', True, transforms=[list(p) for p in pairs])
        check('spawn_pose', math.hypot(probe.truth[0]+1.5, probe.truth[1]+1.5) < 0.05
              and abs(probe.truth[2]) < 0.05, ground_truth=list(probe.truth))
        others = [p for p in probe.get_publishers_info_by_topic('/cmd_vel')
                  if p.node_name != probe.get_name() and p.node_name != 'ros_gz_bridge']
        check('exclusive_control', not others, other_publishers=[p.node_name for p in others])
        launch('slam', ['ros2', 'launch', 'slam_toolbox', 'online_async_launch.py',
                        f'slam_params_file:={ROOT}/src/edgeai_robot_description/config/slam_toolbox.yaml',
                        'use_sim_time:=true'])
        probe.wait_for(lambda: '/map' in probe.latest and probe.tf.can_transform('map', 'base_footprint', Time()),
                       60, 'SLAM map and map-to-robot transform')
        if args.gui:
            import copy
            config = yaml.safe_load((ROOT / 'src/edgeai_robot_description/rviz/realistic_robot.rviz').read_text())
            config = copy.deepcopy(config)
            config['Visualization Manager']['Global Options']['Fixed Frame'] = 'map'
            for display in config['Visualization Manager']['Displays']:
                if display.get('Class') == 'rviz_default_plugins/Map':
                    display['Enabled'] = display['Value'] = True
            rviz_config = output / 'baseline.rviz'
            rviz_config.write_text(yaml.safe_dump(config))
            launch('rviz', ['rviz2', '-d', str(rviz_config), '--ros-args', '-p', 'use_sim_time:=true'])
        start = probe.snapshot()
        print('Moving forward at 0.1 m/s for 2 simulated seconds', flush=True)
        probe.drive(0.1, 0.0, 2.0)
        forward = probe.snapshot()
        gt = math.dist(start['ground_truth'][:2], forward['ground_truth'][:2])
        odom = math.dist(start['odom'][:2], forward['odom'][:2])
        heading = start['ground_truth'][2]
        dx = forward['ground_truth'][0] - start['ground_truth'][0]
        dy = forward['ground_truth'][1] - start['ground_truth'][1]
        check('forward_motion', 0.12 < gt < 0.30 and abs(gt-odom) < 0.05
              and dx*math.cos(heading)+dy*math.sin(heading) > 0.12
              and abs(-dx*math.sin(heading)+dy*math.cos(heading)) < 0.04,
              ground_truth_m=gt, odom_m=odom)
        print('Turning counterclockwise at 0.2 rad/s for 2 simulated seconds', flush=True)
        probe.drive(0.0, 0.2, 2.0)
        turned = probe.snapshot()
        gt_angle = angle(turned['ground_truth'][2]-forward['ground_truth'][2])
        odom_angle = angle(turned['odom'][2]-forward['odom'][2])
        check('turn_motion', 0.25 < gt_angle < 0.60 and abs(gt_angle-odom_angle) < 0.10
              and math.dist(forward['ground_truth'][:2], turned['ground_truth'][:2]) < 0.05,
              ground_truth_rad=gt_angle, odom_rad=odom_angle,
              translation_m=math.dist(forward['ground_truth'][:2], turned['ground_truth'][:2]))
        # Commands are now zero. Observe 1.5 simulated seconds without further commands.
        t0 = probe.sim_time()
        probe.wait_for(lambda: probe.sim_time() >= t0+1.5, 30, 'stop observation')
        stopped = probe.snapshot()
        drift = math.dist(turned['ground_truth'][:2], stopped['ground_truth'][:2])
        rotation = abs(angle(stopped['ground_truth'][2]-turned['ground_truth'][2]))
        check('explicit_stop', drift < 0.01 and rotation < 0.03, drift_m=drift, rotation_rad=rotation)
        report['poses'] = {'start': start, 'after_forward': forward, 'after_turn': turned, 'stopped': stopped}
        # Wait for a fresh SLAM map after movement, not just the initial map.
        map_count = probe.counts['/map']
        probe.wait_for(lambda: probe.counts['/map'] > map_count, 30, 'updated SLAM map')
        grid = probe.latest['/map']
        known = sum(v >= 0 for v in grid.data)
        occupied = sum(v >= 65 for v in grid.data)
        check('slam_map', grid.header.frame_id == 'map' and abs(grid.info.resolution-0.05) < 1e-6
              and known > 100 and occupied > 10,
              width=grid.info.width, height=grid.info.height, resolution=grid.info.resolution,
              known_cells=known, occupied_cells=occupied)
        saved = subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(output/'map'),
                                '--ros-args', '-p', 'use_sim_time:=true', '-p', 'save_map_timeout:=15.0'],
                               cwd=ROOT, capture_output=True, text=True, timeout=30)
        (output/'map_saver.log').write_text(saved.stdout+saved.stderr)
        check('map_export', saved.returncode == 0 and (output/'map.yaml').exists(), returncode=saved.returncode)
        metadata = yaml.safe_load((output/'map.yaml').read_text())
        image = output / metadata['image']
        with PILImage.open(image) as im:
            check('saved_map_image', im.size == (grid.info.width, grid.info.height)
                  and abs(metadata['resolution']-0.05) < 1e-6, image=metadata['image'], size=list(im.size))
            im.convert('RGB').save(output/'map.png')
        launch('map_server', ['ros2', 'run', 'nav2_map_server', 'map_server', '--ros-args',
                              '-r', '__node:=baseline_map_server', '-r', 'map:=/baseline/reloaded_map',
                              '-p', f'yaml_filename:={output}/map.yaml', '-p', 'use_sim_time:=true'])
        for transition in ('configure', 'activate'):
            result = subprocess.run(['ros2', 'lifecycle', 'set', '/baseline_map_server', transition],
                                    capture_output=True, text=True, timeout=30)
            (output/f'map_server_{transition}.log').write_text(result.stdout+result.stderr)
            check('map_server_'+transition, result.returncode == 0 and 'successful' in result.stdout,
                  output=result.stdout.strip())
        probe.wait_for(lambda: '/baseline/reloaded_map' in probe.latest, 20, 'reloaded map publication')
        reloaded = probe.latest['/baseline/reloaded_map']
        check('map_reload', reloaded.info.width == grid.info.width and reloaded.info.height == grid.info.height
              and abs(reloaded.info.resolution-grid.info.resolution) < 1e-6
              and math.dist(pose(reloaded.info.origin)[:2], pose(grid.info.origin)[:2]) < 0.001
              and sum(v >= 65 for v in reloaded.data) > 10,
              width=reloaded.info.width, height=reloaded.info.height,
              origin_rounding_error_m=math.dist(pose(reloaded.info.origin)[:2], pose(grid.info.origin)[:2]),
              occupied_cells=sum(v >= 65 for v in reloaded.data))
        report['status'] = 'passed'
    except (Exception, KeyboardInterrupt) as exc:
        report['error'] = str(exc) or 'Interrupted'
        print('Baseline failed:', report['error'], flush=True)
    finally:
        for _ in range(5):
            probe.zero()
            probe.pump(0.05)
        for child in reversed(children):
            try:
                os.killpg(child.pid, signal.SIGINT)
                child.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGTERM)
                try:
                    child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait()
            except ProcessLookupError:
                pass
        for file in files:
            file.close()
        with open(output/'trajectory.csv', 'w') as file:
            writer = csv.writer(file)
            writer.writerow(['sim_time', 'truth_x', 'truth_y', 'truth_yaw', 'odom_x', 'odom_y', 'odom_yaw'])
            writer.writerows(probe.trace)
        if probe.trace and 'poses' in report:
            # Align odom's initial pose to the Gazebo world for a fair visual comparison.
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            trace = np.array(probe.trace)
            start = report['poses']['start']
            heading = start['ground_truth'][2] - start['odom'][2]
            rotation = np.array([[math.cos(heading), -math.sin(heading)],
                                 [math.sin(heading), math.cos(heading)]])
            aligned = (trace[:, 4:6] - start['odom'][:2]) @ rotation.T + start['ground_truth'][:2]
            fig, axes = plt.subplots(1, 2, figsize=(11, 4), layout='constrained')
            axes[0].plot(trace[:, 1], trace[:, 2], label='Gazebo ground truth')
            axes[0].plot(aligned[:, 0], aligned[:, 1], '--', label='Odometry (aligned)')
            axes[0].set(xlabel='World x (m)', ylabel='World y (m)', title='Forward, turn and stop', aspect='equal')
            axes[1].plot(trace[:, 0], np.degrees(trace[:, 3]), label='Gazebo ground truth')
            axes[1].plot(trace[:, 0], np.degrees(trace[:, 6]+heading), '--', label='Odometry (aligned)')
            axes[1].set(xlabel='Simulation time (s)', ylabel='Heading (degrees)', title='Heading over time')
            for ax in axes:
                ax.legend()
                ax.grid(alpha=0.3)
            fig.savefig(output/'trajectory.png', dpi=160)
            plt.close(fig)
        report['sensor_message_counts'] = probe.counts
        report['finished_at'] = dt.datetime.now(dt.timezone.utc).isoformat()
        (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
        rows = ['# 单机基线验收报告', '', f"结果：{report['status']}", '',
                '| 检查 | 结果 | 数据 |', '| --- | --- | --- |']
        for name, details in report['checks'].items():
            rows.append(f"| {name} | {'通过' if details['passed'] else '失败'} | {json.dumps({k:v for k,v in details.items() if k != 'passed'}, ensure_ascii=False)} |")
        if 'error' in report:
            rows += ['', report['error']]
        rows += ['', '这是已知练习场地中的短距离验收，不代表自主探索、完整地图覆盖或实机导航通过。',
                 'Gazebo 真值只用于验收，没有输入 SLAM 或控制目标。轨迹 CSV 同时保留世界真值和 odom 坐标，两者原点不同。']
        (output/'REPORT.md').write_text('\n'.join(rows)+'\n')
        probe.destroy_node()
        rclpy.shutdown()
        print('Results:', output, flush=True)
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
