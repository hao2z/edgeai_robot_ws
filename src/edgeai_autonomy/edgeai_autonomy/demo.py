"""Practice scene supervisor and independent Gazebo-truth evaluator.

Fixture coordinates are deliberately confined to this test harness. The mission
and detector receive only the marker dictionary/size and practice rescue rules.
"""
import argparse
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
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from tf2_ros import Buffer, TransformListener
from ament_index_python.packages import get_package_share_directory
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V
from PIL import Image
from scipy.spatial.transform import Rotation
import yaml
from .geometry import matrix, yaw
from .acceptance import clearance_report, rectangle, collision_polygons, segment_hits_polygon


def fixture(output, scenario):
    description = Path(get_package_share_directory('edgeai_robot_description'))
    assets = Path(get_package_share_directory('edgeai_autonomy'))/'models/aruco_practice'
    source = description/'worlds'/('competition_rooms.sdf' if scenario == 'rooms' else 'empty.sdf')
    sdf = ET.parse(source)
    if scenario == 'north':
        position, normal, angles = [-0.8, 0.743, 0.19], [0., -1., 0.], [math.pi/2, 0., 0.]
    elif scenario == 'east':
        position, normal, angles = [0.943, 0.5, 0.19], [-1., 0., 0.], [math.pi/2, 0., -math.pi/2]
    elif scenario == 'rooms':
        # Test-only marker inside the upper-left room. Keep the supplied SDF
        # unchanged; only this generated copy contains a practice target.
        position, normal, angles = [-1.75, 2.493, 0.19], [0., -1., 0.], [math.pi/2, 0., 0.]
    else:
        # Back of obstacle_1: invisible from the spawn side, with the straight
        # start-to-rescue route passing through its collision box.
        position, normal, angles = [-0.8, 1.057, 0.19], [0., 1., 0.], [math.pi/2, 0., math.pi]
    model = ET.SubElement(sdf.getroot().find('world'), 'model', name='practice_marker_23')
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = ' '.join(map(str, position+angles))
    link = ET.SubElement(model, 'link', name='board')
    # Render the machine-readable fiducial as flat colored cells. This avoids
    # renderer-specific UV/material behavior that can silently erase a texture.
    from PIL import Image as PILImage
    pixels = np.array(PILImage.open(assets/'marker_23.png').convert('L'))
    for row in range(8):
        for col in range(8):
            white = pixels[row*64+32, col*64+32] > 127
            visual = ET.SubElement(link, 'visual', name=f'cell_{row}_{col}')
            ET.SubElement(visual, 'pose').text = f'{(col-3.5)*0.04} {(3.5-row)*0.04} 0 0 0 0'
            box = ET.SubElement(ET.SubElement(visual, 'geometry'), 'box')
            ET.SubElement(box, 'size').text = '0.04 0.04 0.001'
            ET.SubElement(visual, 'cast_shadows').text = 'false'
            material = ET.SubElement(visual, 'material')
            color = '1 1 1 1' if white else '0 0 0 1'
            for name in ('ambient', 'diffuse', 'emissive'):
                ET.SubElement(material, name).text = color
    world = output/'practice.sdf'
    sdf.write(world, encoding='unicode', xml_declaration=True)
    metadata = {'scenario': scenario, 'marker_id': 23, 'marker_position_world': position,
                'marker_normal_world': normal, 'practice_only': True,
                'source_world': str(source.resolve()),
                'source_world_sha256': hashlib.sha256(source.read_bytes()).hexdigest()}
    (output/'fixture.json').write_text(json.dumps(metadata, indent=2)+'\n')
    return world, metadata


class Monitor(Node):
    def __init__(self):
        super().__init__('autonomy_acceptance_monitor')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.truth, self.sim_time, self.trace = None, 0.0, []
        self.create_subscription(Clock, '/clock', self.clock, qos_profile_sensor_data)
        self.gz = GzNode()
        self.gz.subscribe(Pose_V, '/world/empty/dynamic_pose/info', self.on_truth)

    def clock(self, msg):
        self.sim_time = msg.clock.sec+msg.clock.nanosec*1e-9

    def on_truth(self, msg):
        for pose in msg.pose:
            if pose.name == 'edgeai_robot_real':
                self.truth = (pose.position.x, pose.position.y, yaw(pose.orientation))
                break

    def tick(self):
        rclpy.spin_once(self, timeout_sec=0.02)
        if self.truth and (not self.trace or self.sim_time-self.trace[-1][0] >= 0.05):
            self.trace.append([self.sim_time, *self.truth])


def evaluate(output, monitor, map_to_world, scene):
    mission = json.loads((output/'mission.json').read_text())
    cfg = mission['practice_parameters']
    result = {'status': 'failed', 'mission_status': mission['status'], 'checks': {},
              'map_to_world': map_to_world.tolist()}

    def check(name, ok, **details):
        result['checks'][name] = {'passed': bool(ok), **details}

    check('mission_completed', mission['status'] == 'completed')
    check('frontier_navigation', mission.get('completed_frontier_goals', 0) >= cfg['exploration_min_goals'],
          completed_goals=mission.get('completed_frontier_goals', 0),
          failed_attempts=sum(e['state']=='navigation_result' and e.get('purpose')=='exploration_frontier'
                              and not e.get('succeeded') for e in mission.get('events', [])))
    clearance = clearance_report(output/'practice.sdf', monitor.trace)
    check('ground_truth_obstacle_clearance', clearance['overlap_samples'] == 0, **clearance)
    check('map_and_frozen_targets', (output/'map.yaml').exists() and (output/'targets.json').exists())
    if 'target' in mission:
        target = mission['target']
        target_world = (map_to_world @ np.r_[target['position'], 1])[:3]
        target_error = float(np.linalg.norm(target_world-np.array(scene['marker_position_world'])))
        normal_world = map_to_world[:3,:3] @ target['normal']
        normal_error = math.degrees(math.acos(float(np.clip(np.dot(normal_world, scene['marker_normal_world']), -1, 1))))
        check('target_localization', target_error < 0.15 and normal_error < 20,
              error_m=target_error, normal_error_degrees=normal_error, estimated_world=target_world.tolist())
        zone = np.array(scene['marker_position_world'][:2])+np.array(scene['marker_normal_world'][:2])*cfg['rescue_standoff_m']
        trace = np.array(monitor.trace)
        dwell = trace[(trace[:,0] >= mission['dwell_started_sim_time']) & (trace[:,0] <= mission['completed_sim_time'])]
        distance = np.linalg.norm(dwell[:,1:3]-zone, axis=1) if len(dwell) else np.array([math.inf])
        duration = dwell[-1,0]-dwell[0,0] if len(dwell)>1 else 0.
        movement = float(np.max(np.linalg.norm(dwell[:,1:3]-dwell[0,1:3], axis=1))) if len(dwell) else math.inf
        check('ground_truth_continuous_dwell', duration >= cfg['dwell_seconds']-0.2
              and np.max(distance) <= cfg['rescue_radius_m'] and movement < 0.03,
              sampled_duration_s=float(duration), max_zone_error_m=float(np.max(distance)), movement_m=movement,
              rescue_zone_world=zone.tolist())
    if scene['scenario'] in ('behind', 'rooms'):
        trace = np.array(monitor.trace)
        start = trace[0, 1:3]
        zone = np.array(scene['marker_position_world'][:2])+np.array(scene['marker_normal_world'][:2])*cfg['rescue_standoff_m']
        obstacles = collision_polygons(output/'practice.sdf')
        hidden = any(segment_hits_polygon(start, scene['marker_position_world'][:2], p) for p in obstacles)
        blocked = any(segment_hits_polygon(start, zone, p) for p in obstacles)
        vector = zone-start
        deviations = np.abs(vector[0]*(trace[:,2]-start[1])-vector[1]*(trace[:,1]-start[0]))/np.linalg.norm(vector)
        confirmations = [e for e in mission['events'] if e['state'] == 'target_confirmed']
        first_pose = None
        if confirmations:
            first_pose = trace[np.argmin(abs(trace[:,0]-confirmations[0]['sim_time'])),1:3].tolist()
        check('occluded_target_detour', hidden and blocked and float(deviations.max()) > 0.3
              and mission['status'] == 'completed',
              initial_marker_occluded=hidden, straight_rescue_route_blocked=blocked,
              maximum_lateral_detour_m=float(deviations.max()),
              travelled_distance_m=float(np.linalg.norm(np.diff(trace[:,1:3],axis=0),axis=1).sum()),
              straight_distance_m=float(np.linalg.norm(vector)),
              first_confirmed_robot_world=first_pose)
    if scene['scenario'] == 'rooms':
        trace = np.array(monitor.trace)
        entered = (trace[:,1] < -1.05) & (trace[:,2] > 1.4)
        check('upper_left_room_entry', bool(np.any(entered)), samples_inside=int(entered.sum()))
    expected_checks = {'behind': 7, 'rooms': 8}.get(scene['scenario'], 6)
    if all(c['passed'] for c in result['checks'].values()) and len(result['checks']) == expected_checks:
        result['status'] = 'passed'
    (output/'evaluation.json').write_text(json.dumps(result, indent=2)+'\n')
    rows = ['# 自主闭环仿真验收', '', f"结果：{result['status']}", '',
            '| 检查 | 结果 | 数据 |', '| --- | --- | --- |']
    for name, detail in result['checks'].items():
        rows.append(f"| {name} | {'通过' if detail['passed'] else '失败'} | {json.dumps(detail, ensure_ascii=False)} |")
    rows += ['', '仅针对练习场地和练习 ArUco/救援参数；不是官方比赛参数或实机验收。',
             '任务节点未读取真值或 fixture.json；真值由独立验收器在执行后比较。']
    (output/'REPORT.md').write_text('\n'.join(rows)+'\n')
    if (output/'map.yaml').exists():
        metadata = yaml.safe_load((output/'map.yaml').read_text())
        Image.open(output/metadata['image']).convert('RGB').save(output/'map.png')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon, Circle
    fig, ax = plt.subplots(figsize=(10,7), layout='constrained')
    obstacle_label = True
    for model in ET.parse(output/'practice.sdf').getroot().find('world').findall('model'):
        values = list(map(float, (model.findtext('pose') or '0 0 0 0 0 0').split()))
        for box in model.findall('./link/collision/geometry/box'):
            dimensions = list(map(float, box.findtext('size').split()))
            ax.add_patch(Polygon(rectangle(values[0],values[1],values[5],*dimensions[:2]),
                                 color='0.3', label='Static obstacle' if obstacle_label else None))
            obstacle_label = False
    zone = np.array(scene['marker_position_world'][:2])+np.array(scene['marker_normal_world'][:2])*cfg['rescue_standoff_m']
    ax.add_patch(Circle(zone, cfg['rescue_radius_m'], color='green', alpha=0.25, label='Practice rescue zone'))
    trace = np.array(monitor.trace)
    if trace.size:
        if scene['scenario'] in ('behind', 'rooms'):
            ax.plot([trace[0,1],zone[0]], [trace[0,2],zone[1]], ':', color='red',
                    label='Blocked direct route')
        ax.plot(trace[:,1], trace[:,2], label='Gazebo ground truth')
        ax.scatter(*trace[0,1:3],marker='o',color='blue',s=20,label='Start')
        completed = mission['status'] == 'completed'
        ax.scatter(*trace[-1,1:3], marker='*' if completed else 'X',
                   color='green' if completed else 'crimson', s=100,
                   label='Finish' if completed else 'End (mission failed)')
        if scene['scenario'] in ('behind', 'rooms') and first_pose is not None:
            ax.scatter(*first_pose, marker='D', color='purple', s=35, label='First visual confirmation')
    poses = np.array(mission.get('trajectory_map', []))
    if poses.size:
        world = (map_to_world @ np.c_[poses[:,1:3], np.zeros(len(poses)), np.ones(len(poses))].T).T
        ax.plot(world[:,0], world[:,1], '--', label='SLAM estimated pose')
    ax.scatter(*scene['marker_position_world'][:2], marker='s', label='Practice marker (evaluation only)')
    ax.set(xlabel='World x (m)', ylabel='World y (m)', xlim=(-2.6,2.6), ylim=(-2.6,2.6), aspect='equal')
    ax.grid(alpha=0.3);ax.legend(loc='upper left', bbox_to_anchor=(1.02,1), fontsize=8)
    fig.savefig(output/'trajectory.png', dpi=150, bbox_inches='tight');plt.close(fig)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--scenario', choices=['north', 'east', 'behind', 'rooms'], default='north')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--max-exploration-goals', type=int, help='Bound this test run; includes observation goals')
    args = parser.parse_args()
    if args.max_exploration_goals is not None and args.max_exploration_goals < 1:
        parser.error('--max-exploration-goals must be positive')
    if not os.environ.get('GZ_PARTITION','').startswith('edgeai_autonomy_') or os.environ.get('ROS_DOMAIN_ID') != '72':
        parser.error('Use scripts/run_autonomy_demo.sh for isolated simulation discovery')
    output = (args.output or Path.cwd()/'artifacts'/('autonomy_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S'))).resolve()
    output.mkdir(parents=True, exist_ok=False)
    world, scene = fixture(output, args.scenario)
    package = Path(get_package_share_directory('edgeai_autonomy'))
    description = Path(get_package_share_directory('edgeai_robot_description'))
    for config in ('practice.yaml', 'nav2.yaml', 'slam.yaml'):
        (output/config).write_text((package/'config'/config).read_text())
    if args.max_exploration_goals is not None:
        config = yaml.safe_load((output/'practice.yaml').read_text())
        config['exploration_max_goals'] = args.max_exploration_goals
        (output/'practice.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
    manifest = {'started_at_utc': dt.datetime.now(dt.timezone.utc).isoformat(),
                'scenario': args.scenario, 'platform': platform.platform(),
                'git_commit': subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
                'git_dirty': bool(subprocess.check_output(['git','status','--porcelain'],text=True).strip()),
                'pixi_lock_sha256': hashlib.sha256((Path.cwd()/'pixi.lock').read_bytes()).hexdigest(),
                'ros_domain_id':72, 'gz_partition':os.environ['GZ_PARTITION']}
    (output/'run.json').write_text(json.dumps(manifest,indent=2)+'\n')
    children, logs = [], []
    rclpy.init()
    monitor = Monitor()
    evaluation = {'status': 'failed'}

    def launch(name, command):
        file = open(output/(name+'.log'), 'w');logs.append(file)
        child = subprocess.Popen(command, stdout=file, stderr=subprocess.STDOUT, start_new_session=True)
        children.append(child)
        return child

    def wait(predicate, seconds, message):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            monitor.tick()
            if predicate():
                return
        raise RuntimeError('Timed out: '+message)

    def interrupted(signum, frame):
        raise KeyboardInterrupt()

    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGINT, interrupted)
    try:
        print('Output:', output, flush=True)
        launch('simulation', ['ros2','launch','edgeai_robot_description','simulation_real.launch.py',
                              f'gz_args:=-r {"" if args.gui else "-s "}{world}',
                              'odom_output_topic:=/odom_raw', 'odom_tf_output_topic:=/tf_raw'])
        launch('drive_odometry', ['ros2','run','edgeai_autonomy','drive_odometry'])
        launch('slam', ['ros2','launch','slam_toolbox','online_async_launch.py',
                        f'slam_params_file:={package}/config/slam.yaml','use_sim_time:=true'])
        wait(lambda: monitor.truth is not None and monitor.tf.can_transform('map','base_footprint',Time()), 100, 'simulation and SLAM')
        # Initial alignment is used by the evaluator only; mission receives no world pose.
        initial_map = matrix(monitor.tf.lookup_transform('map','base_footprint',Time()).transform)
        initial_world = np.eye(4)
        initial_world[:3,:3] = Rotation.from_euler('z',monitor.truth[2]).as_matrix()
        initial_world[:2,3] = monitor.truth[:2]
        map_to_world = initial_world @ np.linalg.inv(initial_map)
        common = ['--ros-args','-p',f'config:={output}/practice.yaml','-p',f'output:={output}']
        launch('detector', ['ros2','run','edgeai_autonomy','aruco_detector',*common])
        launch('guard', ['ros2','run','edgeai_autonomy','velocity_guard'])
        launch('navigation', ['ros2','launch','edgeai_autonomy','navigation.launch.py'])
        if args.gui:
            config = yaml.safe_load((description/'rviz/realistic_robot.rviz').read_text())
            config['Visualization Manager']['Global Options']['Fixed Frame']='map'
            for display in config['Visualization Manager']['Displays']:
                if display.get('Class') == 'rviz_default_plugins/Map':
                    display['Enabled']=display['Value']=True
            (output/'autonomy.rviz').write_text(yaml.safe_dump(config))
            launch('rviz', ['rviz2','-d',str(output/'autonomy.rviz'),'--ros-args','-p','use_sim_time:=true'])
        mission = launch('mission', ['ros2','run','edgeai_autonomy','mission',*common])
        last_status, next_print = '', 0.
        deadline = time.monotonic()+2400
        while mission.poll() is None:
            monitor.tick()
            if time.monotonic() > deadline:
                raise RuntimeError('Demo wall time budget exceeded')
            if time.monotonic() > next_print:
                events=output/'events.jsonl'
                if events.exists():
                    lines=events.read_text().splitlines()
                    status=lines[-1] if lines else ''
                    if status and status!=last_status:
                        print(status,flush=True);last_status=status
                next_print=time.monotonic()+2
            if any(c.poll() is not None for c in children if c is not mission):
                raise RuntimeError('A required process exited; inspect logs')
        wait(lambda: True, 1, 'last samples')
        evaluation=evaluate(output,monitor,map_to_world,scene)
        print(json.dumps(evaluation,indent=2),flush=True)
    except (Exception,KeyboardInterrupt) as exc:
        (output/'supervisor_error.txt').write_text(str(exc) or 'Interrupted')
        print('Demo failed:',str(exc) or 'Interrupted',flush=True)
    finally:
        # Stop mission/heartbeat first; guard fails closed before simulator exits.
        for child in reversed(children):
            try:
                os.killpg(child.pid,signal.SIGINT)
                child.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGTERM)
                try:child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid,signal.SIGKILL);child.wait()
            except ProcessLookupError:
                pass
        np.savetxt(output/'ground_truth.csv',np.array(monitor.trace),delimiter=',',
                   header='sim_time,x,y,yaw',comments='')
        for file in logs:file.close()
        monitor.destroy_node();rclpy.shutdown()
    print('Results:',output,flush=True)
    raise SystemExit(0 if evaluation['status']=='passed' else 1)
