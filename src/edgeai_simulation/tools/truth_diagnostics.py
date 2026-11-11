#!/usr/bin/env python3
"""Low-rate, robot-aligned Gazebo truth overlay and timestamped coordinate audit."""
import copy
from collections import deque
import json
import math
from pathlib import Path
import signal
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException
from visualization_msgs.msg import Marker, MarkerArray
import yaml

from truth_geometry import compose, inverse, planar


def sdf_pose(element):
    values = [float(v) for v in element.findtext('pose', '0 0 0 0 0 0').split()]
    if element.find('pose') is not None and element.find('pose').get('relative_to'):
        raise ValueError('Truth overlay requires absolute/default SDF poses')
    if abs(values[3])+abs(values[4]) > 1e-6:
        raise ValueError('Truth overlay supports planar scene poses only')
    return (values[0], values[1], values[5]), values[2]


class TruthDiagnostics(Node):
    def __init__(self):
        super().__init__('simulation_truth')
        for key, default in [('world_file', ''), ('targets_file', ''), ('entity', 'edgeai_robot_real'),
                             ('frame_prefix', ''), ('reference_frame', 'map'), ('log_directory', ''), ('shared_map', False)]:
            self.declare_parameter(key, default)
        p = lambda key: self.get_parameter(key).value
        self.entity = p('entity')
        self.map_frame, self.base_frame = p('frame_prefix')+p('reference_frame'), p('frame_prefix')+'base_footprint'
        if p('shared_map'):self.map_frame='map'
        self.directory = Path(p('log_directory')) if p('log_directory') else None
        self.audit = None
        if self.directory:
            self.directory.mkdir(parents=True, exist_ok=True)
            name = 'truth.jsonl' if not p('frame_prefix') else self.entity+'-truth.jsonl'
            self.audit = (self.directory/name).open('a', buffering=1)
        self.targets = yaml.safe_load(Path(p('targets_file')).read_text())['targets']
        if self.directory and not p('frame_prefix'):
            snapshot=self.directory/'truth-targets.json'
            temporary=snapshot.with_suffix('.tmp')
            temporary.write_text(json.dumps(dict(source=p('targets_file'),targets_world=[
                dict(id=t['id'],pose=[t['pose'][0],t['pose'][1],t['pose'][5]]) for t in self.targets])))
            temporary.replace(snapshot)
        self.scene = self.load_scene(p('world_file'))
        self.truth = self.goal = None
        self.sim_time = self.nav_received = self.truth_received = 0.0
        self.phase, self.task, self.goal_received = 'UNKNOWN', None, None
        self.buffer = Buffer(cache_time=Duration(seconds=2))
        self.pending_tf = deque(maxlen=512)
        # Only the two dynamic navigation edges are needed. Avoid processing
        # wheel/link TF and static robot geometry at sensor frequency.
        self.tf_edges = {(self.map_frame, p('frame_prefix')+'odom'),
                         (p('frame_prefix')+'odom', self.base_frame)}
        self.create_subscription(TFMessage, '/tf', self.receive_tf, qos_profile_sensor_data)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.markers = self.create_publisher(MarkerArray, 'truth/markers', qos)
        self.summary = self.create_publisher(String, 'truth/summary', qos)
        self.create_subscription(TFMessage, 'truth/poses', self.receive_truth, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, 'navigation_authority/dispatched_goal', self.receive_goal, qos)
        self.create_subscription(String, 'navigation_authority/state', self.receive_state, qos)
        self.create_timer(1.0, self.tick)
        self.get_logger().info('Truth diagnostics: 2 Hz pose input, 1 Hz robot-aligned overlay/audit; no navigation TF writes')

    def receive_truth(self, msg):
        for tf in msg.transforms:
            if tf.child_frame_id == self.entity:
                self.truth = tf
                self.truth_received = time.monotonic()

    def receive_tf(self, msg):
        for tf in msg.transforms:
            if (tf.header.frame_id, tf.child_frame_id) in self.tf_edges:
                self.pending_tf.append(tf)
                self.sim_time = max(self.sim_time, tf.header.stamp.sec+tf.header.stamp.nanosec/1e9)
                self.nav_received = time.monotonic()

    def receive_goal(self, msg):
        self.goal = msg
        self.goal_received = self.sim_time

    def receive_state(self, msg):
        state = json.loads(msg.data)
        self.phase, self.task = state.get('phase', 'UNKNOWN'), state.get('generation')

    def marker(self, name, kind, pose, z, scale, color):
        m = Marker(ns=name, id=len(self.scene) if hasattr(self, 'scene') else 0, type=kind,
                   action=Marker.ADD)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(pose[0]), float(pose[1]), float(z)
        m.pose.orientation.z, m.pose.orientation.w = math.sin(pose[2]/2), math.cos(pose[2]/2)
        m.scale.x, m.scale.y, m.scale.z = map(float, scale)
        m.color.r, m.color.g, m.color.b, m.color.a = map(float, color)
        m.lifetime.sec = 2
        return m

    def load_scene(self, path):
        result = []
        world = ET.parse(path).getroot().find('world')
        for model in world.findall('model'):
            if model.findtext('static', 'false') != 'true':
                continue
            mp, mz = sdf_pose(model)
            for link in model.findall('link'):
                lp, lz = sdf_pose(link)
                for collision in link.findall('collision'):
                    cp, cz = sdf_pose(collision)
                    geom = collision.find('geometry')
                    if geom.find('box') is None:
                        continue
                    size = [float(v) for v in geom.findtext('box/size').split()]
                    if size[2] < .01:
                        continue
                    m = self.marker('truth_scene', Marker.CUBE, compose(compose(mp, lp), cp),
                                    mz+lz+cz, size, (.1, .55, 1, .25))
                    m.id = len(result)
                    result.append(m)
        for target in self.targets:
            x, y, z, _, _, angle = target['pose']
            m = self.marker('truth_target', Marker.ARROW, (x, y, angle), z,
                            (.3, .025, .045), (0, .7, 1, 1))
            m.id = target['id']
            result.append(m)
            label = self.marker('truth_label', Marker.TEXT_VIEW_FACING, (x, y, 0), z+.2,
                                (0, 0, .12), (0, .7, 1, 1))
            label.id, label.text = target['id'], 'TRUTH '+target['name']
            result.append(label)
        return result

    def tick(self):
        # Sample timestamps supply simulation time. A wall timer avoids the
        # high-rate /clock subscription dominating Python diagnostic CPU use.
        now = self.sim_time
        row = dict(sim_time=now, status='unavailable', phase=self.phase, task=self.task,
                   reference_frame=self.map_frame, alignment='robot_at_sample_time; visualization_only',
                   time_source='latest_navigation_tf')
        overlay = []
        while self.pending_tf:
            self.buffer.set_transform(self.pending_tf.popleft(), 'simulation_truth')
        try:
            if self.truth is None:
                raise ValueError('No Gazebo world pose received')
            stamp = Time.from_msg(self.truth.header.stamp)
            world_base = planar(self.truth.transform.translation, self.truth.transform.rotation)
            # Rescue verification needs Gazebo poses even if map/odom TF fails.
            row.update(sample_time=stamp.nanoseconds/1e9, robot_world=world_base,
                       truth_valid=time.monotonic()-self.truth_received<=3,
                       targets_world=[dict(id=t['id'], pose=[t['pose'][0], t['pose'][1], t['pose'][5]]) for t in self.targets])
            if not row['truth_valid'] or time.monotonic()-self.nav_received > 2:
                raise ValueError('Truth or navigation TF stream stopped')
            age = now-stamp.nanoseconds/1e9
            row.update(sample_time=stamp.nanoseconds/1e9, pose_age_s=age)
            if not 0 <= age <= .75:
                raise ValueError('Gazebo world pose stale or from future')
            # Never fall back to latest TF: both robot poses refer to this stamp.
            tf = self.buffer.lookup_transform(self.map_frame, self.base_frame, stamp).transform
            map_base = planar(tf.translation, tf.rotation)
            world_map = compose(world_base, inverse(map_base))
            map_world = inverse(world_map)
            row.update(status='ok', robot_world=world_base, robot_estimated=map_base, world_from_reference=world_map,
                       targets_world=[dict(id=t['id'], pose=[t['pose'][0], t['pose'][1], t['pose'][5]]) for t in self.targets])
            overlay = copy.deepcopy(self.scene)
            for m in overlay:
                pose = compose(map_world, planar(m.pose.position, m.pose.orientation))
                m.pose.position.x, m.pose.position.y = pose[:2]
                m.pose.orientation.z, m.pose.orientation.w = math.sin(pose[2]/2), math.cos(pose[2]/2)
            overlay.append(self.marker('truth_robot', Marker.ARROW, map_base, .1,
                                       (.35, .045, .07), (0, .7, 1, 1)))
            if self.goal is not None:
                row['goal_frame'] = self.goal.header.frame_id
                row['goal_received_sim_s'] = self.goal_received
                row['goal_stamp_s'] = self.goal.header.stamp.sec+self.goal.header.stamp.nanosec/1e9
                if self.goal.header.frame_id != self.map_frame:
                    row['goal_error'] = 'Goal frame differs from diagnostic reference frame'
                else:
                    goal = planar(self.goal.pose.position, self.goal.pose.orientation)
                    gw = compose(world_map, goal)
                    row.update(goal_estimated=goal, goal_world=gw,
                               goal_to_targets_m={str(t['id']): math.hypot(gw[0]-t['pose'][0], gw[1]-t['pose'][1])
                                                  for t in self.targets})
                    overlay.append(self.marker('truth_goal', Marker.ARROW, goal, .12,
                                               (.3, .05, .08), (1, .45, 0, 1)))
            if self.directory and (self.directory/'targets.json').exists():
                try:
                    registry = json.loads((self.directory/'targets.json').read_text())
                    if registry.get('frame_id') == self.map_frame:
                        row['target_registry_stamp_s'] = registry.get('stamp_ns', 0)/1e9
                        row['target_estimates'] = []
                        for t in registry.get('targets', []):
                            actual = next((a for a in self.targets if a['id'] == t['id']), None)
                            if actual is None or t.get('yaw') is None:
                                continue
                            tw = compose(world_map, (t['x'], t['y'], t['yaw']))
                            row['target_estimates'].append(dict(id=t['id'], world=tw,
                                observation_age_s=now-t['last_observed_ns']/1e9,
                                error_m=math.hypot(tw[0]-actual['pose'][0], tw[1]-actual['pose'][1])))
                except (OSError, ValueError, KeyError) as error:
                    row['target_error'] = str(error)
        except (ValueError, TransformException) as error:
            row['reason'] = str(error)
        # DELETEALL also removes misleading old overlays when synchronization fails.
        clear = Marker(action=Marker.DELETEALL)
        for m in [clear]+overlay:
            m.header.frame_id = self.map_frame
            m.header.stamp = Time(seconds=now).to_msg()
        self.markers.publish(MarkerArray(markers=[clear]+overlay))
        encoded = json.dumps(row, separators=(',', ':'), allow_nan=False)
        self.summary.publish(String(data=encoded))
        if self.audit:
            self.audit.write(encoded+'\n')

    def destroy_node(self):
        if self.audit:
            self.audit.close()
        super().destroy_node()


def main():
    # Stop spin before shutting down its context (also clean on launch SIGTERM).
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    def stop(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    node = TruthDiagnostics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
