#!/usr/bin/env python3
"""One-shot scan-only initial odom alignment; never broadcasts TF.

API: optional config_file YAML joint_mapping.initializer supplies ROS parameter
defaults; top-level initialization (the joint_mapping.yaml layout) and
joint_mapping.initial_relative_pose are also supported. Explicit ROS
parameters win. The prior relates the initial base frames, sampled at the first
usable scan pair (robots should remain still until initialization completes).
global_odom can be anchored by the consumer to robot1/odom with identity.

Output /joint_mapping/alignment is reliable/transient-local std_msgs/String JSON.
Only accepted=true permits consumption of odom1_from_odom2 [x,y,yaw]. It maps
current odom2 coordinates into odom1. That transform is frozen on confirmation.
covariance is a row-major local 3x3 fit estimate in x,y,yaw, not absolute accuracy.
No ground truth, shared-map TF, or external localization is read.
"""
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                      qos_profile_sensor_data)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException

from edgeai_mapping.relative_registration import (
    RegistrationConfig, TemporalConfirmation, compose, inverse, transform,
    scan_points, remove_peer, register_scans)


DEFAULTS = dict(
    initial_relative_pose=[0., -.6, 0.], processing_hz=2., timeout_s=45.,
    output_directory='', scan1_topic='/robot1/scan', scan2_topic='/robot2/scan',
    odom1_frame='robot1/odom', odom2_frame='robot2/odom',
    base1_frame='robot1/base_footprint', base2_frame='robot2/base_footprint',
    max_scan_age_s=1.5, max_pair_skew_s=.15, confirmation_count=3,
    confirmation_interval_s=.4, confirmation_translation_m=.08,
    confirmation_yaw_rad=.06, peer_bounds=[-.3125, .1125, -.165, .165],
    peer_margin_m=.12)


def config_defaults(filename):
    if not filename:
        return {}
    import yaml
    with open(filename, encoding='utf-8') as stream:
        doc = yaml.safe_load(stream) or {}
    settings = doc.get('joint_mapping', {})
    result = dict(doc.get('initialization', {}))
    result.update(settings.get('initializer', {}))
    if 'initial_relative_pose' in settings:
        result.setdefault('initial_relative_pose', settings['initial_relative_pose'])
    unknown = set(result)-set(DEFAULTS)-set(RegistrationConfig.__dataclass_fields__)
    if unknown:
        raise ValueError('Unknown initializer settings: '+', '.join(sorted(unknown)))
    return result


def planar(tf):
    q = tf.rotation
    return [tf.translation.x, tf.translation.y,
            math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))]


class JointMappingInitializer(Node):
    def __init__(self, **options):
        super().__init__('joint_mapping_initializer', **options)
        self.declare_parameter('config_file', '')
        configured = config_defaults(self.get_parameter('config_file').value)
        defaults = dict(DEFAULTS, **vars(RegistrationConfig()))
        for key, value in defaults.items():
            self.declare_parameter(key, configured.get(key, value))
        self.p = {key: self.get_parameter(key).value for key in defaults}
        self.cfg = RegistrationConfig(**{k: self.p[k] for k in vars(RegistrationConfig())})
        self.confirmation = TemporalConfirmation(
            self.p['confirmation_count'], self.p['confirmation_interval_s'],
            self.p['confirmation_translation_m'], self.p['confirmation_yaw_rad'])
        prior, bounds = self.p['initial_relative_pose'], self.p['peer_bounds']
        if (len(prior) != 3 or not all(math.isfinite(v) for v in prior)
                or len(bounds) != 4 or not all(math.isfinite(v) for v in bounds)
                or bounds[0] >= bounds[1] or bounds[2] >= bounds[3]
                or not math.isfinite(self.p['peer_margin_m']) or self.p['peer_margin_m'] < 0
                or any(not math.isfinite(self.p[k]) or self.p[k] <= 0 for k in
                       ('processing_hz', 'timeout_s', 'max_scan_age_s', 'max_pair_skew_s'))
                or self.p['processing_hz'] > 2.):
            raise ValueError('Invalid initializer geometry/timing settings (processing_hz <= 2)')
        for k in ('scan1_topic', 'scan2_topic', 'odom1_frame', 'odom2_frame',
                  'base1_frame', 'base2_frame'):
            if not self.p[k]:
                raise ValueError('Empty '+k)
        self.started = time.monotonic()
        self.done = False
        self.latest = {}
        self.prior = None
        self.initial_samples = None
        self.last_attempt = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(String, '/joint_mapping/alignment', qos)
        self.create_subscription(LaserScan, self.p['scan1_topic'],
                                 lambda msg: self.receive('primary', msg), qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.p['scan2_topic'],
                                 lambda msg: self.receive('secondary', msg), qos_profile_sensor_data)
        self.report_path = (Path(self.p['output_directory'])/'joint-initialization.json'
                            if self.p['output_directory'] else None)
        self.publish(dict(accepted=False, state='waiting', reason='waiting_for_scans',
                          odom1_from_odom2=None))
        self.create_timer(1./self.p['processing_hz'], self.process)

    def receive(self, key, msg):
        stamp = Time.from_msg(msg.header.stamp).nanoseconds
        previous = self.latest.get(key)
        if previous is None or stamp > Time.from_msg(previous.header.stamp).nanoseconds:
            self.latest[key] = msg

    def publish(self, record):
        record = dict(record, schema_version=1, primary='robot1', secondary='robot2',
                      global_odom_from_odom1=[0., 0., 0.],
                      elapsed_s=time.monotonic()-self.started,
                      initial_relative_pose=list(self.p['initial_relative_pose']))
        record['parameters'] = dict(self.p)
        record.setdefault('parent_frame', self.p['odom1_frame'])
        record.setdefault('child_frame', self.p['odom2_frame'])
        for key, value in dict(residual=None, overlap=0., covariance=None,
                               odom1_from_odom2=None, confirmations=0).items():
            record.setdefault(key, value)
        payload = json.dumps(record, allow_nan=False, sort_keys=True)
        if self.report_path is not None:
            try:
                self.report_path.parent.mkdir(parents=True, exist_ok=True)
                temporary = self.report_path.with_suffix('.json.tmp')
                temporary.write_text(payload+'\n', encoding='utf-8')
                temporary.replace(self.report_path)
            except OSError as exc:
                # Reporting failure must be visible without discarding scan evidence.
                self.get_logger().error('Cannot persist initialization report: '+str(exc))
        self.publisher.publish(String(data=payload))
        self.last_record = record

    def sample(self, scan, index):
        stamp = Time.from_msg(scan.header.stamp)
        if stamp.nanoseconds <= 0:
            raise ValueError('zero_scan_timestamp')
        age = (self.get_clock().now().nanoseconds-stamp.nanoseconds)/1e9
        if not 0 <= age <= self.p['max_scan_age_s']:
            raise ValueError('stale_or_future_scan')
        odom = self.p[f'odom{index}_frame']
        base = self.p[f'base{index}_frame']
        laser_tf = self.buffer.lookup_transform(odom, scan.header.frame_id, stamp)
        base_tf = self.buffer.lookup_transform(odom, base, stamp)
        offset_tf = self.buffer.lookup_transform(base, scan.header.frame_id, stamp)
        laser_pose, base_pose, offset = (planar(t.transform) for t in
                                         (laser_tf, base_tf, offset_tf))
        if not all(math.isfinite(v) for p in (laser_pose, base_pose, offset) for v in p):
            raise ValueError('nonfinite_tf')
        points = scan_points(scan.ranges, scan.angle_min, scan.angle_increment,
                             scan.range_min, scan.range_max, self.cfg.max_points)
        return transform(points, laser_pose), dict(
            scan_stamp_ns=stamp.nanoseconds, scan_frame=scan.header.frame_id,
            odom_frame=odom, base_frame=base, scan_age_s=age,
            odom_from_base=base_pose, odom_from_laser=laser_pose,
            base_from_laser=offset, tf_query_stamp_ns=stamp.nanoseconds,
            odom_pose_stamp_ns=Time.from_msg(base_tf.header.stamp).nanoseconds,
            laser_tf_stamp_ns=Time.from_msg(laser_tf.header.stamp).nanoseconds,
            laser_offset_stamp_ns=Time.from_msg(offset_tf.header.stamp).nanoseconds)

    def process(self):
        if self.done:
            return
        if time.monotonic()-self.started >= self.p['timeout_s']:
            self.done = True
            self.publish(dict(self.last_record, accepted=False, state='failed', reason='deadline',
                              last_reason=self.last_record['reason'], odom1_from_odom2=None))
            return
        now = self.get_clock().now().nanoseconds
        primary = self.latest.get('primary')
        secondary = self.latest.get('secondary')
        if primary is None or secondary is None:
            return
        stamps_ns = tuple(Time.from_msg(s.header.stamp).nanoseconds for s in (primary, secondary))
        try:
            if abs(stamps_ns[0]-stamps_ns[1])/1e9 > self.p['max_pair_skew_s']:
                raise ValueError('scan_pair_skew')
            # Do not mark attempted before TF lookup: latest scans must be retried
            # when TF catches up, even if no newer scan arrives.
            target, sample1 = self.sample(primary, 1)
            source, sample2 = self.sample(secondary, 2)
            key = (stamps_ns, sample1['odom_frame'], sample2['odom_frame'])
            if key == self.last_attempt:
                return
            self.last_attempt = key
            if self.prior is None:
                self.prior = compose(compose(sample1['odom_from_base'],
                                             self.p['initial_relative_pose']),
                                     inverse(sample2['odom_from_base']))
                self.initial_samples = [sample1, sample2]
                self.odom_frames = [sample1['odom_frame'], sample2['odom_frame']]
            if self.odom_frames != [sample1['odom_frame'], sample2['odom_frame']]:
                raise ValueError('odom_frame_changed')
            # Both clouds are already in their own odom frames. Predict peer
            # base at the other scan's timestamp; skew is explicitly bounded.
            peer2_in_odom1 = compose(self.prior, sample2['odom_from_base'])
            peer1_in_odom2 = compose(inverse(self.prior), sample1['odom_from_base'])
            sizes = [len(target), len(source)]
            target = remove_peer(target, inverse(peer2_in_odom1),
                                 self.p['peer_bounds'], self.p['peer_margin_m'])
            source = remove_peer(source, inverse(peer1_in_odom2),
                                 self.p['peer_bounds'], self.p['peer_margin_m'])
            candidate = register_scans(target, source, self.prior, self.cfg)
            if time.monotonic()-self.started >= self.p['timeout_s']:
                self.done = True
                self.publish(dict(candidate, accepted=False, state='failed', reason='deadline',
                                  odom1_from_odom2=None, source_samples=[sample1, sample2]))
                return
            confirmed = self.confirmation.observe(candidate, [s/1e9 for s in stamps_ns])
            record = dict(confirmed or candidate)
            record.update(accepted=confirmed is not None,
                          state='confirmed' if confirmed else 'collecting',
                          confirmations=len(self.confirmation.samples),
                          confirmation_scan_stamps_ns=[
                              [int(round(s*1e9)) for s in pair]
                              for pair in self.confirmation.sample_stamps],
                          required_confirmations=self.p['confirmation_count'],
                          source_samples=[sample1, sample2], initial_samples=self.initial_samples,
                          prior_odom1_from_odom2=self.prior.tolist(),
                          peer_points_removed=[sizes[0]-len(target), sizes[1]-len(source)],
                          parent_frame=self.odom_frames[0], child_frame=self.odom_frames[1])
            # Alias frame IDs must match the numeric coordinates actually used.
            self.publish(record)
            if confirmed:
                self.done = True
                self.get_logger().info('Joint mapping initial alignment confirmed and frozen')
        except (TransformException, ValueError) as exc:
            self.publish(dict(accepted=False, state='waiting', reason=str(exc),
                              odom1_from_odom2=None))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = JointMappingInitializer()
        rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
