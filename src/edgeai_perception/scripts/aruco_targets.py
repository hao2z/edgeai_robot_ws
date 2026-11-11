#!/usr/bin/env python3
"""Observe targets without owning navigation or using Gazebo target truth."""
import json
from collections import deque, Counter
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock,ClockType
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException
from aruco_geometry import detector, estimate, transform_point, rotation_matrix, rotation_quaternion, fuse_quaternions
from landmark_anchors import AnchorGraph


class Targets(Node):
    def __init__(self,**kwargs):
        super().__init__('aruco_targets',**kwargs)
        defaults = dict(image_topic='/camera/image', camera_info_topic='/camera/camera_info',
                        optical_frame='camera_optical_frame', dictionary='DICT_4X4_50',
                        marker_size_m=.20, processing_hz=30.0, max_image_age_s=.5,
                        min_marker_side_px=6.0, max_reprojection_error_px=2.5,
                        min_distance_m=.15, max_distance_m=5.0, confirmation_observations=3,
                        fusion_window_s=20.0, max_observations_per_id=20,
                        reobservation_window_s=1.0,
                        outlier_distance_m=.30, stale_after_s=10.0, output_directory='',
                        target_types='', min_marker_distance_rate=.02,
                        map_anchoring='slam_graph',anchor_max_distance_m=3.0,
                        anchor_processing_hz=5.0,target_publish_hz=5.0)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {k: self.get_parameter(k).value for k in defaults}
        if self.p['marker_size_m'] <= 0 or self.p['processing_hz'] <= 0:
            raise ValueError('Marker size and processing rate must be positive')
        if self.p['confirmation_observations'] < 1 or self.p['max_observations_per_id'] < self.p['confirmation_observations']:
            raise ValueError('Invalid confirmation/window capacity')
        self.detect = detector(self.p['dictionary'], self.p['min_marker_distance_rate'])
        self.bridge = CvBridge()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.latest = self.info = None
        self.last_stamp = None
        self.stats = Counter()
        self.processing_ms = deque(maxlen=200)
        self.decoded_ids = set()
        self.localization_waits=Counter();self.localization_notices={}
        self.stats_started = self.get_clock().now().nanoseconds
        self.records = {};self.reobservations={}
        if self.p['map_anchoring'] not in ('slam_graph','map'):raise ValueError('Invalid map_anchoring')
        if not np.isfinite(self.p['anchor_max_distance_m']) or self.p['anchor_max_distance_m']<=0:
            raise ValueError('anchor_max_distance_m must be positive and finite')
        for name in ('anchor_processing_hz','target_publish_hz','reobservation_window_s'):
            if not np.isfinite(self.p[name]) or self.p[name]<=0:
                raise ValueError(name+' must be positive and finite')
        self.mapping=self.p['map_anchoring']=='slam_graph';self.frozen=False;self.finalizing=False
        self.graph=AnchorGraph(max_distance=self.p['anchor_max_distance_m'])
        self.last_anchor_audit=None
        self.target_types = {}
        for item in str(self.p['target_types']).split(','):
            if ':' in item:
                key, value = item.split(':', 1)
                if key.strip().isdigit() and value.strip():
                    self.target_types[int(key)] = value.strip()
        self.confirmed = set()
        self.started = time.monotonic()
        self.last_warning = {}
        self.output = Path(self.p['output_directory']) if self.p['output_directory'] else None
        if self.output:
            self.output.mkdir(parents=True, exist_ok=True)
        self.create_subscription(Image, self.p['image_topic'], self.image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.p['camera_info_topic'], self.camera, qos_profile_sensor_data)
        self.observations = self.create_publisher(String, 'targets/observations', 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.estimates = self.create_publisher(String, 'targets/estimates', qos)
        self.markers = self.create_publisher(MarkerArray, 'targets/markers', qos)
        self.create_subscription(String,'/slam_toolbox/anchor_graph',self.graph_received,qos)
        self.finalize_group=ReentrantCallbackGroup()
        self.finalize_client=self.create_client(Trigger,'/slam_toolbox/finalize_anchors',callback_group=self.finalize_group)
        self.finalize_service=self.create_service(Trigger,'targets/finalize_map',self.finalize_map,callback_group=self.finalize_group)
        self.create_timer(1/self.p['processing_hz'], self.process)
        self.create_timer(1/self.p['anchor_processing_hz'], self.update_anchors)
        self.create_timer(1/self.p['target_publish_hz'], self.publish)
        self.create_timer(5.0, self.publish_diagnostics)
        self.get_logger().info('TARGET_SEARCH_READY: ArUco探测已启用；仅使用图像、内参和TF，不控制导航')

    def graph_received(self,msg):
        if not self.mapping:return
        try:self.update_graph(json.loads(msg.data))
        except (ValueError,KeyError,TypeError) as error:self.warn('TARGET_ANCHOR_GRAPH_INVALID: '+str(error))

    def update_graph(self,data):
        if self.graph.update(data):
            self.records.clear();self.confirmed.clear();self.reobservations.clear()
            self.get_logger().warning('TARGET_MAP_RESET: SLAM图已更换，清除旧图目标')
        self.update_anchors()

    def localization_wait(self,key,reason,stamp,**details):
        label=f'{key}:{reason}';self.localization_waits[label]+=1
        now=time.monotonic()
        if now-self.localization_notices.get(label,-100)<5.:return
        self.localization_notices[label]=now
        row=dict(id=int(key),reason=reason,stamp_ns=stamp,**details)
        self.get_logger().info('TARGET_LOCALIZATION_WAIT: '+json.dumps(row))

    def update_anchors(self):
        if not self.mapping:return
        for history in self.records.values():
            for index,(stamp,binding) in enumerate(history):
                if not binding.get('pending_anchor'):continue
                try:
                    anchored=self.graph.bind(stamp,binding['point'],binding['rotation'],
                                             binding['robot_map'],frame='map')
                except ValueError:continue
                history[index]=(stamp,anchored)
                self.stats['anchor_attached']+=1

    def accept(self,key,stamp,binding,row):
        point,_=self.graph.project(binding)
        history=self.records.setdefault(key,deque(maxlen=self.p['max_observations_per_id']))
        newest=max(stamp,history[-1][0] if history else stamp)
        cutoff=newest-self.p['fusion_window_s']*1e9
        if stamp<cutoff:return
        while history and history[0][0]<cutoff:history.popleft()
        if len(history)>=self.p['confirmation_observations']:
            center=np.median([self.graph.project(h[1])[0] for h in history],axis=0)
            if np.linalg.norm(point-center)>self.p['outlier_distance_m']:
                self.stats['position_outlier']+=1
                candidates=self.reobservations.setdefault(key,deque(maxlen=self.p['confirmation_observations']))
                if candidates and stamp<=candidates[-1][0]:return
                cutoff=stamp-self.p['reobservation_window_s']*1e9
                while candidates and candidates[0][0]<cutoff:candidates.popleft()
                if candidates:
                    candidate_center=np.median([self.graph.project(item[1])[0] for item in candidates],axis=0)
                    if np.linalg.norm(point-candidate_center)>self.p['outlier_distance_m']:candidates.clear()
                candidates.append((stamp,binding,row))
                if len(candidates)<self.p['confirmation_observations']:
                    self.warn(f'TARGET_OUTLIER: ID={key} 观测偏离历史，等待新位置多帧确认');return
                replacement=list(candidates)
                self.reobservations.pop(key,None);history.clear()
                self.get_logger().info(f'TARGET_RELOCALIZED: ID={key} 新位置已通过多帧一致性检查，替换旧位置')
                for new_stamp,new_binding,new_row in replacement:self.accept(key,new_stamp,new_binding,new_row)
                return
        self.reobservations.pop(key,None)
        first=not history and key not in self.confirmed
        ordered=sorted([*history,(stamp,binding)],key=lambda item:item[0])
        history.clear();history.extend(ordered);self.stats['position_accepted']+=1
        row.update(anchor_kind=binding['kind'],anchor_node_id=binding.get('node_id'),
                   pending_anchor=binding.get('pending_anchor',False),map_epoch=self.graph.epoch)
        self.observations.publish(String(data=json.dumps(row)))
        if self.output:
            with (self.output/'target-observations.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
        if first:self.get_logger().info(f'TARGET_FOUND: 发现 ArUco ID={key}')
        if len(history)>=self.p['confirmation_observations'] and key not in self.confirmed:
            self.confirmed.add(key);self.get_logger().info(f'TARGET_CONFIRMED: 多帧确认 ArUco ID={key}')

    async def finalize_map(self,request,response):
        if self.finalizing:response.message='Target finalization already in progress';return response
        self.finalizing=True;timer=None
        try:
            if self.mapping:
                if not self.finalize_client.service_is_ready():raise RuntimeError('SLAM anchor finalizer unavailable')
                future=self.finalize_client.call_async(Trigger.Request())
                timer=self.create_timer(8.,lambda:future.cancel() if not future.done() else None,
                                        callback_group=self.finalize_group,clock=Clock(clock_type=ClockType.STEADY_TIME))
                snapshot=await future
                if snapshot is None or not snapshot.success:raise RuntimeError('SLAM map/anchor finalization failed')
                data=json.loads(snapshot.message)
                if not data.get('finalized'):raise ValueError('SLAM snapshot was not finalized')
                self.update_graph(data)
                frozen={key:deque(((stamp,self.graph.freeze(binding)) for stamp,binding in history),
                                  maxlen=history.maxlen) for key,history in self.records.items()}
                self.records=frozen;self.reobservations.clear();self.mapping=False
            self.frozen=True;self.publish()
            response.success=True;response.message=json.dumps(self.map_binding())
            self.get_logger().info('TARGET_MAP_FINALIZED: 最终地图与关键帧目标已定稿；后续新观测直接绑定静态地图')
        except Exception as error:response.message=str(error)
        finally:
            if timer:self.destroy_timer(timer)
            self.finalizing=False
        return response

    def map_binding(self):
        return dict(mode='slam_graph' if self.mapping else 'fixed_map',epoch=self.graph.epoch,
                    graph_revision=self.graph.revision,finalized=self.frozen)

    def warn(self, reason):
        now = time.monotonic()
        if now-self.last_warning.get(reason, -100) >= 10:
            self.get_logger().warning(reason)
            self.last_warning[reason] = now

    def image(self, msg):
        self.stats['received_frames'] += 1
        self.latest = msg

    def camera(self, msg):
        self.info = msg

    def process(self):
        if self.latest is None or self.info is None:
            if time.monotonic()-self.started > 10:
                self.warn('TARGET_INPUT_MISSING: 等待相机图像/CameraInfo；检查相机是否被禁用')
            return
        msg, info = self.latest, self.info
        stamp = Time.from_msg(msg.header.stamp)
        if stamp.nanoseconds == self.last_stamp:
            return
        now = self.get_clock().now()
        if not 0 <= (now-stamp).nanoseconds*1e-9 <= self.p['max_image_age_s']:
            self.stats['stale_image'] += 1
            self.warn('TARGET_STALE_IMAGE: 图像时间无效或过期，跳过')
            return
        if (msg.width, msg.height) != (info.width, info.height) or info.k[0] <= 0 or info.k[4] <= 0:
            self.warn('TARGET_BAD_CALIBRATION: 图像尺寸或内参无效')
            return
        if msg.header.frame_id != self.p['optical_frame']:
            self.warn('TARGET_FRAME_MISMATCH: 图像frame与配置光学frame不一致，拒绝定位：'+msg.header.frame_id)
            return
        if info.distortion_model not in ('plumb_bob', 'rational_polynomial', ''):
            self.warn('TARGET_BAD_CALIBRATION: 暂不支持该畸变模型')
            return
        process_started = time.perf_counter()
        ids=None
        try:
            image = self.bridge.imgmsg_to_cv2(msg, 'mono8')
            corners, ids, rejected = self.detect(image)
            self.stats['processed_frames'] += 1
            self.stats['undecoded_quadrilaterals'] += len(rejected)
            if ids is not None:
                self.stats['decoded_observations'] += len(ids)
                for marker_id in ids.flatten():
                    if int(marker_id) not in self.decoded_ids:
                        self.decoded_ids.add(int(marker_id))
                        self.get_logger().info(f'TARGET_DECODED: 图像已识别 ID={int(marker_id)}；定位仍需PnP和TF校验')
            self.last_stamp = stamp.nanoseconds
            if ids is None:
                self.stats['frames_without_decoded_id'] += 1
                return
            tf = self.buffer.lookup_transform('odom', self.p['optical_frame'], stamp)
            if self.mapping:
                base=self.buffer.lookup_transform('odom','base_footprint',stamp).transform
                robot=[base.translation.x,base.translation.y]
                map_odom=self.buffer.lookup_transform('map','odom',stamp).transform
                robot_map=transform_point(map_odom,[*robot,0.])[:2]
            else:map_camera=self.buffer.lookup_transform('map',self.p['optical_frame'],stamp).transform
            k = np.array(info.k, dtype=float).reshape(3, 3)
            d = np.array(info.d, dtype=float)
            accepted_in_frame = set()
            for corner, marker_id in zip(corners, ids.flatten()):
                if int(marker_id) in accepted_in_frame:
                    self.stats['duplicate_id_in_frame'] += 1
                    continue
                result = estimate(corner, self.p['marker_size_m'], k, d,
                                  self.p['min_marker_side_px'], self.p['max_reprojection_error_px'],
                                  self.p['min_distance_m'], self.p['max_distance_m'], self.stats)
                if result is None:
                    self.stats['position_rejected'] += 1
                    c=np.asarray(corner).reshape(4,2)
                    self.localization_wait(int(marker_id),'pnp_or_size_rejected',stamp.nanoseconds,
                        shortest_side_px=float(np.min(np.linalg.norm(c-np.roll(c,-1,axis=0),axis=1))),
                        minimum_side_px=self.p['min_marker_side_px'],maximum_reprojection_error_px=self.p['max_reprojection_error_px'])
                    continue
                error, rvec, tvec = result
                point = transform_point(tf.transform, tvec)
                key = int(marker_id)
                camera_marker = cv2.Rodrigues(rvec)[0]
                marker_rotation = rotation_matrix(tf.transform.rotation) @ camera_marker
                accepted_in_frame.add(key)
                row = dict(id=key, stamp_ns=stamp.nanoseconds, frame_id=self.p['optical_frame'],
                           camera_position=tvec.tolist(), camera_rvec=rvec.tolist(),
                           odom_position=point.tolist(), reprojection_error_px=error)
                if self.mapping:
                    # Correct this observation at its image time before attaching
                    # it to an old node. Old odometry drift must not be reapplied.
                    point_map=transform_point(map_odom,point)
                    rotation_map=rotation_matrix(map_odom.rotation)@marker_rotation
                    row['map_position_at_observation']=point_map.tolist()
                    binding=dict(kind='map',point=point_map,rotation=rotation_map,
                                 pending_anchor=True,robot_map=robot_map)
                else:
                    binding=dict(kind='map',point=transform_point(map_camera,tvec),
                                 rotation=rotation_matrix(map_camera.rotation)@camera_marker)
                self.accept(key,stamp.nanoseconds,binding,row)
        except TransformException:
            self.stats['observation_tf_unavailable'] += 1
            if ids is not None:
                for key in set(ids.flatten()):self.localization_wait(int(key),'image_time_tf_unavailable',stamp.nanoseconds)
            self.warn('TARGET_TF_UNAVAILABLE: 图像时刻的相机TF不可用，跳过定位')
        except (ValueError, RuntimeError, CvBridgeError, cv2.error) as exc:
            self.stats['processing_error'] += 1
            self.warn('TARGET_PROCESS_ERROR: '+str(exc))
        finally:
            self.processing_ms.append((time.perf_counter()-process_started)*1000)

    def publish_diagnostics(self):
        elapsed_sim = (self.get_clock().now().nanoseconds-self.stats_started)/1e9
        elapsed_wall = time.monotonic()-self.started
        output = dict(counters=dict(self.stats), decoded_ids=sorted(self.decoded_ids),
                      localization_waits=dict(self.localization_waits),
                      configured_hz=self.p['processing_hz'],
                      processed_hz_sim=self.stats['processed_frames']/max(elapsed_sim, 1e-6),
                      processed_hz_wall=self.stats['processed_frames']/max(elapsed_wall, 1e-6),
                      processing_ms_median=float(np.median(self.processing_ms)) if self.processing_ms else None,
                      processing_ms_p95=float(np.percentile(self.processing_ms, 95)) if self.processing_ms else None)
        self.get_logger().info('TARGET_DIAGNOSTICS: '+json.dumps(output))
        if self.output:
            tmp = self.output/'target-diagnostics.json.tmp'
            tmp.write_text(json.dumps(output, indent=2))
            tmp.replace(self.output/'target-diagnostics.json')

    def publish(self):
        now = self.get_clock().now()
        rows, markers = [], MarkerArray()
        clear=Marker(action=Marker.DELETEALL);clear.header.frame_id='map';clear.header.stamp=now.to_msg()
        markers.markers.append(clear)
        for key, history in self.records.items():
            if not history:
                continue
            try:projected=[self.graph.project(h[1]) for h in history]
            except ValueError as error:self.warn('TARGET_ANCHOR_UNAVAILABLE: '+str(error));continue
            point=np.median([p for p,_ in projected],axis=0)
            map_rotation=rotation_matrix(fuse_quaternions([rotation_quaternion(r) for _,r in projected]))
            orientation = rotation_quaternion(map_rotation)
            # Marker +Z is its outward normal; ground-plane yaw follows this normal.
            normal = map_rotation[:, 2]
            yaw = float(np.arctan2(normal[1], normal[0])) if np.linalg.norm(normal[:2]) > .1 else None
            stale = (now.nanoseconds-history[-1][0])*1e-9 > self.p['stale_after_s']
            state = 'stale' if stale else ('confirmed' if key in self.confirmed else 'candidate')
            spread = float(np.sqrt(np.mean([np.sum((p-point)**2) for p,_ in projected])))
            rows.append(dict(id=key, type=self.target_types.get(key, 'unknown'),
                             position=point.tolist(), orientation=orientation.tolist(),
                             x=float(point[0]), y=float(point[1]), z=float(point[2]), yaw=yaw,
                             confirmed=key in self.confirmed,
                             frame_id='map', state=state, last_observed_ns=history[-1][0],
                             observations=len(history), spread_m=spread,
                             anchor_nodes=sorted({h[1].get('node_id',h[1].get('source_node_id')) for h in history
                                                  if h[1].get('node_id',h[1].get('source_node_id')) is not None}),
                             pending_anchor_observations=sum(bool(h[1].get('pending_anchor')) for h in history),
                             coordinate_source=('map_observation' if any(h[1].get('pending_anchor') for h in history)
                                                else 'slam_keyframes' if self.mapping else 'fixed_map')))
            m = Marker()
            m.header.frame_id = 'map'; m.header.stamp = now.to_msg()
            m.ns = 'aruco_target'; m.id = key; m.type = Marker.SPHERE; m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, point)
            m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = orientation
            m.scale.x = m.scale.y = m.scale.z = .12
            m.color.a = .85
            m.color.r = 1.0 if state != 'confirmed' else .1
            m.color.g = .7 if state != 'stale' else .2
            markers.markers.append(m)
            label = Marker()
            label.header = m.header; label.ns = 'aruco_label'; label.id = key
            label.type = Marker.TEXT_VIEW_FACING; label.action = Marker.ADD
            label.pose.position.x, label.pose.position.y, label.pose.position.z = map(float, point+[0, 0, .2])
            label.pose.orientation.w = 1.0; label.scale.z = .13
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = f'ID {key} [{state}]'
            markers.markers.append(label)
        output = dict(schema_version=1, frame_id='map', stamp_ns=now.nanoseconds,
                      map_binding=self.map_binding(),
                      orientation_convention='OpenCV marker axes; quaternion xyzw; yaw=outward normal heading',
                      targets=rows)
        self.estimates.publish(String(data=json.dumps(output)))
        self.markers.publish(markers)
        if self.output:
            used={node for row in rows for node in row['anchor_nodes']}
            anchors=[self.graph.nodes[key] for key in sorted(used)]
            signature=json.dumps([self.map_binding()['mode'],self.graph.epoch,anchors])
            if signature!=self.last_anchor_audit:
                self.last_anchor_audit=signature
                with (self.output/'target-anchor-history.jsonl').open('a') as f:
                    f.write(json.dumps(dict(stamp_ns=now.nanoseconds,map_binding=self.map_binding(),nodes=anchors))+'\n')
            temporary = self.output/'targets.json.tmp'
            temporary.write_text(json.dumps(output, indent=2))
            temporary.replace(self.output/'targets.json')


def main():
    rclpy.init()
    node = Targets()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
