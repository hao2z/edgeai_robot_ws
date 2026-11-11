#!/usr/bin/env python3
"""Continuous RGB-D/laser hints with bounded memory, independent of tasks."""
from collections import Counter, deque
import json
from pathlib import Path
import time
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
from aruco_geometry import rotation_matrix
from suspect_geometry import panels, prefer_surface_normal, laser_support


class Suspects(Node):
    def __init__(self):
        super().__init__('suspect_targets')
        defaults=dict(processing_hz=5.,panel_size_m=.20,max_image_age_s=.5,
                      sync_slop_s=.05,confirmation_frames=2,candidate_ttl_s=8.,known_target_radius_m=.30,
                      enabled=True,depth_geometry_enabled=True,laser_enabled=True,
                      laser_topic='/scan',laser_sync_slop_s=.15,laser_support_radius_m=.10,
                      max_candidates=24,fusion_window=20,output_directory='')
        for key,value in defaults.items():
            self.declare_parameter(key,value)
        self.p={k:self.get_parameter(k).value for k in defaults}
        if self.p['processing_hz']<=0 or self.p['panel_size_m']<=0:
            raise ValueError('Invalid suspect detector parameters')
        if self.p['confirmation_frames']<2 or self.p['fusion_window']<self.p['confirmation_frames'] or self.p['max_candidates']<1:
            raise ValueError('Invalid suspect memory limits')
        if self.p['laser_sync_slop_s']<=0 or self.p['laser_support_radius_m']<=0:
            raise ValueError('Invalid laser support limits')
        self.bridge=CvBridge();self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.rgb=deque(maxlen=3);self.depth=deque(maxlen=3);self.info=None
        self.scans=deque(maxlen=8);self.last_stamp=None
        self.known_registry=None;self.known_odom={}
        self.records=[];self.next_id=0;self.stats=Counter();self.last_diag=0.
        self.notified={}
        self.output=Path(self.p['output_directory']) if self.p['output_directory'] else None
        self.create_subscription(Image,'/camera/image',lambda m:self.rgb.append(m),qos_profile_sensor_data)
        self.create_subscription(Image,'/camera/depth_image',lambda m:self.depth.append(m),qos_profile_sensor_data)
        self.create_subscription(CameraInfo,'/camera/camera_info',self.camera,qos_profile_sensor_data)
        if self.p['laser_enabled']:
            self.create_subscription(LaserScan,self.p['laser_topic'],lambda m:self.scans.append(m),qos_profile_sensor_data)
        self.create_subscription(String,'/targets/estimates',self.known_received,1)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher=self.create_publisher(String,'/targets/suspected',qos)
        self.create_timer(1/self.p['processing_hz'],self.process)
        self.get_logger().info(f'SUSPECT_READY: 独立持续检测 {self.p["processing_hz"]:g} Hz，深度几何与激光辅助，多帧候选保留至本次会话结束')

    def camera(self,msg):
        self.info=msg

    def known_received(self,msg):
        self.known_registry=json.loads(msg.data)

    def detecting(self):
        return self.p['enabled']

    def update_known(self):
        registry=self.known_registry
        if not registry or registry.get('frame_id')!='map':return
        tf=self.buffer.lookup_transform('odom','map',Time()).transform
        rotation=rotation_matrix(tf.rotation)
        translation=np.array([tf.translation.x,tf.translation.y,tf.translation.z])
        for target in registry.get('targets',[]):
            position=target.get('position')
            if position is None and target.get('x') is not None and target.get('y') is not None:
                position=[target['x'],target['y'],target.get('z',.25)]
            if position is not None and len(position)==3 and np.isfinite(position).all():
                self.known_odom[target['id']]=rotation@np.asarray(position)+translation
        self.known_registry=None

    def process(self):
        detecting=self.detecting()
        if not detecting and not self.records and self.known_registry is None:
            return
        begin=time.perf_counter()
        now=self.get_clock().now().nanoseconds/1e9
        self.records=[r for r in self.records if now>=r['last_seen'] and
                      (r['observations']>=self.p['confirmation_frames'] or
                       now-r['last_seen']<self.p['candidate_ttl_s'])]
        hints=[]
        try:
            if detecting:self.detect(now)
            self.update_known()
            before=len(self.records)
            self.records=[r for r in self.records if not any(
                np.linalg.norm(np.array(r['position'][:2])-point[:2])<self.p['known_target_radius_m']
                and abs(r['position'][2]-point[2])<.25 for point in self.known_odom.values())]
            self.stats['known_excluded']+=before-len(self.records)
            ids={r['id'] for r in self.records}
            self.notified={k:v for k,v in self.notified.items() if k in ids}
            tf=self.buffer.lookup_transform('map','odom',Time()).transform
            rotation=rotation_matrix(tf.rotation);translation=np.array([tf.translation.x,tf.translation.y,tf.translation.z])
            for record in self.records:
                hint={k:v for k,v in record.items() if not k.startswith('_')}
                hint['retained']=record['observations']>=self.p['confirmation_frames']
                hint['position']=(rotation@np.asarray(record['position'])+translation).tolist()
                n=rotation@np.array([*record['normal'],0.])
                hint['normal']=(n[:2]/np.linalg.norm(n[:2])).tolist()
                hint['normal_yaw_rad']=math.atan2(n[1],n[0])
                self.notify(hint,now)
                if hint['retained']:hints.append(hint)
            self.publisher.publish(String(data=json.dumps(dict(frame_id='map',stamp=now,candidates=hints))))
        except (TransformException,CvBridgeError,ValueError) as exc:
            self.stats[type(exc).__name__]+=1
        self.stats['cycles']+=1
        self.stats['processing_ms_total']+=(time.perf_counter()-begin)*1000
        if time.monotonic()-self.last_diag>5.:
            self.diagnostics(hints,detecting)
            self.last_diag=time.monotonic()

    def notify(self,hint,now):
        stage='RETAINED' if hint['retained'] else 'PROPOSED'
        if self.notified.get(hint['id'])==stage:return
        self.notified[hint['id']]=stage
        p=hint['position'];angle=math.degrees(hint['normal_yaw_rad'])
        kind='薄板侧边' if hint.get('normal_kind')=='thin_edge' else '表面'
        text=(f"疑似目标 #{hint['id']}｜{'多帧保留' if hint['retained'] else '首次观测，待复核'}"
              f"｜map=({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f}) m"
              f"｜{kind}法线轴={angle:.0f}°/{(angle+180+180)%360-180:.0f}°"
              f"｜观测{hint['observations']}次"
              f"｜来源={'深度几何' if hint.get('source')=='depth' else 'RGB-D'}"
              f"，激光支持={'有' if hint.get('laser_supported') else '无'}")
        self.get_logger().info('SUSPECT_'+stage+': '+text)
        if self.output:
            self.output.mkdir(parents=True,exist_ok=True)
            with (self.output/'suspect-events.jsonl').open('a',encoding='utf-8') as stream:
                stream.write(json.dumps(dict(event='SUSPECT_'+stage,sim_time=now,candidate=hint),ensure_ascii=False)+'\n')

    def diagnostics(self,hints,active):
        summary=dict(self.stats,active=active,tracked=len(self.records),candidates=hints,
            mean_processing_ms=self.stats['processing_ms_total']/max(1,self.stats['cycles']))
        brief='；'.join(f"疑似#{h['id']} map=({h['position'][0]:.2f}, {h['position'][1]:.2f}, {h['position'][2]:.2f}) m，法线轴={math.degrees(h['normal_yaw_rad']):.0f}°，{h['observations']}次观测"
                       for h in hints[:3])
        self.get_logger().info(f"SUSPECT_DIAGNOSTICS: 疑似目标检测{'运行' if active else '待机'}，"
            f"已处理{self.stats['frames']}帧，保留{len(hints)}个候选，"
            f"RGB/深度通过{self.stats['accepted_rgbd']}/{self.stats['accepted_depth']}次，激光支持{self.stats['laser_supported']}次，"
            f"平均{summary['mean_processing_ms']:.1f} ms/周期"+('｜'+brief if brief else ''))
        if self.output:
            self.output.mkdir(parents=True,exist_ok=True)
            with (self.output/'suspect-diagnostics.jsonl').open('a') as stream:
                stream.write(json.dumps(dict(sim_time=self.get_clock().now().nanoseconds/1e9,**summary))+'\n')
            snapshot=self.output/'suspect-candidates.json.tmp'
            snapshot.write_text(json.dumps(dict(frame_id='map',
                stamp=self.get_clock().now().nanoseconds/1e9,candidates=hints),ensure_ascii=False))
            snapshot.replace(self.output/'suspect-candidates.json')

    def detect(self,now):
        if not self.rgb or not self.depth or self.info is None:
            self.stats['missing_input']+=1;return
        stamp=lambda m:Time.from_msg(m.header.stamp).nanoseconds/1e9
        image=self.rgb[-1]; image_time=stamp(image)
        if image_time==self.last_stamp:
            return
        depth=min(self.depth,key=lambda m:abs(stamp(m)-image_time))
        if not 0<=now-image_time<=self.p['max_image_age_s'] or abs(stamp(depth)-image_time)>self.p['sync_slop_s']:
            self.stats['stale_or_unsynced']+=1;return
        info=self.info
        if (image.header.frame_id!=depth.header.frame_id or image.header.frame_id!=info.header.frame_id
            or (image.width,image.height)!=(depth.width,depth.height)
            or (image.width,image.height)!=(info.width,info.height) or any(abs(d)>1e-8 for d in info.d)):
            raise ValueError('Registered, undistorted RGB/depth/CameraInfo required')
        if depth.encoding not in ('32FC1','16UC1'):
            raise ValueError('Unsupported depth encoding')
        tf=self.buffer.lookup_transform('odom',image.header.frame_id,Time.from_msg(image.header.stamp)).transform
        rgb=self.bridge.imgmsg_to_cv2(image,'bgr8')
        metres=self.bridge.imgmsg_to_cv2(depth,'passthrough').astype(float)
        if depth.encoding=='16UC1':
            metres*=.001
        proposals=panels(rgb,metres,info.k,rotation_matrix(tf.rotation),
                         [tf.translation.x,tf.translation.y,tf.translation.z],self.p['panel_size_m'],
                         depth_geometry=self.p['depth_geometry_enabled'],diagnostics=self.stats)
        scan_points=self.scan_points(image_time) if proposals and self.p['laser_enabled'] else []
        self.last_stamp=image_time;self.stats['frames']+=1
        used=set()
        for proposal in proposals:
            proposal['laser_supported']=laser_support(proposal,scan_points,self.p['laser_support_radius_m'])
            if proposal['laser_supported']:
                proposal['confidence']=min(.95,proposal['confidence']+.10)
                self.stats['laser_supported']+=1
            matches=[r for r in self.records if r['id'] not in used and
                     np.linalg.norm(np.array(r['position'])-proposal['position'])<.15]
            record=min(matches,key=lambda r:np.linalg.norm(np.array(r['position'])-proposal['position'])) if matches else None
            if record is None:
                if len(self.records)>=self.p['max_candidates']:
                    weak=[r for r in self.records if r['observations']<self.p['confirmation_frames']]
                    if not weak:
                        self.stats['memory_full']+=1;continue
                    self.records.remove(min(weak,key=lambda r:r['last_seen']))
                record=dict(id=self.next_id,observations=0);self.next_id+=1;self.records.append(record)
            samples=record.get('_positions',[])
            samples=(samples+[proposal['position']])[-self.p['fusion_window']:]
            first_seen=record.get('first_seen',image_time)
            record.update(prefer_surface_normal(record,proposal),last_seen=image_time,observations=record['observations']+1)
            record['_positions']=samples;record['first_seen']=first_seen
            record['position']=np.mean(samples,axis=0).tolist()
            record['position_spread_m']=float(np.sqrt(np.mean(np.sum(
                (np.asarray(samples)-record['position'])**2,axis=1))))
            used.add(record['id'])
            self.stats['proposals']+=1

    def scan_points(self,image_time):
        if not self.scans:
            self.stats['laser_missing']+=1;return []
        stamp=lambda m:Time.from_msg(m.header.stamp).nanoseconds/1e9
        scan=min(self.scans,key=lambda m:abs(stamp(m)-image_time))
        # Include scan duration: moving scans must not corroborate an old image.
        if abs(stamp(scan)-image_time)+max(0.,scan.time_increment)*(len(scan.ranges)-1)>self.p['laser_sync_slop_s']:
            self.stats['laser_unsynced']+=1;return []
        try:
            tf=self.buffer.lookup_transform('odom',scan.header.frame_id,Time.from_msg(scan.header.stamp)).transform
        except TransformException:
            self.stats['laser_tf_missing']+=1;return []
        ranges=np.asarray(scan.ranges)
        valid=np.isfinite(ranges)&(ranges>=scan.range_min)&(ranges<=scan.range_max)
        angles=scan.angle_min+np.arange(len(ranges))*scan.angle_increment
        points=np.column_stack((ranges[valid]*np.cos(angles[valid]),
                                ranges[valid]*np.sin(angles[valid]),np.zeros(valid.sum())))
        return points@rotation_matrix(tf.rotation).T+np.array([tf.translation.x,tf.translation.y,tf.translation.z])


def main():
    rclpy.init();executor=SingleThreadedExecutor();node=Suspects()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception:
        # ROS signal handling may invalidate the wait set during SIGTERM.
        # Report real processing failures, but finish normal context shutdown.
        if rclpy.ok():raise
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':
    main()
