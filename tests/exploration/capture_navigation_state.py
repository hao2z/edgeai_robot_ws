"""Read-only raw-costmap/TF snapshots for diagnosing planner/controller disagreement."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PolygonStamped, Twist
from dwb_msgs.msg import LocalPlanEvaluation
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener
from rosidl_runtime_py.convert import message_to_ordereddict


def clean(value):
    if isinstance(value, dict):
        return {k:clean(v) for k,v in value.items()}
    if isinstance(value, (list,tuple)):
        return [clean(v) for v in value]
    if isinstance(value,float) and not math.isfinite(value):
        return None
    return value


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=20)
    parser.add_argument('--with-evaluation', action='store_true',
                        help='Capture optional DWB debug trajectories; disabled for long monitoring runs')
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = rclpy.create_node('edgeai_navigation_state_capture', enable_rosout=False)
    buffer = Buffer(cache_time=Duration(seconds=20))
    listener = TransformListener(buffer,node)
    messages, subscriptions = {}, []
    topics = {'global_raw':('/global_costmap/costmap_raw', Costmap),
              'local_raw':('/local_costmap/costmap_raw',Costmap),
              'map':('/map', OccupancyGrid),
              'global_footprint':('/global_costmap/published_footprint',PolygonStamped),
              'local_footprint':('/local_costmap/published_footprint',PolygonStamped),
              'odom':('/odom',Odometry), 'scan':('/scan',LaserScan),
              'global_plan':('/plan',NavPath), 'local_plan':('/local_plan',NavPath),
              'controller_plan':('/transformed_global_plan',NavPath),
              'cmd_nav':('/cmd_vel_nav',Twist), 'cmd_smoothed':('/cmd_vel_smoothed',Twist),
              'cmd_final':('/cmd_vel',Twist)}
    if args.with_evaluation:
        topics['evaluation']=('/evaluation',LocalPlanEvaluation)
    for key,(topic,kind) in topics.items():
        # Raw costmaps may be volatile; transient-local readers cannot match
        # those publishers. A volatile reader also accepts transient publishers.
        qos = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL) if key == 'map' else qos_profile_sensor_data
        subscriptions.append(node.create_subscription(kind,topic,lambda msg,k=key:messages.__setitem__(k,msg),qos))
    args.output.mkdir(parents=True,exist_ok=True)
    started,last,index = time.monotonic(),0,0
    try:
        while rclpy.ok() and time.monotonic()-started < args.duration:
            rclpy.spin_once(node,timeout_sec=.05)
            if time.monotonic()-last<2 or not all(key in messages for key in ('global_raw','local_raw','odom')):
                continue
            now=node.get_clock().now()
            stamp=Time(nanoseconds=max(1,now.nanoseconds-100_000_000),clock_type=now.clock_type)
            transforms={}
            try:
                for target,source in (('map','base_footprint'),('odom','base_footprint'),('map','odom')):
                    transforms[target+'_from_'+source]=message_to_ordereddict(buffer.lookup_transform(target,source,stamp))
                for key in ('global_footprint','local_footprint'):
                    if key in messages:
                        message=messages[key]
                        transforms[key+'_to_base']=message_to_ordereddict(buffer.lookup_transform(
                            'base_footprint',message.header.frame_id,Time.from_msg(message.header.stamp)))
            except TransformException:
                continue
            index+=1
            data=clean(dict(sim_time=now.nanoseconds/1e9,comparison_time=stamp.nanoseconds/1e9,
                            messages={k:message_to_ordereddict(v) for k,v in messages.items()}, transforms=transforms))
            filename=args.output/f'state-{index:03d}.json'
            filename.write_text(json.dumps(data))
            print(f'{filename.name} sim={data["sim_time"]:.2f}; captured {list(messages)}',flush=True)
            last=time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        if not index:
            print(f'No snapshots; missing required topics: {[k for k in ("global_raw", "local_raw", "odom") if k not in messages]}; '
                  f'received: {list(messages)}', flush=True)
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__=='__main__':
    main()
