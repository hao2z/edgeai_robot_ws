#!/usr/bin/env python3
"""Compare raw/filtered SLAM maps at points removed from a live dual simulation."""
import argparse
import json
import math
from pathlib import Path
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import Costmap
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/edgeai_navigation/scripts'))
from edgeai_tasks.cooperative_policy import cell,compose


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--directory',type=Path,required=True)
    args,ros=parser.parse_known_args();rclpy.init(args=ros);node=Node('fleet_filter_check')
    maps={};raw={};filtered=[];points=[];counts=[];buffer=Buffer();listener=TransformListener(buffer,node)
    global_maps=[]
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for key,topic in [('filtered','/map'),('raw','/map_raw')]:
        node.create_subscription(OccupancyGrid,topic,lambda m,k=key:maps.update({k:m}),qos)
    node.create_subscription(Costmap,'/global_costmap/costmap_raw',lambda m:global_maps.__setitem__(slice(None),[m]),qos)
    node.create_subscription(LaserScan,'/scan',lambda m:raw.update({Time.from_msg(m.header.stamp).nanoseconds:m}),qos_profile_sensor_data)
    node.create_subscription(LaserScan,'/scan_slam',filtered.append,qos_profile_sensor_data)
    end=time.monotonic()+70.;started=None;result=dict(success=False,reason='No paired scan/map observations')
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.02)
        for scan in list(filtered):
            stamp=Time.from_msg(scan.header.stamp);source=raw.get(stamp.nanoseconds)
            if source is None:continue
            try:
                tf=buffer.lookup_transform('map',scan.header.frame_id,stamp).transform
            except TransformException:continue
            pose=[tf.translation.x,tf.translation.y,2*math.atan2(tf.rotation.z,tf.rotation.w)]
            removed=0
            for i,(before,after) in enumerate(zip(source.ranges,scan.ranges)):
                if math.isfinite(before) and math.isnan(after):
                    angle=scan.angle_min+i*scan.angle_increment
                    points.append(compose(pose,[before*math.cos(angle),before*math.sin(angle),0.])[:2]);removed+=1
                elif not (before==after or math.isnan(before) and math.isnan(after)):
                    raise RuntimeError('Filter altered unrelated range')
            counts.append(removed);filtered.remove(scan)
        if counts and len(maps)==2 and global_maps:
            if started is None:started=time.monotonic()
            if time.monotonic()-started>15.:
                raw_occupied=sum(cell(maps['raw'],*p)>=65 for p in points)
                filtered_occupied=sum(cell(maps['filtered'],*p)>=65 for p in points)
                cm=global_maps[0];meta=cm.metadata
                def cost(x,y):
                    u=math.floor((x-meta.origin.position.x)/meta.resolution);v=math.floor((y-meta.origin.position.y)/meta.resolution)
                    return cm.data[v*meta.size_x+u] if 0<=u<meta.size_x and 0<=v<meta.size_y else -1
                dynamic=sum(cost(*p)==254 for p in points)
                grid=maps['filtered'];walls=preserved=0
                for index,value in enumerate(grid.data):
                    if value<65:continue
                    x=grid.info.origin.position.x+(index%grid.info.width+.5)*grid.info.resolution
                    y=grid.info.origin.position.y+(index//grid.info.width+.5)*grid.info.resolution
                    walls+=1;preserved+=cost(x,y)==254
                result=dict(success=len(points)>0 and raw_occupied>0 and filtered_occupied==0 and dynamic>0 and walls>0 and walls==preserved,
                    scans=len(counts),removed_points=len(points),raw_occupied_samples=raw_occupied,
                    filtered_occupied_samples=filtered_occupied,global_dynamic_occupied_samples=dynamic,
                    static_wall_cells=walls,preserved_static_wall_cells=preserved,raw_topic='/scan',slam_topic='/scan_slam')
                break
        # Bound retained diagnostic messages.
        if len(raw)>200:
            for key in sorted(raw)[:-100]:raw.pop(key)
    (args.directory/'filter-result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
    node.destroy_node();rclpy.shutdown();return 0 if result['success'] else 1


if __name__=='__main__':raise SystemExit(main())
