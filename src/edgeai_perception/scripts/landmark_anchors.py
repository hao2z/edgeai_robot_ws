"""Landmarks relative to SLAM keyframes; no navigation or simulator input."""
import math
import numpy as np


def yaw_rotation(angle):
    c,s=math.cos(angle),math.sin(angle)
    return np.array([[c,-s,0.],[s,c,0.],[0.,0.,1.]])


def pose_point(pose,point,inverse=False):
    shift=np.array([pose[0],pose[1],0.]);r=yaw_rotation(pose[2])
    return r.T@(np.asarray(point)-shift) if inverse else r@point+shift


class AnchorGraph:
    def __init__(self,frame='map',max_distance=1.):
        self.frame,self.max_distance=frame,max_distance
        self.epoch=None;self.revision=0;self.nodes={}

    def update(self,snapshot):
        if snapshot.get('frame_id')!=self.frame or not snapshot.get('epoch'):
            raise ValueError('Invalid anchor graph frame/epoch')
        nodes={}
        for node in snapshot['nodes']:
            if (node['id'] in nodes or len(node['odom'])!=3 or len(node['map'])!=3
                or not np.isfinite([*node['odom'],*node['map'],node['stamp_ns']]).all()):
                raise ValueError('Invalid anchor keyframe')
            nodes[node['id']]=node
        changed=self.epoch is not None and self.epoch!=snapshot['epoch']
        self.epoch=snapshot['epoch'];self.revision=snapshot['revision'];self.nodes=nodes
        return changed

    def bind(self,stamp,point,rotation,robot_odom):
        viable=[n for n in self.nodes.values() if
            math.dist(n['odom'][:2],robot_odom[:2])<=self.max_distance and
            (abs(n['stamp_ns']-stamp)<=5e9 or math.dist(n['odom'][:2],robot_odom[:2])<=.25)]
        if not viable:raise ValueError('No nearby SLAM keyframe for target observation')
        node=min(viable,key=lambda n:abs(n['stamp_ns']-stamp))
        return dict(kind='slam_graph',epoch=self.epoch,node_id=node['id'],
            point=pose_point(node['odom'],point,inverse=True),
            rotation=yaw_rotation(-node['odom'][2])@rotation)

    def project(self,binding):
        if binding['kind']=='map':return binding['point'].copy(),binding['rotation'].copy()
        if binding['epoch']!=self.epoch or binding['node_id'] not in self.nodes:
            raise ValueError('Target anchor missing from the current SLAM graph')
        pose=self.nodes[binding['node_id']]['map']
        return pose_point(pose,binding['point']),yaw_rotation(pose[2])@binding['rotation']

    def freeze(self,binding):
        point,rotation=self.project(binding)
        return dict(kind='map',point=point,rotation=rotation,
                    source_node_id=binding.get('node_id',binding.get('source_node_id')))
