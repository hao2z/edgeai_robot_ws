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
    """Robot-tagged nodes use (robot_id, id) keys; legacy nodes use numeric ids."""

    def __init__(self,frame='map',max_distance=3.):
        self.frame,self.max_distance=frame,max_distance
        self.epoch=None;self.revision=0;self.nodes={}

    def update(self,snapshot):
        # Central graphs share one graph_id across robots. Keep epoch as the
        # legacy input and binding field, with graph_id authoritative if given.
        epoch=snapshot.get('graph_id',snapshot.get('epoch'))
        if snapshot.get('frame_id')!=self.frame or not epoch:
            raise ValueError('Invalid anchor graph frame/epoch')
        nodes={}
        for node in snapshot['nodes']:
            key=self._node_key(node)
            if (key in nodes or len(node['odom'])!=3 or len(node['map'])!=3
                or not np.isfinite([*node['odom'],*node['map'],node['stamp_ns']]).all()):
                raise ValueError('Invalid anchor keyframe')
            nodes[key]=node
        changed=self.epoch is not None and self.epoch!=epoch
        self.epoch=epoch;self.revision=snapshot['revision'];self.nodes=nodes
        return changed

    @staticmethod
    def _node_key(node):
        node_id=node.get('node_id',node.get('id'))
        return (node['robot_id'],node_id) if node.get('robot_id') is not None else node_id

    def bind(self,stamp,point,rotation,robot_odom,frame='odom',robot_id=None):
        """Bind in map or odom; robot_id restricts selection to that source.

        Omitting robot_id preserves nearest-node selection across all sources.
        Explicit selection requires tagged nodes, including in legacy graphs.
        """
        if frame not in ('map','odom'):raise ValueError('Invalid observation coordinate frame')
        viable=[n for n in self.nodes.values() if
            (robot_id is None or n.get('robot_id')==robot_id)
            and math.dist(n[frame][:2],robot_odom[:2])<=self.max_distance]
        if not viable:raise ValueError('No nearby SLAM keyframe for target observation')
        # Revisited regions need not generate a new keyframe. Age is a tie
        # breaker, never a rejection; retain the graph epoch/correction link.
        node=min(viable,key=lambda n:(math.dist(n[frame][:2],robot_odom[:2]),abs(n['stamp_ns']-stamp)))
        origin=node[frame]
        binding=dict(kind='slam_graph',epoch=self.epoch,node_id=node['id'],
            point=pose_point(origin,point,inverse=True),
            rotation=yaw_rotation(-origin[2])@rotation)
        for field in ('robot_id','sensor_id'):
            if field in node:binding[field]=node[field]
        return binding

    def project(self,binding):
        if binding['kind']=='map':return binding['point'].copy(),binding['rotation'].copy()
        key=self._node_key(binding)
        if binding['epoch']!=self.epoch or key not in self.nodes:
            raise ValueError('Target anchor missing from the current SLAM graph')
        pose=self.nodes[key]['map']
        return pose_point(pose,binding['point']),yaw_rotation(pose[2])@binding['rotation']

    def freeze(self,binding):
        point,rotation=self.project(binding)
        frozen=dict(kind='map',point=point,rotation=rotation,
                    source_node_id=binding.get('node_id',binding.get('source_node_id')))
        for field in ('epoch','robot_id','sensor_id'):
            source='source_'+field
            if source in binding:frozen[source]=binding[source]
            elif field in binding:frozen[source]=binding[field]
        return frozen
