"""Small ROS adapter around interchangeable search candidate policies."""
import json
import math
import time
from frustum_search import Camera, FrustumPlanner, settings
from search_policy import observation_positions


def camera_model(node):
    """Read only calibrated intrinsics and the fixed camera mounting pose."""
    from rclpy.time import Time
    from tf2_ros import TransformException
    info=node.search_camera_info
    if info is None or any(abs(v)>1e-8 for v in info.d):
        raise ValueError('Rectified CameraInfo unavailable')
    try:
        tf=node.buffer.lookup_transform('base_footprint',info.header.frame_id,Time()).transform
    except TransformException as exc:
        raise ValueError('Camera mounting transform unavailable') from exc
    p,q=tf.translation,tf.rotation
    norm=math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
    if norm<1e-8:
        raise ValueError('Invalid camera mounting quaternion')
    x,y,z,w=q.x/norm,q.y/norm,q.z/norm,q.w/norm
    rotation=(1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),
              2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),
              2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y))
    return Camera(info.width,info.height,info.k[0],info.k[4],info.k[2],info.k[5],
                  (p.x,p.y,p.z),rotation)


class SpatialCandidates:
    def __init__(self,positions):
        self.positions=list(positions)
        self.index=0

    def next_view(self,robot):
        if self.index>=len(self.positions):
            return None
        x,y=self.positions[self.index];self.index+=1
        return dict(x=x,y=y,yaw=robot[2],strategy='spatial')

    def record_sweep(self,actual_pose):
        pass


def make_candidates(node,grid,robot):
    config=settings(node.c.get('search_frustum',{}))
    common=(grid.data,grid.info.width,grid.info.height,grid.info.resolution,
            (grid.info.origin.position.x,grid.info.origin.position.y),(robot['x'],robot['y']),
            node.c.get('search_clearance_m',.4),node.c.get('search_spacing_m',1.))
    if config['enabled']:
        try:
            from tf2_ros import TransformException
            camera=camera_model(node)
            points=observation_positions(*common,config['candidate_pool_size'])
            started=time.perf_counter()
            planner=FrustumPlanner(*common[:5],points,camera,config)
            if not planner.has_views:
                raise ValueError('No candidate has visible samples in the configured height/range band')
            node.event('SEARCH_FRUSTUM_READY',json.dumps(dict(candidate_positions=len(points),
                yaw_samples=config['yaw_samples'],samples=len(planner.cells),
                height_samples_m=config['heights_m'],range_m=[config['near_m'],config['far_m']],
                prepare_ms=(time.perf_counter()-started)*1000,
                coverage='geometric completed-sweep proxy; not detection completeness')))
            return planner
        except (ValueError,TransformException) as exc:
            node.event('SEARCH_FRUSTUM_FALLBACK',str(exc)+'; 使用原有空间分散候选点')
    return SpatialCandidates(observation_positions(*common,node.c.get('search_max_positions',6)))
