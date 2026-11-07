"""ROS-free, bounded camera-view ranking on an axis-aligned occupancy snapshot.

The 3D pinhole frustum is evaluated at configured potential target heights.
Occlusion is a 2D conservative approximation, not a depth reconstruction.
Coverage is a geometric proxy, never evidence that a target is absent.
"""
from dataclasses import dataclass
import math
import numpy as np


DEFAULTS = dict(enabled=False, candidate_pool_size=24, yaw_samples=8,
                direction_bins=4, sample_step_m=.2, max_samples=2500,
                near_m=.25, far_m=2.5, heights_m=[.2,.35,.5],
                travel_weight=.5, turn_weight=.15)


def settings(values):
    if not isinstance(values,dict) or set(values)-set(DEFAULTS):
        raise ValueError('Unknown or invalid search_frustum settings')
    c = dict(DEFAULTS, **values)
    if not isinstance(c['enabled'], bool):
        raise ValueError('search_frustum.enabled must be boolean')
    for name,low,high in [('candidate_pool_size',1,30),('yaw_samples',4,32),
                          ('direction_bins',2,16),('max_samples',100,10000)]:
        v=c[name]
        if isinstance(v,bool) or not isinstance(v,int) or not low<=v<=high:
            raise ValueError('Invalid search_frustum.'+name)
    for name in ('near_m','far_m','sample_step_m','travel_weight','turn_weight'):
        v=c[name]
        if isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or v<0:
            raise ValueError('Invalid search_frustum.'+name)
    if c['far_m']<=c['near_m'] or c['sample_step_m']<.1:
        raise ValueError('Invalid frustum range or sampling step (<0.1 m)')
    heights=c['heights_m']
    if not isinstance(heights,(list,tuple)) or not 1<=len(heights)<=8 or any(
            isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or v<0 for v in heights):
        raise ValueError('Invalid search_frustum.heights_m')
    return c


@dataclass(frozen=True)
class Camera:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    position: tuple  # camera optical origin in base_footprint
    rotation: tuple  # row-major base_from_optical rotation

    def __post_init__(self):
        if (self.width<=0 or self.height<=0 or self.fx<=0 or self.fy<=0 or
                not all(math.isfinite(v) for v in [self.fx,self.fy,self.cx,self.cy,*self.position,*self.rotation])):
            raise ValueError('Invalid camera calibration/extrinsic')
        if len(self.position)!=3 or len(self.rotation)!=9:
            raise ValueError('Invalid camera extrinsic dimensions')
        r=np.array(self.rotation).reshape(3,3)
        if not np.allclose(r.T@r,np.eye(3),atol=1e-5) or np.linalg.det(r)<.99:
            raise ValueError('Camera rotation is not a proper orthonormal rotation')


def line_clear(data, width, height, start, end):
    """Supercover grid ray; never look through an occupied/unknown corner."""
    x,y=start; ex,ey=end
    nx,ny=abs(ex-x),abs(ey-y)
    sx,sy=1 if x<ex else -1,1 if y<ey else -1
    ix=iy=0
    def clear(px,py):
        return (0<=px<width and 0<=py<height and data[py*width+px]>=0 and
                (data[py*width+px]==0 or (px,py)==(ex,ey)))
    while True:
        if not clear(x,y):
            return False
        if (x,y)==(ex,ey):
            return True
        decision=(1+2*ix)*ny-(1+2*iy)*nx
        if decision==0:
            if not clear(x+sx,y) or not clear(x,y+sy):
                return False
            x+=sx;y+=sy;ix+=1;iy+=1
        elif decision<0:
            x+=sx;ix+=1
        else:
            y+=sy;iy+=1


class FrustumPlanner:
    """Propose one untried XY/yaw; only completed sweeps change coverage.

    Inputs: plain grid, camera model, candidate XY positions and settings.
    Outputs: dictionaries with XY/yaw, directional gain and score. No ROS,
    navigation, rescue IDs, world truth or perception pipeline dependencies.
    """
    def __init__(self,data,width,height,resolution,origin,positions,camera,config=None):
        self.c=settings(config or {})
        if resolution<=0 or len(data)!=width*height:
            raise ValueError('Invalid frustum occupancy map')
        self.data,self.width,self.height=data,width,height
        self.res,self.origin=resolution,origin
        self.positions=[tuple(p) for p in positions[:self.c['candidate_pool_size']]]
        self.camera=camera
        self.covered=set();self.attempted=set();self._features={}
        stride=max(1,math.ceil(self.c['sample_step_m']/resolution))
        stride=max(stride,math.ceil(math.sqrt(width*height/self.c['max_samples'])))
        cells=[(x,y) for y in range(stride//2,height,stride) for x in range(stride//2,width,stride)
               if data[y*width+x]>=0]
        self.cells=np.array(cells,dtype=int).reshape(-1,2)
        self.xy=np.column_stack([origin[0]+(self.cells[:,0]+.5)*resolution,
                                 origin[1]+(self.cells[:,1]+.5)*resolution])
        self.sample_area=(stride*resolution)**2/len(self.c['heights_m'])
        for i,(x,y) in enumerate(self.positions):
            for j in range(self.c['yaw_samples']):
                angle=-math.pi+j*2*math.pi/self.c['yaw_samples']
                self._features[i,j]=self.visible_keys((x,y,angle))

    @property
    def has_views(self):
        return any(self._features.values())

    def visible_keys(self,view):
        x,y,angle=view
        c,s=math.cos(angle),math.sin(angle)
        rotation=np.array([[c,-s],[s,c]])
        camera_xy=np.array([x,y])+rotation@np.array(self.camera.position[:2])
        dx=self.xy-np.array([x,y])
        local_xy=dx@rotation-np.array(self.camera.position[:2])
        r=np.array(self.camera.rotation).reshape(3,3)
        visible=[]
        near,far=self.c['near_m'],self.c['far_m']
        for h in self.c['heights_m']:
            base=np.column_stack([local_xy,np.full(len(dx),h-self.camera.position[2])])
            optical=base@r
            z=optical[:,2]
            safe_z=np.maximum(z,1e-8)
            u=self.camera.fx*optical[:,0]/safe_z+self.camera.cx
            v=self.camera.fy*optical[:,1]/safe_z+self.camera.cy
            distance=np.linalg.norm(base,axis=1)
            visible.append((z>0)&(distance>=near)&(distance<=far)&
                (u>=0)&(u<self.camera.width)&(v>=0)&(v<self.camera.height))
        visible=np.array(visible)
        candidates=np.flatnonzero(visible.any(axis=0))
        start=(math.floor((camera_xy[0]-self.origin[0])/self.res),
               math.floor((camera_xy[1]-self.origin[1])/self.res))
        keys=set()
        for i in candidates:
            if not line_clear(self.data,self.width,self.height,start,tuple(self.cells[i])):
                continue
            # Direction FROM this potential target TO the camera. Different
            # sides remain valuable even if the same XY was previously seen.
            bearing=math.atan2(camera_xy[1]-self.xy[i,1],camera_xy[0]-self.xy[i,0])
            side=int((bearing%(2*math.pi))/(2*math.pi)*self.c['direction_bins'])%self.c['direction_bins']
            for h in np.flatnonzero(visible[:,i]):
                keys.add((int(i)*len(self.c['heights_m'])+int(h))*self.c['direction_bins']+side)
        return keys

    def next_view(self,robot):
        choices=[]
        for (i,j),keys in self._features.items():
            if i in self.attempted or not keys:
                continue
            x,y=self.positions[i];angle=-math.pi+j*2*math.pi/self.c['yaw_samples']
            gain=len(keys-self.covered)*self.sample_area
            distance=math.hypot(x-robot[0],y-robot[1])
            turn=abs(math.atan2(math.sin(angle-robot[2]),math.cos(angle-robot[2])))
            score=gain/(1+self.c['travel_weight']*distance+self.c['turn_weight']*turn)
            choices.append((score,gain,-distance,-turn,-i,-j,i,x,y,angle))
        if not choices:
            return None
        score,gain,_,_,_,_,i,x,y,angle=max(choices)
        self.attempted.add(i)
        return dict(x=x,y=y,yaw=angle,gain_m2=gain,score=score,
                    strategy='frustum',coverage_keys=len(self.covered))

    def record_sweep(self,actual_pose):
        # Full completed sweep, at its measured position. This deliberately
        # does not credit planned, failed or partially canceled actions.
        for j in range(self.c['yaw_samples']):
            self.covered.update(self.visible_keys((actual_pose[0],actual_pose[1],
                actual_pose[2]+j*2*math.pi/self.c['yaw_samples'])))
