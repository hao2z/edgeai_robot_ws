"""Small, ROS-independent geometry and joint rescue completion rules."""
import math
from edgeai_nav.geometry import compose, relative






def cell(grid,x,y):
    info=grid.info;o=info.origin
    q=o.orientation
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    p=relative([o.position.x,o.position.y,yaw],[x,y,0.])
    u,v=math.floor(p[0]/info.resolution),math.floor(p[1]/info.resolution)
    if not (0<=u<info.width and 0<=v<info.height):return -1
    return grid.data[v*info.width+u]


def footprint_clear(grid,pose,footprint,pad=.005):
    # Sample the entire padded rectangular envelope, including its boundary.
    xs,ys=zip(*footprint);xmin,xmax=min(xs)-pad,max(xs)+pad;ymin,ymax=min(ys)-pad,max(ys)+pad
    nx=max(1,math.ceil((xmax-xmin)/(grid.info.resolution/2)))
    ny=max(1,math.ceil((ymax-ymin)/(grid.info.resolution/2)))
    for i in range(nx+1):
        for j in range(ny+1):
            p=compose(pose,[xmin+(xmax-xmin)*i/nx,ymin+(ymax-ymin)*j/ny,0.])
            if not 0<=cell(grid,*p[:2])<50:return False
    return True


def separated(a,b,footprint,pad=.04):
    xs,ys=zip(*footprint)
    corners=[(min(xs)-pad,min(ys)-pad),(max(xs)+pad,min(ys)-pad),
             (max(xs)+pad,max(ys)+pad),(min(xs)-pad,max(ys)+pad)]
    polygons=[[compose(p,[x,y,0.])[:2] for x,y in corners] for p in (a,b)]
    for polygon in polygons:
        for u,v in zip(polygon,polygon[1:]+polygon[:1]):
            axis=[u[1]-v[1],v[0]-u[0]]
            ranges=[[sum(x*y for x,y in zip(point,axis)) for point in shape] for shape in polygons]
            if max(ranges[0])<min(ranges[1]) or max(ranges[1])<min(ranges[0]):return True
    return False


def path_clear_of_peer(poses,parked,footprint,margin=.05):
    """Check the interpolated rectangular sweep, including rotation in place."""
    if not poses or not all(math.isfinite(v) for p in (*poses,parked) for v in p):return False
    radius=max(math.hypot(x,y) for x,y in footprint)
    if not separated(poses[0],parked,footprint,margin/2):return False
    for a,b in zip(poses,poses[1:]):
        turn=math.remainder(b[2]-a[2],2*math.pi)
        travel=math.dist(a[:2],b[:2])+radius*abs(turn)
        steps=max(1,math.ceil(travel/.04))
        # Half a sample's maximum corner motion covers the interpolated segment.
        pad=margin/2+travel/(2*steps)
        for i in range(steps):
            t=(i+.5)/steps
            sample=[a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1]),a[2]+t*turn]
            if not separated(sample,parked,footprint,pad):return False
    return separated(poses[-1],parked,footprint,margin/2)


def rescue_yaw_error(pose,target):
    """Face the target from the robot's actual position."""
    desired=math.atan2(target['y']-pose[1],target['x']-pose[0])
    return math.atan2(math.sin(pose[2]-desired),math.cos(pose[2]-desired))


def parking_pairs(grid,target,footprint,config):
    if target.get('yaw') is None:raise ValueError('C1 normal not available')
    candidates=[]
    for radius in config.get('parking_radii_m',[.50,.55,.60]):
        if not math.isfinite(radius) or radius<=0:raise ValueError('Invalid C1 parking radius')
        if radius>config['radius_m']-config['position_margin_m']+1e-6:continue
        for degrees in (0,25,-25,45,-45,65,-65):
            yaw=target['yaw']+math.radians(degrees)
            pose=[target['x']+radius*math.cos(yaw),target['y']+radius*math.sin(yaw),
                  math.atan2(-math.sin(yaw),-math.cos(yaw))]
            if footprint_clear(grid,pose,footprint,config.get('footprint_padding_m',.01)):candidates.append(pose)
    pairs=[(a,b) for i,a in enumerate(candidates) for b in candidates[i+1:]
           if math.dist(a[:2],b[:2])>=config['min_separation_m'] and separated(
               a,b,footprint,config.get('parking_peer_clearance_m',.08)/2)]
    if not pairs:raise ValueError('No clear, separated C1 parking pair')
    return pairs


class JointDwell:
    def __init__(self,radius,duration):
        self.radius,self.duration=radius,duration;self.start=None;self.last=None

    def awaiting_data(self,now,max_gap=.5):
        """Hold recent confirmed evidence during callback skew; never advance it."""
        keep=self.last is not None and 0<=now-self.last<=max_gap
        if not keep:self.start=self.last=None
        return keep

    def update(self,stamp,distances,valid=True):
        if (not valid or len(distances)!=2 or not all(math.isfinite(d) and d<=self.radius for d in distances)
                or (self.last is not None and (stamp<self.last or stamp-self.last>.5))):
            self.start=None;self.last=stamp;return False
        if self.start is None:self.start=stamp
        self.last=stamp
        return stamp-self.start>=self.duration
