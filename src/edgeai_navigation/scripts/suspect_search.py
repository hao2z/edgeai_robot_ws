"""Finish each suspected panel's available views before baseline search."""
from collections import deque
import math
import numpy as np
from frustum_search import line_clear


def without_known(hints,registry,radius=.30):
    known=[]
    for target in (registry or {}).get('targets',[]):
        point=target.get('position')
        if point is None and target.get('x') is not None and target.get('y') is not None:
            point=[target['x'],target['y'],target.get('z',.25)]
        if point is not None and len(point)==3 and np.isfinite(point).all():known.append(point)
    return [hint for hint in hints if not any(
        math.dist(hint['position'][:2],point[:2])<radius and abs(hint['position'][2]-point[2])<.25
        for point in known)]


def full_panel_view(camera, view, centre, normal, panel_size, margin=.10, min_side_px=24.):
    """Project the whole assumed face, not only its centre; return ray endpoints."""
    x,y,yaw=view;c,s=math.cos(yaw),math.sin(yaw)
    rz=np.array([[c,-s,0.],[s,c,0.],[0.,0.,1.]])
    camera_world=np.array([x,y,0.])+rz@np.array(camera.position)
    nx,ny=normal
    tangent=np.array([-ny,nx,0.]);centre=np.asarray(centre)
    # Sample centre, edges and corners for conservative 2D map occlusion.
    points=np.array([centre+a*panel_size/2*tangent+np.array([0.,0.,b*panel_size/2])
                     for a in (-1,0,1) for b in (-1,0,1)])
    base=(points-np.array([x,y,0.]))@rz-np.array(camera.position)
    optical=base@np.array(camera.rotation).reshape(3,3)
    z=optical[:,2]
    if np.any(z<=.15):return None
    u=camera.fx*optical[:,0]/z+camera.cx
    v=camera.fy*optical[:,1]/z+camera.cy
    if not (np.all((u>=margin*camera.width)&(u<=(1-margin)*camera.width))
            and np.all((v>=margin*camera.height)&(v<=(1-margin)*camera.height))
            and np.ptp(u)>=min_side_px and np.ptp(v)>=min_side_px):
        return None
    return camera_world,points,dict(projected_width_px=float(np.ptp(u)),
        projected_height_px=float(np.ptp(v)),camera_distance_m=float(np.linalg.norm(camera_world-centre)))


def normal_views(hints, data, width, height, resolution, origin, robot,
                 clearance=.4, standoff=.8, camera=None, config=None):
    if camera is None:return []  # Never assume a view is valid without calibration.
    config=config or {}
    panel_size=config.get('panel_size_m',.20)
    near,far=config.get('min_standoff_m',.55),config.get('max_standoff_m',1.4)
    if not 0<near<=standoff<=far or panel_size<=0:
        raise ValueError('Invalid suspect viewing distance or panel size')
    distances=sorted(set([standoff,*np.linspace(near,far,7)]),key=lambda d:abs(d-standoff))
    half_angle=config.get('view_half_angle_deg',70.)
    angle_samples=config.get('view_angle_samples',7)
    if not 0<=half_angle<=70 or angle_samples<1 or angle_samples>15 or angle_samples%2!=1:
        raise ValueError('Suspect fan needs an odd 1..15 samples and half-angle 0..70 degrees')
    angles=sorted(set(np.linspace(-half_angle,half_angle,angle_samples)),key=abs)
    ox,oy=origin
    cell=lambda x,y:(math.floor((x-ox)/resolution),math.floor((y-oy)/resolution))
    def free(x,y):
        return 0<=x<width and 0<=y<height and data[y*width+x]==0
    start=cell(*robot[:2])
    if not free(*start):return []
    reached={start};queue=deque([start])
    while queue:
        x,y=queue.popleft()
        for p in ((x-1,y),(x+1,y),(x,y-1),(x,y+1)):
            if p not in reached and free(*p):
                reached.add(p);queue.append(p)
    radius=math.ceil(clearance/resolution)+1
    offsets=[(x,y) for x in range(-radius,radius+1) for y in range(-radius,radius+1)
             if math.hypot(x,y)*resolution<=clearance+resolution/math.sqrt(2)]
    views=[]
    for hint in hints:
        x,y,_=hint['position'];nx,ny=hint['normal']
        if nx<0 or (abs(nx)<1e-8 and ny<0):nx,ny=-nx,-ny
        thin_edge=(hint.get('normal_kind')=='thin_edge' or
                   (0<hint.get('width_m',float('inf'))<.10*panel_size))
        # A millimetre-wide face normal points along the board's edge, not
        # toward the printed broad face. Do not offer that same side view again.
        axes=[(1,-ny,nx,'inferred_face_from_edge',0)] if thin_edge else [(0,nx,ny,'measured_surface',0)]
        if hint.get('normal_ambiguous',False) and not thin_edge:
            axes=[(0,nx,ny,'measured_surface',0),(1,-ny,nx,'alternate_face',1)]
        for axis,ax,ay,basis,rank in axes:
            for sign in (-1,1):
                for offset in angles:
                    angle=math.radians(offset);ca,sa=math.cos(angle),math.sin(angle)
                    dx,dy=ca*ax-sa*ay,sa*ax+ca*ay
                    for distance in distances:
                        vx,vy=x+sign*distance*dx,y+sign*distance*dy
                        cx,cy=cell(vx,vy)
                        if (cx,cy) not in reached or not all(free(cx+a,cy+b) for a,b in offsets):
                            continue
                        yaw=math.atan2(y-vy,x-vx)
                        # Rotate the viewpoint, never the hypothesized panel.
                        frame=full_panel_view(camera,(vx,vy,yaw),hint['position'],(ax,ay),panel_size,
                            config.get('image_margin',.1),config.get('min_panel_side_px',24.))
                        if frame is None:continue
                        camera_world,points,metrics=frame
                        if not all(line_clear(data,width,height,cell(*camera_world[:2]),cell(*p[:2])) for p in points):
                            continue
                        views.append(dict(x=vx,y=vy,yaw=yaw,standoff_m=float(distance),
                            strategy='suspected_panel',require_heading=True,suspect_id=hint['id'],
                            suspect_position=hint['position'],normal_axis=axis,normal_sign=sign,
                            normal_basis=basis,normal_hypothesis_rank=rank,
                            face_normal_yaw=math.atan2(sign*ay,sign*ax),view_angle_offset_deg=float(offset),
                            confidence=hint['confidence'],**metrics))
                        break
    return sorted(views,key=lambda v:(v['normal_hypothesis_rank'],
        abs(v['view_angle_offset_deg']),
        math.hypot(v['x']-robot[0],v['y']-robot[1])-.5*v['confidence']))


class ParallelCandidates:
    def __init__(self, baseline, node, camera_provider=None):
        self.baseline=baseline;self.node=node;self.positions=baseline.positions
        self.used=[];self.used_positions=[];self.active_id=None;self.visits=0
        if camera_provider is None:
            from search_candidates import camera_model
            camera_provider=lambda:camera_model(node)
        self.camera_provider=camera_provider

    def next_view(self,robot):
        node=self.node;c=node.c.get('search_suspects',{})
        hints=node.search_suspects or {}
        now=node.get_clock().now().nanoseconds/1e9
        grid=node.search_map
        views=[]
        if (self.visits<c.get('max_positions',4) and hints.get('frame_id')=='map'
            and 0<=now-hints.get('stamp',-100)<2.):
            fresh=[h for h in hints.get('candidates',[]) if
                   now>=h['last_seen'] and (now-h['last_seen']<c.get('max_age_s',8.) or
                   (h.get('retained',False) and h.get('observations',0)>=2))
                   and h['confidence']>=c.get('min_confidence',.5)]
            fresh=without_known(fresh,node.read_target_registry(allow_handover=True),c.get('known_target_radius_m',.30))
            try:
                camera=self.camera_provider()
            except ValueError:
                camera=None
            views=normal_views(fresh,grid.data,grid.info.width,grid.info.height,grid.info.resolution,
                (grid.info.origin.position.x,grid.info.origin.position.y),robot,
                node.c.get('search_clearance_m',.4),c.get('standoff_m',.8),camera,c)
            views=[v for v in views if (v['suspect_id'],v['normal_axis'],v['normal_sign'],v['view_angle_offset_deg']) not in self.used
                   and all(math.dist((v['x'],v['y']),p)>=c.get('min_view_spacing_m',.30) for p in self.used_positions)]
        same_round=[v for v in views if v['suspect_id']==self.active_id]
        if same_round:
            views=same_round
        if views:
            # Within the best face hypothesis, visit the opposite side before
            # spending the budget on more angles on an already attempted side.
            def side_visits(view):
                return sum(sid==view['suspect_id'] and axis==view['normal_axis'] and sign==view['normal_sign']
                           for sid,axis,sign,_ in self.used)
            views.sort(key=lambda v:(v['normal_hypothesis_rank'],side_visits(v)))
            view=views[0]
            self.used.append((view['suspect_id'],view['normal_axis'],view['normal_sign'],view['view_angle_offset_deg']))
            self.used_positions.append((view['x'],view['y']));self.visits+=1
            self.active_id=view['suspect_id']
            return view
        self.active_id=None
        return self.baseline.next_view(robot)

    def record_sweep(self,actual_pose):
        self.baseline.record_sweep(actual_pose)
