"""RGB-D panel proposals, independent of ROS, IDs and navigation."""
import cv2
import numpy as np


def prefer_surface_normal(record, proposal):
    """Keep a measured face orientation when a later frame only sees its edge."""
    merged=dict(proposal)
    if (record.get('normal_kind')=='surface' and
        (proposal.get('normal_kind')=='thin_edge' or
         (not record.get('normal_ambiguous',True) and proposal.get('normal_ambiguous',False)))):
        for key in ('normal','normal_kind','normal_ambiguous','width_m','confidence'):
            if key in record:merged[key]=record[key]
    return merged


def depth_regions(depth, jump_m=.05):
    """Small disconnected surfaces, bounded by depth jumps rather than colour."""
    valid=np.isfinite(depth)&(depth>.20)&(depth<3.)
    mask=valid.copy()
    finite=np.where(valid,depth,0.)
    horizontal=valid[:,1:]&valid[:,:-1]&(np.abs(np.diff(finite,axis=1))>jump_m)
    vertical=valid[1:,:]&valid[:-1,:]&(np.abs(np.diff(finite,axis=0))>jump_m)
    mask[:,1:]&=~horizontal;mask[:,:-1]&=~horizontal
    mask[1:,:]&=~vertical;mask[:-1,:]&=~vertical
    count,labels,stats,_=cv2.connectedComponentsWithStats(mask.astype('uint8'),connectivity=4)
    return [(labels==i).astype('uint8') for i in sorted(
        range(1,count),key=lambda i:stats[i,cv2.CC_STAT_AREA],reverse=True)[:12]
        if stats[i,cv2.CC_STAT_AREA]>=18]


def laser_support(proposal, points, radius_m=.10):
    """Positive corroboration only: a planar scan may miss the panel height."""
    if len(points)==0:return False
    points=np.asarray(points)
    centre=np.asarray(proposal['position'])
    return bool(np.any((np.linalg.norm(points[:,:2]-centre[:2],axis=1)<radius_m)&
                       (np.abs(points[:,2]-centre[2])<proposal['height_m']/2+.02)))


def panels(rgb, depth, k, rotation, translation, panel_size=.20, max_regions=12,
           depth_geometry=True, diagnostics=None):
    """Registered optical RGB/depth in metres; pose transforms optical to odom.

    A narrow side is deliberately tentative: its measured surface normal need
    not be the printed face normal. The consumer must test both axis pairs.
    """
    if rgb.shape[:2] != depth.shape or depth.ndim != 2:
        raise ValueError('RGB/depth must be registered and have identical dimensions')
    k=np.asarray(k).reshape(3,3)
    if k[0,0]<=0 or k[1,1]<=0:
        raise ValueError('Invalid intrinsics')
    small=rgb[::2,::2]
    high=small.max(axis=2);low=small.min(axis=2)
    white=((low>125)&((high.astype(float)-low)<65)).astype('uint8')*255
    mask=cv2.morphologyEx(white,cv2.MORPH_CLOSE,np.ones((5,3),np.uint8))
    contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    regions=[]
    for contour in sorted(contours,key=cv2.contourArea,reverse=True)[:max_regions]:
        region=np.zeros(mask.shape,np.uint8)
        cv2.drawContours(region,[contour],-1,1,-1)
        regions.append(('rgbd',region))
    if depth_geometry:
        regions.extend(('depth',region) for region in depth_regions(depth[::2,::2]))
    proposals=[]
    def reject(reason):
        if diagnostics is not None:diagnostics['reject_'+reason]+=1
    for source,region in regions:
        x,y,w,h=cv2.boundingRect(region)
        if h<6 or w<2 or h*w>small.shape[0]*small.shape[1]*.30:
            reject('image_extent')
            continue
        v,u=np.nonzero(region)
        u=u*2;v=v*2;z=depth[v,u]
        valid=np.isfinite(z)&(z>.20)&(z<3.)
        u,v,z=u[valid],v[valid],z[valid]
        if len(z)<18:
            reject('depth_samples')
            continue
        # Reject depth discontinuities rather than fitting a plane through
        # background and panel together. At least 70% must be one surface.
        keep=np.abs(z-np.median(z))<max(.045,panel_size*.75)
        if keep.mean()<.70:
            reject('depth_discontinuity')
            continue
        u,v,z=u[keep][::2],v[keep][::2],z[keep][::2]
        if len(z)<12:
            reject('plane_samples')
            continue
        optical=np.column_stack(((u-k[0,2])*z/k[0,0],(v-k[1,2])*z/k[1,1],z))
        points=optical@np.asarray(rotation).T+np.asarray(translation)
        centre=np.median(points,axis=0)
        centred=points-centre
        _,_,axes=np.linalg.svd(centred,full_matrices=False)
        normal=axes[-1]
        residual=float(np.sqrt(np.mean((centred@normal)**2)))
        if abs(normal[2])>.30 or residual>.012:
            reject('plane')
            continue
        normal=normal[:2]/np.linalg.norm(normal[:2])
        tangent=np.array([-normal[1],normal[0]])
        height=float(np.ptp(np.percentile(points[:,2],[3,97])))
        width=float(np.ptp(np.percentile(points[:,:2]@tangent,[3,97])))
        if not .65*panel_size<height<1.35*panel_size or not .006<width<1.35*panel_size:
            reject('physical_size')
            continue
        if not .12<centre[2]<.75:
            reject('height')
            continue
        crop=rgb[2*y:min(2*(y+h),rgb.shape[0]),2*x:min(2*(x+w),rgb.shape[1])]
        gray=cv2.cvtColor(crop,cv2.COLOR_BGR2GRAY)
        black_fraction=float((gray<80).mean())
        ambiguous=width<.35*panel_size
        # White backs/sides remain candidates, never IDs. Black print boosts score.
        confidence=min(.9,.45+.25*min(1.,black_fraction/.15)+.15*(1-residual/.012))
        # One physical observation must never count as two confirmation frames.
        if any(np.linalg.norm(centre-p['position'])<.08 for p in proposals):continue
        if diagnostics is not None:diagnostics['accepted_'+source]+=1
        proposals.append(dict(position=centre.tolist(),normal=normal.tolist(),source=source,
            normal_kind='thin_edge' if width<.10*panel_size else 'surface',
            width_m=width,height_m=height,plane_rmse_m=residual,
            black_fraction=black_fraction,normal_ambiguous=ambiguous,confidence=confidence))
    return proposals
