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


def panels(rgb, depth, k, rotation, translation, panel_size=.20, max_regions=12):
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
    proposals=[]
    for contour in sorted(contours,key=cv2.contourArea,reverse=True)[:max_regions]:
        x,y,w,h=cv2.boundingRect(contour)
        if h<6 or w<2 or h*w>small.shape[0]*small.shape[1]*.30:
            continue
        region=np.zeros(mask.shape,np.uint8)
        cv2.drawContours(region,[contour],-1,1,-1)
        v,u=np.nonzero(region)
        u=u*2;v=v*2;z=depth[v,u]
        valid=np.isfinite(z)&(z>.20)&(z<3.)
        u,v,z=u[valid],v[valid],z[valid]
        if len(z)<18:
            continue
        # Reject depth discontinuities rather than fitting a plane through
        # background and panel together. At least 70% must be one surface.
        keep=np.abs(z-np.median(z))<max(.045,panel_size*.75)
        if keep.mean()<.70:
            continue
        u,v,z=u[keep][::2],v[keep][::2],z[keep][::2]
        if len(z)<12:
            continue
        optical=np.column_stack(((u-k[0,2])*z/k[0,0],(v-k[1,2])*z/k[1,1],z))
        points=optical@np.asarray(rotation).T+np.asarray(translation)
        centre=np.median(points,axis=0)
        centred=points-centre
        _,_,axes=np.linalg.svd(centred,full_matrices=False)
        normal=axes[-1]
        residual=float(np.sqrt(np.mean((centred@normal)**2)))
        if abs(normal[2])>.30 or residual>.012:
            continue
        normal=normal[:2]/np.linalg.norm(normal[:2])
        tangent=np.array([-normal[1],normal[0]])
        height=float(np.ptp(np.percentile(points[:,2],[3,97])))
        width=float(np.ptp(np.percentile(points[:,:2]@tangent,[3,97])))
        if not .65*panel_size<height<1.35*panel_size or not .006<width<1.35*panel_size:
            continue
        if not .12<centre[2]<.75:
            continue
        crop=rgb[2*y:min(2*(y+h),rgb.shape[0]),2*x:min(2*(x+w),rgb.shape[1])]
        gray=cv2.cvtColor(crop,cv2.COLOR_BGR2GRAY)
        black_fraction=float((gray<80).mean())
        ambiguous=width<.35*panel_size
        # White backs/sides remain candidates, never IDs. Black print boosts score.
        confidence=min(.9,.45+.25*min(1.,black_fraction/.15)+.15*(1-residual/.012))
        proposals.append(dict(position=centre.tolist(),normal=normal.tolist(),
            normal_kind='thin_edge' if width<.10*panel_size else 'surface',
            width_m=width,height_m=height,plane_rmse_m=residual,
            black_fraction=black_fraction,normal_ambiguous=ambiguous,confidence=confidence))
    return proposals
