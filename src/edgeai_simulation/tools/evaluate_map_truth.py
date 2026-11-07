#!/usr/bin/env python3
"""Offline map/scene comparison anchored by same-run robot truth diagnostics."""
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import numpy as np
from PIL import Image
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
import yaml

from truth_geometry import compose


def pose(element):
    p = [float(v) for v in element.findtext('pose', '0 0 0 0 0 0').split()]
    assert abs(p[3])+abs(p[4]) < 1e-8, 'Requires planar scene'
    return (p[0], p[1], p[5]), p[2]


def transform(points, p):
    c, s = math.cos(p[2]), math.sin(p[2])
    return points @ np.array([[c, s], [-s, c]])+p[:2]


def boxes_from_world(path, height):
    boxes = []
    for model in ET.parse(path).getroot().findall('world/model'):
        if model.findtext('static') != 'true':
            continue
        mp, mz = pose(model)
        for link in model.findall('link'):
            lp, lz = pose(link)
            for collision in link.findall('collision'):
                size = collision.findtext('geometry/box/size')
                if not size:
                    continue
                size = np.array([float(v) for v in size.split()])
                cp, cz = pose(collision)
                if abs(height-(mz+lz+cz)) > size[2]/2:
                    continue
                p = compose(compose(mp, lp), cp)
                corners = transform(np.array([[-1,-1],[1,-1],[1,1],[-1,1]])*size[:2]/2, p)
                boxes.append(dict(name=model.get('name'), pose=p, size=size, corners=corners))
    return boxes


def inside_boxes(points, boxes):
    occupied = np.zeros(len(points), dtype=bool)
    for b in boxes:
        p = b['pose']; c, s = math.cos(p[2]), math.sin(p[2])
        d = points-p[:2]
        occupied |= (np.abs(c*d[:,0]+s*d[:,1]) <= b['size'][0]/2) & (
                     np.abs(-s*d[:,0]+c*d[:,1]) <= b['size'][1]/2)
    return occupied


def segments_for(boxes):
    corners = np.array([b['corners'] for b in boxes])
    return corners.reshape(-1,2), np.roll(corners,-1,axis=1).reshape(-1,2)


def boundary_distance(points, starts, ends):
    vectors = ends-starts
    d = points[:,None,:]-starts
    t = np.clip(np.sum(d*vectors,axis=2)/np.sum(vectors*vectors,axis=1),0,1)
    return np.linalg.norm(d-t[:,:,None]*vectors,axis=2).min(axis=1)


def stats(distances):
    return dict(mean_m=float(np.mean(distances)), median_m=float(np.median(distances)),
                p95_m=float(np.quantile(distances,.95)), max_m=float(np.max(distances)),
                within_005_pct=float(100*np.mean(distances<=.05)),
                within_010_pct=float(100*np.mean(distances<=.10)))


def exposed_boundaries(boxes):
    points, owners = [], []
    for i,b in enumerate(boxes):
        corners = b['corners']
        for a,z in zip(corners,np.roll(corners,-1,axis=0)):
            v = z-a
            n = np.array([v[1],-v[0]])/np.linalg.norm(v)
            count = max(1,math.ceil(np.linalg.norm(v)/.01))
            ps = a+((np.arange(count)+.5)/count)[:,None]*v
            outward = ps+.006*n
            visible = ((outward[:,0]>0)&(outward[:,0]<5)&
                       (outward[:,1]>0)&(outward[:,1]<5)&~inside_boxes(outward,boxes))
            points.extend(ps[visible]);owners.extend([i]*int(visible.sum()))
    return np.array(points), np.array(owners)


def evaluate(world_points, boxes, image, config, anchor):
    starts, ends = segments_for(boxes)
    distance = boundary_distance(world_points,starts,ends)
    exposed, owners = exposed_boundaries(boxes)
    reverse = cKDTree(world_points).query(exposed)[0]
    step = .025
    yy,xx = np.mgrid[step/2:5:step,step/2:5:step]
    grid = np.column_stack([xx.ravel(),yy.ravel()])
    true_free = ~inside_boxes(grid,boxes)
    clear = true_free & (boundary_distance(grid,starts,ends)>.10)
    c,s=math.cos(anchor[2]),math.sin(anchor[2])
    local=(grid-anchor[:2]) @ np.array([[c,-s],[s,c]])
    indices=np.floor((local-np.array(config['origin'][:2]))/config['resolution']).astype(int)
    indices[:,1]=image.shape[0]-1-indices[:,1]
    in_map=(indices[:,0]>=0)&(indices[:,0]<image.shape[1])&(indices[:,1]>=0)&(indices[:,1]<image.shape[0])
    probabilities=np.full(len(grid),np.nan)
    pix=image[indices[in_map,1],indices[in_map,0]]
    probabilities[in_map]=(255.-pix)/255 if not config.get('negate',0) else pix/255.
    free=probabilities<config['free_thresh']
    occupied=probabilities>config['occupied_thresh']
    known=free|occupied
    regions={'all':np.ones(len(grid),dtype=bool),
             'north_y_above_3.1':grid[:,1]>3.1,
             'south_y_below_3.1':grid[:,1]<=3.1}
    coverage={}
    for name,region in regions.items():
        mask=true_free&region
        inner=clear&region
        coverage[name]=dict(true_free_m2=float(mask.sum()*step**2),
            known_pct=float(100*np.mean(known[mask])),
            mapped_free_pct=float(100*np.mean(free[mask])),
            clear_free_known_pct=float(100*np.mean(known[inner])),
            clear_free_mapped_free_pct=float(100*np.mean(free[inner])),
            clear_free_marked_occupied_pct=float(100*np.mean(occupied[inner])))
    per_object={b['name']:stats(reverse[owners==i]) for i,b in enumerate(boxes) if np.any(owners==i)}
    result=dict(occupied_to_true_boundary=stats(distance), exposed_true_boundary_to_occupied=stats(reverse),
        occupied_cells=len(world_points), occupied_farther_than_010_m=int((distance>.10).sum()),
        free_space_coverage=coverage, per_object_boundary=per_object)
    return result,(xx,yy,true_free,known,free,occupied),distance


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('run',type=Path)
    parser.add_argument('--height',type=float,default=.16,help='Approximate lidar plane in world metres')
    parser.add_argument('--anchor-time',type=float,help='Proxy map time; default handover target stamp')
    args=parser.parse_args()
    directory=args.run;out=directory/'map-quality';out.mkdir(exist_ok=True)
    config=yaml.safe_load((directory/'map.yaml').read_text())
    assert abs(config['origin'][2])<1e-8, 'Requires axis-aligned map origin'
    image=np.array(Image.open(directory/config['image']))
    handover=json.loads((directory/'handover-targets.json').read_text())
    stamp=args.anchor_time if args.anchor_time is not None else handover['stamp_ns']/1e9
    rows=[json.loads(l) for l in (directory/'truth.jsonl').read_text().splitlines()]
    rows=[r for r in rows if r.get('status')=='ok' and r.get('reference_frame')=='map']
    row=min(rows,key=lambda r:abs(r['sample_time']-stamp))
    anchor=np.array(row['world_from_reference'])
    boxes=boxes_from_world(directory/'observation.sdf',args.height)
    probabilities=(255.-image)/255 if not config.get('negate',0) else image/255.
    ys,xs=np.where(probabilities>config['occupied_thresh'])
    points=np.column_stack([config['origin'][0]+(xs+.5)*config['resolution'],
                           config['origin'][1]+(image.shape[0]-ys-.5)*config['resolution']])
    world_points=transform(points,anchor)
    result,grid,distance=evaluate(world_points,boxes,image,config,anchor)
    # Secondary rigid fit separates anchor error from remaining geometric shape
    # error. It is not used for the primary, truth-anchored measurements.
    starts,ends=segments_for(boxes)
    center=np.array([2.5,2.5])
    adjust=lambda p:transform(world_points-center,p)+center
    fit=least_squares(lambda p:boundary_distance(adjust(p),starts,ends),np.zeros(3),
        bounds=([-.25,-.25,-math.radians(5)],[.25,.25,math.radians(5)]),loss='soft_l1',f_scale=.025)
    fitted=adjust(fit.x)
    result['best_rigid_fit']=dict(translation_at_arena_center_m=fit.x[:2].tolist(),
        rotation_deg=float(math.degrees(fit.x[2])), occupied_to_true_boundary=stats(boundary_distance(fitted,starts,ends)))
    # Fixed initial gauge is separate from the per-sample robot alignment.
    # It measures pose consistency; re-anchoring on every robot would hide it.
    initial=min(rows,key=lambda r:r['sample_time'])['world_from_reference']
    result['initial_anchor_pose_consistency']={}
    for name,selection in [('before_map_save',[r for r in rows if r['sample_time']<=stamp]),
                           ('whole_run',rows)]:
        errors=[]; angles=[]
        for r in selection:
            estimated=compose(initial,r['robot_estimated'])
            errors.append(math.dist(estimated[:2],r['robot_world'][:2]))
            diff=estimated[2]-r['robot_world'][2]
            angles.append(abs(math.degrees(math.atan2(math.sin(diff),math.cos(diff)))))
        result['initial_anchor_pose_consistency'][name]=dict(samples=len(errors),
            position_rmse_m=float(np.sqrt(np.mean(np.square(errors)))),
            position_p95_m=float(np.quantile(errors,.95)),position_max_m=float(max(errors)),
            yaw_p95_deg=float(np.quantile(angles,.95)),yaw_max_deg=float(max(angles)))
    # Long uncovered sections of the north boundary deserve separate review;
    # a small occupied-to-truth error alone cannot reveal missing wall cells.
    exposed,owners=exposed_boundaries(boxes)
    reverse=cKDTree(world_points).query(exposed)[0]
    missing=(reverse>.10)&np.array([boxes[i]['name']=='north_wall' for i in owners])
    result['north_wall_uncovered_samples_x']=exposed[missing,0].tolist()
    result['alignment']=dict(map_time_proxy_s=stamp,truth_sample_s=row['sample_time'],world_from_map=anchor.tolist(),
        caveat='Saved OccupancyGrid header stamp was not recorded; handover target timestamp is a proxy. Map predates rescue/search.')
    result['method']=dict(arena_world_xy=[0,5,0,5],map_resolution_m=config['resolution'],
        evaluation_step_m=.025,lidar_plane_approx_m=args.height,
        caveat='Static box collisions intersecting lidar height; occupied cell centres, not raw laser hits. Boundary distances include grid quantization; visibility inferred only by free-facing surfaces, not full ray tracing.')
    # Repeat with neighbouring truth anchors to expose timing sensitivity.
    result['anchor_sensitivity']=[]
    for target in (stamp-.5,stamp,stamp+.5):
        neighbour=min(rows,key=lambda r:abs(r['sample_time']-target))
        ds=boundary_distance(transform(points,neighbour['world_from_reference']),starts,ends)
        result['anchor_sensitivity'].append(dict(sample_time=neighbour['sample_time'],
            median_m=float(np.median(ds)),p95_m=float(np.quantile(ds,.95))))
    (out/'metrics.json').write_text(json.dumps(result,indent=2)+'\n')
    fig,axes=plt.subplots(1,2,figsize=(12,6),layout='constrained')
    for ax in axes:
        ax.add_collection(LineCollection(np.stack([starts,ends],axis=1),colors='black',linewidths=1.3,label='True obstacle edges'))
        ax.set(xlim=(-.15,5.15),ylim=(-.15,5.15),xlabel='World X (m)',ylabel='World Y (m)',aspect='equal')
        ax.grid(alpha=.15)
    axes[0].scatter(world_points[:,0],world_points[:,1],s=7,color='#d74742',label='SLAM occupied cells')
    axes[0].plot(row['robot_world'][0],row['robot_world'][1],'bo',label='Robot at handover')
    for name,x,y in [('S1',1,4.1),('C1',4.1,1)]:
        axes[0].plot(x,y,'c*',ms=9);axes[0].text(x+.08,y+.08,name,color='teal')
    axes[0].set_title('Map snapshot vs simulator geometry\nRobot-anchored; 5 cm map cells')
    axes[0].legend(loc='lower left',fontsize=8)
    xx,yy,true_free,known,free,occupied=grid
    rgba=np.ones((len(true_free),4))
    rgba[true_free&~known]=[.95,.63,.1,.8]
    rgba[true_free&free]=[.35,.75,.55,.55]
    rgba[true_free&occupied]=[.85,.22,.25,.9]
    rgba[~true_free]=[.7,.7,.7,.5]
    axes[1].imshow(rgba.reshape(*xx.shape,4),origin='lower',extent=(0,5,0,5),interpolation='nearest')
    axes[1].set_title('True free-space classification\nGreen: mapped free; orange: unknown; red: occupied')
    fig.savefig(out/'map-truth-comparison.png',dpi=180)
    fig.savefig(out/'map-truth-comparison.svg')
    plt.close(fig)
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    main()
