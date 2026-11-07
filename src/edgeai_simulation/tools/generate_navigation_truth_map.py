#!/usr/bin/env python3
"""Rasterize static SDF collision boxes in world XY for navigation benchmarks."""
import argparse
import json
import math
from pathlib import Path
from urllib.parse import unquote,urlparse
import xml.etree.ElementTree as ET
import cv2
import numpy as np
import yaml
from truth_geometry import compose


def pose(element):
    values=list(map(float,element.findtext('pose','0 0 0 0 0 0').split()))
    if abs(values[3])+abs(values[4])>1e-8:raise ValueError('Planar SDF required')
    return (values[0],values[1],values[5]),values[2]


def collision_boxes(world_file):
    world=ET.parse(world_file).getroot().find('world');models=[]
    for model in world.findall('model'):models.append((model,(0.,0.,0.),0.))
    for include in world.findall('include'):
        uri=include.findtext('uri','');parsed=urlparse(uri)
        if parsed.scheme!='file':raise ValueError('Benchmark requires local file model includes')
        path=Path(unquote(parsed.path));path=path/'model.sdf' if path.is_dir() else path
        models.append((ET.parse(path).getroot().find('model'),*pose(include)))
    boxes=[]
    for model,parent,pz in models:
        if model.findtext('static')!='true':continue
        if model.get('name')=='ground_plane':continue
        mp,mz=pose(model);mp=compose(parent,mp)
        for link in model.findall('link'):
            lp,lz=pose(link)
            for collision in link.findall('collision'):
                text=collision.findtext('geometry/box/size')
                if text is None:raise ValueError('Unsupported static collision shape in benchmark')
                sx,sy,sz=map(float,text.split());cp,cz=pose(collision)
                # Project obstacles above the floor, including bases and raised panels.
                if pz+mz+lz+cz+sz/2<=.02:continue
                boxes.append(dict(name=model.get('name'),pose=compose(compose(mp,lp),cp),size=[sx,sy,sz]))
    return boxes


def generate(world_file,output,resolution=.05):
    if resolution<=0:raise ValueError('Positive resolution required')
    boxes=collision_boxes(world_file)
    if not boxes:raise ValueError('No static obstacle geometry')
    # Arena bounds include the boundary walls. Cells beyond them stay unknown.
    corners=[]
    for b in boxes:
        x,y,a=b['pose'];sx,sy,_=b['size'];c,s=math.cos(a),math.sin(a)
        corners.extend((x+c*dx-s*dy,y+s*dx+c*dy) for dx in (-sx/2,sx/2) for dy in (-sy/2,sy/2))
    bounds=np.asarray(corners)
    origin=np.floor((bounds.min(axis=0)-resolution)/resolution)*resolution
    shape=np.ceil((bounds.max(axis=0)+resolution-origin)/resolution).astype(int)
    xx,yy=np.meshgrid(origin[0]+(np.arange(shape[0])+.5)*resolution,
                      origin[1]+(np.arange(shape[1])+.5)*resolution)
    image=np.full(xx.shape,254,np.uint8)
    outside=(xx<bounds[:,0].min())|(xx>bounds[:,0].max())|(yy<bounds[:,1].min())|(yy>bounds[:,1].max())
    image[outside]=205
    for b in boxes:
        x,y,a=b['pose'];sx,sy,_=b['size'];c,s=math.cos(a),math.sin(a);dx,dy=xx-x,yy-y
        # Separating-axis test: mark every cell intersecting each collision rectangle.
        half=resolution/2;pad=half*(abs(c)+abs(s))
        hit=((abs(dx)<=abs(c)*sx/2+abs(s)*sy/2+half)&
             (abs(dy)<=abs(s)*sx/2+abs(c)*sy/2+half)&
             (abs(c*dx+s*dy)<=sx/2+pad)&(abs(-s*dx+c*dy)<=sy/2+pad))
        image[hit]=0
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True)
    pgm=output.with_suffix('.pgm')
    if not cv2.imwrite(str(pgm),np.flipud(image)):raise RuntimeError('Map image write failed')
    output.with_suffix('.yaml').write_text(yaml.safe_dump(dict(image=pgm.name,mode='trinary',
        resolution=resolution,origin=[float(origin[0]),float(origin[1]),0.],negate=0,
        occupied_thresh=.65,free_thresh=.196),sort_keys=False))
    output.with_suffix('.geometry.json').write_text(json.dumps(dict(frame='map = Gazebo world XY',
        boxes=boxes,resolution=resolution,source=str(world_file)),indent=2))
    return image,origin,boxes


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('world',type=Path);p.add_argument('output',type=Path)
    args=p.parse_args();generate(args.world,args.output)
