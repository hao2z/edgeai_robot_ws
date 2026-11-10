"""Independent test-only geometry. Not imported by mission or perception."""
import math
import xml.etree.ElementTree as ET
import numpy as np


def rectangle(x, y, heading, length, width, offset_x=0.):
    points = np.array([[-length/2,-width/2], [length/2,-width/2],
                       [length/2,width/2], [-length/2,width/2]])
    points[:,0] += offset_x
    c,s=math.cos(heading),math.sin(heading)
    return points @ np.array([[c,s],[-s,c]]) + [x,y]


def polygon_distance(a, b):
    # Separating axis test distinguishes overlap from disjoint edge distances.
    separated=False
    for polygon in (a,b):
        for edge in np.roll(polygon,-1,axis=0)-polygon:
            axis=np.array([-edge[1],edge[0]])
            pa,pb=a@axis,b@axis
            if pa.max()<pb.min() or pb.max()<pa.min():
                separated=True
    if not separated:
        return 0.
    result=math.inf
    for points,polygon in ((a,b),(b,a)):
        starts,ends=polygon,np.roll(polygon,-1,axis=0)
        edges=ends-starts
        delta=points[:,None,:]-starts[None,:,:]
        t=np.clip(np.sum(delta*edges[None,:,:],axis=2)/np.sum(edges*edges,axis=1),0,1)
        projections=starts[None,:,:]+t[:,:,None]*edges[None,:,:]
        result=min(result,float(np.linalg.norm(points[:,None,:]-projections,axis=2).min()))
    return result


def segment_hits_polygon(start, finish, polygon):
    """Intersect a closed segment with a convex polygon using slab clipping."""
    start, finish = np.asarray(start), np.asarray(finish)
    lower, upper = 0., 1.
    for edge in np.roll(polygon, -1, axis=0)-polygon:
        axis = np.array([-edge[1], edge[0]])
        extent = polygon @ axis
        origin, direction = start @ axis, (finish-start) @ axis
        if abs(direction) < 1e-12:
            if origin < extent.min() or origin > extent.max():
                return False
            continue
        a, b = sorted(((extent.min()-origin)/direction, (extent.max()-origin)/direction))
        lower, upper = max(lower, a), min(upper, b)
        if lower > upper:
            return False
    return True


def collision_polygons(world):
    obstacles=[]
    for model in ET.parse(world).getroot().find('world').findall('model'):
        values=list(map(float,(model.findtext('pose') or '0 0 0 0 0 0').split()))
        for box in model.findall('./link/collision/geometry/box'):
            dimensions=list(map(float,box.findtext('size').split()))
            obstacles.append(rectangle(values[0],values[1],values[5],*dimensions[:2]))
    return obstacles


def clearance_report(world, trajectory):
    obstacles = collision_polygons(world)
    minimum=math.inf
    intersections=0
    for _,x,y,heading in trajectory:
        # Conservative planar rectangle encloses chassis, wheels and casters.
        robot=rectangle(x,y,heading,0.44,0.34,offset_x=-0.005)
        distance=min(polygon_distance(robot,obstacle) for obstacle in obstacles)
        minimum=min(minimum,distance)
        intersections+=distance==0.
    return {'minimum_clearance_m':minimum,'overlap_samples':int(intersections)}
