"""Derive a robot's runtime config without duplicating controller tuning."""
from copy import deepcopy
import json
import math


def parking_geometry(config, settings):
    """Derive task checks from the same runtime geometry as native planning."""
    grid=config['global_costmap']['global_costmap']['ros__parameters']
    padding=float(grid.get('footprint_padding',0.))
    settings['footprint_padding_m']=padding
    # Peer cells are conservatively rasterized by half a cell diagonal.
    settings['parking_peer_clearance_m']=max(float(settings.get('parked_peer_clearance_m',0.)),
                                           2*padding+math.sqrt(.5)*grid['resolution'])
    return json.loads(grid['footprint'])


def configure_peer_costmaps(config, namespace, settings):
    """Subscribe only to the other robot's C1 pose, before inflation."""
    if not settings.get('enabled',True):return
    global_map=config['global_costmap']['global_costmap']['ros__parameters']
    xs,ys=zip(*json.loads(global_map['footprint']))
    for key in ('global_costmap','local_costmap'):
        costmap=config[key][key]['ros__parameters']
        plugins=costmap['plugins']
        if 'peer_footprint' not in plugins:
            plugins.insert(plugins.index('inflation_layer'), 'peer_footprint')
        costmap['peer_footprint']=dict(plugin='edgeai_navigation::PeerFootprintLayer',
            pose_topic='/cooperative_rescue/peer_pose/'+('robot1' if namespace else 'robot2'),
            bounds=[min(xs),max(xs),min(ys),max(ys)],padding_m=float(global_map.get('footprint_padding',0.)),
            max_age_s=float(settings.get('max_age_s',.5)))


def namespaced_config(config, namespace):
    namespace=namespace.strip('/')
    if not namespace or '/' in namespace:
        raise ValueError('Expected one robot namespace')
    def visit(value):
        if isinstance(value,dict):return {k:visit(v) for k,v in value.items()}
        if isinstance(value,list):return [visit(v) for v in value]
        if isinstance(value,str):
            if value in ('base_footprint','base_link','odom'):
                return namespace+'/'+value
            if value=='map' or value=='/map':return '/map' if value.startswith('/') else 'map'
            if value.startswith('/') and not value.startswith(('/opt/','/home/','/mnt/')):
                return '/'+namespace+value
        return value
    result=visit(deepcopy(config))
    for name in ('local_costmap','global_costmap'):
        static=result[name][name]['ros__parameters'].get('static_layer')
        if static is not None:static['map_topic']='/map'
    amcl=result['amcl']['ros__parameters']
    amcl.update(map_topic='/map',scan_topic='scan',update_min_d=.05,update_min_a=.05,
                transform_tolerance=.2,save_pose_rate=-1.)
    return result
