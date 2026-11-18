"""Derive a robot's runtime config without duplicating controller tuning."""
from copy import deepcopy
import json
import math
from pathlib import Path


def frontier_endpoint_geometry(config):
    """Give the exporter the padded global footprint and native lattice angles."""
    grid = config['global_costmap']['global_costmap']['ros__parameters']
    padding = float(grid.get('footprint_padding') or 0.)
    planner = config['planner_server']['ros__parameters']
    lattice = planner.get('GridBasedFrontier', planner['GridBased']).get('lattice_filepath')
    headings = [i * math.tau / 16 for i in range(16)]
    if lattice:
        headings = json.loads(Path(lattice).read_text())['lattice_metadata']['heading_angles']
    points = json.loads(grid.get('footprint') or '[]')
    if points:
        footprint = [float(v) + (math.copysign(padding, v) if v else 0.) for pt in points for v in pt]
    else:
        radius = (float(grid['robot_radius']) + padding) / math.cos(math.pi / 16)
        footprint = [v * radius for i in range(16)
                     for v in (math.cos(i * math.tau / 16), math.sin(i * math.tau / 16))]
    return dict(frontier_footprint=footprint, frontier_heading_angles=list(map(float, headings)))


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
            pose_topic='/cooperative_rescue/peer_pose/'+('robot2' if namespace.strip('/') == 'robot1' else 'robot1'),
            bounds=[min(xs),max(xs),min(ys),max(ys)],padding_m=float(global_map.get('footprint_padding',0.)),
            max_age_s=float(settings.get('max_age_s',.5)))


def namespaced_config(config, namespace, source_namespace=''):
    namespace=namespace.strip('/')
    if not namespace or '/' in namespace:
        raise ValueError('Expected one robot namespace')
    def visit(value,key=''):
        if isinstance(value,dict):return {k:visit(v,k) for k,v in value.items()}
        if isinstance(value,list):return [visit(v,key) for v in value]
        if isinstance(value,str):
            for frame in ('base_footprint','base_link','odom','camera_optical_frame','lidar_link'):
                if not key.endswith('_topic') and value == ((source_namespace.strip('/')+'/') if source_namespace else '')+frame:
                    return namespace+'/'+frame
            if source_namespace and value.startswith('/'+source_namespace.strip('/')+'/'):
                return '/'+namespace+value[len(source_namespace.strip('/'))+1:]
            if value in ('/tf','/tf_static','/clock') or value.startswith(('/targets/','/joint_mapping/','/cooperative_rescue/')):return value
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
