"""Derive a robot's runtime config without duplicating controller tuning."""
from copy import deepcopy


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
