"""Explicit mapping/localization/task membership; one mapper is a fleet subset."""
import math
from edgeai_tasks.cooperative_policy import compose,relative


def fleet_layout(joint_mapping,cooperative,relative_pose):
    robots={'robot1':dict(namespace='robot1',base_frame='robot1/base_footprint',odom_frame='robot1/odom',initial_relative_pose=[0.,0.,0.]),
            'robot2':dict(namespace='robot2',base_frame='robot2/base_footprint',odom_frame='robot2/odom',
                          initial_relative_pose=list(relative_pose))}
    localized=['robot1','robot2'] if joint_mapping or cooperative else ['robot1']
    mappers=['robot1','robot2'] if joint_mapping else ['robot1']
    return dict(robots=robots,mapping_robots=mappers,localization_robots=localized,
                running_navigation_robots=list(mappers),
                task_robots=dict(s1=list(localized),c1=['robot1','robot2'] if cooperative else []))


def configured_fleet(mission,relative_pose):
    """Normalize earlier runtime snapshots at the configuration boundary only."""
    fleet=mission.get('fleet')
    if fleet is None:
        fleet=fleet_layout(mission.get('joint_mapping',False),mission.get('cooperative',False),relative_pose)
    validate_fleet(fleet)
    return fleet


def validate_fleet(fleet):
    robots=fleet['robots'];mappers=fleet['mapping_robots'];localized=fleet['localization_robots']
    if (not mappers or not localized or len(set(mappers))!=len(mappers) or
            len(set(localized))!=len(localized) or not set(mappers)<=set(localized)<=set(robots)):
        raise ValueError('Invalid mapping/localization membership')
    if not set(fleet['running_navigation_robots'])<=set(localized):
        raise ValueError('Running navigation robots must participate in localization')
    if any(not set(group)<=set(localized) for group in fleet['task_robots'].values()):
        raise ValueError('Task robot is not included in localization')
    if len(fleet['task_robots'].get('c1',[])) not in (0,2):
        raise ValueError('C1 requires two localized robots')
    for robot in localized:
        pose=robots[robot]['initial_relative_pose']
        if len(pose)!=3 or not all(math.isfinite(v) for v in pose):raise ValueError('Invalid relative pose')


def localization_priors(fleet,map_poses,odom_poses,reference=None):
    """Mapped robots use final SLAM; others propagate the fixed startup prior."""
    validate_fleet(fleet)
    poses={}
    for robot in fleet['localization_robots']:
        if robot in fleet['mapping_robots']:
            poses[robot]=list(map_poses[robot])
        else:
            if reference is None:raise ValueError('Initial fleet reference missing for '+robot)
            initial_odom=reference.get('initial_odoms',{}).get(robot)
            # Read previously recorded two-robot sessions too.
            if initial_odom is None and robot=='robot2':initial_odom=reference.get('initial_odom2')
            if initial_odom is None:raise ValueError('Initial odometry reference missing for '+robot)
            poses[robot]=compose(compose(reference['mapping_start_pose'],
                fleet['robots'][robot]['initial_relative_pose']),relative(initial_odom,odom_poses[robot]))
    return poses
