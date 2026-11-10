"""Bind arrival rules to each managed action, without mutable selector topics."""
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
from copy import deepcopy


POLICIES = {
    'position': ('general_goal_checker', 'GridBased'),
    'heading': ('heading_goal_checker', 'GridBasedHeading'),
    'rescue': ('rescue_goal_checker', 'GridBasedHeading'),
}
CONTROLLERS = {'position':'FollowPath','heading':'FollowPathHeading','rescue':'FollowPathRescue'}


def configure_controllers(config):
    """Separate XY-only tracking from yaw alignment using the same base tuning."""
    control=config['controller_server']['ros__parameters']
    base=deepcopy(control['FollowPath'])
    for policy,(checker,_) in POLICIES.items():
        follow=deepcopy(base)
        primary=follow.get('primary_controller',follow['plugin'])
        if primary=='dwb_core::DWBLocalPlanner':
            critics=[c for c in follow['critics'] if c!='RotateToGoal']
            if policy!='position':critics.insert(0,'RotateToGoal')
            follow['critics']=critics
            # DWB cannot lock translation outside the selected checker's XY window.
            follow['xy_goal_tolerance']=control[checker]['xy_goal_tolerance']
            if policy=='position':
                for name in list(follow):
                    if name.startswith('RotateToGoal.'):del follow[name]
        elif primary=='nav2_mppi_controller::MPPIController':
            follow['critics']=[c for c in follow['critics'] if c!='GoalAngleCritic']
            if policy!='position':follow['critics'].append('GoalAngleCritic')
        elif primary=='nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController':
            # Reverse paths encode vehicle yaw, not the direction of travel.
            planner=config['planner_server']['ros__parameters']['GridBased']
            reverse=(policy=='position' and follow.get('allow_reversing',False)
                     and planner.get('allow_reverse_expansion',False))
            follow['allow_reversing']=reverse
            follow['use_rotate_to_heading']=policy!='position'
        if follow['plugin']=='nav2_rotation_shim_controller::RotationShimController':
            if primary=='nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController':
                # Native Shim owns initial/final rotation; RPP only tracks.
                follow['rotate_to_goal_heading']=policy!='position' and base.get('rotate_to_goal_heading',True)
                follow['use_path_orientations']=base.get('use_path_orientations',True)
                if follow['use_path_orientations']:
                    # Jazzy native Shim compares path yaw and robot yaw directly.
                    # Keep its controller pose and planner path in the same frame.
                    config['local_costmap']['local_costmap']['ros__parameters']['global_frame']=(
                        config['global_costmap']['global_costmap']['ros__parameters']['global_frame'])
                follow['use_rotate_to_heading']=False
            else:
                follow['rotate_to_goal_heading']=False
        control[CONTROLLERS[policy]]=follow
    control['controller_plugins']=list(dict.fromkeys(control['controller_plugins']+list(CONTROLLERS.values())))
    # Native behavior collision checking consumes bare Pose2D coordinates.
    # Its pose frame must match the costmap frame; it does not transform them.
    behavior=config['behavior_server']['ros__parameters']
    behavior['local_frame']=config['local_costmap']['local_costmap']['ros__parameters']['global_frame']
    behavior['global_frame']=config['global_costmap']['global_costmap']['ros__parameters']['global_frame']


def managed_trees(source, directory):
    if not source:
        return dict.fromkeys(POLICIES, '')  # Test backends without a behavior tree.
    runtime=Path(directory)/'nav2-runtime.yaml'
    actual='[]'
    robot_frame='base_footprint'
    if runtime.is_file():
        config=yaml.safe_load(runtime.read_text())
        actual=config['local_costmap']['local_costmap']['ros__parameters'].get('footprint','[]')
        robot_frame=config.get('bt_navigator',{}).get('ros__parameters',{}).get('robot_base_frame',robot_frame)
    directory = Path(directory)/'goal-policy-trees'
    directory.mkdir(parents=True, exist_ok=True)
    paths = {}
    for policy, (checker, planner) in POLICIES.items():
        tree = ET.parse(source)
        for parent in tree.iter():
            for child in list(parent):
                if child.tag in ('GoalCheckerSelector', 'PlannerSelector', 'ControllerSelector'):
                    parent.remove(child)
            if parent.tag == 'ComputePathToPose':
                parent.set('planner_id', planner)
            elif parent.tag == 'FollowPath':
                parent.set('goal_checker_id', checker)
                parent.set('controller_id', CONTROLLERS[policy])
            elif parent.tag == 'SelectRecovery':
                parent.set('controller_id', CONTROLLERS[policy])
                parent.set('actual_footprint',actual)
            if parent.tag in ('SelectRecovery','PathNearRobot'):
                parent.set('robot_frame',robot_frame)
        path = directory/f'{policy}.xml'
        tree.write(path, encoding='unicode')
        paths[policy] = str(path.resolve())
    return paths
