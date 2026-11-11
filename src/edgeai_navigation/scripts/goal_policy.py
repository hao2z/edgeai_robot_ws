"""Bind arrival rules to each managed action, without mutable selector topics."""
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
from copy import deepcopy


POLICIES = {
    'position': ('general_goal_checker', 'GridBased'),
    'heading': ('heading_goal_checker', 'GridBasedHeading'),
    'rescue': ('rescue_goal_checker', 'GridBasedRescue'),
    # Input yaw is a tie/order preference among feasible terminal headings.
    'frontier': ('frontier_goal_checker', 'GridBasedFrontier'),
    'cooperative': ('cooperative_goal_checker', 'GridBasedCooperative'),
}
CONTROLLERS = {'position':'FollowPath','heading':'FollowPathHeading','rescue':'FollowPathRescue',
               'frontier':'FollowPathFrontier','cooperative':'FollowPathCooperative'}

# Task/request adapters own policy selection; the authority only arbitrates execution.
ACTION_ROUTES = (
    ('exploration/navigate_to_pose', 'AUTO', 'frontier'),
    ('navigate_to_pose', 'MANUAL', 'position'),
    ('navigation/heading_to_pose', 'MANUAL', 'heading'),
    ('navigation/rescue_to_pose', 'MANUAL', 'rescue'),
    ('navigation/cooperative_to_pose', 'MANUAL', 'cooperative'),
)


def configure_controllers(config, profile_dir=None):
    """Separate XY-only tracking from yaw alignment using the same base tuning."""
    control=config['controller_server']['ros__parameters']
    base=deepcopy(control['FollowPath'])
    profile_dir=Path(profile_dir) if profile_dir else Path(__file__).resolve().parents[1]/'config/navigation_tasks'
    from navigation_config import merge
    planners=config['planner_server']['ros__parameters']
    planner_templates=deepcopy(planners)
    checker_templates=deepcopy(control)
    configured=set()
    for policy,(checker,planner_id) in POLICIES.items():
        settings=yaml.safe_load((profile_dir/(policy+'.yaml')).read_text())
        if set(settings)-{'require_heading','allow_reverse','controller','planner','goal_checker'}:
            raise ValueError('Unknown navigation task option: '+policy)
        if any(type(settings.get(k)) is not bool for k in ('require_heading','allow_reverse')):
            raise ValueError('Navigation task flags must be booleans: '+policy)
        heading=settings['require_heading'];reverse=settings['allow_reverse']
        for key in ('controller','planner','goal_checker'):
            if not isinstance(settings.get(key,{}),dict):raise ValueError('Invalid '+policy+'.'+key)
        if 'allow_reversing' in settings.get('controller',{}) or 'allow_reverse_expansion' in settings.get('planner',{}):
            raise ValueError('Use allow_reverse to coordinate planner and controller: '+policy)
        template='rescue_goal_checker' if policy in ('rescue','cooperative') else 'heading_goal_checker' if heading else 'general_goal_checker'
        control[checker]=merge(checker_templates.get(template,checker_templates['general_goal_checker']),settings.get('goal_checker',{}))
        if not heading:
            control[checker]['plugin']='nav2_controller::PositionGoalChecker'
            control[checker].pop('yaw_goal_tolerance',None)
        elif control[checker]['plugin']=='nav2_controller::PositionGoalChecker':
            control[checker]['plugin']='nav2_controller::SimpleGoalChecker'
            control[checker].setdefault('yaw_goal_tolerance',.25)
        planner=deepcopy(planner_templates.get('GridBasedHeading',planner_templates['GridBased']) if heading else planner_templates['GridBased'])
        planner=merge(planner,settings.get('planner',{}))
        if 'Lattice' in planner['plugin']:planner['allow_reverse_expansion']=reverse
        planners[planner_id]=planner
        controller=CONTROLLERS[policy]
        if controller in configured:continue
        configured.add(controller)
        follow=merge(base,settings.get('controller',{}))
        primary=follow.get('primary_controller',follow['plugin'])
        if primary=='dwb_core::DWBLocalPlanner':
            critics=[c for c in follow['critics'] if c!='RotateToGoal']
            if heading:critics.insert(0,'RotateToGoal')
            follow['critics']=critics
            # DWB cannot lock translation outside the selected checker's XY window.
            follow['xy_goal_tolerance']=control[checker]['xy_goal_tolerance']
            if not reverse:follow['min_vel_x']=max(0.,follow.get('min_vel_x',0.))
            if not heading:
                for name in list(follow):
                    if name.startswith('RotateToGoal.'):del follow[name]
        elif primary=='nav2_mppi_controller::MPPIController':
            follow['critics']=[c for c in follow['critics'] if c!='GoalAngleCritic']
            if heading:follow['critics'].append('GoalAngleCritic')
            if not reverse:follow['vx_min']=max(0.,follow.get('vx_min',0.))
        elif primary=='nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController':
            # Reverse paths encode vehicle yaw, not the direction of travel.
            follow['allow_reversing']=reverse and planner.get('allow_reverse_expansion',False)
            follow['use_rotate_to_heading']=heading and not follow['allow_reversing']
        if follow['plugin']=='nav2_rotation_shim_controller::RotationShimController':
            if primary=='nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController':
                # Native Shim owns initial/final rotation; RPP only tracks.
                follow['rotate_to_goal_heading']=heading and follow.get('rotate_to_goal_heading',True)
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
    control['goal_checker_plugins']=list(dict.fromkeys(control['goal_checker_plugins']+[c for c,_ in POLICIES.values()]))
    planners['planner_plugins']=list(dict.fromkeys(planners['planner_plugins']+[p for _,p in POLICIES.values()]))
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
