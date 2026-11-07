"""Build the official non-circular differential-drive lattice profile."""
from copy import deepcopy
from pathlib import Path
import yaml


def configure_lattice(config, resource_dir, smac_share):
    resource_dir=Path(resource_dir)
    planner=yaml.safe_load((resource_dir/'smac_lattice.yaml').read_text())['GridBased']
    relative=Path(planner['lattice_filepath'])
    planner['lattice_filepath']=str(Path(smac_share).joinpath(*relative.parts[1:]))
    if not Path(planner['lattice_filepath']).is_file():
        raise ValueError('Official differential-drive lattice primitives are missing')
    rectangle=yaml.safe_load((resource_dir/'nav2_exploration_rectangle.yaml').read_text())
    for section in ('local_costmap','global_costmap'):
        costmap=config[section][section]['ros__parameters']
        if abs(costmap['resolution']-.05)>1e-9:
            raise ValueError('Selected lattice primitives require 0.05 m costmap resolution')
        # The old MPPI comparison enclosed the body in a circle. SE2 planning
        # must use the actual axle-centred polygon in planner and controller.
        physical=rectangle[section][section]['ros__parameters']
        costmap.pop('robot_radius',None)
        costmap['footprint']=physical['footprint']
        costmap['footprint_padding']=physical['footprint_padding']
    global_map=config['global_costmap']['global_costmap']['ros__parameters']
    global_map['footprint_padding']=max(.02,global_map['footprint_padding'])
    planners=config['planner_server']['ros__parameters']
    planners['GridBased']=deepcopy(planner)
    # Ordinary position goals choose a reachable heading; explicit yaw goals
    # use the unmodified native planner and the requested goal orientation.
    planners['GridBasedHeading']=deepcopy(planner)
    planners['GridBasedHeading']['plugin']='nav2_smac_planner::SmacPlannerLattice'
    planners['GridBasedHeading']['allow_reverse_expansion']=False
    follow=config['controller_server']['ros__parameters']['FollowPath']
    primary=follow.get('primary_controller',follow['plugin'])
    if primary=='nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController':
        # A planner must not emit reverse segments for a forward-only tracker.
        planners['GridBased']['allow_reverse_expansion']=(
            planner['allow_reverse_expansion'] and follow.get('allow_reversing',False))
    if primary=='nav2_mppi_controller::MPPIController':
        follow['CostCritic']['consider_footprint']=True
        follow['PathAlignCritic']['use_path_orientations']=True
        follow['PathAngleCritic']['mode']=2
