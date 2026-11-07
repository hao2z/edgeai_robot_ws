"""Failure routing and alternative parking geometry for one fixed rescue target."""
import math


PLANNER_ERRORS={200:'unknown_planner_error',201:'invalid_planner',202:'tf_error',
    203:'start_outside_map',204:'goal_outside_map',205:'start_occupied',206:'goal_occupied',
    207:'planning_timeout',208:'no_valid_path'}


def failure_reason(code):
    if code==0:return 'none'
    if code in PLANNER_ERRORS:return PLANNER_ERRORS[code]
    return {100:'unknown_controller_error',101:'invalid_controller',102:'tf_error',103:'invalid_path',
        104:'controller_patience_exceeded',105:'no_progress',106:'no_valid_control',107:'controller_timeout'}.get(code,'unknown')


def should_change_approach(code):return code in (204,206,208)


def approach_goal(nominal,config,angle=0.,radius=None):
    """Rebuild the chosen offset from the latest estimate, not the old goal."""
    radius=config['normal_offset_m'] if radius is None else radius
    center_x=nominal['x']+config['normal_offset_m']*math.cos(nominal['yaw'])
    center_y=nominal['y']+config['normal_offset_m']*math.sin(nominal['yaw'])
    normal=nominal['yaw']+math.pi+angle
    result=dict(nominal,x=center_x+radius*math.cos(normal),y=center_y+radius*math.sin(normal),
                yaw=math.atan2(-math.sin(normal),-math.cos(normal)))
    result.update(approach_angle_rad=angle,approach_distance_m=radius)
    return result


def approaches(nominal,config):
    settings=config.get('approach_selection',{})
    angles=settings.get('angles_deg',[0,30,-30,60,-60])
    radii=[config['normal_offset_m']]+settings.get('extra_distances_m',[])
    limit=config.get('target_radius_m',config['normal_offset_m']+config['position_tolerance_m'])
    seen=set();result=[]
    for degrees in angles:
        for radius in radii:
            if not 0<radius<=limit-config['position_tolerance_m']+1e-8:continue
            key=(round(float(degrees),6),round(float(radius),6))
            if key in seen:continue
            seen.add(key)
            result.append(approach_goal(nominal,config,math.radians(degrees),radius))
    return result[:settings.get('max_candidates',12)]
