"""Derive RPP limits from the final actuator envelope; report tuning conflicts."""
import math

RPP='nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController'


def configure_motion_limits(config):
    smooth=config['velocity_smoother']['ros__parameters']
    control=config['controller_server']['ros__parameters']
    lo,hi=smooth['min_velocity'],smooth['max_velocity']
    accel,decel=smooth['max_accel'],smooth['max_decel']
    for label,values in (('min_velocity',lo),('max_velocity',hi),('max_accel',accel),('max_decel',decel)):
        if len(values)!=3 or not all(math.isfinite(v) for v in values):
            raise ValueError('MOTION_LIMITS: invalid velocity_smoother.'+label)
    if not (lo[0]<0<hi[0] and lo[2]<0<hi[2] and accel[0]>0 and accel[2]>0 and decel[0]<0 and decel[2]<0):
        raise ValueError('MOTION_LIMITS: require positive forward/reverse/angular and acceleration limits')
    angular=min(hi[2],-lo[2])
    angular_accel=min(accel[2],-decel[2])
    report={'source':'velocity_smoother','forward_mps':hi[0],'reverse_mps':-lo[0],
            'angular_rps':angular,'controllers':{},'warnings':[]}
    for name in control['controller_plugins']:
        follow=control[name]
        if follow.get('primary_controller',follow['plugin'])!=RPP:
            continue
        def positive(key):
            value=follow[key]
            if not math.isfinite(value) or value<=0:raise ValueError(f'MOTION_LIMITS: {name}.{key} must be positive')
            return value
        cruise=positive('desired_linear_vel')
        reverse=positive('reverse_velocity_limit')
        effective=min(cruise,hi[0])
        follow['motion_forward_limit']=hi[0]
        follow['motion_reverse_limit']=min(reverse,-lo[0])
        follow['motion_angular_limit']=angular
        if positive('rotate_to_heading_angular_vel')>angular or positive('max_angular_accel')>angular_accel:
            report['warnings'].append(f'{name}: rotation speed/acceleration exceeds smoother; generated limits cap it')
        follow['rotate_to_heading_angular_vel']=min(positive('rotate_to_heading_angular_vel'),angular)
        follow['max_angular_accel']=min(positive('max_angular_accel'),angular_accel)
        if 'primary_controller' in follow:follow['primary_controller']='edgeai_navigation::MatchedRPP'
        else:follow['plugin']='edgeai_navigation::MatchedRPP'
        recommended=(effective**2-min(effective,follow['min_approach_linear_velocity'])**2)/(2*-decel[0])
        report['controllers'][name]={'cruise_mps':effective,'reverse_mps':min(effective,reverse,-lo[0]),
                                    'reverse_enabled':follow.get('allow_reversing',False),
                                    'angular_rps':angular,'ideal_approach_distance_m':round(recommended,3)}
        if cruise>hi[0] or reverse>-lo[0]:
            report['warnings'].append(f'{name}: requested speed exceeds smoother; generated limits cap it')
        if follow['approach_velocity_scaling_dist']<recommended:
            report['warnings'].append(f'{name}: approach distance {follow["approach_velocity_scaling_dist"]:.2f} m '
                                      f'< ideal braking estimate {recommended:.2f} m; check overshoot')
    if report['controllers']:
        # Coordinate acceleration increments; hard speed matching is handled above.
        smooth['scale_velocities']=True
        behavior=config['behavior_server']['ros__parameters']
        behavior['max_rotational_vel']=min(behavior['max_rotational_vel'],angular)
        behavior['min_rotational_vel']=min(behavior['min_rotational_vel'],behavior['max_rotational_vel'])
        behavior['rotational_acc_lim']=min(behavior['rotational_acc_lim'],angular_accel)
        if min(hi[0],-lo[0])<.06:
            report['warnings'].append('BT recovery requests 0.06 m/s; check backup/forward speed and time_allowance')
    return report
