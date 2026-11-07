"""Normal rescue geometry and continuous dwell checks, independent of ROS."""
import math


class RescueAttemptFailed(RuntimeError):
    """A confirmed terminal failure; ownership loss/cancellation are not retryable."""
    def __init__(self,message,phase,error_code=None):
        super().__init__(message)
        self.phase,self.error_code=phase,error_code


def rescue_position(config,goal,measurement):
    """Return the task distance/limit; a parking point is only an approach aid."""
    if config.get('goal_source')=='detected_target' and 'target_radius_m' in config:
        radius=goal.get('approach_distance_m',config['normal_offset_m'])
        x=goal['x']+radius*math.cos(goal['yaw']);y=goal['y']+radius*math.sin(goal['yaw'])
        return math.hypot(measurement['x']-x,measurement['y']-y),config['target_radius_m']
    return measurement['distance_m'],config['position_tolerance_m']


def validate_config(c):
    retries=c.get('rescue_max_retries',2)
    if isinstance(retries,bool) or not isinstance(retries,int) or not 0<=retries<=10:
        raise ValueError('rescue_max_retries must be an integer in 0..10')
    from frustum_search import settings
    settings(c.get('search_frustum',{}))
    approach=c.get('approach_selection',{})
    if not isinstance(approach,dict) or not isinstance(approach.get('enabled',False),bool):
        raise ValueError('Invalid approach_selection')
    if approach.get('enabled',False):
        count=approach.get('max_candidates',12)
        if isinstance(count,bool) or not isinstance(count,int) or not 1<=count<=30:
            raise ValueError('approach_selection.max_candidates must be in 1..30')
        angles=approach.get('angles_deg',[0,30,-30,60,-60])
        if not angles or any(not math.isfinite(float(a)) or abs(float(a))>70 for a in angles):
            raise ValueError('Approach angles must lie within +/-70 degrees')
        if any(not math.isfinite(float(r)) or float(r)<=0 for r in approach.get('extra_distances_m',[])):
            raise ValueError('Approach distances must be finite and positive')
        timeout=float(approach.get('planning_timeout_s',10.))
        if not math.isfinite(timeout) or not 0<timeout<=30:
            raise ValueError('Approach planning timeout must be in (0,30]')
    if not isinstance(c.get('alignment_enabled',False),bool):
        raise ValueError('alignment_enabled must be boolean')
    for key,default,limit in (('alignment_step_m',.03,.10),('alignment_speed_mps',.03,.06),
                              ('alignment_timeout_s',60.,180.)):
        value=c.get(key,default)
        if not math.isfinite(float(value)) or not 0<value<=limit:
            raise ValueError('Invalid rescue correction parameter: '+key)
    count=c.get('alignment_max_actions',20)
    if isinstance(count,bool) or not isinstance(count,int) or not 1<=count<=100:
        raise ValueError('alignment_max_actions must be an integer in 1..100')
    if not isinstance(c.get('search_enabled', False), bool):
        raise ValueError('search_enabled must be boolean')
    count = c.get('search_max_positions', 6)
    if isinstance(count, bool) or not isinstance(count, int) or not 1 <= count <= 30:
        raise ValueError('search_max_positions must be an integer in 1..30')
    retries = c.get('navigation_max_retries', 2)
    if isinstance(retries, bool) or not isinstance(retries, int) or not 0 <= retries <= 10:
        raise ValueError('navigation_max_retries must be an integer in 0..10')
    for key, default in (('navigation_retry_delay_s', 1.0), ('navigation_retry_wait_s', 5.0),
                         ('retry_tf_max_age_s', .15), ('target_update_distance_m', .10),
                         ('target_update_yaw_rad', .25), ('target_update_interval_s', 1.0),
                         ('search_timeout_s', 120.0), ('search_navigation_timeout_s', 20.0),
                         ('search_clearance_m', .40), ('search_spacing_m', 1.0),
                         ('search_spin_timeout_s', 30.0)):
        value = c.get(key, default)
        if not math.isfinite(float(value)) or value <= 0:
            raise ValueError('Expected positive finite rescue parameter: '+key)
    if c.get('navigation_retry_wait_s', 5.0) <= c.get('navigation_retry_delay_s', 1.0):
        raise ValueError('Retry readiness timeout must exceed retry delay')
    if c.get('retry_tf_max_age_s', .15) > c['max_data_age_s']:
        raise ValueError('Retry TF age limit must not exceed the measurement age limit')
    for key in ('normal_offset_m', 'position_tolerance_m', 'yaw_tolerance_rad', 'dwell_s',
                'stopped_linear_mps', 'stopped_angular_rps', 'max_data_age_s',
                'navigation_timeout_s', 'wall_timeout_s'):
        if not math.isfinite(float(c[key])) or c[key] <= 0:
            raise ValueError('Expected positive finite rescue parameter: '+key)
    if c['position_tolerance_m'] > .18:
        raise ValueError('Rescue tolerance exceeds development rescue zone radius .18 m')
    if 'target_radius_m' in c:
        radius = float(c['target_radius_m'])
        if (not math.isfinite(radius) or radius <= 0 or
                c['normal_offset_m'] + c['position_tolerance_m'] > radius):
            raise ValueError('Rescue goal offset and tolerance exceed the target radius')


def select_goal(config, registry):
    if config['goal_source'] == 'map_pose':
        x, y, yaw = map(float, config['map_pose'])
        target_id = config['target_id']
    elif config['goal_source'] == 'detected_target':
        rows = [r for r in registry.get('targets', [])
                if r.get('type') == 'normal' and r.get('yaw') is not None
                and (not config.get('require_confirmation', False) or r.get('confirmed'))
                and (config['target_id'] < 0 or r['id'] == config['target_id'])]
        if registry.get('frame_id') != 'map' or not rows:
            raise ValueError('No eligible localized normal target with map orientation')
        row = min(rows, key=lambda r: (-r.get('observations', 1), r['spread_m'], r['id']))
        target_id = row['id']
        yaw = row['yaw']
        x = row['x'] + config['normal_offset_m']*math.cos(yaw)
        y = row['y'] + config['normal_offset_m']*math.sin(yaw)
        yaw = math.atan2(math.sin(yaw+math.pi), math.cos(yaw+math.pi))
    else:
        raise ValueError('Unknown goal_source')
    if not all(math.isfinite(v) for v in (x, y, yaw)):
        raise ValueError('Nonfinite rescue goal')
    return dict(target_id=target_id, x=x, y=y, yaw=yaw, frame_id='map')


class Dwell:
    def __init__(self, config):
        self.config = config
        self.since = self.previous = None

    def update(self, now, distance, yaw_error, linear, angular, fresh=True,position_limit=None):
        c = self.config
        valid = (fresh and distance <= (c['position_tolerance_m'] if position_limit is None else position_limit)
                 and abs(yaw_error) <= c['yaw_tolerance_rad']
                 and abs(linear) <= c['stopped_linear_mps']
                 and abs(angular) <= c['stopped_angular_rps'])
        if self.previous is not None and (now < self.previous or now-self.previous > c['max_data_age_s']):
            self.since = None
        self.previous = now
        if not valid:
            self.since = None
            return False
        if self.since is None:
            self.since = now
        return now-self.since >= c['dwell_s']
