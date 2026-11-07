"""Compose shared navigation, controller tuning and explicit legacy profile deltas."""
from copy import deepcopy
from pathlib import Path
import yaml


def merge(base, patch):
    """Apply nested overrides; null removes a key (e.g. polygon vs radius)."""
    result = deepcopy(base)
    for key, value in patch.items():
        if value is None:
            result.pop(key, None)
        elif isinstance(value, dict):
            result[key] = merge(result.get(key, {}), value)
        else:
            result[key] = deepcopy(value)
    return result


def load_navigation(resource_dir, controller='dwb', profile='rectangle', depth_obstacles=None):
    resource_dir = Path(resource_dir)
    if controller not in ('dwb', 'mppi', 'rpp'):
        raise ValueError('Unknown controller: '+controller)
    if profile == 'conservative':
        profile = 'safe'
    if profile not in ('rectangle', 'baseline', 'safe'):
        raise ValueError('Unknown navigation profile: '+profile)
    config = yaml.safe_load((resource_dir/'nav2_exploration_rectangle.yaml').read_text())
    profiles = yaml.safe_load((resource_dir/'navigation_profiles.yaml').read_text())
    selected = profiles.get(profile, {})
    config = merge(config, selected.get('navigation', {}))
    config = merge(config, selected.get('controller_navigation', {}).get(controller, {}))
    tuning = yaml.safe_load((resource_dir/f'{controller}_controller.yaml').read_text())
    follow = merge(tuning['FollowPath'], tuning.get('profiles', {}).get(profile, {}))
    config['controller_server']['ros__parameters']['FollowPath'] = follow
    depth = yaml.safe_load((resource_dir/'depth_obstacles.yaml').read_text())
    enabled = depth['enabled'] if depth_obstacles is None else depth_obstacles
    if not isinstance(enabled, bool):
        raise ValueError('depth_obstacles must be a boolean')
    local = config['local_costmap']['local_costmap']['ros__parameters']
    # Insert before inflation so new lethal cells receive the same soft margin.
    if enabled:
        index = local['plugins'].index('inflation_layer')
        local['plugins'].insert(index, 'depth_layer')
        local['depth_layer'] = deepcopy(depth['layer'])
        config['depth_obstacle_filter'] = {'ros__parameters': dict(depth['filter'], use_sim_time=True)}
    return config
