"""Explicit regression selections; never starts Gazebo or stops user sessions."""
import argparse
import os
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
PROFILES = {
    'navigation': ['test_goal_policy', 'test_motion_limits', 'test_yield_routes', 'test_navigation_authority'],
    'mission': ['test_exploration_assignment', 'test_frontier_updates', 'test_fleet_search', 'test_mission_config'],
    'ros-smoke': [
        'test_peer_yield_ros.PeerYieldROS.test_generated_tree_endpoints_are_available',
        'test_peer_yield_ros.PeerYieldROS.test_active_stopped_requester_and_idle_peer',
        'test_task_units_ros.TaskUnitsROS.test_real_service_adapter_can_wait_for_authority_and_dwell',
        'test_fleet_authority_ros.FleetAuthority.test_robot2_goal_reaches_only_robot2_backend',
    ],
    'ros-tasks': ['test_peer_yield_ros', 'test_fleet_search_ros', 'test_task_units_ros',
                  'test_fleet_rescue_ros', 'test_fleet_coordination_ros', 'test_mission_ros'],
}
PROFILES['quick'] = PROFILES['navigation'] + PROFILES['mission']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', choices=PROFILES, default='quick')
    parser.add_argument('--list', action='store_true', help='Show selection without importing tests')
    parser.add_argument('tests', nargs='*', help='Optional unittest module/class/method names')
    args = parser.parse_args()
    selected = args.tests or PROFILES[args.profile]
    if args.list:
        print('\n'.join(selected))
        return 0
    # Reserve a separate domain even for lightweight profiles, so selecting an
    # explicit ROS case never joins the normal mission domain by accident.
    domain = int(os.environ.setdefault('ROS_DOMAIN_ID', '197'))
    if domain in (0, 86, 93) or not 1 <= domain <= 232:
        parser.error('Use an isolated ROS_DOMAIN_ID (default: 197; mission domains excluded)')
    sys.path[:0] = [str(ROOT/'tests/exploration')]
    label = 'explicit selection' if args.tests else args.profile
    print(f'Test profile: {label}; ROS_DOMAIN_ID={domain}', flush=True)
    suite = unittest.defaultTestLoader.loadTestsFromNames(selected)
    return 0 if unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful() else 1


if __name__ == '__main__':
    raise SystemExit(main())
