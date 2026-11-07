from pathlib import Path
import sys,unittest
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/edgeai_navigation/scripts'))
from navigation_config import load_navigation
from goal_policy import configure_controllers
from motion_limits import configure_motion_limits


def prepared(controller='rpp'):
    config=load_navigation(ROOT/'src/edgeai_navigation/config',controller)
    control=config['controller_server']['ros__parameters']
    for name in ('heading_goal_checker','rescue_goal_checker'):
        control[name]=dict(control['general_goal_checker'])
    control['FollowPath']['allow_reversing']=True
    config['planner_server']['ros__parameters']['GridBased']['allow_reverse_expansion']=True
    configure_controllers(config)
    return config


class MotionLimits(unittest.TestCase):
    def test_all_rpp_policies_use_pre_prediction_limits(self):
        config=prepared()
        # Exercise a fixed envelope independently of user-adjustable YAML tuning.
        config['velocity_smoother']['ros__parameters'].update(
            max_velocity=[.6,0.,.9],min_velocity=[-.25,0.,-.9])
        for name in ('FollowPath','FollowPathHeading','FollowPathRescue'):
            config['controller_server']['ros__parameters'][name]['reverse_velocity_limit']=.25
        report=configure_motion_limits(config)
        self.assertEqual(len(report['controllers']),3)
        control=config['controller_server']['ros__parameters']
        for name in ('FollowPath','FollowPathHeading','FollowPathRescue'):
            follow=control[name]
            self.assertEqual(follow['primary_controller'],'edgeai_navigation::MatchedRPP')
            self.assertEqual(follow['motion_angular_limit'],.9)
            self.assertEqual(follow['motion_reverse_limit'],.25)
        self.assertTrue(control['FollowPath']['allow_reversing'])
        self.assertFalse(control['FollowPathHeading']['allow_reversing'])
        self.assertFalse(control['FollowPathRescue']['allow_reversing'])
        self.assertTrue(config['velocity_smoother']['ros__parameters']['scale_velocities'])

    def test_changing_actuator_envelope_derives_every_rpp_instance_and_warns(self):
        config=prepared();smooth=config['velocity_smoother']['ros__parameters']
        smooth.update(max_velocity=[.2,0.,.4],min_velocity=[-.1,0.,-.5],
                      max_accel=[.3,0.,.3],max_decel=[-.3,0.,-.4])
        report=configure_motion_limits(config)
        for item in report['controllers'].values():
            self.assertEqual(item['cruise_mps'],.2)
            self.assertEqual(item['reverse_mps'],.1)
            self.assertEqual(item['angular_rps'],.4)
        follow=config['controller_server']['ros__parameters']['FollowPathRescue']
        self.assertEqual(follow['max_angular_accel'],.3)
        self.assertEqual(follow['rotate_to_heading_angular_vel'],.4)
        self.assertTrue(any('exceeds smoother' in w for w in report['warnings']))
        self.assertEqual(config['behavior_server']['ros__parameters']['max_rotational_vel'],.4)

    def test_invalid_limits_fail_before_launch(self):
        config=prepared()
        config['velocity_smoother']['ros__parameters']['max_velocity'][2]=0.
        with self.assertRaisesRegex(ValueError,'MOTION_LIMITS'):
            configure_motion_limits(config)

    def test_other_controllers_are_not_replaced(self):
        for profile in ('dwb','mppi'):
            config=prepared(profile)
            before=config['controller_server']['ros__parameters']['FollowPath']['plugin']
            report=configure_motion_limits(config)
            self.assertFalse(report['controllers'])
            self.assertEqual(config['controller_server']['ros__parameters']['FollowPath']['plugin'],before)


if __name__=='__main__':unittest.main()
