import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts/ros'))
from controller_probe import sample_flags,summarize,controller_view


class ControllerProbe(unittest.TestCase):
    def test_idle_is_not_a_motion_failure_and_stale_data_not_compared(self):
        row={'values':{'cmd_vel':[0.,0.],'odom':[0.,0.]},'wall_age_s':{'cmd_vel':0.,'odom':0.}}
        self.assertEqual(sample_flags(row),[])
        row['values']['cmd_vel']=[.2,0.]
        self.assertIn('command_without_measured_motion',sample_flags(row))
        row['wall_age_s']['odom']=2.
        self.assertEqual(sample_flags(row),[])

    def test_speed_stage_difference_excludes_unreceived_data(self):
        row={'values':{'cmd_vel_nav':[.6,1.5],'cmd_vel_smoothed':[.3,.9]},
             'wall_age_s':{'cmd_vel_nav':0.,'cmd_vel_smoothed':0.}}
        self.assertEqual(sample_flags(row),['smoother_difference'])
        row.update(wall_elapsed_s=0.,flags=sample_flags(row))
        result=summarize([row],{'cmd_vel_nav':2},1.)
        self.assertEqual(result['channels']['cmd_vel_nav']['received_wall_hz'],2.)
        self.assertEqual(result['channels']['odom']['fresh_samples'],0)

    def test_feedback_from_old_goal_is_not_attributed_to_current_controller(self):
        row={'values':{'authority':{'arrival_policy':'rescue','following_path':True},
                       'follow_path_status':{'goal_id':'new','name':'EXECUTING'},
                       'follow_path_feedback':{'goal_id':'old','distance_to_goal_m':9.}},
             'wall_age_s':{'authority':0.,'follow_path_status':0.,'follow_path_feedback':0.}}
        cv=controller_view(row,{'FollowPathRescue':{'primary_controller':'MatchedRPP'}})
        self.assertEqual(cv['controller_id'],'FollowPathRescue')
        self.assertEqual(cv['primary'],'MatchedRPP')
        self.assertFalse(cv['feedback'])
        row['values']['follow_path_feedback']['goal_id']='new'
        self.assertEqual(controller_view(row,{})['feedback']['distance_to_goal_m'],9.)


if __name__=='__main__':unittest.main()
