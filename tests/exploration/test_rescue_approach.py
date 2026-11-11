from pathlib import Path
import sys
import math
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'src/edgeai_navigation/scripts'))
from edgeai_tasks.rescue_approach import approaches,approach_goal,failure_reason,should_change_approach


class RescueApproach(unittest.TestCase):
    def test_all_offsets_stay_inside_rescue_radius_with_arrival_margin(self):
        config=dict(normal_offset_m=.35,position_tolerance_m=.10,target_radius_m=.5,
                    approach_selection=dict(extra_distances_m=[.30,.40,.45]))
        nominal=dict(target_id=1,x=1.,y=2.,yaw=0.,frame_id='map')
        for goal in approaches(nominal,config):
            distance=math.dist([goal['x'],goal['y']],[1.35,2.])
            self.assertLessEqual(distance+.1,.5000001)
            bearing=math.atan2(2.-goal['y'],1.35-goal['x'])
            self.assertAlmostEqual(math.cos(goal['yaw']-bearing),1.)
            self.assertEqual(goal['target_id'],1)

    def test_latest_target_rebuild_preserves_selected_side(self):
        config=dict(normal_offset_m=.35)
        a=approach_goal(dict(target_id=1,x=1.,y=2.,yaw=0.),config,math.pi/6,.3)
        b=approach_goal(dict(target_id=1,x=1.2,y=2.1,yaw=0.),config,a['approach_angle_rad'],a['approach_distance_m'])
        self.assertAlmostEqual(b['x']-a['x'],.2);self.assertAlmostEqual(b['y']-a['y'],.1)

    def test_endpoint_errors_do_not_include_start_tf_or_controller_failures(self):
        for code in (204,206,208):self.assertTrue(should_change_approach(code))
        for code in (201,202,203,205,207,102,104,105,106):self.assertFalse(should_change_approach(code))
        self.assertEqual(failure_reason(208),'no_valid_path')


if __name__=='__main__':unittest.main()
