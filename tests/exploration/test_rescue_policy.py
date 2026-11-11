from pathlib import Path
import sys
import unittest
import yaml
import tempfile
import json
sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'src/edgeai_navigation/scripts'))
from edgeai_tasks.rescue_policy import Dwell, select_goal, validate_config, rescue_position
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'src/edgeai_mission/scripts'))
from export_handover import export


class RescueTests(unittest.TestCase):
    def test_rescue_circle_accepts_navigation_point_error_above_tolerance(self):
        config=dict(goal_source='detected_target',target_radius_m=.5,normal_offset_m=.35,
                    position_tolerance_m=.1,yaw_tolerance_rad=.25,dwell_s=.2,
                    stopped_linear_mps=.02,stopped_angular_rps=.05,max_data_age_s=.5)
        goal=dict(x=1.,y=2.,yaw=0.)
        measurement=dict(x=.893,y=2.,yaw=0.,yaw_error_rad=0.,distance_m=.107)
        distance,limit=rescue_position(config,goal,measurement)
        self.assertAlmostEqual(distance,.457);self.assertEqual(limit,.5)
        dwell=Dwell(config)
        self.assertFalse(dwell.update(0.,distance,0.,0.,0.,position_limit=limit))
        self.assertTrue(dwell.update(.25,distance,0.,0.,0.,position_limit=limit))
        measurement.update(x=.8,distance_m=.2)
        self.assertGreater(rescue_position(config,goal,measurement)[0],limit)

    def setUp(self):
        self.c = yaml.safe_load((Path(__file__).resolve().parents[2]/'src/edgeai_navigation/config/normal_rescue.yaml').read_text())

    def test_localized_normal_without_confirmation(self):
        rows = [dict(id=10, type='cooperative', confirmed=True, yaw=0., x=1., y=2., spread_m=.01),
                dict(id=1, type='normal', confirmed=False, yaw=0., x=1., y=2., spread_m=.01)]
        goal = select_goal(self.c, dict(frame_id='map', targets=rows))
        self.assertEqual(goal['target_id'], 1)
        self.c['require_confirmation'] = True
        with self.assertRaises(ValueError):
            select_goal(self.c, dict(frame_id='map', targets=rows))
        rows[1]['confirmed'] = True
        goal = select_goal(self.c, dict(frame_id='map', targets=rows))
        self.assertAlmostEqual(goal['x'], 1.7)
        self.assertEqual(goal['y'], 2.)
        self.assertAlmostEqual(abs(goal['yaw']), 3.141592653589793)
        self.assertEqual(goal['target_id'], 1)

    def test_search_selects_only_requested_cooperative_id(self):
        self.c.update(target_id=2,target_type='cooperative')
        rows=[dict(id=1,type='normal',yaw=0.,x=1.,y=2.,spread_m=.01),
              dict(id=2,type='cooperative',yaw=0.,x=3.,y=2.,spread_m=.01)]
        registry=dict(frame_id='map',targets=rows)
        self.assertEqual(select_goal(self.c,registry)['target_id'],2)
        rows[1]['yaw']=None
        with self.assertRaisesRegex(ValueError,'cooperative'):select_goal(self.c,registry)

    def test_continuous_dwell_resets_on_motion_or_missing_data(self):
        d = Dwell(self.c)
        for i in range(30):
            self.assertFalse(d.update(i*.1, .02, 0., 0., 0.))
        self.assertFalse(d.update(3., .02, 0., .1, 0.))
        for i in range(31, 62):
            success = d.update(i*.1, .02, 0., 0., 0.)
        self.assertTrue(success)
        self.assertFalse(d.update(6.2, .02, 0., 0., 0., fresh=False))
        self.assertFalse(d.update(10., .02, 0., 0., 0.))
        self.assertFalse(d.update(1., .02, 0., 0., 0.))

    def test_export_freezes_task1_and_filters_candidates(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            registry = dict(frame_id='map', stamp_ns=42, targets=[
                dict(id=1, type='normal', confirmed=True, x=1., y=2., yaw=.3),
                dict(id=2, type='normal', confirmed=False, x=2., y=2., yaw=.3)])
            (directory/'targets.json').write_text(json.dumps(registry))
            (directory/'map.yaml').write_text('resolution: 0.05\norigin: [0, 0, 0]\nimage: map.pgm\n')
            export(directory, directory/'map.yaml')
            self.assertEqual(len((directory/'target-list.csv').read_text().splitlines()), 3)
            self.assertEqual(json.loads((directory/'task1-output.json').read_text())['resolution'], .05)
            (directory/'targets.json').write_text('{}')
            self.assertEqual(json.loads((directory/'handover-targets.json').read_text())['stamp_ns'], 42)

    def test_invalid_config(self):
        validate_config(self.c)
        self.c['dwell_s'] = -1.
        with self.assertRaises(ValueError):
            validate_config(self.c)

    def test_invalid_retry_settings(self):
        for settings in (dict(navigation_max_retries=-1), dict(navigation_max_retries=1.5),
                         dict(navigation_max_retries=True), dict(navigation_retry_delay_s=0.),
                         dict(navigation_retry_wait_s=.5), dict(retry_tf_max_age_s=.6)):
            with self.subTest(settings=settings), self.assertRaises(ValueError):
                validate_config(dict(self.c, **settings))

    def test_invalid_search_settings(self):
        for settings in (dict(search_enabled=1), dict(search_max_positions=0),
                         dict(search_max_positions=True), dict(search_timeout_s=0.),
                         dict(search_spacing_m=-1.), dict(search_spin_timeout_s=float('nan'))):
            with self.subTest(settings=settings), self.assertRaises(ValueError):
                validate_config(dict(self.c, **settings))

    def test_explicit_goal(self):
        self.c.update(goal_source='map_pose', map_pose=[1., 2., .5])
        self.assertEqual(select_goal(self.c, {})['yaw'], .5)

    def test_midterm_s1_and_continuous_ten_seconds(self):
        config = Path(__file__).resolve().parents[2]/'src/edgeai_navigation/config/midterm_single_rescue.yaml'
        c = yaml.safe_load(config.read_text())
        validate_config(c)
        registry = dict(frame_id='map', targets=[
            dict(id=1, type='normal', yaw=-1.5707963267948966, x=1., y=4.1, spread_m=.01),
            dict(id=2, type='cooperative', yaw=0., x=4.1, y=1., spread_m=.01)])
        goal = select_goal(c, registry)
        self.assertEqual(goal['target_id'], 1)
        self.assertAlmostEqual(goal['y'], 3.75)
        dwell = Dwell(c)
        for i in range(110):
            self.assertFalse(dwell.update(i*.1, .05, 0., 0., 0.))
        self.assertTrue(dwell.update(11., .05, 0., 0., 0.))
        self.assertFalse(dwell.update(11.1, .11, 0., 0., 0.))
        self.assertFalse(dwell.update(11.2, .05, 0., 0., 0.))
        c['normal_offset_m'] = .45
        with self.assertRaises(ValueError):
            validate_config(c)


if __name__ == '__main__':
    unittest.main()
