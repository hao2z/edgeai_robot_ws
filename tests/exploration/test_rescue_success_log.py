"""Independent sampled world-pose judgement and human-readable errors."""
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'src/edgeai_mission/scripts'))
from rescue_success_log import success_evidence


class TruthEvidence(unittest.TestCase):
    def evaluate(self,task='s1',alter=None):
        with tempfile.TemporaryDirectory() as tmp:
            directory=Path(tmp)
            (directory/'truth-targets.json').write_text(json.dumps(dict(targets_world=[dict(id=1,pose=[0.,0.,0.])])) )
            for robot in range(1,3):
                rows=[dict(status='unavailable',truth_valid=True,sample_time=float(t),
                           robot_world=[.4,0.,math.pi],targets_world=[dict(id=1,pose=[0.,0.,0.])],
                           target_estimates=[dict(id=1,error_m=.03)]) for t in range(12)]
                if alter:alter(rows,robot)
                (directory/('truth.jsonl' if robot==1 else 'robot2-truth.jsonl')).write_text(
                    '\n'.join(json.dumps(r) for r in rows)+'\n{"partial":')
            count=1 if task=='s1' else 2
            return success_evidence(directory,task,1,[0.,0.,0.],[[1.,2.,0.]]*count,
                                    [.45]*count,[.1]*count,.5,.25,10.,10.5)

    def test_world_history_passes_without_map_tf_and_reports_errors(self):
        evidence,messages=self.evaluate('c1')
        self.assertEqual(evidence['truth_verification']['status'],'passed')
        self.assertEqual(len(evidence['robots']),2)
        truth=evidence['robots'][0]['truth']
        self.assertAlmostEqual(truth['distance_error_m'],.05)
        self.assertAlmostEqual(truth['yaw_estimation_error_rad'],.1)
        self.assertIn('目标定位误差 0.030 m','\n'.join(messages))
        self.assertIn('C1 真值核验：通过',messages)

    def test_one_robot_outside_circle_fails_joint_verification(self):
        def alter(rows,robot):
            if robot==2:
                for row in rows:row['robot_world'][0]=.6
        evidence,_=self.evaluate('c1',alter)
        self.assertEqual(evidence['truth_verification']['status'],'failed')

    def test_bad_heading_during_dwell_is_not_hidden_by_final_pose(self):
        def alter(rows,robot):rows[4]['robot_world'][2]=0.
        evidence,_=self.evaluate(alter=alter)
        self.assertEqual(evidence['truth_verification']['status'],'failed')

    def test_motion_within_circle_fails_stopped_check(self):
        def alter(rows,robot):rows[4]['robot_world'][0]=.45
        evidence,_=self.evaluate(alter=alter)
        self.assertEqual(evidence['truth_verification']['status'],'failed')

    def test_missing_history_does_not_claim_dwell_pass(self):
        def alter(rows,robot):del rows[:5]
        evidence,_=self.evaluate(alter=alter)
        self.assertEqual(evidence['truth_verification']['status'],'unavailable')

    def test_stale_truth_does_not_claim_pass(self):
        def alter(rows,robot):del rows[7:]
        evidence,_=self.evaluate(alter=alter)
        self.assertEqual(evidence['truth_verification']['status'],'unavailable')

    def test_legacy_truth_rows_use_saved_target_manifest(self):
        def alter(rows,robot):
            for row in rows:del row['targets_world']
        evidence,_=self.evaluate(alter=alter)
        self.assertEqual(evidence['truth_verification']['status'],'passed')

    def test_old_success_replay_reads_back_past_large_tail(self):
        def alter(rows,robot):
            for t in range(11,2000):
                rows.append(dict(rows[0],sample_time=float(t)))
        evidence,_=self.evaluate(alter=alter)
        self.assertEqual(evidence['truth_verification']['status'],'passed')

    def test_rotation_before_task_dwell_does_not_fail_truth(self):
        def alter(rows,robot):
            # Task checks [0.5,10.5]; earlier turn must not enter that window.
            rows.insert(0,dict(rows[0],sample_time=-.5,robot_world=[.4,0.,2.8]))
        evidence,_=self.evaluate(alter=alter)
        truth=evidence['robots'][0]['truth']
        self.assertEqual(truth['window_start_s'],.5)
        self.assertEqual(truth['window_end_s'],10.5)
        self.assertEqual(truth['status'],'passed')
        self.assertEqual(truth['max_angular_rps'],0.)


if __name__=='__main__':unittest.main()
