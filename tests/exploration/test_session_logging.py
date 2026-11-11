"""Exercise live log persistence, event evidence and shutdown classification."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
import importlib.util

ROS_SCRIPTS = Path(__file__).resolve().parents[2]/'scripts/ros'
sys.path.insert(0, str(ROS_SCRIPTS))
spec = importlib.util.spec_from_file_location('session_logs', ROS_SCRIPTS/'stream_session_logs.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class SessionLoggingTest(unittest.TestCase):
    def test_truth_verification_is_visible_with_correct_severity(self):
        records=[]
        reducer=module.EventReducer(lambda *args,**kw:records.append(args))
        reducer.consume('rescue','RESCUE_SUCCESS_POSE: S1 真值核验：通过')
        reducer.consume('cooperative','COOPERATIVE_SUCCESS_POSE: {"summary":"C1 真值核验：未通过"}')
        reducer.consume('rescue','RESCUE_SUCCESS_POSE: 真值核验：无法确认')
        self.assertEqual([r[1] for r in records],['OK','ERROR','WARN'])

    def test_language_switch_localizes_operator_text_but_keeps_evidence(self):
        records = []
        reducer = module.EventReducer(lambda *args, **kwargs: records.append((args, kwargs)), selected='en')
        reducer.consume('nav', 'Goal succeeded')
        self.assertEqual(records[0][0][2], 'Navigation goal reached')
        self.assertEqual(records[0][0][3], 'Goal succeeded')
        evidence = 'MISSION_TASK_FEEDBACK: {"task":"c1","unit":"check_rescue"}'
        reducer.consume('mission', evidence)
        self.assertEqual(records[-1][0][2], 'C1 cooperative rescue · Check rescue conditions and dwell time')
        self.assertEqual(records[-1][0][3], evidence)

    def test_language_validation(self):
        from language import language, localize
        self.assertEqual(language('zh-CN'), 'zh')
        self.assertEqual(localize('已到达导航目标', 'en'), 'Navigation goal reached')
        with self.assertRaises(ValueError):
            language('sv')

    def test_mission_stage_failure_is_visible_in_main_event_stream(self):
        records=[]
        reducer=module.EventReducer(lambda *args,**kw:records.append(args))
        reducer.consume('mission','[INFO] [1] [mission_coordinator]: MISSION_STAGE_FINISHED: {"step":"s1","success":false}')
        self.assertEqual(records[0][0],'mission')
        self.assertEqual(records[0][1],'ERROR')
        self.assertEqual(records[0][2], 's1 · 失败')

    def test_cooperative_navigation_events_belong_to_each_robot(self):
        records = []
        reducer = module.EventReducer(lambda *args, **kw: records.append((args, kw)))
        for source, prefix in [('nav', ''), ('cooperative-nav', 'robot2.')]:
            reducer.consume(source, f'[INFO] [1] [{prefix}bt_navigator]: Goal succeeded')
        reducer.consume('cooperative-nav', '[WARN] [1] [amcl]: laser update delayed')
        reducer.consume('cooperative-nav', '[INFO] [1] [robot2.navigation_authority]: '
                        'RECOVERY_STARTED: 执行恢复动作; task=3; owner=MANUAL; phase=ACTIVE')
        self.assertEqual([fields['robot'] for _, fields in records], ['robot1', 'robot2', 'robot1', 'robot2'])
        self.assertEqual(reducer.robots['robot1']['arrivals'], 1)
        self.assertEqual(reducer.robots['robot2']['arrivals'], 1)
        self.assertEqual(reducer.robots['robot2']['recovery_attempts'], 1)

    def test_mission_polling_is_quiet_but_long_phases_have_progress(self):
        records, now = [], [0.]
        reducer = module.EventReducer(lambda *args, **kw: records.append((args, kw)), lambda: now[0])
        feedback = '[INFO] [1] [mission_coordinator]: MISSION_TASK_FEEDBACK: '
        for _ in range(50):
            reducer.consume('mission', feedback+'{"task":"c1","unit":"check_rescue","details":{"valid":true}}')
        self.assertEqual(len(records), 1)
        now[0] = 20
        reducer.consume('cooperative', '[INFO] [1] [joint_rescue]: COOPERATIVE_JOINT_STATUS: '
                        '{"valid":true,"distances_m":[0.41,0.62],"dwell_s":3.0}')
        reducer.flush()
        self.assertIn('机器人1 0.41 m / 机器人2 0.62 m', records[-1][0][2])
        self.assertEqual(records[-1][1]['event'], 'MISSION_PROGRESS')
        reducer.consume('mission', 'MISSION_RESULT: {"success":true}')
        now[0] = 60
        reducer.flush()
        self.assertEqual(len(records), 3)

    def test_suspect_discovery_is_visible_in_live_monitor(self):
        records=[]
        reducer=module.EventReducer(lambda *args,**kw:records.append(args))
        reducer.consume('suspects','[INFO] [1] [suspect_targets]: SUSPECT_PROPOSED: 疑似目标 #0，法线轴=90°')
        reducer.consume('suspects','[INFO] [2] [suspect_targets]: SUSPECT_RETAINED: 疑似目标 #0，多帧保留')
        self.assertEqual(len(records),2)
        self.assertIn('法线轴',records[0][2])
        self.assertIn('SUSPECT_RETAINED',records[1][2])

    def test_success_duplicates_and_recovered_timing_do_not_spam(self):
        records, now = [], [0.0]
        reducer = module.EventReducer(lambda *a, **kw: records.append(a), lambda: now[0])
        for _ in range(3):
            reducer.consume('nav', 'Goal succeeded')
        self.assertEqual(len(records), 1)
        reducer.consume('alignment', 'WAIT/STALE')
        reducer.consume('alignment', 'WAIT/STALE; waiting for map')
        reducer.consume('alignment', 'TIME CHECK OK')
        now[0] = 20
        reducer.flush()
        self.assertEqual(len(records), 3)
        self.assertEqual(records[-1][1], 'OK')
        reducer.consume('nav', 'Failed to get result for follow_path', stopping=True)
        reducer.consume('nav', 'Aborting', stopping=True)
        self.assertEqual(len(records), 4)

    def test_repeated_goal_and_error_counts_survive_reduction(self):
        records, now = [], [0.0]
        reducer = module.EventReducer(lambda *args, **kw: records.append((args, kw)), lambda: now[0])
        reducer.frontier = True
        for _ in range(43):
            reducer.consume('nav', 'Begin navigating from current location (0, 0) to (-0.24, 3.30)')
            reducer.consume('explore', 'Sending frontier goal (mrtsp): dispatch=(-0.24, 3.30), size=28')
        self.assertEqual((reducer.sends, reducer.changes), (43, 1))
        for _ in range(28):
            reducer.consume('nav', '[ERROR] [123] [navigator]: Failed to get result for follow_path in node halt!')
            reducer.consume('rosout', '[ERROR] [123] [navigator]: Failed to get result for follow_path in node halt!')
        self.assertEqual(len(records), 3)
        now[0] = 16
        reducer.flush()
        self.assertEqual(records[-1][1]['count'], 28)
        self.assertEqual(records[-2][1]['count'], 42)
        self.assertEqual(records[-1][0][1], 'ERROR')

    def test_unclassified_error_is_visible(self):
        records = []
        reducer = module.EventReducer(lambda *args, **kw: records.append(args))
        reducer.consume('rviz', '[ERROR] [123] [rviz]: Failed to load panel')
        self.assertEqual(records[0][1], 'ERROR')
        self.assertIn('Failed to load panel', records[0][2])

    def test_blocked_candidate_coordinates_are_not_goal_dispatches(self):
        records = []
        reducer = module.EventReducer(lambda *args, **kw: records.append(args))
        reducer.frontier = True
        reducer.consume('explore', 'Sending frontier goal (mrtsp): dispatch=(1.0, 2.0), size=8')
        for _ in range(23):
            reducer.consume('explore', 'Skipping blocked frontier goal before dispatch: '
                            'cost=90; dispatch=(3.76, 2.31), size=8')
        reducer.consume('explore', 'Sending frontier goal after blocked-goal skip: '
                        'dispatch=(-0.79, 0.76), size=5')
        reducer.consume('explore', 'Sending updated frontier goal (mrtsp): '
                        'dispatch=(-0.79, 0.76), size=5')
        self.assertEqual((reducer.sends, reducer.changes), (3, 2))

    def test_live_stream_and_events(self):
        script = Path(__file__).resolve().parents[2] / 'scripts/ros/stream_session_logs.py'
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            for name in ('sim', 'nav', 'explore', 'rviz', 'rosout', 'watch'):
                (directory / f'{name}.log').touch()
            process = subprocess.Popen([sys.executable, str(script), str(directory)],
                                       stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)

            def wait_for(predicate):
                deadline = time.monotonic() + 5
                while time.monotonic() < deadline:
                    if predicate():
                        return
                    time.sleep(.03)
                self.fail('Log writer did not update within five seconds')

            def content(name):
                path = directory / name
                return path.read_text() if path.exists() else ''

            try:
                (directory / 'explore.log').write_text(
                    "all_frontiers_suppressed_behavior=stay\n"
                    "Completion event enabled: topic='exploration_complete'\n")
                with (directory / 'nav.log').open('a') as output:
                    output.write('Start occupied\nGoal succeeded\n')
                wait_for(lambda: '已到达导航目标' in content('events.log'))
                (directory / 'cooperative-nav.log').write_text(
                    '[INFO] [1] [robot2.bt_navigator]: Goal succeeded\n', encoding='utf-8')
                wait_for(lambda: '[机器人2]' in content('events.log'))
                self.assertNotIn('探索器报告完成', content('events.log'))
                self.assertNotIn('前沿候选均被抑制', content('events.log'))
                self.assertIn('不等于已发生物理碰撞', content('events.log'))
                (directory / 'stopping').touch()
                with (directory / 'nav.log').open('a') as output:
                    output.write('Aborting\n')
                wait_for(lambda: '测试停止阶段' in content('events.log'))
                (directory / 'screenshots').mkdir()
                (directory / 'screenshots/capture.event').write_text('[capture] 截图已保存', encoding='utf-8')
                wait_for(lambda: '截图已保存' in content('console.log'))
                self.assertNotIn('截图已保存', content('events.log'))
                records = [json.loads(line) for line in content('events.jsonl').splitlines()]
                self.assertEqual(records[0]['evidence'], 'Start occupied')
                self.assertIn('Goal succeeded', content('console.log'))
                self.assertNotIn('\x1b', content('events.log'))
                (directory / 'stream.stop').touch()
                _, errors = process.communicate(timeout=5)
                self.assertEqual(process.returncode, 0, errors.decode())
                self.assertTrue((directory/'log-summary.json').exists())
            finally:
                if process.poll() is None:
                    process.kill()
                    process.communicate()


if __name__ == '__main__':
    unittest.main()
