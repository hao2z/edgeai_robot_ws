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
    def test_routine_checks_are_quiet_for_every_task_but_failures_remain(self):
        records, now = [], [0.]
        reducer = module.EventReducer(lambda *a, **kw: records.append((a, kw)), lambda: now[0])
        for task in ('explore', 'find_targets', 's1', 'c1'):
            reducer.consume('mission', 'MISSION_TASK_REQUESTED: '+json.dumps(dict(task=task)))
            for unit in ('select_target', 'diagnose_blockage', 'select_yield', 'dispatch_yield', 'poll_yield'):
                for tick in range(3):
                    now[0] += 16
                    reducer.consume('mission', 'MISSION_TASK_FEEDBACK: '+json.dumps(
                        dict(task=task, unit=unit, details=dict(success=True))))
            self.assertEqual(records[-1][1]['event'], 'MISSION_TASK_REQUESTED')
        reducer.consume('mission', 'MISSION_TASK_FEEDBACK: '+json.dumps(dict(
            task='s1', unit='dispatch_yield', details=dict(success=False, reason='Rejected'))))
        self.assertEqual(records[-1][0][1], 'ERROR')
        self.assertEqual(len(records), 5)

    def test_top_level_localization_has_translated_ready_and_failure_messages(self):
        records=[]
        reducer=module.EventReducer(lambda *a,**kw:records.append(a),selected='en')
        reducer.consume('mission','MISSION_LOCALIZATION_READY: {"phase":"LOCALIZATION_READY"}')
        reducer.consume('mission','MISSION_LOCALIZATION_FAILED: {"phase":"LOCALIZATION_FAILED","reason":"AMCL failed"}')
        self.assertEqual(records[0][1],'OK')
        self.assertIn('Localization ready',records[0][2])
        self.assertEqual(records[1][1],'ERROR')
        self.assertIn('Localization handover failed',records[1][2])

    def test_fleet_loop_feedback_preserves_task_status_without_false_action_labels(self):
        records=[]
        reducer=module.EventReducer(lambda *a,**kw:records.append(a),selected='en')
        reducer.consume('mission','MISSION_TASK_REQUESTED: {"task":"explore"}')
        previous = reducer.mission
        reducer.consume('mission','MISSION_TASK_FEEDBACK: {"task":"explore","unit":"poll_yield"}')
        self.assertEqual(len(records),1)
        self.assertEqual(reducer.mission, previous)

    def test_fleet_assignments_preserve_robot_and_failure_reason(self):
        records=[]
        reducer=module.EventReducer(lambda *a,**kw:records.append((a,kw)),selected='en')
        reducer.consume('fleet-exploration',json.dumps(dict(event='FLEET_EXPLORATION_ASSIGNED',
            robot='robot2',goal=dict(x=1.234,y=2.345))))
        reducer.consume('fleet-exploration',json.dumps(dict(event='FLEET_EXPLORATION_NAV_FAILED',
            robot='robot2',reason='Navigation timeout')))
        self.assertIn('Navigate to assigned frontier',records[0][0][2])
        self.assertEqual(records[0][1]['robot'],'robot2')
        self.assertEqual(records[1][0][1],'WARN')
        self.assertIn('Navigation timeout',records[1][0][2])

    def test_frontier_replacements_are_readable_info_events(self):
        for selected, expected in (('en','cancel old navigation'),('zh','取消旧导航')):
            records=[]
            reducer=module.EventReducer(lambda *a,**kw:records.append((a,kw)),selected=selected)
            reducer.consume('fleet-exploration',json.dumps(dict(
                event='FLEET_EXPLORATION_FRONTIER_UPDATE_REQUESTED',
                robot='robot1',goal=dict(x=2.,y=3.))))
            self.assertEqual(records[0][0][1],'INFO')
            self.assertIn(expected,records[0][0][2])
            self.assertEqual(records[0][1]['robot'],'robot1')
            reducer.consume('mission','MISSION_TASK_FEEDBACK: {"task":"explore","unit":"refresh_frontier_goals"}')
            self.assertEqual(len(records),1)

    def test_s1_allocation_and_empty_assignment_fallback_are_readable(self):
        records=[]
        reducer=module.EventReducer(lambda *a,**kw:records.append((a,kw)),selected='en')
        reducer.consume('rescue',json.dumps(dict(event='FLEET_RESCUE_SELECTED',
            robot='robot2',goal=dict(x=1.,y=2.))))
        reducer.consume('rescue',json.dumps(dict(event='FLEET_RESCUE_REASSIGN',
            robot=None,reason='No eligible parking')))
        self.assertIn('Assign single rescue parking',records[0][0][2])
        self.assertEqual(records[0][1]['robot'],'robot2')
        self.assertIn('Rescue fallback',records[1][0][2])

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

    def test_joint_explorer_and_authority_count_secondary_without_double_navigation_sends(self):
        records=[]
        reducer=module.EventReducer(lambda *a,**kw:records.append((a,kw)))
        prefix='[INFO] [1] [robot2.'
        reducer.consume('joint-mapping',prefix+'frontier_explorer]: Sending frontier goal (mrtsp): dispatch=(1.0, 2.0)')
        reducer.consume('joint-mapping',prefix+'bt_navigator]: Begin navigating from current location (0, 0) to (1.0, 2.0)')
        reducer.consume('joint-mapping',prefix+'navigation_authority]: RECOVERY_STARTED: 恢复; task=3')
        reducer.consume('joint-mapping',prefix+'bt_navigator]: Goal succeeded')
        self.assertEqual(reducer.robots['robot2']['goal_sends'],1)
        self.assertEqual(reducer.robots['robot2']['recovery_attempts'],1)
        self.assertEqual(reducer.robots['robot2']['arrivals'],1)

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
