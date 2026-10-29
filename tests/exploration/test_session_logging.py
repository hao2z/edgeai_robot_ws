"""Exercise live log persistence, event evidence and shutdown classification."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
import importlib.util

spec = importlib.util.spec_from_file_location('session_logs', Path(__file__).resolve().parents[2]/'scripts/ros/stream_session_logs.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class SessionLoggingTest(unittest.TestCase):
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
