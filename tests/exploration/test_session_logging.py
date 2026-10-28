"""Exercise live log persistence, event evidence and shutdown classification."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest


class SessionLoggingTest(unittest.TestCase):
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
                wait_for(lambda: '截图已保存' in content('events.log'))
                records = [json.loads(line) for line in content('events.jsonl').splitlines()]
                self.assertEqual(records[0]['evidence'], 'Start occupied')
                self.assertIn('Goal succeeded', content('console.log'))
                (directory / 'stream.stop').touch()
                _, errors = process.communicate(timeout=5)
                self.assertEqual(process.returncode, 0, errors.decode())
            finally:
                if process.poll() is None:
                    process.kill()
                    process.communicate()


if __name__ == '__main__':
    unittest.main()
