"""Real task Action/XML/Python units against non-motion Nav2 and authority peers."""
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
import threading
import time
import unittest
import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rosgraph_msgs.msg import Clock
from ament_index_python.packages import get_package_prefix
from edgeai_mission.action import ExecuteTask
from test_rescue_ros import FakeNavigation

ROOT=Path(__file__).resolve().parents[2]


class TaskUnitsROS(unittest.TestCase):
    def exercise(self,correction=False,retry=False,find_targets=False):
        rclpy.init();fake=FakeNavigation();fake.follow_goal=not correction;fake.alignment_motion=correction
        if correction:
            fake.transform.transform.rotation.z=math.sin(math.pi/4)
            fake.transform.transform.rotation.w=math.cos(math.pi/4)
        if retry:fake.errors=[208,0]
        clock=fake.create_publisher(Clock,'/clock',10)
        fake.create_timer(.02,lambda:clock.publish(Clock(clock=fake.get_clock().now().to_msg())))
        executor=MultiThreadedExecutor(num_threads=4);executor.add_node(fake)
        thread=threading.Thread(target=executor.spin);thread.start();process=None
        try:
            with tempfile.TemporaryDirectory() as temporary:
                d=Path(temporary)
                settings=yaml.safe_load((ROOT/'src/edgeai_navigation/config/normal_rescue.yaml').read_text())
                settings.update(goal_source='map_pose',target_id=1,map_pose=[1.4 if retry else 1.,2.,0.],
                    dwell_s=.2,alignment_enabled=correction,wall_timeout_s=25.,approach_selection=dict(enabled=False))
                if find_targets:
                    settings.update(goal_source='detected_target',search_enabled=True,search_max_positions=3,search_timeout_s=15.)
                    fake.search_target_id=2;fake.search_target_type='cooperative';fake.search_discover_at=1
                    fake.retained_search_targets=[dict(id=1,type='normal',x=1.,y=2.,yaw=0.,spread_m=.01)]
                    fake.search_file=d/'targets.json'
                    fake.search_file.write_text(json.dumps(dict(frame_id='map',targets=fake.retained_search_targets)))
                (d/'rescue-config.yaml').write_text(yaml.safe_dump(settings))
                task='find_targets' if find_targets else 's1'
                c=dict(required_targets=[dict(id=1,type='normal'),dict(id=2,type='cooperative')],directory=str(d),project_root=str(ROOT),wall_timeout_s=30.,
                    unit_worker_script=str(ROOT/'src/edgeai_mission/scripts/task_units.py'),
                    task_trees={task:str(ROOT/'src/edgeai_mission/config/tasks'/f'{task}.xml')})
                config=d/'config.json';config.write_text(json.dumps(c))
                binary=Path(get_package_prefix('edgeai_mission'))/'lib/edgeai_mission/task_executor'
                with (d/'task.log').open('w+') as log:
                    process=subprocess.Popen([str(binary),'--ros-args','-p','config:='+str(config),'-p','task_name:='+task],stdout=log,stderr=log)
                    client=ActionClient(fake,ExecuteTask,'/mission/tasks/'+task+'/execute')
                    self.assertTrue(client.wait_for_server(timeout_sec=8.))
                    sent=client.send_goal_async(ExecuteTask.Goal(task_id='test-s1'))
                    def wait(future):
                        end=time.monotonic()+30.
                        while not future.done() and time.monotonic()<end:time.sleep(.02)
                        self.assertTrue(future.done(),'Action timed out');return future.result()
                    handle=wait(sent);self.assertTrue(handle.accepted)
                    terminal=wait(handle.get_result_async());log.flush();log.seek(0)
                    evidence=log.read()+'\n'+(d/('target-search.log' if find_targets else 'rescue.log')).read_text()
                    evidence+='\n'+(d/'task-unit-events.jsonl').read_text()[-2400:]
                    self.assertEqual(terminal.status,4,evidence+'\n'+terminal.result.reason)
                    result=json.loads(terminal.result.result_json)
                    self.assertTrue(result['success'],evidence)
                    self.assertTrue(result['task_result']['success'])
                    if find_targets:
                        self.assertTrue((d/'target-search-result.json').exists())
                        self.assertFalse((d/'rescue-result.json').exists())
                        self.assertEqual(len(result['task_result']['required_targets']),2)
                        self.assertEqual(len(fake.calls),1)
                        return
                    self.assertTrue((d/'rescue-result.json').exists())
                    self.assertEqual(len(fake.calls),2 if retry else 1)
                    if correction:self.assertTrue(fake.spin_calls)
        finally:
            if process and process.poll() is None:process.terminate();process.wait(timeout=20.)
            executor.shutdown();thread.join(timeout=5.);fake.destroy_node();rclpy.shutdown()

    def test_first_task_searches_missing_cooperative_target_before_rescue(self):self.exercise(find_targets=True)

    def test_real_service_adapter_can_wait_for_authority_and_dwell(self):self.exercise()
    def test_real_tree_corrects_heading_after_navigation(self):self.exercise(correction=True)
    def test_real_tree_retries_only_after_navigation_failure(self):self.exercise(retry=True)


if __name__=='__main__':
    if os.environ.get('ROS_DOMAIN_ID','0') in ('0','86'):raise SystemExit('Use an isolated test domain')
    unittest.main()
