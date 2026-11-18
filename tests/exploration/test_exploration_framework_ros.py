"""Real generic task server and worker shell with an injected source-only algorithm."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import time
import unittest

import rclpy
from rclpy.action import ActionClient
from ament_index_python.packages import get_package_prefix
from edgeai_mission.action import ExecuteTask
from rosgraph_msgs.msg import Clock

ROOT=Path(__file__).resolve().parents[2]
ALGORITHM='''import json
from pathlib import Path
from rclpy.node import Node
class ExplorationUnits:
 def __init__(self,c):
  self.c=c;self.directory=Path(c['directory']);self.success=False;self.ticks=0
  self.node=Node('fleet_exploration_coordinator')
  self.node.create_timer(.02,self.tick)
 def tick(self):self.ticks+=1
 def invoke(self,op):
  with (self.directory/'algorithm-ops.jsonl').open('a') as f:f.write(json.dumps(op)+'\\n')
  if op=='acquire' and self.ticks==0:raise RuntimeError('Data callbacks not pumped')
  if op=='check_complete':return dict(complete=not self.c.get('incomplete',False))
  if op=='record_success':
   self.success=True
   (self.directory/'exploration-result.json').write_text(json.dumps(dict(success=True,ticks=self.ticks)))
  return {}
 def close(self):
  (self.directory/'algorithm-closed').touch();self.node.destroy_node()
  if self.c.get('cleanup_fail'):raise RuntimeError('Stop was not confirmed')
'''


class ExplorationFrameworkROS(unittest.TestCase):
    def exercise(self,cleanup_fail=False,cancel=False):
        rclpy.init();peer=rclpy.create_node('explore_framework_test')
        clock=peer.create_publisher(Clock,'/clock',10);process=None
        try:
            with tempfile.TemporaryDirectory() as temporary:
                d=Path(temporary);package=d/'src/edgeai_mission/edgeai_tasks';package.mkdir(parents=True)
                (package/'__init__.py').touch();(package/'exploration.py').write_text(ALGORITHM)
                (d/'exploration-result.json').write_text(json.dumps(dict(success=True,stale=True)))
                c=dict(directory=str(d),project_root=str(d),wall_timeout_s=20.,cleanup_fail=cleanup_fail,incomplete=cancel,
                    task_wall_timeout_s=dict(explore=15.),unit_worker_script=str(ROOT/'src/edgeai_mission/scripts/task_units.py'),
                    task_trees=dict(explore=str(ROOT/'src/edgeai_mission/config/tasks/explore.xml')))
                config=d/'mission.json';config.write_text(json.dumps(c))
                binary=Path(get_package_prefix('edgeai_mission'))/'lib/edgeai_mission/task_executor'
                with (d/'executor.log').open('w+') as log:
                    process=subprocess.Popen([str(binary),'--ros-args','-p','config:='+str(config),'-p','task_name:=explore'],stdout=log,stderr=log)
                    client=ActionClient(peer,ExecuteTask,'/mission/tasks/explore/execute')
                    self.assertTrue(client.wait_for_server(timeout_sec=8.))
                    def wait(predicate):
                        deadline=time.monotonic()+20.
                        while not predicate() and time.monotonic()<deadline:
                            clock.publish(Clock(clock=peer.get_clock().now().to_msg()))
                            rclpy.spin_once(peer,timeout_sec=.02)
                        log.flush();log.seek(0)
                        self.assertTrue(predicate(),log.read())
                    sent=client.send_goal_async(ExecuteTask.Goal(task_id='framework-explore'))
                    wait(sent.done);handle=sent.result();self.assertTrue(handle.accepted)
                    terminal=handle.get_result_async()
                    if cancel:
                        wait(lambda:(d/'algorithm-ops.jsonl').exists() and 'check_complete' in (d/'algorithm-ops.jsonl').read_text())
                        canceled=handle.cancel_goal_async();wait(canceled.done)
                    wait(terminal.done)
                    result=terminal.result()
                    self.assertEqual(result.status,5 if cancel else 6 if cleanup_fail else 4,result.result.reason)
                    self.assertTrue((d/'algorithm-closed').exists())
                    self.assertFalse((d/'task-explore-worker-process.json').exists())
                    worker=json.loads((d/'task-explore-worker-result.json').read_text())
                    self.assertEqual(worker['success'],not(cancel or cleanup_fail))
                    self.assertTrue((d/'fleet-exploration.log').exists())
                    if not(cancel or cleanup_fail):
                        self.assertGreater(worker['task_result']['ticks'],0)
                        self.assertNotIn('stale',worker['task_result'])
                        ops=[json.loads(line) for line in (d/'algorithm-ops.jsonl').read_text().splitlines()]
                        self.assertEqual(ops,['acquire','poll','refresh_frontier_goals','poll_yield','diagnose_blockage',
                            'select_yield','dispatch_yield','dispatch','check_complete',
                            'cancel_all_navigation','record_success'])
                    if cleanup_fail:self.assertIn('Stop was not confirmed',result.result.reason)
                    if cancel:self.assertFalse((d/'exploration-result.json').exists())
        finally:
            if process and process.poll() is None:process.terminate();process.wait(timeout=20.)
            peer.destroy_node();rclpy.shutdown()

    def test_source_module_contract_and_cleanup_result(self):self.exercise()

    def test_unconfirmed_cleanup_cannot_report_task_success(self):self.exercise(cleanup_fail=True)

    def test_canceling_task_reaps_exploration_backend(self):self.exercise(cancel=True)


if __name__=='__main__':
    if os.environ.get('ROS_DOMAIN_ID','0') in ('0','86','92'):raise SystemExit('Use an isolated test domain')
    unittest.main()
