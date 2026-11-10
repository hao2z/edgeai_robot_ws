"""Real coordinator/BT and ROS events; isolated domain, fake non-motion workers."""
import json
import os
import signal
from pathlib import Path
import subprocess
import tempfile
import time
import unittest
import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import StaticTransformBroadcaster

ROOT=Path(__file__).resolve().parents[2]
WORKER='''import argparse,json,time,signal,subprocess
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--step');p.add_argument('--config');a=p.parse_args()
c=json.loads(Path(a.config).read_text());d=Path(c['directory'])
with (d/'calls.jsonl').open('a') as f:f.write(json.dumps(a.step)+'\\n')
signal.signal(signal.SIGTERM,lambda *_:(_ for _ in ()).throw(KeyboardInterrupt()))
okay=a.step!=c.get('fail_step')
try:
 if a.step=='s1' and c.get('owned_navigation'):
  owned=subprocess.Popen(['python3','-c','import time;time.sleep(60)'],start_new_session=True)
  (d/'mission-owned-navigation.json').write_text(json.dumps(dict(pid=owned.pid,start_ticks=Path('/proc/'+str(owned.pid)+'/stat').read_text().split()[21])))
 if a.step==c.get('block_step'):
  while True:time.sleep(.02)
except KeyboardInterrupt:okay=False
if a.step!=c.get('omit_result'):
 (d/('stage-'+a.step+'-result.json')).write_text(json.dumps(dict(success=okay,reason='fake failure' if not okay else '')))
raise SystemExit(0 if okay or c.get('false_zero_exit') else 1)
'''

# The real C++ task Action server and XML run against these non-motion units.
UNITS='''import argparse,json,time,signal,subprocess
from pathlib import Path
import rclpy
from std_srvs.srv import Trigger
p=argparse.ArgumentParser();p.add_argument('--task');p.add_argument('--config');a=p.parse_args()
c=json.loads(Path(a.config).read_text());d=Path(c['directory']);okay=False;first=True;counts={}
rclpy.init();node=rclpy.create_node(a.task+'_fake_units')
signal.signal(signal.SIGTERM,lambda *_:(_ for _ in ()).throw(KeyboardInterrupt()))
def handle(op,response):
 global first,okay
 with (d/'unit-calls.jsonl').open('a') as f:f.write(json.dumps(dict(task=a.task,unit=op))+'\\n')
 if first:
  first=False
  with (d/'calls.jsonl').open('a') as f:f.write(json.dumps(a.task)+'\\n')
  if a.task=='s1' and c.get('owned_navigation'):
   owned=subprocess.Popen(['python3','-c','import time;time.sleep(60)'],start_new_session=True)
   (d/'mission-owned-navigation.json').write_text(json.dumps(dict(pid=owned.pid,start_ticks=Path('/proc/'+str(owned.pid)+'/stat').read_text().split()[21])))
  if a.task==c.get('block_step'):
   while True:time.sleep(.02)
 success=a.task!=c.get('fail_step')
 counts[op]=counts.get(op,0)+1
 data=dict(complete=True,needs_correction=False,category='failure',reason='fake failure')
 scenario=c.get('scenario')
 if a.task=='find_targets' and scenario in ('missing_c1','search_exhausted'):
  if op=='select_target':success=False
  if op=='search_target':success=scenario!='search_exhausted'
 if scenario in ('retry','exhausted','updated') and op=='poll_navigation':
  success=counts[op]>({'retry':1,'exhausted':100,'updated':2}[scenario])
  if scenario=='updated':data['category']='target_updated'
 if scenario in ('retry','exhausted','updated') and op=='inside_rescue_region':
  success=False
  if scenario=='updated':data['category']='target_updated'
 if scenario=='correction' and op=='check_rescue' and counts[op]==1:data.update(complete=False,needs_correction=True)
 if scenario in ('activation_retry','activation_exhausted') and a.task=='c1' and op=='wait_navigation_ready':
  success=counts[op]>({'activation_retry':1,'activation_exhausted':100}[scenario])
 if scenario=='localization_retry' and a.task=='c1' and op=='localize':success=counts[op]>1
 if op=='record_success' and success:okay=True
 response.success=success
 if a.task==c.get('fail_step'):data['category']='fatal'
 response.message=json.dumps(dict(success=success,**data))
 return response
ops=['next_target','check_targets','acquire','select_target','search_target','select_approach','inside_rescue_region','correct','handover','wait_navigation_ready','localize','prepare_target','select_parking','select_parking_locked','cancel_navigation','cancel_all_navigation','dispatch_navigation','poll_navigation','check_rescue','record_success']
services=[node.create_service(Trigger,'/mission/tasks/'+a.task+'/units/'+op,lambda req,res,op=op:handle(op,res)) for op in ops]
try:
 while rclpy.ok():rclpy.spin_once(node,timeout_sec=.02)
except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
finally:
 (d/('task-'+a.task+'-worker-result.json')).write_text(json.dumps(dict(success=okay)))
 node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
raise SystemExit(0 if okay or c.get('false_zero_exit') else 1)
'''


class MissionROS(unittest.TestCase):
    def exercise(self, *, cooperative=False, rescue=True, fail=None, block=None, timeout=5., complete=True, dead_process=False, omit_result=None, false_zero_exit=False, owned_navigation=False, scenario=None, kill_task_server=False):
        rclpy.init();peer=rclpy.create_node('mission_test_peer')
        publisher=peer.create_publisher(Empty,'/exploration_complete',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        pub1=peer.create_publisher(Odometry,'/odom',10);pub2=peer.create_publisher(Odometry,'/robot2/odom',10)
        broadcaster=StaticTransformBroadcaster(peer)
        tf=TransformStamped();tf.header.frame_id='map';tf.child_frame_id='odom';tf.transform.rotation.w=1.
        broadcaster.sendTransform(tf)
        # The stop endpoint confirms that mission cancellation reaches robot authority.
        stops=[]
        def stop(_,response):stops.append(True);response.success=True;return response
        peer.create_service(Trigger,'/navigation_authority/stop',stop)
        if cooperative:peer.create_service(Trigger,'/robot2/navigation_authority/stop',stop)
        process=None
        try:
            with tempfile.TemporaryDirectory() as temporary:
                d=Path(temporary);worker=d/'worker.py';worker.write_text(WORKER)
                units=d/'units.py';units.write_text(UNITS)
                c=dict(directory=str(d),project_root=str(ROOT),cooperative=cooperative,rescue_enabled=rescue,
                    observation=False,backend='frontier',initial_relative_pose=[0.,-.6,0.],
                    worker_script=str(worker),tree=str(ROOT/'src/edgeai_mission/config/integrated.xml'),
                    unit_worker_script=str(units),task_trees={t:str(ROOT/'src/edgeai_mission/config/tasks'/f'{t}.xml') for t in ('find_targets','s1','c1')},
                    exploration_timeout_s=timeout,wall_timeout_s=25.,fail_step=fail,block_step=block,
                    omit_result=omit_result,false_zero_exit=false_zero_exit,owned_navigation=owned_navigation,scenario=scenario)
                config=d/'config.json';config.write_text(json.dumps(c));(d/'ready').touch()
                identity=Path('/proc/self/stat').read_text().split()[21]
                (d/'mission-processes.json').write_text(json.dumps(dict(test_peer=dict(
                    pid=99999999 if dead_process else os.getpid(),start_ticks=identity))))
                log=(d/'coordinator.log').open('w+')
                executable=Path(get_package_prefix('edgeai_mission'))/'lib/edgeai_mission/mission_coordinator'
                process=subprocess.Popen([str(executable),'--ros-args','-p','use_sim_time:=false','-p','config:='+str(config)],stdout=log,stderr=log)
                deadline=time.monotonic()+35.;cancel_sent=None
                while process.poll() is None and time.monotonic()<deadline:
                    rclpy.spin_once(peer,timeout_sec=.02)
                    odom=Odometry();odom.header.stamp=peer.get_clock().now().to_msg();odom.pose.pose.orientation.w=1.
                    pub1.publish(odom);pub2.publish(odom)
                    events=(d/'mission-events.jsonl').read_text() if (d/'mission-events.jsonl').exists() else ''
                    if complete and 'EXPLORING' in events and (not cooperative or (d/'mission-reference.json').exists()):publisher.publish(Empty())
                    if block and (d/'calls.jsonl').exists() and block in (d/'calls.jsonl').read_text() and cancel_sent is None:
                        if kill_task_server:
                            record=json.loads((d/f'task-{block}-worker-process.json').read_text())
                            server_pid=int(Path('/proc',str(record['pid']),'stat').read_text().split()[3])
                            os.kill(server_pid,signal.SIGKILL);cancel_sent=True
                            continue
                        client=peer.create_client(Trigger,'/mission/stop')
                        if client.service_is_ready():cancel_sent=client.call_async(Trigger.Request())
                if process.poll() is None:
                    process.terminate();process.wait(timeout=20.)
                    log.seek(0);self.fail('Coordinator timeout: '+log.read())
                log.seek(0);output=log.read();log.close()
                self.assertTrue((d/'mission-result.json').exists(),output)
                result=json.loads((d/'mission-result.json').read_text())
                calls=[json.loads(line) for line in (d/'calls.jsonl').read_text().splitlines()] if (d/'calls.jsonl').exists() else []
                events=[json.loads(line) for line in (d/'mission-events.jsonl').read_text().splitlines()]
                if (d/'unit-calls.jsonl').exists():
                    events.extend(dict(phase='UNIT_CALL',**json.loads(line)) for line in (d/'unit-calls.jsonl').read_text().splitlines())
                self.assertTrue(stops,'No authority stop at mission end')
                if block:self.assertIsNotNone(cancel_sent)
                if cooperative:self.assertTrue((d/'mission-reference.json').exists())
                if owned_navigation:
                    pid=json.loads((d/'mission-owned-navigation.json').read_text())['pid']
                    path=Path('/proc')/str(pid)/'stat'
                    alive=path.exists() and path.read_text().split()[2]!='Z'
                    if alive:os.killpg(pid,9)  # Clean even when the assertion exposes a regression.
                    self.assertFalse(alive,'Owned navigation launch survived mission cancellation')
                return result,calls,events
        finally:
            if process and process.poll() is None:process.kill();process.wait()
            peer.destroy_node();rclpy.shutdown()

    def test_fixed_single_robot_order(self):
        result,calls,_=self.exercise()
        self.assertTrue(result['success']);self.assertEqual(calls,['quiesce','find_targets','save_map','s1'])

    def test_cooperative_follows_s1_without_task_scoring(self):
        result,calls,events=self.exercise(cooperative=True)
        self.assertTrue(result['success']);self.assertEqual(calls,['quiesce','find_targets','save_map','s1','c1'])
        self.assertEqual(events[0]['scheduler'],'fixed_rules')

    def test_missing_c1_is_searched_in_first_task_before_s1(self):
        result,calls,events=self.exercise(cooperative=True,scenario='missing_c1')
        self.assertTrue(result['success'])
        self.assertEqual(calls,['quiesce','find_targets','save_map','s1','c1'])
        units=[e for e in events if e['phase']=='UNIT_CALL']
        self.assertTrue(any(e['task']=='find_targets' and e['unit']=='search_target' for e in units))
        self.assertFalse(any(e['task']=='c1' and e['unit']=='search_target' for e in units))

    def test_exhausted_first_task_search_cannot_start_rescue_or_handover(self):
        result,calls,events=self.exercise(cooperative=True,scenario='search_exhausted')
        self.assertFalse(result['success'])
        self.assertEqual(calls,['quiesce','find_targets'])
        self.assertFalse(any(e.get('unit')=='handover' for e in events))

    def test_s1_failure_never_dispatches_c1(self):
        result,calls,_=self.exercise(cooperative=True,fail='s1')
        self.assertFalse(result['success']);self.assertEqual(calls,['quiesce','find_targets','save_map','s1'])

    def test_c1_retries_activation_without_repeating_map_handover(self):
        result,_,events=self.exercise(cooperative=True,scenario='activation_retry')
        self.assertTrue(result['success'])
        units=[e.get('unit') for e in events if e.get('task')=='c1' and e['phase']=='UNIT_CALL']
        self.assertEqual(units.count('handover'),1)
        self.assertEqual(units.count('wait_navigation_ready'),2)
        self.assertEqual(units.count('localize'),1)

    def test_c1_retries_localization_through_readiness_gate(self):
        result,_,events=self.exercise(cooperative=True,scenario='localization_retry')
        self.assertTrue(result['success'])
        units=[e.get('unit') for e in events if e.get('task')=='c1' and e['phase']=='UNIT_CALL']
        self.assertEqual(units.count('handover'),1)
        self.assertEqual(units.count('localize'),2)
        self.assertEqual(units.count('wait_navigation_ready'),2)

    def test_c1_activation_retries_are_bounded_and_do_not_dispatch_navigation(self):
        result,_,events=self.exercise(cooperative=True,scenario='activation_exhausted')
        self.assertFalse(result['success'])
        units=[e.get('unit') for e in events if e.get('task')=='c1' and e['phase']=='UNIT_CALL']
        self.assertEqual(units.count('handover'),1)
        self.assertEqual(units.count('wait_navigation_ready'),3)
        self.assertNotIn('localize',units);self.assertNotIn('dispatch_navigation',units)

    def test_disabled_rescue_still_saves_map(self):
        result,calls,_=self.exercise(rescue=False)
        self.assertTrue(result['success']);self.assertEqual(calls,['quiesce','save_map'])

    def test_cancel_active_worker_confirms_termination(self):
        result,calls,_=self.exercise(block='s1')
        self.assertFalse(result['success']);self.assertIn('canceled',result['reason'])
        self.assertEqual(calls,['quiesce','find_targets','save_map','s1'])

    def test_timeout_is_not_exploration_completion(self):
        result,calls,_=self.exercise(timeout=.3,complete=False)
        self.assertFalse(result['success']);self.assertIn('time limit',result['reason']);self.assertEqual(calls,[])

    def test_required_process_exit_does_not_dispatch_stages(self):
        result,calls,_=self.exercise(dead_process=True)
        self.assertFalse(result['success']);self.assertIn('Required process exited',result['reason'])
        self.assertEqual(calls,[])

    def test_zero_exit_without_result_cannot_start_rescue(self):
        result,calls,_=self.exercise(omit_result='save_map')
        self.assertFalse(result['success']);self.assertIn('without confirmed result',result['reason'])
        self.assertEqual(calls,['quiesce','find_targets','save_map'])

    def test_failure_payload_cannot_be_overridden_by_zero_exit(self):
        result,calls,_=self.exercise(fail='s1',false_zero_exit=True)
        self.assertFalse(result['success']);self.assertEqual(calls,['quiesce','find_targets','save_map','s1'])

    def test_cancel_cleans_owned_navigation_launch_group(self):
        result,_,events=self.exercise(block='s1',owned_navigation=True)
        self.assertFalse(result['success'])
        self.assertTrue(any(e['phase']=='CLEANUP_OWNED_NAVIGATION' for e in events))

    def test_task_tree_retries_failed_navigation(self):
        result,_,events=self.exercise(scenario='retry')
        self.assertTrue(result['success'])
        self.assertEqual(sum(e.get('unit')=='select_approach' and e['phase']=='UNIT_CALL' for e in events),2)

    def test_task_tree_bounds_navigation_attempts(self):
        result,_,events=self.exercise(scenario='exhausted')
        self.assertFalse(result['success'])
        self.assertEqual(sum(e.get('unit')=='select_approach' and e['phase']=='UNIT_CALL' for e in events),9)
        self.assertFalse(any(e.get('unit')=='record_success' and e.get('task')=='s1' for e in events))

    def test_target_updates_rebuild_without_spending_failure_budget(self):
        result,_,events=self.exercise(scenario='updated')
        self.assertTrue(result['success'])
        self.assertEqual(sum(e.get('unit')=='select_approach' and e['phase']=='UNIT_CALL' for e in events),3)

    def test_task_tree_corrects_before_reporting_success(self):
        result,_,events=self.exercise(scenario='correction')
        self.assertTrue(result['success'])
        units=[e.get('unit') for e in events if e['phase']=='TASK_FEEDBACK' and e.get('task')=='s1']
        self.assertLess(units.index('correct'),units.index('record_success'))

    def test_task_server_death_cleans_orphan_backend(self):
        result,_,events=self.exercise(block='s1',kill_task_server=True)
        self.assertFalse(result['success']);self.assertIn('task server exited',result['reason'])
        self.assertTrue(any(e['phase']=='CLEANUP_ORPHAN_TASK_BACKEND' for e in events))


if __name__=='__main__':
    if os.environ.get('ROS_DOMAIN_ID','0') in ('0','86'):raise SystemExit('Use an isolated test domain')
    unittest.main()
