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
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Empty, Int32, String
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
 if a.task=='explore' and op=='check_complete' and scenario=='explore_incomplete':data['complete']=False
 if a.task=='s1' and op=='check_rescue' and scenario=='localization_runtime_died':data['complete']=False
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
 if scenario in ('preplan_occupied','preplan_occupied_exhausted') and a.task=='c1':
  if op in ('select_parking','select_parking_locked'):
   success=counts.get('recover_preplan_start',0)>0 and scenario=='preplan_occupied'
   data['category']='failure' if success else 'start_occupied'
  if op=='recover_preplan_start':
   success=scenario=='preplan_occupied';data['category']='start_occupied'
 if scenario=='preplan_occupied_exhausted' and op=='yield_peer':success=False;data['category']='start_occupied'
 if scenario in ('preplan_peer','preplan_peer_unavailable','preplan_peer_fatal') and a.task=='c1':
  if op=='select_parking_locked':success=False
  if op=='select_parking':success=counts.get('yield_peer',0)>0 or scenario=='preplan_peer_unavailable'
  if op=='yield_peer':
   success=scenario=='preplan_peer'
   if scenario=='preplan_peer_fatal':data['category']='fatal'
 if op=='record_success' and success:okay=True
 response.success=success
 if a.task==c.get('fail_step'):data['category']='fatal'
 response.message=json.dumps(dict(success=success,**data))
 return response
import xml.etree.ElementTree as ET
ops=sorted({unit.get('operation') for unit in ET.parse(c['task_trees'][a.task]).iter('TaskUnit')})
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


LOCALIZATION='''import argparse,json,time,signal
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--config');a=p.parse_args()
c=json.loads(Path(a.config).read_text());d=Path(c['directory'])
with (d/'calls.jsonl').open('a') as f:f.write(json.dumps('localization')+'\\n')
scenario=c.get('scenario');count=d/'localization-attempts'
n=int(count.read_text())+1 if count.exists() else 1;count.write_text(str(n))
state=d/'localization-state.json'
def publish(**data):
 tmp=state.with_suffix('.tmp');tmp.write_text(json.dumps(data));tmp.replace(state)
if scenario=='localization_start_failed' or scenario=='localization_start_retry' and n==1:
 publish(ready=False,phase='FAILED',reason='test AMCL startup failure');raise SystemExit(1)
publish(ready=False,phase='STARTING')
if scenario=='localization_state_unreadable':state.chmod(0)
time.sleep(.6 if scenario=='localization_state_unreadable' else .4)
if scenario=='localization_state_unreadable':state.chmod(0o600)
publish(ready=True,phase='READY')
signal.signal(signal.SIGTERM,lambda *_:exit(0))
while True:
 if scenario=='localization_runtime_died' and (d/'unit-calls.jsonl').exists() and 's1' in (d/'unit-calls.jsonl').read_text():raise SystemExit(1)
 time.sleep(.02)
'''


class MissionROS(unittest.TestCase):
    def exercise(self, *, cooperative=False, rescue=True, joint=False, localization=False, ready_delay=0., fail=None, block=None, timeout=5., complete=True, dead_process=False, omit_result=None, false_zero_exit=False, owned_navigation=False, scenario=None, kill_task_server=False, known_cells=100, grow_map=False, publish_map=True, tree='integrated.xml'):
        rclpy.init();peer=rclpy.create_node('mission_test_peer')
        publisher=peer.create_publisher(Empty,'/exploration_complete',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        maps=peer.create_publisher(OccupancyGrid,'/map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        monitor=peer.create_publisher(Int32,'/explore/available_frontiers',1)
        pub1=peer.create_publisher(Odometry,'/robot1/odom',10);pub2=peer.create_publisher(Odometry,'/robot2/odom',10)
        broadcaster=StaticTransformBroadcaster(peer)
        tf=TransformStamped();tf.header.frame_id='map';tf.child_frame_id='robot1/odom';tf.transform.rotation.w=1.
        broadcaster.sendTransform(tf)
        # The stop endpoint confirms that mission cancellation reaches robot authority.
        stops=[]
        def stop(_,response):stops.append(True);response.success=True;return response
        peer.create_service(Trigger,'/robot1/navigation_authority/stop',stop)
        if cooperative or joint:peer.create_service(Trigger,'/robot2/navigation_authority/stop',stop)
        process=None
        try:
            with tempfile.TemporaryDirectory() as temporary:
                d=Path(temporary);worker=d/'worker.py';worker.write_text(WORKER)
                units=d/'units.py';units.write_text(UNITS)
                c=dict(directory=str(d),project_root=str(ROOT),cooperative=cooperative,rescue_enabled=rescue,
                    joint_mapping=joint,initial_pose_seed=not joint,
                    backend='frontier',initial_relative_pose=[0.,-.6,0.],
                    required_targets=[dict(id=1,type='normal')]+([dict(id=2,type='cooperative')] if cooperative else []),
                    worker_script=str(worker),tree=str(ROOT/'src/edgeai_mission/config'/tree),
                    unit_worker_script=str(units),task_trees={t:str(ROOT/'src/edgeai_mission/config/tasks'/f'{t}.xml') for t in ('explore','find_targets','s1','c1')},
                    task_wall_timeout_s=dict(explore=timeout),
                    exploration_timeout_s=timeout,wall_timeout_s=25.,fail_step=fail,
                    block_step=block or ('supplement_mapping' if scenario in ('frontier_during_supplement','monitor_missing') else None),
                    mapping=dict(min_known_area_m2=18.,supplement_attempts=3,reposition_timeout_s=60.),
                    omit_result=omit_result,false_zero_exit=false_zero_exit,owned_navigation=owned_navigation,scenario=scenario)
                if tree=='midterm_part2.xml':
                    c.update(demonstration_part='part2',initial_pose_seed=False,external_navigation=True)
                if scenario=='missing_mapping_reference':c['initial_pose_seed']=False
                config=d/'config.json';config.write_text(json.dumps(c))
                if localization:
                    localization_worker=d/'localization.py';localization_worker.write_text(LOCALIZATION)
                    c.update(post_mapping_localization=True,localization_worker_script=str(localization_worker),
                             localization=dict(startup_attempts=2,startup_timeout_s=5.))
                    config.write_text(json.dumps(c))
                if not ready_delay:(d/'ready').touch()
                identity=Path('/proc/self/stat').read_text().split()[21]
                (d/'mission-processes.json').write_text(json.dumps(dict(test_peer=dict(
                    pid=99999999 if dead_process else os.getpid(),start_ticks=identity))))
                log=(d/'coordinator.log').open('w+')
                executable=Path(get_package_prefix('edgeai_mission'))/'lib/edgeai_mission/mission_coordinator'
                process=subprocess.Popen([str(executable),'--ros-args','-p','use_sim_time:=false','-p','config:='+str(config)],stdout=log,stderr=log)
                began=time.monotonic();deadline=began+35.;cancel_sent=None
                while process.poll() is None and time.monotonic()<deadline:
                    rclpy.spin_once(peer,timeout_sec=.02)
                    odom=Odometry();odom.header.stamp=peer.get_clock().now().to_msg();odom.pose.pose.orientation.w=1.
                    pub1.publish(odom);pub2.publish(odom)
                    events=(d/'mission-events.jsonl').read_text() if (d/'mission-events.jsonl').exists() else ''
                    called=(d/'calls.jsonl').read_text() if (d/'calls.jsonl').exists() else ''
                    if ready_delay and not (d/'ready').exists():
                        self.assertNotIn('explore',called,'Task dispatched before startup readiness')
                        if 'WAITING_MAPPING_READY' in events and time.monotonic()-began>=ready_delay:
                            (d/'ready').touch()
                    if scenario!='monitor_missing':
                        monitor.publish(Int32(data=1 if scenario=='frontier_during_supplement' and 'supplement_mapping' in called else 0))
                    if publish_map:
                        count=100 if grow_map and 'step":"supplement_mapping"' in events else known_cells
                        grid=OccupancyGrid();grid.header.frame_id='map';grid.info.width=grid.info.height=10
                        grid.info.resolution=.5;grid.info.origin.orientation.w=1.
                        grid.data=([0,100,50]*34)[:count]+[-1]*(100-count)
                        maps.publish(grid)
                    if complete and 'EXPLORING' in events and (not cooperative or not c.get('initial_pose_seed',True) or (d/'mission-reference.json').exists()):publisher.publish(Empty())
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
                if cooperative and c.get('initial_pose_seed',True):self.assertTrue((d/'mission-reference.json').exists())
                if not c.get('initial_pose_seed',True):self.assertFalse((d/'mission-reference.json').exists())
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


    def test_localization_precedes_s1_and_is_reused_by_c1(self):
        result,calls,events=self.exercise(cooperative=True,localization=True)
        self.assertTrue(result['success'],result)
        self.assertEqual(calls,['explore','quiesce','find_targets','save_map','localization','s1','c1'])
        ready=next(i for i,e in enumerate(events) if e['phase']=='LOCALIZATION_READY')
        s1=next(i for i,e in enumerate(events) if e['phase']=='TASK_REQUESTED' and e['task']=='s1')
        self.assertLess(ready,s1)
        c1_units=[e.get('unit') for e in events if e.get('task')=='c1' and e['phase']=='UNIT_CALL']
        self.assertIn('wait_navigation_ready',c1_units)
        self.assertNotIn('localize',c1_units)

    def test_transient_localization_state_read_failure_does_not_abort_handover(self):
        result,calls,events=self.exercise(cooperative=True,localization=True,
                                        scenario='localization_state_unreadable')
        self.assertTrue(result['success'],result)
        self.assertEqual(calls.count('localization'),1)
        self.assertIn('s1',calls);self.assertIn('c1',calls)
        self.assertTrue(any(e['phase']=='LOCALIZATION_STATE_RETRY' for e in events))

    def test_localization_start_failure_blocks_rescue(self):
        result,calls,_=self.exercise(localization=True,scenario='localization_start_failed')
        self.assertFalse(result['success']);self.assertIn('AMCL startup failure',result['reason'])
        self.assertEqual(calls.count('localization'),2)
        self.assertNotIn('s1',calls)


    def test_joint_exploration_is_task_before_search_and_rescue(self):
        result,calls,events=self.exercise(joint=True,ready_delay=.5)
        self.assertTrue(result['success'],result)
        self.assertEqual(calls,['explore','quiesce','find_targets','save_map','s1'])
        self.assertTrue(any(e['phase']=='MAPPING_READY' for e in events))
        self.assertTrue(any(e['phase']=='FRONTIERS_EXHAUSTED' and e.get('source')=='exploration_task' for e in events))


    def test_joint_task_completion_cannot_be_replaced_by_aggregate_event(self):
        result,calls,_=self.exercise(joint=True,rescue=False,timeout=2.,scenario='explore_incomplete')
        self.assertFalse(result['success']);self.assertIn('budget exhausted',result['reason'])
        self.assertEqual(calls,['explore'])


    def test_joint_exploration_cancellation_reaps_worker(self):
        result,calls,_=self.exercise(joint=True,rescue=False,block='explore',timeout=15.)
        self.assertFalse(result['success']);self.assertIn('canceled',result['reason'].lower())
        self.assertEqual(calls,['explore'])

    def test_joint_exploration_server_is_supervised_without_rescue(self):
        result,calls,_=self.exercise(joint=True,rescue=False,block='explore',kill_task_server=True,timeout=15.)
        self.assertFalse(result['success']);self.assertEqual(result['reason'],'Exploration task server exited')
        self.assertEqual(calls,['explore'])


    def test_c1_unknown_yield_terminal_cannot_dispatch(self):
        result,_,events=self.exercise(cooperative=True,scenario='preplan_peer_fatal')
        self.assertFalse(result['success'])
        units=[e.get('unit') for e in events if e.get('task')=='c1' and e['phase']=='UNIT_CALL']
        self.assertIn('yield_peer',units);self.assertNotIn('dispatch_navigation',units)


    def test_required_process_exit_does_not_dispatch_stages(self):
        result,calls,_=self.exercise(dead_process=True)
        self.assertFalse(result['success']);self.assertIn('Required process exited',result['reason'])
        self.assertEqual(calls,[])


if __name__=='__main__':
    if os.environ.get('ROS_DOMAIN_ID','0') in ('0','86'):raise SystemExit('Use an isolated test domain')
    unittest.main()
