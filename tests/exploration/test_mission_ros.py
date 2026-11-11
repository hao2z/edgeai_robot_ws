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
 if scenario in ('preplan_occupied','preplan_occupied_exhausted') and a.task=='c1':
  if op in ('select_parking','select_parking_locked'):
   success=counts.get('recover_preplan_start',0)>0 and scenario=='preplan_occupied'
   data['category']='failure' if success else 'start_occupied'
  if op=='recover_preplan_start':
   success=scenario=='preplan_occupied';data['category']='start_occupied'
 if op=='record_success' and success:okay=True
 response.success=success
 if a.task==c.get('fail_step'):data['category']='fatal'
 response.message=json.dumps(dict(success=success,**data))
 return response
ops=['next_target','check_targets','acquire','select_target','search_target','select_approach','inside_rescue_region','correct','handover','wait_navigation_ready','localize','prepare_target','select_parking','select_parking_locked','recover_preplan_start','cancel_navigation','cancel_all_navigation','dispatch_navigation','poll_navigation','check_rescue','record_success']
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
    def exercise(self, *, cooperative=False, rescue=True, fail=None, block=None, timeout=5., complete=True, dead_process=False, omit_result=None, false_zero_exit=False, owned_navigation=False, scenario=None, kill_task_server=False, known_cells=100, grow_map=False, publish_map=True, tree='integrated.xml'):
        rclpy.init();peer=rclpy.create_node('mission_test_peer')
        publisher=peer.create_publisher(Empty,'/exploration_complete',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        maps=peer.create_publisher(OccupancyGrid,'/map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        monitor=peer.create_publisher(Int32,'/explore/available_frontiers',1)
        observation_test=scenario=='observation_target_gate'
        if observation_test:
            targets=peer.create_publisher(String,'/targets/estimates',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            estimates=[dict(id=1,type='normal',x=1.,y=1.,yaw=0.),dict(id=2,type='cooperative',x=2.,y=2.,yaw=None)]
            observation_started=None
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
                    observation=observation_test,backend='frontier',initial_relative_pose=[0.,-.6,0.],
                    required_targets=[dict(id=1,type='normal')]+([dict(id=2,type='cooperative')] if cooperative else []),
                    worker_script=str(worker),tree=str(ROOT/'src/edgeai_mission/config'/tree),
                    unit_worker_script=str(units),task_trees={t:str(ROOT/'src/edgeai_mission/config/tasks'/f'{t}.xml') for t in ('find_targets','s1','c1')},
                    exploration_timeout_s=timeout,wall_timeout_s=25.,fail_step=fail,
                    block_step=block or ('supplement_mapping' if scenario in ('frontier_during_supplement','monitor_missing') else None),
                    mapping=dict(min_known_area_m2=18.,supplement_attempts=3,reposition_timeout_s=60.),
                    omit_result=omit_result,false_zero_exit=false_zero_exit,owned_navigation=owned_navigation,scenario=scenario)
                if tree=='midterm_part2.xml':
                    c.update(demonstration_part='part2',initial_pose_seed=False,external_navigation=True)
                if scenario=='missing_mapping_reference':c['initial_pose_seed']=False
                config=d/'config.json';config.write_text(json.dumps(c));(d/'ready').touch()
                if observation_test:
                    (d/'observation-scan-runtime.yaml').write_text('observation_scan:\n  ros__parameters:\n    use_sim_time: false\n')
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
                    called=(d/'calls.jsonl').read_text() if (d/'calls.jsonl').exists() else ''
                    if scenario!='monitor_missing':
                        monitor.publish(Int32(data=1 if scenario=='frontier_during_supplement' and 'supplement_mapping' in called else 0))
                    if publish_map:
                        count=100 if grow_map and 'step":"supplement_mapping"' in events else known_cells
                        grid=OccupancyGrid();grid.header.frame_id='map';grid.info.width=grid.info.height=10
                        grid.info.resolution=.5;grid.info.origin.orientation.w=1.
                        grid.data=([0,100,50]*34)[:count]+[-1]*(100-count)
                        maps.publish(grid)
                    can_complete=True
                    if observation_test:
                        obs_log=d/'observation.log'
                        text=obs_log.read_text() if obs_log.exists() else ''
                        if observation_started is None and 'OBSERVATION_READY' in text:observation_started=time.monotonic()
                        if observation_started is not None:
                            if time.monotonic()-observation_started<.4:
                                self.assertNotIn('OBSERVATION_SUPPRESSED_TARGETS_LOCATED',events)
                            else:estimates[1]['yaw']=0.
                            targets.publish(String(data=json.dumps(dict(frame_id='map',targets=estimates))))
                        can_complete='OBSERVATION_SUPPRESSED:' in text
                    if complete and can_complete and 'EXPLORING' in events and (not cooperative or not c.get('initial_pose_seed',True) or (d/'mission-reference.json').exists()):publisher.publish(Empty())
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

    def test_fixed_single_robot_order(self):
        result,calls,_=self.exercise()
        self.assertTrue(result['success']);self.assertEqual(calls,['quiesce','find_targets','save_map','s1'])

    def test_midterm_part1_stops_after_mapping_and_target_search(self):
        result,calls,events=self.exercise(cooperative=True,tree='midterm_part1.xml',scenario='missing_c1')
        self.assertTrue(result['success'],result)
        self.assertEqual(calls,['quiesce','find_targets','save_map'])
        self.assertTrue(any(e.get('task')=='find_targets' and e.get('unit')=='search_target' for e in events))
        self.assertTrue(any(e['phase']=='EXPLORING' and e.get('wall_monotonic_s',0)>0 for e in events))

    def test_midterm_part1_exhausted_search_fails_before_map_export(self):
        result,calls,_=self.exercise(cooperative=True,tree='midterm_part1.xml',scenario='search_exhausted')
        self.assertFalse(result['success'])
        self.assertEqual(calls,['quiesce','find_targets'])

    def test_midterm_part2_runs_rescues_without_mapping_or_search(self):
        result,calls,_=self.exercise(cooperative=True,tree='midterm_part2.xml',complete=False,publish_map=False)
        self.assertTrue(result['success'],result)
        self.assertEqual(calls,['s1','c1'])
        self.assertGreater(result['completed_wall_monotonic_s'],0)

    def test_integrated_c1_requires_mapping_reference(self):
        result,calls,_=self.exercise(cooperative=True,scenario='missing_mapping_reference')
        self.assertFalse(result['success'])
        self.assertEqual(result['reason'],'Missing mapping-start reference')
        self.assertNotIn('c1',calls)

    def test_suppresses_observation_only_after_both_targets_are_localized(self):
        result,calls,events=self.exercise(cooperative=True,scenario='observation_target_gate',timeout=15.)
        self.assertTrue(result['success'],result)
        phases=[e['phase'] for e in events]
        gate='OBSERVATION_SUPPRESSED_TARGETS_LOCATED'
        self.assertEqual(phases.count(gate),1)
        self.assertLess(phases.index(gate),phases.index('FRONTIERS_EXHAUSTED'))
        self.assertEqual(calls,['quiesce','find_targets','save_map','s1','c1'])

    def test_mapping_at_18_m2_accepts_free_and_occupied_cells(self):
        result,calls,events=self.exercise(known_cells=72)
        self.assertTrue(result['success']);self.assertNotIn('supplement_mapping',calls)
        area=next(e for e in events if e['phase']=='MAPPING_AREA_READY')
        self.assertEqual(area['known_area_m2'],18.)

    def test_insufficient_mapping_repositions_and_rechecks_before_tasks(self):
        result,calls,events=self.exercise(known_cells=20,grow_map=True,timeout=20.)
        self.assertTrue(result['success'])
        self.assertEqual(calls,['quiesce','supplement_mapping','resume_exploration','quiesce','find_targets','save_map','s1'])
        self.assertTrue(any(e['phase']=='MAPPING_AREA_INSUFFICIENT' for e in events))

    def test_mapping_below_threshold_exhausts_without_dispatching_rescue(self):
        result,calls,_=self.exercise(known_cells=71,timeout=20.)
        self.assertFalse(result['success']);self.assertIn('Mapping incomplete',result['reason'])
        self.assertEqual(calls.count('supplement_mapping'),3)
        self.assertNotIn('find_targets',calls);self.assertNotIn('save_map',calls);self.assertNotIn('s1',calls)

    def test_missing_map_cannot_complete_mapping(self):
        result,calls,_=self.exercise(publish_map=False,timeout=20.)
        self.assertFalse(result['success']);self.assertNotIn('save_map',calls)

    def test_failed_supplement_does_not_reacquire_control_or_resume(self):
        result,calls,_=self.exercise(known_cells=20,fail='supplement_mapping',timeout=20.)
        self.assertFalse(result['success'])
        self.assertEqual(calls,['quiesce','supplement_mapping'])

    def test_tree_halts_supplement_on_new_frontier_before_resuming(self):
        result,calls,events=self.exercise(known_cells=20,grow_map=True,timeout=20.,scenario='frontier_during_supplement')
        self.assertTrue(result['success'])
        self.assertEqual(calls,['quiesce','supplement_mapping','resume_exploration','quiesce','find_targets','save_map','s1'])
        self.assertTrue(any(e['phase']=='MAPPING_FRONTIER_AVAILABLE' for e in events))
        self.assertFalse(any(e['phase']=='STAGE_FINISHED' and e.get('step')=='supplement_mapping' for e in events))

    def test_missing_monitor_halts_supplement_without_auto_resume(self):
        result,calls,_=self.exercise(known_cells=20,timeout=20.,scenario='monitor_missing')
        self.assertFalse(result['success']);self.assertIn('Frontier monitoring unavailable',result['reason'])
        self.assertEqual(calls,['quiesce','supplement_mapping'])

    def test_cancel_supplement_does_not_resume_exploration(self):
        result,calls,_=self.exercise(known_cells=20,block='supplement_mapping',timeout=20.)
        self.assertFalse(result['success']);self.assertEqual(result['reason'],'Mission canceled')
        self.assertEqual(calls,['quiesce','supplement_mapping'])

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

    def test_c1_start_occupancy_recovers_then_requires_new_preplan(self):
        result,_,events=self.exercise(cooperative=True,scenario='preplan_occupied')
        self.assertTrue(result['success'])
        units=[e.get('unit') for e in events if e.get('task')=='c1' and e['phase']=='UNIT_CALL']
        i=units.index('recover_preplan_start')
        self.assertEqual(units[i+1],'select_parking_locked')
        self.assertLess(i,units.index('dispatch_navigation'))
        self.assertNotIn('select_parking',units)

    def test_c1_start_recovery_failure_cannot_dispatch_or_reselect_goals(self):
        result,_,events=self.exercise(cooperative=True,scenario='preplan_occupied_exhausted')
        self.assertFalse(result['success'])
        units=[e.get('unit') for e in events if e.get('task')=='c1' and e['phase']=='UNIT_CALL']
        self.assertIn('recover_preplan_start',units)
        self.assertNotIn('dispatch_navigation',units);self.assertNotIn('select_parking',units)

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
