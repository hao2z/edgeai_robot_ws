"""Assigned S1 tree, real fleet coordinator and real S1 checks with fake motion."""
import copy
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
import threading
import time
import unittest

import rclpy
from rclpy.action import ActionServer, ActionClient, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_prefix
from edgeai_mission.action import ExecuteTask
from nav2_msgs.action import NavigateToPose
from rosgraph_msgs.msg import Clock
import yaml

from fake_navigation import FakeNavigation

ROOT=Path(__file__).resolve().parents[2]


class FleetRescueROS(unittest.TestCase):
    def exercise(self, fail_first=False, correction=False, fail_all=False, target_update=False, start_occupied=False,
                 multiple=False, partial=False):
        # Separate participants and action/service discovery between cases.
        domain=205+sorted(unittest.defaultTestLoader.getTestCaseNames(type(self))).index(self._testMethodName)
        rclpy.init(domain_id=domain)
        peers={r:FakeNavigation(r) for r in ('robot1','robot2')}
        for r,n in peers.items():
            n.mode='STOPPED'
            n.follow_goal=True
            n.alignment_motion=correction
            n.transform.transform.translation.x=0. if r=='robot1' else 1.5
            n.transform.transform.translation.y=2.
            n.yield_action=ActionServer(n,NavigateToPose,
                ('/'+r)+'/navigation/assigned_frontier_to_pose',
                lambda h,n=n:n.navigate(h,'yield'),callback_group=ReentrantCallbackGroup(),
                cancel_callback=lambda _:CancelResponse.ACCEPT)
        if fail_first:
            peers['robot2'].errors=[208]*10
        if start_occupied:peers['robot2'].plan_errors=[205]
        if fail_all:
            for n in peers.values():n.errors=[208]*10
        if partial:
            for node in peers.values():
                original = node.navigate
                def abort_second(handle, policy, original=original, node=node):
                    if handle.request.pose.pose.position.x < 1.:
                        node.errors = [208]
                    return original(handle, policy)
                node.navigate = abort_second
        if correction:
            original=peers['robot2'].navigate
            def arrive_sideways(handle,policy):
                result=original(handle,policy)
                q=peers['robot2'].transform.transform.rotation
                q.z=math.sin(math.pi/4);q.w=math.cos(math.pi/4)
                return result
            # ActionServer captured a lambda that resolves navigate at call time.
            peers['robot2'].navigate=arrive_sideways
        first=peers['robot1']
        clock=first.create_publisher(Clock,'/clock',10)
        def publish_alias():
            clock.publish(Clock(clock=first.get_clock().now().to_msg()))
            tf=copy.deepcopy(first.transform)
            tf.child_frame_id='robot1/base_footprint'
            first.tf.sendTransform(tf)
        first.create_timer(.05,publish_alias)
        executor=MultiThreadedExecutor(num_threads=6)
        for n in peers.values():executor.add_node(n)
        thread=threading.Thread(target=executor.spin);thread.start()
        process=None
        try:
            with tempfile.TemporaryDirectory(prefix='fleet-rescue-') as temporary:
                d=Path(temporary)
                settings=yaml.safe_load((ROOT/'src/edgeai_navigation/config/normal_rescue.yaml').read_text())
                settings.update(goal_source='detected_target',target_id=1,dwell_s=.2,settle_margin_s=0.,
                    # This tests assignment/recovery, not subsecond sensor deadlines.
                    # Leave scheduling headroom while Gazebo may be running elsewhere.
                    max_data_age_s=2.,retry_tf_max_age_s=.8,
                    wall_timeout_s=35.,alignment_enabled=correction,
                    approach_selection=dict(enabled=True,angles_deg=[0],max_candidates=4,planning_timeout_s=3.))
                (d/'rescue-config.yaml').write_text(yaml.safe_dump(settings))
                targets = [dict(id=1,type='normal',x=3.,y=2.,yaw=math.pi,spread_m=.01)]
                if multiple:
                    targets.append(dict(id=3,type='normal',x=.2,y=3.,yaw=-math.pi/2,spread_m=.01))
                (d/'targets.json').write_text(json.dumps(dict(frame_id='map',targets=targets)))
                if target_update:peers['robot2'].update_file=d/'targets.json'
                geometry=dict(footprint=json.dumps([[-.2,-.15],[.2,-.15],[.2,.15],[-.2,.15]]),footprint_padding=.01)
                (d/'nav2-runtime.yaml').write_text(yaml.safe_dump(dict(global_costmap=dict(global_costmap=dict(ros__parameters=geometry)))))
                c=dict(directory=str(d),project_root=str(ROOT),joint_mapping=True,wall_timeout_s=40.,
                    task_wall_timeout_s=dict(s1=38.),task_attempts=dict(s1=1),task_navigation_attempts=dict(s1=3),
                    unit_worker_script=str(ROOT/'src/edgeai_mission/scripts/task_units.py'),
                    task_trees=dict(s1=str(ROOT/'src/edgeai_mission/config/tasks/s1.xml')),
                    single_rescue_assignment=dict(max_navigation_attempts=1,max_retry_rounds=1,
                        reservation_radius_m=.15,failure_cooldown_s=10.))
                if multiple:
                    c['required_targets'] = [dict(id=t['id'], type=t['type']) for t in targets]
                    settings['dwell_s'] = .8
                    (d/'rescue-config.yaml').write_text(yaml.safe_dump(settings))
                if partial:
                    c.update(cooperative=True, post_mapping_localization=True)
                config=d/'config.json';config.write_text(json.dumps(c))
                binary=Path(get_package_prefix('edgeai_mission'))/'lib/edgeai_mission/task_executor'
                action='/mission/tests/'+d.name.replace('-','_')+'/execute'
                with (d/'executor.log').open('w+') as log:
                    process=subprocess.Popen([str(binary),'--ros-args','-p','config:='+str(config),
                        '-p','task_name:=s1','-r','/mission/tasks/s1/execute:='+action],stdout=log,stderr=log,
                        env=dict(os.environ,ROS_DOMAIN_ID=str(domain)))
                    client=ActionClient(first,ExecuteTask,action)
                    self.assertTrue(client.wait_for_server(timeout_sec=8.))
                    def wait(future):
                        end=time.monotonic()+42.
                        while not future.done() and time.monotonic()<end:time.sleep(.02)
                        evidence=(d/'rescue.log').read_text() if (d/'rescue.log').exists() else (d/'executor.log').read_text()
                        self.assertTrue(future.done(),evidence)
                        return future.result()
                    handle=wait(client.send_goal_async(ExecuteTask.Goal(task_id='assigned-s1')))
                    self.assertTrue(handle.accepted)
                    terminal=wait(handle.get_result_async())
                    evidence=(d/'rescue.log').read_text()+'\n'+(d/'task-unit-events.jsonl').read_text()
                    self.assertEqual(terminal.status,6 if fail_all else 4,terminal.result.reason+'\n'+evidence)
                    self.assertTrue(all(n.mode=='STOPPED' for n in peers.values()),evidence)
                    history=json.loads((d/'s1-assignment-result.json').read_text())
                    if fail_all:
                        self.assertFalse(history['success'])
                        self.assertEqual(sum(f['kind']=='rescue' for f in history['failures']),2,evidence)
                        self.assertFalse((d/'rescue-result.json').exists())
                    elif multiple:
                        result = json.loads((d/'rescue-result.json').read_text())
                        self.assertEqual(result['success'], not partial)
                        self.assertEqual({t['target_id'] for t in result['target_results']}, {1} if partial else {1,3})
                        if partial:
                            self.assertEqual([t['target_id'] for t in result['skipped_targets']], [3])
                        else:
                            assignments = history['assignments']
                            self.assertEqual({a['robot'] for a in assignments}, {'robot1','robot2'})
                            self.assertLess(abs(peers['robot1'].calls[0][0]-peers['robot2'].calls[0][0]), .8)
                    else:
                        result=json.loads((d/'rescue-result.json').read_text())
                        robot='robot1' if fail_first else 'robot2'
                        self.assertEqual(result['robot'],robot,evidence)
                        self.assertEqual(result['success_evidence']['robots'][0]['robot'],robot)
                        self.assertEqual(len(history['failures']),1 if fail_first else 0,evidence)
                        self.assertEqual(len(peers['robot2'].calls),2 if target_update else 1)
                        self.assertEqual(len(peers['robot1'].calls),1 if fail_first else 0)
                        if correction:self.assertTrue(peers['robot2'].spin_calls,evidence)
                        if target_update:self.assertTrue(peers['robot2'].canceled_update,evidence)
                        if start_occupied:self.assertIn('RESCUE_APPROACH_START_OCCUPIED',evidence)
        finally:
            if process and process.poll() is None:process.terminate();process.wait(timeout=20.)
            executor.shutdown();thread.join(timeout=5.)
            for n in peers.values():n.destroy_node()
            rclpy.shutdown()

    def test_nearer_robot2_rescues_and_uses_its_own_pose(self):self.exercise()
    def test_failed_robot2_falls_back_to_robot1(self):self.exercise(fail_first=True)
    def test_robot2_uses_existing_s1_final_heading_correction(self):self.exercise(correction=True)
    def test_both_fail_without_false_success_or_unbounded_retry(self):self.exercise(fail_all=True)
    def test_target_update_reassigns_without_charging_failure(self):self.exercise(target_update=True)
    def test_preplan_start_occupied_still_dispatches_nav2_recovery(self):self.exercise(start_occupied=True)
    def test_distinct_targets_dispatched_to_both_robots(self):self.exercise(multiple=True)
    def test_unreachable_target_does_not_prevent_cooperative_handoff(self):self.exercise(multiple=True, partial=True)


if __name__=='__main__':
    if os.environ.get('ROS_DOMAIN_ID','0') in ('0','86','93'):raise SystemExit('Use an isolated domain')
    unittest.main()
