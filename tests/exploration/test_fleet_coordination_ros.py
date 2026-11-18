"""Real task tree/coordinator with two fake authorities and navigation Actions."""
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
import time
import unittest
import sys

import rclpy
from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from rclpy.task import Future
from rclpy.qos import QoSProfile, DurabilityPolicy
from ament_index_python.packages import get_package_prefix
from edgeai_mission.action import ExecuteTask
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import TransformBroadcaster

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'src/edgeai_navigation'),str(ROOT/'src/edgeai_navigation/scripts')]
from peer_yield_server import PeerYieldServer


class FleetCoordinationROS(unittest.TestCase):
    def exercise(self, reject_yield=False, replace_frontier=False, shared_only=False):
        rclpy.init()
        peer=rclpy.create_node('fleet_coordination_test')
        yield_node=PeerYieldServer(dict(max_data_age_s=.5,yielding=dict(enabled=True,
            yield_body_radius_m=.15,yield_max_cells=20000,yield_max_attempts=2,
            yield_min_distance_m=1.,yield_max_distance_m=3.,yield_requester_clearance_m=1.,
            yield_route_clearance_m=.5,yield_sample_spacing_m=.2,yield_max_candidates=20,
            yield_navigation_timeout_s=5.)))
        from rclpy.executors import MultiThreadedExecutor
        import threading
        yield_executor=MultiThreadedExecutor(num_threads=2);yield_executor.add_node(yield_node)
        yield_thread=threading.Thread(target=yield_executor.spin,daemon=True);yield_thread.start()
        yield_client=ActionClient(peer,NavigateToPose,'/robot1/navigation/yield_peer')
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        clock=peer.create_publisher(Clock,'/clock',10)
        map_pub=peer.create_publisher(OccupancyGrid,'/map',qos)
        tf=TransformBroadcaster(peer)
        # U detour: robot1 is nearest by Euclidean distance while robot2 blocks
        # the only corridor. Shared allocation must retain the yield scenario.
        positions=dict(robot1=[.55,6.05],robot2=[2.55,2.05])
        states={r:dict(mode='STOPPED',phase='IDLE',command_sequence=0,explorer_paused=True)
                for r in positions}
        odom_pubs={}
        publishers={};candidates={};services=[];servers=[];received=[];events=[];statuses=[]
        done=False
        canceled_at=None
        first_terminal_at=None
        replacement_at=None
        release=Future()
        subscriptions=[peer.create_subscription(String,'/exploration/task_state',
            lambda m:events.append(json.loads(m.data)),qos),
            peer.create_subscription(String,'/exploration/fleet_status',
            lambda m:statuses.append(json.loads(m.data)),qos)]
        for robot in positions:
            prefix='/'+robot
            odom_pubs[robot]=peer.create_publisher(Odometry,prefix+'/odom',10)
            publishers[robot]=peer.create_publisher(String,prefix+'/navigation_authority/state',qos)
            candidates[robot]=peer.create_publisher(String,'/'+robot+'/explore/candidates',10)
            def manual(req,res,r=robot):
                states[r].update(mode='MANUAL',command_sequence=states[r]['command_sequence']+1)
                res.success=True;return res
            def stop(req,res,r=robot):
                states[r].update(mode='STOPPED',command_sequence=states[r]['command_sequence']+1)
                res.success=True;return res
            services.extend([peer.create_service(SetBool,prefix+'/navigation_authority/set_manual',manual),
                peer.create_service(Trigger,prefix+'/navigation_authority/stop',stop)])
            def cancel(handle,r=robot):
                nonlocal canceled_at
                canceled_at=time.monotonic()
                states[r]['phase']='CANCELING'
                return CancelResponse.ACCEPT
            async def execute(handle,r=robot):
                nonlocal done,first_terminal_at,replacement_at
                p=handle.request.pose.pose.position
                received.append((r,p.x,p.y))
                result=NavigateToPose.Result()
                if replace_frontier:
                    if len(received)==1:
                        states[r]['phase']='ACTIVE'
                        await release
                        self.assertTrue(handle.is_cancel_requested)
                        first_terminal_at=time.monotonic()
                        handle.canceled()
                        states[r]['phase']='IDLE'
                    else:
                        replacement_at=time.monotonic()
                        positions[r]=[p.x,p.y]
                        done=True
                        handle.succeed()
                    return result
                if r=='robot1' and math.dist(positions['robot2'],[2.55,2.05])<.35:
                    states[r]['phase']='ACTIVE'
                    if yield_client.server_is_ready():
                        sent=await yield_client.send_goal_async(NavigateToPose.Goal(pose=handle.request.pose))
                        if sent.accepted:await sent.get_result_async()
                    states[r]['phase']='IDLE'
                    if math.dist(positions['robot2'],[2.55,2.05])<.35:
                        result.error_code=208;handle.abort()
                    else:
                        positions[r]=[p.x,p.y];done=True;handle.succeed()
                else:
                    positions[r]=[p.x,p.y]
                    if r=='robot1':done=True
                    handle.succeed()
                return result
            servers.append(ActionServer(peer,NavigateToPose,prefix+'/navigation/assigned_frontier_to_pose',
                execute_callback=execute,cancel_callback=cancel,goal_callback=lambda req,r=robot:
                    GoalResponse.REJECT if reject_yield and r=='robot2' else GoalResponse.ACCEPT))

        def yield_plan(handle):
            result=ComputePathToPose.Result();result.path.header.frame_id='map'
            result.path.poses=[handle.request.goal];handle.succeed();return result
        servers.append(ActionServer(peer,ComputePathToPose,
            '/robot2/navigation_backend/compute_path_to_pose',yield_plan))

        async def yield_motion(handle):
            p=handle.request.pose.pose.position;received.append(('robot2',p.x,p.y))
            positions['robot2']=[p.x,p.y];handle.succeed();return NavigateToPose.Result()
        servers.append(ActionServer(peer,NavigateToPose,'/robot2/navigation/yield_to_pose',yield_motion,
            goal_callback=lambda _:GoalResponse.REJECT if reject_yield else GoalResponse.ACCEPT))

        def publish():
            if canceled_at is not None and time.monotonic()-canceled_at>=.4 and not release.done():
                release.set_result(True)
            now=peer.get_clock().now()
            for robot,pub in odom_pubs.items():
                msg=Odometry();msg.header.stamp=now.to_msg();pub.publish(msg)
            clock.publish(Clock(clock=now.to_msg()))
            grid=OccupancyGrid();grid.header.frame_id='map';grid.header.stamp=now.to_msg()
            grid.info.width=50;grid.info.height=80;grid.info.resolution=.1
            grid.info.origin.orientation.w=1.
            grid.data=[0 if x<18 or x>32 or 18<=y<=22 else 100
                       for y in range(80) for x in range(50)]
            map_pub.publish(grid)
            transforms=[]
            for robot,p in positions.items():
                t=TransformStamped();t.header.frame_id='map';t.child_frame_id=robot+'/base_footprint'
                t.header.stamp=now.to_msg();t.transform.translation.x,t.transform.translation.y=p
                t.transform.rotation.w=1.;transforms.append(t)
                if robot=='robot1':
                    import copy
                    root_tf=copy.deepcopy(t);root_tf.child_frame_id='base_footprint';transforms.append(root_tf)
                publishers[robot].publish(String(data=json.dumps(states[robot])))
                source='robot2' if shared_only else 'robot1'
                goals=[] if done or robot!=source else [dict(x=4.45,y=6.05,size=20.)]
                if replace_frontier and received and goals:
                    # Inside the old goal's .7m reservation, so another idle
                    # car cannot take the replacement before cancellation.
                    goals=[dict(x=3.95,y=6.05,size=20.)]
                candidates[robot].publish(String(data=json.dumps(dict(frame_id='map',available=True,
                    stamp_ns=now.nanoseconds,candidates=goals))))
            tf.sendTransform(transforms)
        timer=peer.create_timer(.05,publish)
        process=None
        try:
            with tempfile.TemporaryDirectory(prefix='fleet-coordination-') as temporary:
                directory=Path(temporary)
                config=dict(project_root=str(ROOT),directory=str(directory),wall_timeout_s=30.,
                    exploration_timeout_s=25.,task_wall_timeout_s=dict(explore=28.),
                    unit_worker_script=str(ROOT/'src/edgeai_mission/scripts/task_units.py'),
                    task_trees=dict(explore=str(ROOT/'src/edgeai_mission/config/tasks/explore.xml')),
                    exploration_assignment=dict(completion_hold_s=.2,failure_cooldown_s=.1,max_navigation_attempts=2,
                        max_retry_rounds=1,settle_s=.1))
                if replace_frontier:
                    config['exploration_assignment'].update(
                        frontier_update_hold_s=.2,frontier_min_navigation_s=.2,frontier_match_radius_m=.3)
                path=directory/'config.json';path.write_text(json.dumps(config))
                binary=Path(get_package_prefix('edgeai_mission'))/'lib/edgeai_mission/task_executor'
                # A fresh endpoint prevents DDS discovery of a previous test's
                # terminated executor from satisfying wait_for_server().
                action='/mission/tests/'+directory.name.replace('-','_')+'/execute'
                with (directory/'executor.log').open('w+') as log:
                    process=subprocess.Popen([str(binary),'--ros-args','-p','config:='+str(path),
                        '-p','task_name:=explore','-r','/mission/tasks/explore/execute:='+action],stdout=log,stderr=log)
                    client=ActionClient(peer,ExecuteTask,action)
                    self.assertTrue(client.wait_for_server(timeout_sec=8.))
                    def wait(predicate):
                        end=time.monotonic()+32.
                        while not predicate() and time.monotonic()<end:
                            rclpy.spin_once(peer,timeout_sec=.02)
                        evidence=(directory/'fleet-exploration.log')
                        self.assertTrue(predicate(),evidence.read_text() if evidence.exists() else log.name)
                    sent=client.send_goal_async(ExecuteTask.Goal(task_id='fleet-coordination'))
                    wait(sent.done);handle=sent.result();self.assertTrue(handle.accepted)
                    terminal=handle.get_result_async();wait(terminal.done)
                    result=terminal.result()
                    evidence=(directory/'fleet-exploration.log').read_text()
                    self.assertEqual(result.status,4,
                                     result.result.reason+'\n'+evidence)
                    # Yielding is internal to navigation, never a task assignment.
                    self.assertNotIn('FLEET_EXPLORATION_YIELD_STARTED',evidence)
                    self.assertTrue(all(s['mode']=='STOPPED' for s in states.values()),evidence)
                    saved=json.loads((directory/'exploration-result.json').read_text())
                    self.assertTrue(saved['success'])
                    if replace_frontier:
                        self.assertEqual([(r,round(x,2)) for r,x,y in received],
                                         [('robot1',4.45),('robot1',3.95)],evidence)
                        self.assertGreaterEqual(first_terminal_at-canceled_at,.4)
                        self.assertGreaterEqual(replacement_at,first_terminal_at)
                        self.assertEqual(saved['failures'],[])
                        self.assertEqual(saved['failure_history']['robot1'],[])
                        self.assertEqual([a['state'] for a in saved['assignments']],['CANCELED','SUCCEEDED'])
                        self.assertIn('FLEET_EXPLORATION_FRONTIER_UPDATE_REQUESTED',evidence)
                        self.assertIn('FLEET_EXPLORATION_FRONTIER_RELEASED',evidence)
                    elif not reject_yield:
                        self.assertEqual([r for r,x,y in received],['robot1','robot2'],evidence)
                        self.assertFalse(saved['failures'])
                        self.assertTrue(statuses,'No fleet state snapshot received')
                    else:
                        self.assertLessEqual(sum(a.get('kind')=='yield' for a in saved['assignments']),2)
                        self.assertEqual({r: sum(h['attempts'] for h in saved['failure_history'][r])
                                          for r in states}, {'robot1':2,'robot2':2})
                        self.assertIn('FLEET_EXPLORATION_COMPLETED',evidence)
        finally:
            if process and process.poll() is None:
                process.terminate();process.wait(timeout=20.)
            for server in servers:server.destroy()
            yield_executor.shutdown(timeout_sec=10.);yield_thread.join(timeout=10.)
            yield_node.destroy_node();peer.destroy_node();rclpy.shutdown()

    def test_blocked_robot_resumes_after_idle_peer_yields(self):self.exercise()
    def test_both_robots_exhaust_retries_and_skip_unreachable_frontier(self):self.exercise(True)
    def test_frontier_update_cancels_then_dispatches_without_navigation_failure(self):
        self.exercise(replace_frontier=True)

    def test_robot1_uses_robot2_frontiers_and_shared_updates(self):
        self.exercise(replace_frontier=True,shared_only=True)


if __name__=='__main__':
    if os.environ.get('ROS_DOMAIN_ID','0') in ('0','86','93'):
        raise SystemExit('Use an isolated test domain')
    unittest.main()
