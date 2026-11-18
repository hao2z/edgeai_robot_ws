"""Isolated real ROS actions: yielding works during requester recovery and cancels cleanly."""
from pathlib import Path
import json
import os
import sys
import threading
import time
import unittest

import rclpy
from rclpy.action import ActionClient,ActionServer,CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile,DurabilityPolicy
from rclpy.task import Future
from geometry_msgs.msg import TransformStamped,PoseStamped
from nav_msgs.msg import OccupancyGrid,Odometry
from nav2_msgs.action import NavigateToPose,ComputePathToPose
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'src/edgeai_navigation'),str(ROOT/'src/edgeai_navigation/scripts')]
from peer_yield_server import PeerYieldServer


class PeerYieldROS(unittest.TestCase):
    def exercise(self,case):
        rclpy.init()
        settings=dict(lightweight=True, max_data_age_s=.5,yielding=dict(enabled=True,yield_body_radius_m=.15,
            yield_max_cells=20000,yield_max_attempts=2,yield_sample_spacing_m=.2,
            yield_min_distance_m=1.,yield_max_distance_m=3.,yield_requester_clearance_m=1.,
            yield_route_clearance_m=.5,yield_max_candidates=20,yield_navigation_timeout_s=5.))
        server=PeerYieldServer(settings);peer=rclpy.create_node('yield_fake_nav2')
        executor=MultiThreadedExecutor(num_threads=4);executor.add_node(server);executor.add_node(peer)
        thread=threading.Thread(target=executor.spin,daemon=True)
        poses={'robot1':[.55,2.05],'robot2':[2.55,2.05]}
        states={r:dict(phase='ACTIVE' if r=='robot1' else 'IDLE',mode='MANUAL',command_sequence=1) for r in poses}
        if case=='busy_peer':states['robot2']['phase']='ACTIVE'
        if case=='no_blockage':poses['robot2']=[1.,1.]
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        pubs={r:peer.create_publisher(String,('/'+r if r else '')+'/navigation_authority/state',qos) for r in poses}
        odom={r:peer.create_publisher(Odometry,('/'+r if r else '')+'/odom',10) for r in poses}
        maps=peer.create_publisher(OccupancyGrid,'/map',qos);tf=TransformBroadcaster(peer)
        nav_calls=[];plan_calls=[];gate=Future();started=threading.Event();canceled=threading.Event()
        events=[];subscription=peer.create_subscription(String,'/navigation/peer_yield/state',
            lambda m:events.append(json.loads(m.data)),qos)
        def publish():
            now=peer.get_clock().now().to_msg()
            for r,p in poses.items():
                pubs[r].publish(String(data=json.dumps(states[r])))
                msg=Odometry();msg.header.stamp=now;odom[r].publish(msg)
                t=TransformStamped();t.header.frame_id='map';t.child_frame_id=(r+'/' if r else '')+'base_footprint'
                t.header.stamp=now;t.transform.translation.x,t.transform.translation.y=p;t.transform.rotation.w=1.;tf.sendTransform(t)
            grid=OccupancyGrid();grid.header.frame_id='map';grid.header.stamp=now
            grid.info.resolution=.1;grid.info.width=50;grid.info.height=40;grid.info.origin.orientation.w=1.
            grid.data=[0 if x<18 or x>32 or 18<=y<=22 else 100 for y in range(40) for x in range(50)]
            maps.publish(grid)
        timer=peer.create_timer(.04,publish)
        def plan(h):
            plan_calls.append(h.request.planner_id)
            result=ComputePathToPose.Result();result.path.header.frame_id='map'
            if case=='planner_reject':result.error_code=208;h.abort()
            else:result.path.poses=[h.request.goal];h.succeed()
            return result
        def cancel(h):
            canceled.set();gate.set_result(True);return CancelResponse.ACCEPT
        async def navigate(h):
            nav_calls.append(h.request);states['robot2']['phase']='ACTIVE';started.set()
            if case=='cancel':
                await gate
                p=h.request.pose.pose.position;poses['robot2']=[p.x,p.y];h.succeed()
            else:
                p=h.request.pose.pose.position;poses['robot2']=[p.x,p.y];h.succeed()
            states['robot2']['phase']='IDLE'
            return NavigateToPose.Result()
        planner=ActionServer(peer,ComputePathToPose,'/robot2/navigation_backend/compute_path_to_pose',plan,
            callback_group=ReentrantCallbackGroup())
        navigation=ActionServer(peer,NavigateToPose,'/robot2/navigation/yield_to_pose',navigate,
            cancel_callback=cancel,callback_group=ReentrantCallbackGroup())
        client=ActionClient(peer,NavigateToPose,'/robot1/navigation/yield_peer',callback_group=ReentrantCallbackGroup())
        def wait(predicate,timeout=8.):
            end=time.monotonic()+timeout
            while not predicate() and time.monotonic()<end:time.sleep(.02)
            self.assertTrue(predicate())
        thread.start()
        try:
            wait(lambda:all(r in server.states for r in poses) and server.map is not None)
            self.assertFalse(server.odom_subscriptions)
            self.assertIsNone(server.listener)
            goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.pose.position.x=4.45
            goal.pose.pose.position.y=2.05;goal.pose.pose.orientation.w=1.
            self.assertTrue(client.wait_for_server(timeout_sec=3.))
            sent=client.send_goal_async(goal);wait(sent.done);handle=sent.result();self.assertTrue(handle.accepted)
            result=handle.get_result_async()
            if case=='cancel':
                wait(started.is_set)
                # Busy serialization also rejects recursive requests from peer recovery.
                recursive=client.send_goal_async(goal);wait(recursive.done);self.assertFalse(recursive.result().accepted)
                cancellation=handle.cancel_goal_async();wait(cancellation.done)
                self.assertFalse(cancellation.result().goals_canceling)
                # Requester moves and hands over its task while the peer continues.
                states['robot1']['command_sequence']=2
                states['robot1']['mode']='STOPPED'
                time.sleep(.15)
                self.assertFalse(result.done())
                self.assertFalse(canceled.is_set())
                gate.set_result(True)
            wait(result.done)
            self.assertEqual(result.result().status,4 if case in ('cancel','success') else 6)
            self.assertFalse(server.pending)
            self.assertFalse(server.handles)
            self.assertFalse(server.odom_subscriptions)
            self.assertIsNone(server.listener)
            if case=='success':
                self.assertEqual(plan_calls,['GridBased']);self.assertEqual(len(nav_calls),1)
                self.assertTrue(any(e.get('route_released') for e in events))
            elif case=='cancel':self.assertFalse(canceled.is_set())
            else:self.assertFalse(nav_calls)
        finally:
            executor.shutdown(timeout_sec=10.);thread.join(timeout=10.)
            planner.destroy();navigation.destroy();server.destroy_node();peer.destroy_node();rclpy.shutdown()

    def test_generated_tree_endpoints_are_available(self):
        # Exercise the installed entry and generated endpoints together: TF
        # prefixes must not be mistaken for the root robot's ROS namespace.
        import signal
        import subprocess
        import tempfile
        import xml.etree.ElementTree as ET
        import yaml
        from ament_index_python.packages import get_package_share_directory
        from goal_policy import managed_trees
        script=Path(get_package_share_directory('edgeai_navigation'))/'scripts/peer_yield_server.py'
        self.assertTrue(script.is_file(),str(script))
        with tempfile.TemporaryDirectory() as tmp:
            log_path=Path(tmp)/'server.log'
            with log_path.open('w') as output:
                process=subprocess.Popen([sys.executable,'-u',str(script)],stdout=output,stderr=output)
                rclpy.init();node=rclpy.create_node('yield_generated_tree_check')
                try:
                    for namespace,frame in (('/robot1','robot1/base_footprint'),('/robot2','robot2/base_footprint')):
                        (Path(tmp)/'nav2-runtime.yaml').write_text(yaml.safe_dump({
                            'local_costmap':{'local_costmap':{'ros__parameters':{}}},
                            'bt_navigator':{'ros__parameters':{'robot_base_frame':frame}}}))
                        paths=managed_trees(ROOT/'src/edgeai_navigation/config/behavior_trees/explore_recovery.xml',tmp,namespace)
                        for path in paths.values():
                            endpoint=ET.parse(path).find('.//NavigateToPose').get('server_name')
                            client=ActionClient(node,NavigateToPose,endpoint)
                            try:self.assertTrue(client.wait_for_server(timeout_sec=5.),endpoint+'\n'+log_path.read_text())
                            finally:client.destroy()
                    self.assertIsNone(process.poll(),log_path.read_text())
                finally:
                    if process.poll() is None:process.send_signal(signal.SIGINT)
                    try:process.wait(timeout=5.)
                    except subprocess.TimeoutExpired:process.kill();process.wait()
                    node.destroy_node();rclpy.shutdown()

    def test_active_stopped_requester_and_idle_peer(self):self.exercise('success')
    def test_busy_peer_is_never_dispatched(self):self.exercise('busy_peer')
    def test_unconfirmed_blockage_never_dispatched(self):self.exercise('no_blockage')
    def test_native_planner_rejection_never_dispatched(self):self.exercise('planner_reject')
    def test_requester_cannot_cancel_committed_peer_yield(self):self.exercise('cancel')


if __name__=='__main__':
    if os.environ.get('ROS_DOMAIN_ID','0') in ('0','86','93'):raise SystemExit('Use isolated ROS domain')
    unittest.main()
