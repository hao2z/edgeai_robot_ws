"""Real dual action endpoints and synchronized joint dwell against a fake fleet."""
from pathlib import Path
import sys
import tempfile
import threading
import time
import json
import math
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from rclpy.action import ActionServer
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster
from tf2_ros import TransformException
import yaml

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/edgeai_navigation/scripts'))
from cooperative_rescue import CooperativeRescue


class FakeFleet(Node):
    def __init__(self):
        super().__init__('fake_cooperative_fleet')
        self.calls=[];self.on_arrival=None;self.poses={'':[1.,1.,0.],'robot2':[1.7,1.,0.]};self.tf=TransformBroadcaster(self)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub=self.create_publisher(OccupancyGrid,'/map',qos)
        self.odom={};self.amcl={};self.state={};self.servers=[]
        for ns in ('','robot2'):
            prefix='/'+(ns+'/' if ns else '')
            self.odom[ns]=self.create_publisher(Odometry,prefix+'odom',10)
            self.amcl[ns]=self.create_publisher(PoseWithCovarianceStamped,prefix+'amcl_pose',qos)
            self.state[ns]=self.create_publisher(String,prefix+'navigation_authority/state',qos)
            self.servers.append(ActionServer(self,NavigateToPose,prefix+'navigation/rescue_to_pose',
                execute_callback=lambda h,n=ns:self.navigate(n,h)))
            self.servers.append(ActionServer(self,ComputePathToPose,prefix+'navigation_backend/compute_path_to_pose',
                execute_callback=self.plan))
        self.create_timer(.04,self.publish)

    def navigate(self,ns,handle):
        p=handle.request.pose.pose
        self.calls.append(ns);self.poses[ns]=[p.position.x,p.position.y,2*math.atan2(p.orientation.z,p.orientation.w)]
        if self.on_arrival:self.on_arrival(ns)
        handle.succeed();return NavigateToPose.Result()

    def plan(self,handle):
        result=ComputePathToPose.Result();result.path.header.frame_id='map';result.path.poses=[handle.request.goal]
        handle.succeed();return result

    def publish(self):
        stamp=self.get_clock().now().to_msg()
        grid=OccupancyGrid();grid.header.frame_id='map';grid.header.stamp=stamp
        grid.info.width=grid.info.height=120;grid.info.resolution=.05;grid.info.origin.orientation.w=1.;grid.data=[0]*14400
        self.map_pub.publish(grid)
        frames=[]
        for ns,p in self.poses.items():
            tf=TransformStamped();tf.header.frame_id='map';tf.child_frame_id=(ns+'/' if ns else '')+'base_footprint'
            tf.header.stamp=stamp;tf.transform.translation.x,tf.transform.translation.y=p[:2]
            tf.transform.rotation.z=math.sin(p[2]/2);tf.transform.rotation.w=math.cos(p[2]/2);frames.append(tf)
            odom=Odometry();odom.header.stamp=stamp;odom.pose.pose.orientation.w=1.;self.odom[ns].publish(odom)
            amcl=PoseWithCovarianceStamped();amcl.header.frame_id='map';amcl.header.stamp=stamp
            amcl.pose.pose.position.x,amcl.pose.pose.position.y=p[:2];amcl.pose.pose.orientation=tf.transform.rotation
            amcl.pose.covariance[0]=amcl.pose.covariance[7]=amcl.pose.covariance[35]=.001
            self.amcl[ns].publish(amcl)
            self.state[ns].publish(String(data=json.dumps(dict(mode='MANUAL',phase='IDLE',command_sequence=2))))
        self.tf.sendTransform(frames)


class CooperativeROS(unittest.TestCase):
    def activation_gate(self,activate):
        rclpy.init();peer=Node('activation_peer');behavior=LifecycleNode('behavior_server',namespace='robot2')
        behavior.trigger_configure()
        names=['/map_server']+['/'+(ns+'/' if ns else '')+name
            for ns in ('','robot2') for name in ('amcl','controller_server','planner_server',
                'behavior_server','velocity_smoother','collision_monitor','bt_navigator')]
        def active(_,response):
            response.current_state.id=State.PRIMARY_STATE_ACTIVE;response.current_state.label='active';return response
        services=[peer.create_service(GetState,name+'/get_state',active)
                  for name in names if name!='/robot2/behavior_server']
        activation=[None];begin=time.monotonic()
        def tick():
            if activate and activation[0] is None and time.monotonic()-begin>.45:
                behavior.trigger_activate();activation[0]=time.monotonic()
        peer.create_timer(.02,tick)
        executor=SingleThreadedExecutor();executor.add_node(peer);executor.add_node(behavior)
        thread=threading.Thread(target=executor.spin);thread.start()
        try:
            with tempfile.TemporaryDirectory() as directory:
                config=yaml.safe_load((ROOT/'src/edgeai_navigation/config/cooperative_rescue.yaml').read_text())
                node=CooperativeRescue(directory,config);node.seed_enabled=False
                try:
                    if activate:
                        node.wait_navigation_active(timeout=5.)
                        self.assertIsNotNone(activation[0]);self.assertGreaterEqual(time.monotonic(),activation[0])
                        events=[json.loads(s) for s in (Path(directory)/'cooperative-events.jsonl').read_text().splitlines()]
                        self.assertTrue(any(e['phase']=='NAVIGATION_WAITING' for e in events))
                        self.assertEqual(events[-1]['phase'],'NAVIGATION_READY')
                    else:
                        with self.assertRaisesRegex(RuntimeError,'timeout'):
                            node.wait_navigation_active(timeout=.5)
                        self.assertLess(time.monotonic()-begin,3.)
                finally:node.destroy_node()
        finally:
            executor.shutdown();thread.join(timeout=5.);peer.destroy_node();behavior.destroy_node();rclpy.shutdown()

    def test_navigation_gate_waits_for_discovered_but_inactive_behavior_server(self):
        self.activation_gate(True)

    def test_navigation_gate_times_out_when_behavior_never_activates(self):
        self.activation_gate(False)

    def test_finalized_map_rejects_targets_from_another_map_epoch(self):
        rclpy.init()
        try:
            with tempfile.TemporaryDirectory() as directory:
                config=yaml.safe_load((ROOT/'src/edgeai_navigation/config/cooperative_rescue.yaml').read_text())
                node=CooperativeRescue(directory,config);node.seed_enabled=False
                try:
                    node.targets=dict(map_binding=dict(mode='fixed_map',epoch='final-map',finalized=True))
                    node.target=dict(id=2,x=3.,y=3.,yaw=0.,last_observed_ns=1)
                    binding=dict(mode='fixed_map',epoch='other-map',finalized=True)
                    candidate=dict(id=2,x=3.2,y=3.,yaw=0.,confirmed=True,last_observed_ns=10)
                    path=Path(directory)/'targets.json'
                    path.write_text(json.dumps(dict(frame_id='map',map_binding=binding,targets=[candidate])))
                    self.assertFalse(node.refresh_target(force=True));self.assertFalse(node.target_ready)
                    binding['epoch']='final-map'
                    path.write_text(json.dumps(dict(frame_id='map',map_binding=binding,targets=[candidate])))
                    self.assertTrue(node.refresh_target(force=True));self.assertTrue(node.target_ready)
                finally:node.destroy_node()
        finally:rclpy.shutdown()

    def test_explicit_mission_handover_uses_original_reference_without_waiting_for_s1_file(self):
        rclpy.init()
        try:
            with tempfile.TemporaryDirectory() as directory:
                d=Path(directory)
                c=yaml.safe_load((ROOT/'src/edgeai_navigation/config/cooperative_rescue.yaml').read_text())
                reference=dict(mapping_start_pose=[.2,.1,math.pi/2],initial_odom2=[0.,0.,0.])
                path=d/'reference.json';path.write_text(json.dumps(reference))
                (d/'targets.json').write_text(json.dumps(dict(targets=[])))
                node=CooperativeRescue(directory,c);node.seed_enabled=False;node.handover_reference=str(path)
                node.odom={'':Odometry(),'robot2':Odometry()};node.map=OccupancyGrid()
                node.odom['robot2'].pose.pose.orientation.w=1.
                try:
                    with patch.object(node,'tf_pose',side_effect=[TransformException('TF still connecting'),[3.,1.,0.],[3.,1.,0.]]), \
                         patch.object(node,'stop_slam') as stop,patch.object(node,'start_localization') as start, \
                         patch.object(node,'finalize_targets'), \
                         patch('cooperative_rescue.subprocess.run',return_value=SimpleNamespace(returncode=0,stdout='saved')):
                        node.handover()
                    self.assertFalse((d/'rescue-result.json').exists())
                    self.assertEqual(node.mapping_start_pose,reference['mapping_start_pose'])
                    self.assertAlmostEqual(start.call_args.args[1][0],.8)
                    self.assertAlmostEqual(start.call_args.args[1][1],.1)
                    stop.assert_called_once()
                finally:node.destroy_node()
        finally:rclpy.shutdown()

    def test_invalid_live_target_blocks_completion_and_recovers_on_valid_update(self):
        rclpy.init()
        try:
            with tempfile.TemporaryDirectory() as directory:
                config=yaml.safe_load((ROOT/'src/edgeai_navigation/config/cooperative_rescue.yaml').read_text())
                node=CooperativeRescue(directory,config)
                try:
                    node.target=dict(id=2,x=3.,y=3.,yaw=0.)
                    path=Path(directory)/'targets.json'
                    path.write_text(json.dumps(dict(frame_id='odom',targets=[])))
                    self.assertFalse(node.refresh_target(force=True));self.assertFalse(node.target_ready)
                    self.assertEqual(node.target['x'],3.)
                    target=dict(id=2,x=3.2,y=3.,yaw=0.,confirmed=True,last_observed_ns=1)
                    path.write_text(json.dumps(dict(frame_id='map',targets=[target])))
                    self.assertTrue(node.refresh_target(force=True));self.assertTrue(node.target_ready)
                    self.assertEqual(node.target['x'],3.2)
                finally:node.destroy_node()
        finally:rclpy.shutdown()

    def test_republished_old_observation_cannot_move_fixed_map_target(self):
        rclpy.init()
        try:
            with tempfile.TemporaryDirectory() as directory:
                config=yaml.safe_load((ROOT/'src/edgeai_navigation/config/cooperative_rescue.yaml').read_text())
                node=CooperativeRescue(directory,config)
                try:
                    node.target=dict(id=2,x=3.,y=3.,yaw=0.,last_observed_ns=73)
                    node.target_observation_floor=105
                    path=Path(directory)/'targets.json'
                    target=dict(id=2,x=3.2,y=3.,yaw=0.,confirmed=True,last_observed_ns=73)
                    path.write_text(json.dumps(dict(frame_id='map',stamp_ns=110,targets=[target])))
                    self.assertFalse(node.refresh_target(force=True))
                    self.assertEqual(node.target['x'],3.)
                    target['last_observed_ns']=111
                    path.write_text(json.dumps(dict(frame_id='map',stamp_ns=112,targets=[target])))
                    self.assertTrue(node.refresh_target(force=True))
                    self.assertEqual(node.target['x'],3.2)
                finally:node.destroy_node()
        finally:rclpy.shutdown()

    def test_manual_acquisition_waits_for_delayed_initial_state_and_exact_ack(self):
        rclpy.init();peer=Node('delayed_manual_peer');sequence=[0];requests=[]
        publisher=peer.create_publisher(String,'/robot2/navigation_authority/state',
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        ready_at=time.monotonic()+.2;publish_after=[ready_at]
        def request(_,response):
            requests.append(sequence[0]);sequence[0]+=1
            publish_after[0]=time.monotonic()+.2
            response.success=True;return response
        peer.create_service(SetBool,'/robot2/navigation_authority/set_manual',request)
        def publish():
            if time.monotonic()>=publish_after[0]:
                publisher.publish(String(data=json.dumps(dict(command_sequence=sequence[0],mode='MANUAL',phase='IDLE'))))
        peer.create_timer(.02,publish)
        executor=SingleThreadedExecutor();executor.add_node(peer)
        thread=threading.Thread(target=executor.spin);thread.start()
        try:
            with tempfile.TemporaryDirectory() as directory:
                config=yaml.safe_load((ROOT/'src/edgeai_navigation/config/cooperative_rescue.yaml').read_text())
                node=CooperativeRescue(directory,config);node.seed_enabled=False
                try:
                    node.acquire_manual('robot2')
                    self.assertEqual(requests,[0]);self.assertEqual(node.sequences['robot2'],1)
                    until=time.monotonic()+.3
                    node.wait(lambda:time.monotonic()>=until,1.,True)
                    sequence[0]=2
                    with self.assertRaisesRegex(RuntimeError,'expected sequence=1, observed=2'):
                        node.wait(lambda:False,1.,True)
                finally:node.destroy_node()
        finally:
            executor.shutdown();thread.join(timeout=5.);peer.destroy_node();rclpy.shutdown()

    def test_waiting_robot_prior_does_not_follow_robot1_odometry_corrections(self):
        rclpy.init()
        try:
            with tempfile.TemporaryDirectory() as directory:
                config=yaml.safe_load((ROOT/'src/edgeai_navigation/config/cooperative_rescue.yaml').read_text())
                node=CooperativeRescue(directory,config)
                try:
                    node.initial_odom={'':[0.,0.,0.],'robot2':[0.,0.,0.]}
                    tf=TransformStamped();tf.header.frame_id='map';tf.child_frame_id='odom'
                    tf.header.stamp=node.get_clock().now().to_msg();tf.transform.rotation.w=1.
                    node.buffer.set_transform(tf,'test');node.publish_initial_estimate()
                    original=list(node.mapping_start_pose)
                    tf.header.stamp=node.get_clock().now().to_msg();tf.transform.translation.x=1.
                    tf.transform.rotation.z=math.sin(.3/2);tf.transform.rotation.w=math.cos(.3/2)
                    node.buffer.set_transform(tf,'test');node.publish_initial_estimate()
                    self.assertEqual(node.mapping_start_pose,original)
                    self.assertAlmostEqual(original[0],0.)
                finally:node.destroy_node()
        finally:rclpy.shutdown()

    def test_active_slam_stops_with_real_lifecycle_services(self):
        rclpy.init();slam=LifecycleNode('handover_test_slam')
        slam.trigger_configure();slam.trigger_activate()
        executor=SingleThreadedExecutor();executor.add_node(slam)
        thread=threading.Thread(target=executor.spin);thread.start()
        try:
            with tempfile.TemporaryDirectory() as directory:
                config=yaml.safe_load((ROOT/'src/edgeai_navigation/config/cooperative_rescue.yaml').read_text())
                node=CooperativeRescue(directory,config)
                try:
                    node.stop_slam('/handover_test_slam')
                    node.stop_slam('/handover_test_slam')  # Already finalized: no invalid transition.
                    events=[json.loads(s) for s in (Path(directory)/'cooperative-events.jsonl').read_text().splitlines()]
                    self.assertEqual([e['transition'] for e in events if e['phase']=='SLAM_TRANSITION'],[4,2,5])
                    self.assertEqual(events[-1]['phase'],'SLAM_STOPPED')
                finally:node.destroy_node()
        finally:
            executor.shutdown();thread.join(timeout=5.);slam.destroy_node();rclpy.shutdown()

    def test_rejected_transition_reports_node_and_transition(self):
        class RefusesDeactivate(LifecycleNode):
            def on_deactivate(self,state):return TransitionCallbackReturn.FAILURE
        rclpy.init();slam=RefusesDeactivate('handover_test_refuses')
        slam.trigger_configure();slam.trigger_activate()
        executor=SingleThreadedExecutor();executor.add_node(slam)
        thread=threading.Thread(target=executor.spin);thread.start()
        try:
            with tempfile.TemporaryDirectory() as directory:
                config=yaml.safe_load((ROOT/'src/edgeai_navigation/config/cooperative_rescue.yaml').read_text())
                node=CooperativeRescue(directory,config)
                try:
                    with self.assertRaisesRegex(RuntimeError,'/handover_test_refuses/change_state; transition=4'):
                        node.stop_slam('/handover_test_refuses')
                finally:node.destroy_node()
        finally:
            executor.shutdown();thread.join(timeout=5.);slam.destroy_node();rclpy.shutdown()

    def test_two_distinct_goals_then_common_dwell(self):
        rclpy.init();fleet=FakeFleet();executor=SingleThreadedExecutor();executor.add_node(fleet)
        thread=threading.Thread(target=executor.spin);thread.start()
        try:
            with tempfile.TemporaryDirectory() as directory:
                config=yaml.safe_load((ROOT/'src/edgeai_navigation/config/cooperative_rescue.yaml').read_text())
                config.update(dwell_s=.2,localization_stable_s=.1)
                nav=yaml.safe_load((ROOT/'src/edgeai_navigation/config/nav2_exploration_rectangle.yaml').read_text())
                (Path(directory)/'nav2-runtime.yaml').write_text(yaml.safe_dump(nav))
                node=CooperativeRescue(directory,config);node.seed_enabled=False
                def handover():
                    node.deadline=time.monotonic()+20.
                    node.wait(lambda:len(node.localization)==2 and len(node.states)==2 and node.map is not None,5.)
                    node.sequences={'':2,'robot2':2}
                    node.targets=dict(targets=[dict(id=2,x=3.,y=3.,yaw=math.pi/2)])
                try:
                    # Isolate map matching/handover hardware; exercise real ROS action and TF sequencing.
                    with patch.object(node,'handover',handover),patch.object(node,'wait_navigation_active'),patch.object(node,'laser_match',return_value=1.):
                        result=node.execute()
                    self.assertTrue(result['success']);self.assertEqual(fleet.calls,['','robot2'])
                    self.assertNotEqual(result['goals'][0],result['goals'][1])
                    self.assertTrue(all(d<=.7 for d in result['distances_m']))
                finally:node.destroy_node()
        finally:
            executor.shutdown();thread.join(timeout=5.);fleet.destroy_node();rclpy.shutdown()

    def test_live_target_shift_after_arrival_reselects_parking_before_success(self):
        rclpy.init();fleet=FakeFleet();executor=SingleThreadedExecutor();executor.add_node(fleet)
        thread=threading.Thread(target=executor.spin);thread.start()
        try:
            with tempfile.TemporaryDirectory() as directory:
                config=yaml.safe_load((ROOT/'src/edgeai_navigation/config/cooperative_rescue.yaml').read_text())
                config.update(dwell_s=.2,localization_stable_s=.1)
                nav=yaml.safe_load((ROOT/'src/edgeai_navigation/config/nav2_exploration_rectangle.yaml').read_text())
                (Path(directory)/'nav2-runtime.yaml').write_text(yaml.safe_dump(nav))
                node=CooperativeRescue(directory,config);node.seed_enabled=False
                def handover():
                    node.deadline=time.monotonic()+20.
                    node.wait(lambda:len(node.localization)==2 and len(node.states)==2 and node.map is not None,5.)
                    node.sequences={'':2,'robot2':2}
                    node.targets=dict(targets=[dict(id=2,x=3.,y=3.,yaw=math.pi/2)])
                def shift_target(ns):
                    if len(fleet.calls)==2:
                        (Path(directory)/'targets.json').write_text(json.dumps(dict(frame_id='map',stamp_ns=123,
                            targets=[dict(id=2,x=4.,y=3.,yaw=math.pi/2,confirmed=True,last_observed_ns=123)])))
                fleet.on_arrival=shift_target
                try:
                    with patch.object(node,'handover',handover),patch.object(node,'wait_navigation_active'),patch.object(node,'laser_match',return_value=1.):
                        result=node.execute()
                    self.assertTrue(result['success']);self.assertEqual(len(fleet.calls),4)
                    self.assertTrue(all(math.dist(p[:2],[4.,3.])<=.7 for p in fleet.poses.values()))
                    events=[json.loads(line) for line in (Path(directory)/'cooperative-events.jsonl').read_text().splitlines()]
                    self.assertTrue(any(e['phase']=='TARGET_UPDATED' for e in events))
                    self.assertEqual(sum(e['phase']=='RESCUE_REPAIR' for e in events),1)
                    self.assertTrue(any('Outside rescue circle' in e.get('reasons',[]) for e in events))
                finally:node.destroy_node()
        finally:
            executor.shutdown();thread.join(timeout=5.);fleet.destroy_node();rclpy.shutdown()


if __name__=='__main__':unittest.main()
