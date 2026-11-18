"""Exercise split adapter nodes and freeze handover on an isolated ROS graph."""
import importlib.util
import json
import os
from pathlib import Path
import time
import unittest
from types import SimpleNamespace
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String, Int32

ROOT=Path(__file__).resolve().parents[2]


def load(name,path):
    spec=importlib.util.spec_from_file_location(name,ROOT/path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


class JointAdaptersROS(unittest.TestCase):
    def test_peer_obstacles_require_active_fresh_mapping_and_stop_when_frozen(self):
        if os.environ.get('ROS_DOMAIN_ID','0') in ('0','86'):
            self.skipTest('Use an isolated ROS domain')
        rclpy.init()
        module=load('peer_adapter','src/edgeai_navigation/scripts/joint_peer_poses.py')
        node=module.JointPeerPoses();reader=rclpy.create_node('peer_obstacle_reader')
        received={r:[] for r in ('robot1','robot2')}
        subs=[reader.create_subscription(PoseStamped,'/cooperative_rescue/peer_pose/'+r,
            lambda msg,key=r:received[key].append(msg),1) for r in received]
        def lookup(*args):
            t=TransformStamped();t.header.frame_id='map';t.header.stamp=node.get_clock().now().to_msg()
            t.transform.rotation.w=1.;return t
        node.buffer=SimpleNamespace(lookup_transform=lookup)
        def pump():
            until=time.monotonic()+.3
            while time.monotonic()<until:
                rclpy.spin_once(reader,timeout_sec=.02)
        try:
            pump();node.update();pump()
            self.assertFalse(any(received.values()))
            node.status(String(data=json.dumps(dict(active=True,alignment_accepted=True))))
            for _ in range(3):node.update();pump()
            self.assertTrue(all(received.values()))
            before={r:len(v) for r,v in received.items()}
            node.status(String(data=json.dumps(dict(active=True,alignment_accepted=True,frozen=True))))
            node.update();pump()
            self.assertEqual(before,{r:len(v) for r,v in received.items()})
            node.status(String(data=json.dumps(dict(active=True,alignment_accepted=True,frozen=True,finalized=True))))
            node.update();pump()
            self.assertTrue(all(len(received[r])>before[r] for r in received))
            before={r:len(v) for r,v in received.items()}
            node.status(String(data=json.dumps(dict(active=True,alignment_accepted=True))))
            stale=TransformStamped();stale.header.stamp.sec=1
            node.buffer=SimpleNamespace(lookup_transform=lambda *args:stale)
            node.update();pump()
            self.assertEqual(before,{r:len(v) for r,v in received.items()})
        finally:
            node.destroy_node();reader.destroy_node();rclpy.shutdown()

    def test_frontier_monitor_starts_unavailable_and_preserves_negative(self):
        rclpy.init()
        module=load('frontier_adapter','src/edgeai_mission/scripts/joint_frontier_monitor.py')
        node=module.JointFrontierMonitor()
        try:
            node.update()
            self.assertEqual(module.aggregate(node.counts,node.received,time.monotonic(),node.max_age),-1)
            node.receive('robot1',Int32(data=0));node.receive('robot2',Int32(data=-1))
            self.assertEqual(module.aggregate(node.counts,node.received,time.monotonic(),node.max_age),-1)
            node.receive('robot2',Int32(data=2))
            self.assertEqual(module.aggregate(node.counts,node.received,time.monotonic(),node.max_age),2)
        finally:
            node.destroy_node();rclpy.shutdown()


if __name__=='__main__':unittest.main()
