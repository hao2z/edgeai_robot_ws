"""Quiesce must await fresh authority state, not merely a service response."""
import json
import os
from pathlib import Path
import sys
import threading
import time
import unittest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile,DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/edgeai_mission/scripts'))
from mission_stage import quiesce


class QuiesceROS(unittest.TestCase):
    def test_waits_for_stop_and_explorer_pause_acknowledgement(self):
        # quiesce owns rclpy's default context; the peer uses an independent context.
        context=rclpy.Context();rclpy.init(context=context)
        peer=rclpy.create_node('mission_quiesce_peer',context=context)
        pub=peer.create_publisher(String,'/navigation_authority/state',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        state=dict(command_sequence=7,mode='AUTO',phase='ACTIVE',explorer_paused=False)
        requested=[]
        def stop(_,response):requested.append(time.monotonic());response.success=True;return response
        peer.create_service(Trigger,'/navigation_authority/stop',stop)
        def publish():
            if requested and time.monotonic()-requested[0]>.25:
                state.update(command_sequence=8,mode='STOPPED',phase='IDLE',explorer_paused=True)
            pub.publish(String(data=json.dumps(state)))
        peer.create_timer(.02,publish)
        executor=SingleThreadedExecutor(context=context);executor.add_node(peer)
        thread=threading.Thread(target=executor.spin);thread.start()
        try:
            quiesce()
            self.assertEqual(len(requested),1)
            self.assertGreaterEqual(time.monotonic()-requested[0],.25)
            self.assertTrue(state['explorer_paused'])
        finally:
            executor.shutdown();thread.join(timeout=5.);peer.destroy_node();rclpy.shutdown(context=context)


if __name__=='__main__':
    if os.environ.get('ROS_DOMAIN_ID','0') in ('0','86'):raise SystemExit('Use an isolated test domain')
    unittest.main()
