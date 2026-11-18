"""Fleet data delivery must survive legacy nested waits and service pumping."""
from pathlib import Path
import sys
import time
import unittest
from unittest.mock import Mock, patch
from types import SimpleNamespace

import rclpy
from std_msgs.msg import String

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/edgeai_mission/scripts'))
from task_units import drain_data_callbacks


class FleetCallbacks(unittest.TestCase):
    def test_zero_timeout_after_ready_batch_does_not_leave_next_batch_queued(self):
        calls=[]
        def callback():calls.append('data')
        callback.exception=lambda:None
        callback.result=lambda:None
        timeout=rclpy.executors.TimeoutException
        executor=SimpleNamespace(wait_for_ready_callbacks=Mock(
            side_effect=[(callback,None,None),timeout(),(callback,None,None),timeout(),timeout()]))
        with patch('task_units.time.monotonic',return_value=0.):
            drain_data_callbacks(executor,32)
        self.assertEqual(calls,['data','data'])
        self.assertEqual(executor.wait_for_ready_callbacks.call_count,5)

    def test_both_robot_streams_survive_nested_waits(self):
        rclpy.init(domain_id=167)
        nodes=[rclpy.create_node('fleet_callback_test_'+str(i)) for i in range(2)]
        executor=rclpy.get_global_executor()
        received=set()
        subscriptions=[]
        publishers=[]
        try:
            for i,node in enumerate(nodes):
                executor.add_node(node)
                topic='/callback_test/robot'+str(i)
                subscriptions.append(node.create_subscription(String,topic,lambda m:received.add(m.data),1))
                publishers.append(node.create_publisher(String,topic,1))
            deadline=time.monotonic()+5.
            while received!={'0','1'} and time.monotonic()<deadline:
                for i,publisher in enumerate(publishers):publisher.publish(String(data=str(i)))
                # Same nested-spin contract used by the existing algorithms.
                rclpy.spin_once(nodes[0],timeout_sec=.01)
                drain_data_callbacks(executor,64)
            self.assertEqual(received,{'0','1'})
            self.assertEqual(set(executor.get_nodes()),set(nodes))
        finally:
            for node in nodes:node.destroy_node()
            rclpy.shutdown()
