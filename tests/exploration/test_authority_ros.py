"""Real ROS action handshakes against a controllable backend, no simulator."""
from pathlib import Path
import sys
import tempfile
import time
import unittest

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool
from frontier_exploration_ros2.srv import ControlExploration

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'src/edgeai_navigation/scripts'))
from navigation_authority import NavigationAuthority


class ActionHandshakeTest(unittest.TestCase):
    def test_same_owner_replacement_waits_for_backend_terminal(self):
        rclpy.init()
        with tempfile.TemporaryDirectory() as directory:
            backend_node = Node('replacement_backend')
            group = ReentrantCallbackGroup()
            pending, executions = [], []

            async def execute(handle):
                done = Future()
                pending.append(done)
                executions.append(handle)
                await done
                if handle.is_cancel_requested:
                    handle.canceled()
                else:
                    handle.succeed()
                return NavigateToPose.Result()

            server = ActionServer(backend_node, NavigateToPose,
                '/navigation_backend/navigate_to_pose', execute_callback=execute,
                cancel_callback=lambda handle: CancelResponse.ACCEPT, callback_group=group)
            authority = NavigationAuthority(directory)
            client_node = Node('replacement_owner')
            client = ActionClient(client_node, NavigateToPose, '/exploration/navigate_to_pose')
            executor = SingleThreadedExecutor()
            for node in (backend_node, authority, client_node):
                executor.add_node(node)

            def until(predicate):
                deadline = time.monotonic()+8
                while not predicate() and time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=.02)
                self.assertTrue(predicate(), 'Timed out waiting for replacement protocol')

            def result(future):
                until(future.done)
                return future.result()

            try:
                until(lambda: authority.backend.server_is_ready() and client.server_is_ready())
                first = result(client.send_goal_async(NavigateToPose.Goal()))
                self.assertTrue(first.accepted)
                first_result = first.get_result_async()
                until(lambda: len(executions) == 1 and authority.policy.phase == 'ACTIVE')
                second = result(client.send_goal_async(NavigateToPose.Goal()))
                self.assertTrue(second.accepted)
                second_result = second.get_result_async()
                until(lambda: executions[0].is_cancel_requested)
                # Cancel acknowledgement is insufficient: backend has not ended yet.
                self.assertEqual(len(executions), 1)
                self.assertFalse(second_result.done())
                pending[0].set_result(True)
                result(first_result)
                until(lambda: len(executions) == 2 and authority.policy.phase == 'ACTIVE')
                self.assertFalse(executions[1].is_cancel_requested)
                pending[1].set_result(True)
                self.assertEqual(result(second_result).status, 4)
                until(lambda: authority.policy.phase == 'IDLE')
                # A canceled queued goal must never reach the backend.
                third = result(client.send_goal_async(NavigateToPose.Goal()))
                third_result = third.get_result_async()
                until(lambda: len(executions) == 3 and authority.policy.phase == 'ACTIVE')
                fourth = result(client.send_goal_async(NavigateToPose.Goal()))
                self.assertTrue(fourth.accepted)
                fourth_result = fourth.get_result_async()
                self.assertEqual(len(result(fourth.cancel_goal_async()).goals_canceling), 1)
                self.assertEqual(result(fourth_result).status, 5)
                pending[2].set_result(True)
                result(third_result)
                until(lambda: authority.policy.phase == 'IDLE')
                self.assertEqual(len(executions), 3)
            finally:
                executor.shutdown()
                server.destroy()
                authority.audit.close()
                for node in (client_node, authority, backend_node):
                    node.destroy_node()
                rclpy.shutdown()

    def test_takeover_and_uncertain_cancel_block_replacement(self):
        rclpy.init()
        with tempfile.TemporaryDirectory() as directory:
            backend_node = Node('test_backend')
            group = ReentrantCallbackGroup()
            pending, executions = [], []

            async def execute(handle):
                done = Future()
                pending.append(done)
                executions.append(handle)
                await done
                if handle.is_cancel_requested:
                    handle.canceled()
                else:
                    handle.succeed()
                return NavigateToPose.Result()

            server = ActionServer(backend_node, NavigateToPose, '/navigation_backend/navigate_to_pose',
                execute_callback=execute, cancel_callback=lambda handle: CancelResponse.ACCEPT,
                callback_group=group)
            authority = NavigationAuthority(directory)
            client_node = Node('test_owner')
            automatic = ActionClient(client_node, NavigateToPose, '/exploration/navigate_to_pose')
            manual = ActionClient(client_node, NavigateToPose, '/navigate_to_pose')
            control = client_node.create_client(SetBool, '/navigation_authority/set_manual')
            automatic_handle, pause_calls = [], []

            def explorer_control(request, response):
                if request.action == 2:
                    pause_calls.append(time.monotonic())
                    if automatic_handle:
                        automatic_handle[-1].cancel_goal_async()
                response.accepted = True
                return response

            explorer_service = client_node.create_service(ControlExploration,
                '/exploration/control_exploration', explorer_control)
            executor = SingleThreadedExecutor()
            for node in (backend_node, authority, client_node):
                executor.add_node(node)

            def until(predicate):
                deadline = time.monotonic()+8
                while not predicate() and time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=.02)
                self.assertTrue(predicate(), 'Timed out waiting for asynchronous protocol')

            def result(future):
                until(future.done)
                return future.result()

            try:
                until(lambda: authority.backend.server_is_ready() and automatic.server_is_ready()
                      and manual.server_is_ready() and control.service_is_ready())
                first = result(automatic.send_goal_async(NavigateToPose.Goal()))
                automatic_handle.append(first)
                self.assertTrue(first.accepted)
                first_result = first.get_result_async()
                until(lambda: len(pending) == 1 and authority.policy.phase == 'ACTIVE')
                self.assertFalse(result(manual.send_goal_async(NavigateToPose.Goal())).accepted)
                self.assertTrue(result(control.call_async(SetBool.Request(data=True))).success)
                until(lambda: executions[0].is_cancel_requested)
                self.assertTrue(pause_calls)
                self.assertFalse(result(manual.send_goal_async(NavigateToPose.Goal())).accepted)
                self.assertEqual(len(executions), 1)
                self.assertFalse(first_result.done())
                pending[0].set_result(True)
                self.assertEqual(result(first_result).status, 5)
                until(lambda: authority.policy.phase == 'IDLE')
                odom = Odometry()
                now = authority.get_clock().now().nanoseconds
                odom.header.stamp.sec = now//10**9-2
                authority.odometry(odom)
                self.assertIsNone(authority.fresh_pose())
                odom.header.stamp = authority.get_clock().now().to_msg()
                authority.odometry(odom)
                self.assertIsNotNone(authority.fresh_pose())
                second = result(manual.send_goal_async(NavigateToPose.Goal()))
                self.assertTrue(second.accepted)
                second_result = second.get_result_async()
                until(lambda: len(pending) == 2 and authority.policy.phase == 'ACTIVE')
                cancel = result(second.cancel_goal_async())
                self.assertEqual(len(cancel.goals_canceling), 1)
                until(lambda: executions[1].is_cancel_requested)
                authority.current['cancel_at'] = time.monotonic()-6
                authority.tick()
                self.assertEqual(authority.policy.phase, 'UNKNOWN')
                self.assertFalse(result(manual.send_goal_async(NavigateToPose.Goal())).accepted)
                self.assertEqual(len(executions), 2)
                pending[1].set_result(True)
                self.assertEqual(result(second_result).status, 5)
                until(lambda: authority.policy.phase == 'IDLE')
            finally:
                executor.shutdown()
                server.destroy()
                authority.audit.close()
                for node in (client_node, authority, backend_node):
                    node.destroy_node()
                rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
