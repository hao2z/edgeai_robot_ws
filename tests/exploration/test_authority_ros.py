"""Real ROS action handshakes against a controllable backend, no simulator."""
from pathlib import Path
import sys
import tempfile
import time
import unittest
import xml.etree.ElementTree as ET
from types import SimpleNamespace

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.task import Future
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool, Trigger
from action_msgs.msg import GoalStatus,GoalStatusArray
from unique_identifier_msgs.msg import UUID

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'src/edgeai_navigation/scripts'))
from navigation_authority import NavigationAuthority


class ActionHandshakeTest(unittest.TestCase):
    def test_yield_admission_never_preempts_or_gets_replaced(self):
        from rclpy.action import GoalResponse
        rclpy.init()
        with tempfile.TemporaryDirectory() as directory:
            authority=NavigationAuthority(directory,mode='MANUAL')
            authority.backend.server_is_ready=lambda:True
            try:
                request=NavigateToPose.Goal()
                self.assertEqual(authority.admit(request,'MANUAL'),GoalResponse.ACCEPT)
                self.assertEqual(authority.admit(NavigateToPose.Goal(),'MANUAL'),GoalResponse.REJECT)
            finally:
                authority.audit.close();authority.destroy_node();rclpy.shutdown()

    def test_terminal_status_requires_matching_uuid_and_does_not_release_slot(self):
        rclpy.init()
        with tempfile.TemporaryDirectory() as directory:
            authority=NavigationAuthority(directory, mode='MANUAL')
            try:
                token=authority.policy.reserve('MANUAL');authority.policy.accepted(token)
                expected=UUID(uuid=[7]*16)
                authority.current={'backend':SimpleNamespace(goal_id=expected),'terminal_observed':None}
                status=GoalStatus(status=4);status.goal_info.goal_id=UUID(uuid=[8]*16)
                authority.terminal_status(GoalStatusArray(status_list=[status]))
                self.assertIsNone(authority.current['terminal_observed'])
                status.goal_info.goal_id=expected
                authority.terminal_status(GoalStatusArray(status_list=[status]))
                self.assertEqual(authority.current['terminal_observed'][0],4)
                self.assertEqual(authority.policy.phase,'ACTIVE')
            finally:
                authority.audit.close();authority.destroy_node();rclpy.shutdown()


    def test_owner_cancels_and_waits_for_backend_terminal_before_dispatch(self):
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
            tree=Path(__file__).resolve().parents[2]/'src/edgeai_navigation/config/behavior_trees/explore_recovery.xml'
            authority = NavigationAuthority(directory, mode='MANUAL', tree=str(tree))
            client_node = Node('replacement_owner')
            dispatched = []
            client_node.create_subscription(PoseStamped, '/navigation_authority/dispatched_goal',
                dispatched.append, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            client = ActionClient(client_node, NavigateToPose, '/navigation/assigned_frontier_to_pose')
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
                request = NavigateToPose.Goal()
                request.pose.header.frame_id = 'map'
                request.pose.pose.position.x, request.pose.pose.position.y = 3.594, -.518
                request.pose.pose.orientation.z = .3826834323650898
                request.pose.pose.orientation.w = .9238795325112867
                first = result(client.send_goal_async(request))
                self.assertTrue(first.accepted)
                first_result = first.get_result_async()
                until(lambda: len(executions) == 1 and authority.policy.phase == 'ACTIVE')
                until(lambda: bool(dispatched))
                self.assertEqual(dispatched[-1], executions[0].request.pose)
                self.assertEqual(executions[0].request.pose,request.pose)
                frontier_tree=ET.parse(executions[0].request.behavior_tree)
                self.assertEqual(frontier_tree.find('.//ComputePathToPose').get('planner_id'),'GridBasedFrontier')
                self.assertEqual(frontier_tree.find('.//FollowPath').get('controller_id'),'FollowPathFrontier')
                self.assertEqual(frontier_tree.find('.//FollowPath').get('goal_checker_id'),'frontier_goal_checker')
                self.assertFalse(result(client.send_goal_async(request)).accepted)
                self.assertEqual(len(result(first.cancel_goal_async()).goals_canceling), 1)
                until(lambda: executions[0].is_cancel_requested)
                self.assertFalse(first_result.done())
                self.assertFalse(result(client.send_goal_async(request)).accepted)
                pending[0].set_result(True)
                self.assertEqual(result(first_result).status, 5)
                until(lambda: authority.policy.phase == 'IDLE')

                ordinary=ActionClient(client_node,NavigateToPose,'/navigate_to_pose')
                heading=ActionClient(client_node,NavigateToPose,'/navigation/heading_to_pose')
                rescue=ActionClient(client_node,NavigateToPose,'/navigation/rescue_to_pose')
                until(lambda: ordinary.server_is_ready() and heading.server_is_ready()
                      and rescue.server_is_ready())
                policies=[(ordinary,'general_goal_checker','GridBased'),
                          (heading,'heading_goal_checker','GridBasedHeading'),
                          (rescue,'rescue_goal_checker','GridBasedRescue'),
                          (ordinary,'general_goal_checker','GridBased')]
                for expected,(route,checker,planner) in enumerate(policies,2):
                    handle=result(route.send_goal_async(request))
                    self.assertTrue(handle.accepted)
                    done=handle.get_result_async()
                    until(lambda:len(executions)==expected and authority.policy.phase=='ACTIVE')
                    bound=ET.parse(executions[-1].request.behavior_tree)
                    self.assertEqual(bound.find('.//FollowPath').get('goal_checker_id'),checker)
                    self.assertEqual(bound.find('.//ComputePathToPose').get('planner_id'),planner)
                    pending[-1].set_result(True)
                    self.assertEqual(result(done).status,4)
                    until(lambda:authority.policy.phase=='IDLE')
                # RViz can select a per-point yaw rule through the new preference.
                preference=client_node.create_client(SetBool,'/navigation_authority/set_goal_heading')
                manual_pose=client_node.create_publisher(PoseStamped,'/goal_pose',1)
                until(lambda:preference.service_is_ready() and manual_pose.get_subscription_count()>0)
                for expected,require_heading,controller in [(6,True,'FollowPathHeading'),(7,False,'FollowPath')]:
                    response=result(preference.call_async(SetBool.Request(data=require_heading)))
                    self.assertTrue(response.success)
                    manual_pose.publish(request.pose)
                    until(lambda:len(executions)==expected)
                    bound=ET.parse(executions[-1].request.behavior_tree)
                    self.assertEqual(bound.find('.//FollowPath').get('controller_id'),controller)
                    pending[-1].set_result(True)
                    until(lambda:authority.policy.phase=='IDLE')
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
            authority = NavigationAuthority(directory, mode='MANUAL')
            client_node = Node('test_owner')
            automatic = ActionClient(client_node, NavigateToPose, '/navigation/assigned_frontier_to_pose')
            manual = ActionClient(client_node, NavigateToPose, '/navigate_to_pose')
            control = client_node.create_client(SetBool, '/navigation_authority/set_manual')
            automatic_handle = []

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
                self.assertTrue(result(control.call_async(SetBool.Request(data=False))).success)
                until(lambda: executions[0].is_cancel_requested)
                self.assertFalse(result(manual.send_goal_async(NavigateToPose.Goal())).accepted)
                self.assertEqual(len(executions), 1)
                self.assertFalse(first_result.done())
                pending[0].set_result(True)
                self.assertEqual(result(first_result).status, 6)
                until(lambda: authority.policy.phase == 'IDLE')
                self.assertTrue(result(control.call_async(SetBool.Request(data=True))).success)
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
