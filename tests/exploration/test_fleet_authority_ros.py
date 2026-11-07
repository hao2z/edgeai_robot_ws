"""Exercise robot2 authority handover without exposing a root navigation server."""
from pathlib import Path
import sys
import tempfile
import time
import unittest
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import SingleThreadedExecutor
from nav2_msgs.action import NavigateToPose

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'src/edgeai_navigation/scripts'))
from navigation_authority import NavigationAuthority


class FleetAuthority(unittest.TestCase):
    def test_robot2_goal_reaches_only_robot2_backend(self):
        rclpy.init(args=['--ros-args','-r','__ns:=/robot2','-p','allow_auto:=false'])
        with tempfile.TemporaryDirectory() as directory:
            authority=NavigationAuthority(directory,mode='MANUAL',tree='')
            backend=Node('fleet_test_backend');client_node=Node('fleet_test_client',use_global_arguments=False)
            calls=[]
            def execute(handle):
                calls.append(handle.request.pose.pose.position.x)
                handle.succeed();return NavigateToPose.Result()
            server=ActionServer(backend,NavigateToPose,'navigation_backend/navigate_to_pose',execute_callback=execute)
            robot2=ActionClient(client_node,NavigateToPose,'/robot2/navigation/rescue_to_pose')
            root=ActionClient(client_node,NavigateToPose,'/navigation/rescue_to_pose')
            executor=SingleThreadedExecutor()
            for node in (authority,backend,client_node):executor.add_node(node)
            def until(predicate):
                end=time.monotonic()+8.
                while not predicate() and time.monotonic()<end:executor.spin_once(timeout_sec=.02)
                self.assertTrue(predicate())
            try:
                until(lambda:robot2.server_is_ready() and authority.backend.server_is_ready())
                self.assertFalse(root.server_is_ready())
                request=NavigateToPose.Goal();request.pose.header.frame_id='map'
                request.pose.pose.position.x=2.;request.pose.pose.orientation.w=1.
                sent=robot2.send_goal_async(request);until(sent.done)
                self.assertTrue(sent.result().accepted)
                result=sent.result().get_result_async();until(result.done)
                self.assertEqual(result.result().status,4)
                self.assertEqual(calls,[2.])
                self.assertFalse(authority.explorer_sync)
            finally:
                executor.shutdown();server.destroy()
                for node in (authority,backend,client_node):node.destroy_node()
                authority.audit.close();rclpy.shutdown()


if __name__=='__main__':unittest.main()
