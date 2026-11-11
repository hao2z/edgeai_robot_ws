"""Real frontier node: pause dispatch, detect new boundary, then dispatch on resume."""
import os
from pathlib import Path
import subprocess
import tempfile
import time
import unittest
import rclpy
from rclpy.action import ActionServer
from rclpy.qos import QoSProfile,DurabilityPolicy
from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Int32
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from frontier_exploration_ros2.srv import ControlExploration


class FrontierMonitorROS(unittest.TestCase):
    def test_paused_node_keeps_detection_and_dispatches_only_after_resume(self):
        rclpy.init();peer=rclpy.create_node('frontier_monitor_peer')
        maps=peer.create_publisher(OccupancyGrid,'/map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        costs=peer.create_publisher(OccupancyGrid,'/global_costmap/costmap',10)
        tf=StaticTransformBroadcaster(peer)
        transform=TransformStamped();transform.header.frame_id='map';transform.child_frame_id='base_footprint'
        transform.transform.rotation.w=1.;tf.sendTransform(transform)
        counts=[];goals=[];boundary=False
        subscription=peer.create_subscription(Int32,'/explore/available_frontiers',lambda m:counts.append(m.data),1)
        def execute(handle):goals.append(handle.request);handle.succeed();return NavigateToPose.Result()
        server=ActionServer(peer,NavigateToPose,'/monitor_test/navigate_to_pose',execute)
        client=peer.create_client(ControlExploration,'/control_exploration')
        def publish():
            grid=OccupancyGrid();grid.header.frame_id='map';grid.info.width=grid.info.height=60
            grid.info.resolution=.1;grid.info.origin.position.x=grid.info.origin.position.y=-2.
            grid.info.origin.orientation.w=1.
            grid.data=[-1 if boundary and x>=40 else 0 for y in range(60) for x in range(60)]
            maps.publish(grid);grid.data=[0]*3600;costs.publish(grid)
        timer=peer.create_timer(.05,publish)
        def wait(predicate,timeout=8.):
            end=time.monotonic()+timeout
            while time.monotonic()<end:
                rclpy.spin_once(peer,timeout_sec=.02)
                if predicate():return
            self.fail('Timed out waiting for explorer monitor')
        def control(action):
            wait(client.service_is_ready)
            req=ControlExploration.Request();req.action=action
            future=client.call_async(req);wait(future.done)
            self.assertTrue(future.result().accepted)
        process=None
        try:
            with tempfile.TemporaryDirectory() as directory:
                executable=Path(get_package_prefix('frontier_exploration_ros2'))/'lib/frontier_exploration_ros2/frontier_explorer'
                with (Path(directory)/'node.log').open('w+') as log:
                    snapshots=Path(directory)/'snapshots'
                    process=subprocess.Popen([str(executable),'--ros-args','-p','paused_map_processing_enabled:=true',
                        '-p','map_qos_autodetect_on_startup:=false','-p','map_processing_rate_hz:=2.0',
                        '-p','navigate_to_pose_action_name:=/monitor_test/navigate_to_pose'],stdout=log,stderr=log,
                        env=dict(os.environ,EDGEAI_FRONTIER_DIAGNOSTICS_DIR=str(snapshots)))
                    wait(lambda:bool(list(snapshots.glob('*/global_costmap.txt'))))
                    control(2)
                    wait(lambda:0 in counts)
                    self.assertFalse(goals)
                    boundary=True
                    wait(lambda:any(c>0 for c in counts))
                    self.assertFalse(goals,'Explorer dispatched while navigation was paused')
                    control(1)
                    wait(lambda:bool(goals))
                    process.terminate();process.wait(timeout=10.)
                    log.seek(0);text=log.read()
                    self.assertIn('FRONTIER_MONITOR',text)
                    self.assertIn('FRONTIER_EMPTY_DIAGNOSTICS',text)
        finally:
            if process and process.poll() is None:process.kill();process.wait()
            server.destroy();peer.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    if os.environ.get('ROS_DOMAIN_ID','0') in ('0','86'):raise SystemExit('Use an isolated test domain')
    unittest.main()
