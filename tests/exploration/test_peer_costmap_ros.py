"""Real Nav2 costmap and path validation as a cooperative peer moves."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

import rclpy
import yaml
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, TransformStamped
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import GetCostmap, IsPathValid
from tf2_ros import StaticTransformBroadcaster

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/edgeai_navigation/scripts'))
from fleet_config import configure_peer_costmaps


class PeerCostmapROS(unittest.TestCase):
    def test_old_path_invalidates_and_peer_costs_move_and_expire(self):
        rclpy.init();node=rclpy.create_node('peer_costmap_test')
        publisher=node.create_publisher(PoseStamped,'/cooperative_rescue/peer_pose/robot2',1)
        tf=StaticTransformBroadcaster(node)
        transform=TransformStamped();transform.header.frame_id='map';transform.child_frame_id='base_footprint'
        transform.transform.translation.x=.5;transform.transform.translation.y=2.
        transform.transform.rotation.w=1.;tf.sendTransform(transform)
        process=None

        def pose(x,y):
            p=PoseStamped();p.header.frame_id='map';p.header.stamp=node.get_clock().now().to_msg()
            p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.w=1.;return p

        def wait(predicate,timeout=8.):
            end=time.monotonic()+timeout
            while time.monotonic()<end:
                rclpy.spin_once(node,timeout_sec=.02)
                if predicate():return
            self.fail('ROS wait timed out')

        def call(client,request):
            future=client.call_async(request);wait(future.done);return future.result()

        try:
            with tempfile.TemporaryDirectory() as directory:
                path=Path(directory)/'nav.yaml'
                config=yaml.safe_load((ROOT/'src/edgeai_navigation/config/nav2_exploration_rectangle.yaml').read_text())
                cost=config['global_costmap']['global_costmap']['ros__parameters']
                cost.update(use_sim_time=False,global_frame='map',robot_base_frame='base_footprint',
                            plugins=['inflation_layer'],rolling_window=False,track_unknown_space=False,
                            width=5,height=5,origin_x=0.,origin_y=0.)
                configure_peer_costmaps(config,'',dict(max_age_s=.5))
                config['planner_server']['ros__parameters']=dict(use_sim_time=False,planner_plugins=['GridBased'],
                    GridBased=dict(plugin='nav2_navfn_planner::NavfnPlanner',tolerance=0.))
                path.write_text(yaml.safe_dump({k:config[k] for k in ('planner_server','global_costmap')}))
                process=subprocess.Popen(['ros2','run','nav2_planner','planner_server','--ros-args',
                    '--params-file',str(path)],start_new_session=True)
                lifecycle=node.create_client(ChangeState,'/planner_server/change_state');wait(lifecycle.service_is_ready)
                for transition in (1,3):
                    req=ChangeState.Request();req.transition.id=transition
                    self.assertTrue(call(lifecycle,req).success)
                validity=node.create_client(IsPathValid,'/is_path_valid');wait(validity.service_is_ready)
                costmap=node.create_client(GetCostmap,'/global_costmap/get_costmap');wait(costmap.service_is_ready)
                planner=ActionClient(node,ComputePathToPose,'/compute_path_to_pose');wait(planner.server_is_ready)
                request=IsPathValid.Request();request.path.header.frame_id='map'
                request.path.poses=[pose(.5+i*.05,2.) for i in range(61)]
                self.assertTrue(call(validity,request).is_valid)

                def at(x,y):
                    grid=call(costmap,GetCostmap.Request()).map
                    meta=grid.metadata
                    u=int((x-meta.origin.position.x)/meta.resolution)
                    v=int((y-meta.origin.position.y)/meta.resolution)
                    return grid.data[v*meta.size_x+u]

                wait(lambda:publisher.get_subscription_count()>0)
                def put(x,y):
                    publisher.publish(pose(x,y));return at(x,y)==254
                wait(lambda:put(2.,2.))
                self.assertFalse(call(validity,request).is_valid)
                timer=node.create_timer(.05,lambda:publisher.publish(pose(2.,2.)))
                try:
                    future=planner.send_goal_async(ComputePathToPose.Goal(goal=pose(3.5,2.),planner_id='GridBased'))
                    wait(future.done);self.assertTrue(future.result().accepted)
                    terminal=future.result().get_result_async();wait(terminal.done)
                    planned=terminal.result().result
                    self.assertEqual(planned.error_code,0)
                    self.assertGreater(max(abs(p.pose.position.y-2.) for p in planned.path.poses),.25)
                finally:node.destroy_timer(timer)
                wait(lambda:put(2.,3.5))
                self.assertEqual(at(2.,2.),0)
                self.assertTrue(call(validity,request).is_valid)
                wait(lambda:at(2.,3.5)==0)
        finally:
            if process and process.poll() is None:
                os.killpg(process.pid,signal.SIGTERM)
                try:process.wait(timeout=5.)
                except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
            node.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    if os.environ.get('ROS_DOMAIN_ID','0') in ('0','86'):raise SystemExit('Use an isolated test domain')
    unittest.main()
