"""Real Nav2 planner plugins on a static narrow slot; no Gazebo or motion."""
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import tempfile
import time
import unittest
import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile,DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import TransformStamped,PoseStamped
from lifecycle_msgs.srv import ChangeState
from tf2_ros import StaticTransformBroadcaster
from ament_index_python.packages import get_package_share_directory

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/edgeai_navigation/scripts'))
from navigation_config import load_navigation
from planner_profile import configure_lattice


class MultiHeadingPlannerROS(unittest.TestCase):
    def test_blocked_input_yaw_has_alternate_heading_and_stable_replan(self):
        rclpy.init();peer=rclpy.create_node('multi_heading_test_peer')
        publisher=peer.create_publisher(OccupancyGrid,'/map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        grid=OccupancyGrid();grid.header.frame_id='map';grid.info.width=grid.info.height=80
        grid.info.resolution=.05;grid.info.origin.orientation.w=1.;grid.data=[0]*6400
        for y in range(80):grid.data[y*80+34]=grid.data[y*80+46]=100
        peer.create_timer(.1,lambda:publisher.publish(grid))
        tf=StaticTransformBroadcaster(peer)
        transform=TransformStamped();transform.header.frame_id='map';transform.child_frame_id='base_footprint'
        transform.transform.translation.x=2.;transform.transform.translation.y=.6
        transform.transform.rotation.z=transform.transform.rotation.w=math.sqrt(.5);tf.sendTransform(transform)
        process=None;log=None
        def wait(predicate,timeout=20.):
            end=time.monotonic()+timeout
            while time.monotonic()<end:
                rclpy.spin_once(peer,timeout_sec=.02)
                if predicate():return
            if log:log.flush();log.seek(0);text=log.read()[-7000:]
            else:text=''
            self.fail('Planner test timed out: '+text)
        def pose(x,y,yaw):
            p=PoseStamped();p.header.frame_id='map';p.pose.position.x=x;p.pose.position.y=y
            p.pose.orientation.z=math.sin(yaw/2);p.pose.orientation.w=math.cos(yaw/2);return p
        try:
            with tempfile.TemporaryDirectory() as temporary:
                d=Path(temporary)
                config=load_navigation(ROOT/'src/edgeai_navigation/config','rpp')
                configure_lattice(config,ROOT/'src/edgeai_navigation/config',get_package_share_directory('nav2_smac_planner'))
                planners=config['planner_server']['ros__parameters']
                planners.update(use_sim_time=False,planner_plugins=['GridBased','GridBasedHeading'],expected_planner_frequency=.1)
                for name in ('GridBased','GridBasedHeading'):
                    planners[name].update(tolerance=0.,allow_unknown=False)
                cost=config['global_costmap']['global_costmap']['ros__parameters']
                cost.update(use_sim_time=False,global_frame='map',robot_base_frame='base_footprint',
                    plugins=['static_layer','inflation_layer'],track_unknown_space=True,update_frequency=20.,publish_frequency=10.)
                config_path=d/'planner.yaml'
                config_path.write_text(yaml.safe_dump({name:config[name] for name in ('planner_server','global_costmap')}))
                log=(d/'planner.log').open('w+')
                process=subprocess.Popen(['ros2','run','nav2_planner','planner_server','--ros-args',
                    '--params-file',str(config_path)],stdout=log,stderr=log,start_new_session=True)
                lifecycle=peer.create_client(ChangeState,'/planner_server/change_state')
                wait(lifecycle.service_is_ready)
                for transition in (1,3):
                    req=ChangeState.Request();req.transition.id=transition
                    future=lifecycle.call_async(req);wait(future.done)
                    self.assertTrue(future.result().success)
                action=ActionClient(peer,ComputePathToPose,'/compute_path_to_pose');wait(action.server_is_ready)
                def plan(plugin,x=2.):
                    req=ComputePathToPose.Goal(planner_id=plugin,use_start=True)
                    req.start=pose(x,.6,math.pi/2);req.goal=pose(x,2.,0.)
                    future=action.send_goal_async(req);wait(future.done)
                    self.assertTrue(future.result().accepted)
                    terminal=future.result().get_result_async();wait(terminal.done)
                    return terminal.result()
                fixed=plan('GridBasedHeading')
                self.assertEqual(fixed.status,6,'The fixed yaw must collide with the narrow slot')
                first=plan('GridBased')
                self.assertEqual(first.status,4);self.assertEqual(first.result.error_code,0)
                end=first.result.path.poses[-1].pose
                yaw=2*math.atan2(end.orientation.z,end.orientation.w)
                self.assertGreater(abs(math.remainder(yaw,2*math.pi)),.5)
                self.assertLess(math.hypot(end.position.x-2.,end.position.y-2.),.1)
                again=plan('GridBased');self.assertEqual(again.status,4)
                q=again.result.path.poses[-1].pose.orientation
                self.assertAlmostEqual(math.remainder(2*math.atan2(q.z,q.w)-yaw,2*math.pi),0.,places=3)
                # In open space more than one heading is feasible and must be compared.
                open_space=plan('GridBased',3.2);self.assertEqual(open_space.status,4)
                q=open_space.result.path.poses[-1].pose.orientation
                self.assertAlmostEqual(math.remainder(2*math.atan2(q.z,q.w)-math.pi/2,2*math.pi),0.,places=2)
                log.flush();log.seek(0);text=log.read()
                rows=re.findall(r'candidates (\d+)/(\d+), feasible (\d+).*reused (\d)',text)
                self.assertTrue(any(int(r[2])>1 for r in rows),text[-6000:])
                self.assertTrue(any(r[3]=='1' for r in rows),text[-6000:])
                self.assertTrue(all(int(r[1])==4 for r in rows),text[-6000:])
                print('Native planner comparison:',rows)
        finally:
            if process and process.poll() is None:
                os.killpg(process.pid,signal.SIGTERM)
                try:process.wait(timeout=10.)
                except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
            if log:log.close()
            peer.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    if os.environ.get('ROS_DOMAIN_ID','0') in ('0','86'):raise SystemExit('Use an isolated test domain')
    unittest.main()
