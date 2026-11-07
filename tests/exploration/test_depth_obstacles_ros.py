"""Native Nav2 + synthetic sensors: no simulator, goals or motion commands.

Run in an isolated ROS_DOMAIN_ID with the native Jazzy environment sourced.
"""
from pathlib import Path
import os
import math
import struct
import subprocess
import sys
import tempfile
import time
import unittest
import yaml
import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.msg import Costmap
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import StaticTransformBroadcaster

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'src/edgeai_navigation/scripts'))
from navigation_config import load_navigation


class DepthFusionROS(unittest.TestCase):
    def test_independent_marking_and_clearing(self):
        if os.environ.get('ROS_DOMAIN_ID', '0') in ('0', '86'):
            self.skipTest('Use an isolated ROS_DOMAIN_ID, e.g. 192; never the mission domain')
        rclpy.init()
        node = rclpy.create_node('depth_fusion_test')
        processes = []
        try:
            with tempfile.TemporaryDirectory() as temporary:
                params = load_navigation(ROOT/'src/edgeai_navigation/config', depth_obstacles=True)
                local = params['local_costmap']['local_costmap']['ros__parameters']
                local.pop('fractional_window', None)
                local['plugins'].remove('fractional_window')
                local.update(width=4, height=4, use_sim_time=False, update_frequency=10., publish_frequency=10.)
                params['depth_obstacle_filter']['ros__parameters']['use_sim_time'] = False
                params['depth_obstacle_filter']['ros__parameters']['input_mode'] = 'pointcloud'
                params['controller_server'] = {'ros__parameters': dict(use_sim_time=False,
                    controller_plugins=['FollowPath'], goal_checker_plugins=['goal_checker'],
                    progress_checker_plugins=['progress_checker'],
                    FollowPath={'plugin':'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController'},
                    goal_checker={'plugin':'nav2_controller::SimpleGoalChecker'},
                    progress_checker={'plugin':'nav2_controller::SimpleProgressChecker'})}
                path = Path(temporary)/'params.yaml'
                path.write_text(yaml.safe_dump(params))
                logfile = open(Path(temporary)/'nodes.log', 'w+')
                try:
                    executable = Path(get_package_prefix('nav2_controller'))/'lib/nav2_controller/controller_server'
                    for command in ([str(executable), '--ros-args', '--params-file', str(path)],
                                    ['python3', str(ROOT/'src/edgeai_navigation/scripts/depth_obstacle_filter.py'),
                                     '--ros-args', '--params-file', str(path)]):
                        processes.append(subprocess.Popen(command, stdout=logfile, stderr=logfile))
                    broadcaster = StaticTransformBroadcaster(node)
                    transforms = []
                    for parent, child, z in [('odom','base_footprint',0.), ('base_footprint','depth_frame',.1)]:
                        transform = TransformStamped()
                        transform.header.frame_id, transform.child_frame_id = parent, child
                        transform.transform.translation.z = z
                        transform.transform.rotation.w = 1.
                        transforms.append(transform)
                    broadcaster.sendTransform(transforms)
                    scan_pub = node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
                    cloud_pub = node.create_publisher(PointCloud2, '/camera/points', qos_profile_sensor_data)
                    state = {'map':None, 'point':(.6,.25,0.), 'laser':.9}
                    def receive(message):
                        state['map'] = message
                    node.create_subscription(Costmap, '/local_costmap/costmap_raw', receive, qos_profile_sensor_data)
                    def publish():
                        stamp = node.get_clock().now().to_msg()
                        scan = LaserScan()
                        scan.header.stamp, scan.header.frame_id = stamp, 'depth_frame'
                        angle = math.atan2(.25,.6)
                        scan.angle_min, scan.angle_max, scan.angle_increment = 0.,angle,angle
                        scan.range_min, scan.range_max = .05,3.
                        # Second laser ray crosses depth obstacle XY but reports
                        # free range, as happens for an object below its plane.
                        scan.ranges = [state['laser'],2.5]
                        scan_pub.publish(scan)
                        # Include visible floor: it must clear rays without being marked.
                        points = [state['point'] if i%8 < 4 else (.4,-.25,-.1) for i in range(64)]
                        cloud = PointCloud2()
                        cloud.header.stamp, cloud.header.frame_id = stamp,'depth_frame'
                        cloud.height, cloud.width, cloud.point_step, cloud.row_step = 8,8,12,96
                        cloud.fields = [PointField(name=name,offset=i*4,datatype=PointField.FLOAT32,count=1)
                                        for i,name in enumerate(('x','y','z'))]
                        cloud.data = b''.join(struct.pack('<fff',*p) for p in points)
                        cloud_pub.publish(cloud)
                    def wait(predicate, timeout=12.):
                        deadline = time.monotonic()+timeout
                        while time.monotonic()<deadline:
                            publish()
                            rclpy.spin_once(node, timeout_sec=.05)
                            if predicate():
                                return
                        logfile.flush(); logfile.seek(0)
                        self.fail('Native sensor/costmap check timed out:\n'+logfile.read()[-7000:])
                    client = node.create_client(ChangeState, '/controller_server/change_state')
                    self.assertTrue(client.wait_for_service(timeout_sec=10.))
                    for transition in (1,3):
                        request = ChangeState.Request()
                        request.transition.id = transition
                        future = client.call_async(request)
                        wait(future.done)
                        self.assertTrue(future.result().success)
                    def cost(x,y):
                        message = state['map']
                        if message is None:
                            return None
                        meta = message.metadata
                        mx = int((x-meta.origin.position.x)/meta.resolution)
                        my = int((y-meta.origin.position.y)/meta.resolution)
                        # Synthetic float32 endpoints on cell boundaries may
                        # round into a neighboring cell; check one-cell margin.
                        return max(message.data[y*meta.size_x+x]
                                   for y in range(my-1,my+2) for x in range(mx-1,mx+2))
                    # Raw cloud -> decimator -> native VoxelLayer; laser remains active.
                    wait(lambda: cost(.6,.25)==254 and cost(.9,0.)==254)
                    self.assertNotEqual(cost(.4,-.25),254)
                    # Camera clear ray crosses laser cell, but may not remove its obstacle.
                    state['point'] = (1.3,0.,0.)
                    time_end = time.monotonic()+.7
                    wait(lambda: time.monotonic()>time_end)
                    self.assertEqual(cost(.9,0.),254)
                    # Laser free ray doesn't erase off-plane depth obstacle.
                    self.assertEqual(cost(.6,.25),254)
                    # A new depth return beyond the old obstacle clears its own voxel.
                    state['point'] = (1.2,.5,0.)
                    wait(lambda: cost(.6,.25)!=254 and cost(.9,0.)==254)
                finally:
                    for process in processes:
                        process.terminate()
                    for process in processes:
                        try:
                            process.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            process.kill(); process.wait()
                    logfile.close()
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
