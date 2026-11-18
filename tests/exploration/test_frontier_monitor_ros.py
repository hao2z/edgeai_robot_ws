"""Real passive candidate detector, isolated domain 158."""
import json
import math
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
from std_msgs.msg import Int32, Empty, String
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from frontier_exploration_ros2.srv import ControlExploration


class FrontierMonitorROS(unittest.TestCase):
    def setUp(self):
        if os.environ.get('ROS_DOMAIN_ID') != '158':
            self.skipTest('Frontier monitor tests are restricted to ROS_DOMAIN_ID=158')

    def test_namespaced_passive_candidates_use_free_cost_filtered_goals(self):
        rclpy.init()
        self.addCleanup(rclpy.shutdown)
        peer = rclpy.create_node('passive_candidates_peer')
        self.addCleanup(peer.destroy_node)
        directory = tempfile.TemporaryDirectory(prefix='passive-frontiers-')
        self.addCleanup(directory.cleanup)
        maps = peer.create_publisher(OccupancyGrid, '/map',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        costs = {i: peer.create_publisher(OccupancyGrid, f'/robot{i}/global_costmap/costmap', 10)
                 for i in (1, 2)}
        locals_ = {i: peer.create_publisher(OccupancyGrid, f'/robot{i}/local_costmap/costmap', 10)
                   for i in (1, 2)}
        records = {1: [], 2: []}
        goals = []
        subscriptions, servers, processes, logs = [], [], [], []
        tf = StaticTransformBroadcaster(peer)
        state = dict(blocked=True, local_blocked=False, boundary=True)
        for i in (1, 2):
            subscriptions.append(peer.create_subscription(String, f'/robot{i}/explore/candidates',
                lambda msg, i=i: records[i].append(json.loads(msg.data)), 10))
            def execute(handle):
                goals.append(handle.request)
                handle.succeed()
                return NavigateToPose.Result()
            server = ActionServer(peer, NavigateToPose, f'/robot{i}/navigate_to_pose', execute)
            servers.append(server)
            self.addCleanup(server.destroy)

        def grid(data):
            msg = OccupancyGrid()
            msg.header.frame_id = 'map'
            msg.header.stamp = peer.get_clock().now().to_msg()
            msg.info.width = msg.info.height = 60
            msg.info.resolution = .1
            msg.info.origin.position.x = msg.info.origin.position.y = -2.
            msg.info.origin.orientation.w = 1.
            msg.data = data
            return msg

        def publish():
            maps.publish(grid([-1 if state['boundary'] and x >= 40 else 0
                               for y in range(60) for x in range(60)]))
            # A lethal/inscribed clearance band blocks otherwise free endpoints.
            global_data = [100 if state['blocked'] and x >= 38 else 0
                           for y in range(60) for x in range(60)]
            for i in (1, 2):
                costs[i].publish(grid(global_data))
                locals_[i].publish(grid([100 if state['local_blocked'] else 0]*3600))
        timer = peer.create_timer(.05, publish)

        def stop(process):
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=3.)

        def diagnostics():
            return '; '.join(Path(log.name).read_text()[-4000:] for log in logs)

        def wait(predicate, description, timeout=12.):
            end = time.monotonic()+timeout
            while time.monotonic() < end:
                rclpy.spin_once(peer, timeout_sec=.02)
                self.assertTrue(all(p.poll() is None for p in processes), diagnostics())
                if predicate():
                    return
            self.fail(f'Timed out: {description}; records={records}; logs={diagnostics()}')

        def latest(i, available, nonempty=None):
            if not records[i] or records[i][-1]['available'] != available:
                return False
            return nonempty is None or bool(records[i][-1]['candidates']) == nonempty

        executable = Path(get_package_prefix('frontier_exploration_ros2'))/'lib/frontier_exploration_ros2/frontier_explorer'
        for i in (1, 2):
            log = (Path(directory.name)/f'robot{i}.log').open('w+')
            logs.append(log)
            self.addCleanup(log.close)
            process = subprocess.Popen([str(executable), '--ros-args', '-r', f'__ns:=/robot{i}',
                '-r', f'/explore/available_frontiers:=/robot{i}/explore/available_frontiers',
                '-p', 'use_sim_time:=false', '-p', 'autostart:=false',
                '-p', 'paused_map_processing_enabled:=true',
                '-p', 'map_qos_autodetect_on_startup:=false',
                '-p', 'map_processing_rate_hz:=10.0', '-p', 'map_topic:=/map',
                '-p', f'robot_base_frame:=robot{i}/base_footprint'], stdout=log, stderr=log)
            processes.append(process)
            self.addCleanup(stop, process)
        wait(lambda: all(latest(i, False) for i in (1, 2)), 'missing TF is unavailable')
        transform = TransformStamped()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'robot1/base_footprint'
        transform.transform.rotation.w = 1.
        tf.sendTransform(transform)
        wait(lambda: latest(1, True, False), 'global clearance band filters endpoints')
        self.assertTrue(latest(2, False), 'Missing TF must not masquerade as exhaustion')
        state['blocked'] = False
        wait(lambda: latest(1, True, True), 'reachable free-neighbor endpoints')
        snapshot = records[1][-1]
        for candidate in snapshot['candidates']:
            self.assertTrue(all(math.isfinite(candidate[key]) for key in ('x', 'y')))
            self.assertGreater(candidate['size'], 0)
            x, y = (math.floor((candidate[key]+2.)/.1) for key in ('x', 'y'))
            self.assertTrue(0 <= x < 40 and 0 <= y < 60,
                            'Candidate must be on known free side, not unknown center')
        transform2 = TransformStamped()
        transform2.header.frame_id = 'map'
        transform2.child_frame_id = 'robot2/base_footprint'
        transform2.transform.rotation.w = 1.
        tf.sendTransform([transform, transform2])
        wait(lambda: latest(2, True, True), 'second independent namespace candidates')
        before = [len(records[i]) for i in (1, 2)]
        wait(lambda: all(len(records[i])-before[i-1] >= 3 for i in (1, 2)), 'fresh periodic snapshots')
        for i in (1, 2):
            stamps = [record['stamp_ns'] for record in records[i]]
            self.assertTrue(all(isinstance(stamp, int) for stamp in stamps))
            self.assertTrue(all(b-a >= 490_000_000 for a, b in zip(stamps, stamps[1:])),
                            'Candidate publishing must remain capped at 2 Hz')
            self.assertTrue(all(record['frame_id'] == 'map' for record in records[i]))
            age = (peer.get_clock().now().nanoseconds-stamps[-1])/1e9
            self.assertLess(age, 1.)
        state['local_blocked'] = True
        wait(lambda: all(latest(i, True, False) for i in (1, 2)), 'local costs filter candidates')
        state['local_blocked'], state['boundary'] = False, False
        wait(lambda: all(latest(i, True, False) for i in (1, 2)), 'valid empty map snapshots')
        state['boundary'] = True
        wait(lambda: all(latest(i, True, True) for i in (1, 2)), 'passive detectors reopen')
        self.assertFalse(goals, 'Passive detectors must never dispatch navigation')
        print('PASSIVE_CANDIDATES_ROS_EVIDENCE '+json.dumps(dict(
            topics=['/robot1/explore/candidates', '/robot2/explore/candidates'],
            candidates=snapshot['candidates'], missing_tf_unavailable=True,
            cost_filtering=True, max_hz=2, action_goals=len(goals))), flush=True)
        timer.cancel()



if __name__=='__main__':
    if os.environ.get('ROS_DOMAIN_ID') != '158':raise SystemExit('Use ROS_DOMAIN_ID=158 only')
    unittest.main()
