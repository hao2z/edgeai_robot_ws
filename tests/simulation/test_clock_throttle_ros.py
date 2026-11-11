"""Synthetic clock only, isolated ROS domain; never launch or control Gazebo."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock


class ClockThrottleROS(unittest.TestCase):
    def test_standard_throttle_limits_rate_preserves_pause_and_clock_reset(self):
        if os.environ.get('ROS_DOMAIN_ID', '0') in ('0', '86', '87'):
            self.skipTest('Requires an isolated ROS domain, e.g. 198')
        executable = Path(get_package_prefix('topic_tools'))/'lib/topic_tools/throttle'
        with tempfile.TemporaryFile(mode='w+') as log:
            process = subprocess.Popen([str(executable), 'messages', '/simulation/clock_raw', '100.0',
                '/clock', '--ros-args', '-r', '__node:=simulation_clock', '-p', 'use_sim_time:=false',
                '-p', 'use_wall_clock:=true', '-p', 'lazy:=false'], stdout=log, stderr=log, start_new_session=True)
            rclpy.init()
            node = rclpy.create_node('clock_test_source')
            publisher = node.create_publisher(Clock, '/simulation/clock_raw',
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            received, inputs = [], set()
            subscription = node.create_subscription(Clock, '/clock',
                lambda m:received.append((time.monotonic(), m.clock.sec*10**9+m.clock.nanosec)),
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            def drive(seconds, base=None):
                started = time.monotonic()
                while time.monotonic()-started < seconds:
                    if base is not None:
                        stamp = base+int((time.monotonic()-started)*10**9)
                        inputs.add(stamp)
                        msg = Clock()
                        msg.clock.sec, msg.clock.nanosec = divmod(stamp, 10**9)
                        publisher.publish(msg)
                    rclpy.spin_once(node, timeout_sec=.001)
                    if process.poll() is not None:
                        log.seek(0)
                        self.fail('Clock relay exited: '+log.read())
            try:
                deadline = time.monotonic()+6
                while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
                    drive(.05, 10*10**9)
                self.assertGreater(publisher.get_subscription_count(), 0)
                drive(.1, 11*10**9)
                received.clear()
                drive(1.2, 12*10**9)
                self.assertGreater(len(received), 20)
                self.assertLessEqual(len(received), 125)
                self.assertTrue(all(stamp in inputs for _, stamp in received))
                self.assertTrue(all(a[1] <= b[1] for a, b in zip(received, received[1:])))
                drive(.1)  # Drain DDS samples without advancing time.
                count = len(received)
                drive(.12)
                self.assertEqual(len(received), count)
                # A simulator reset must still propagate while output ROS time is in the past.
                drive(.15, 10**9)
                self.assertGreater(len(received), count)
                self.assertLess(received[-1][1], 2*10**9)
                self.assertIn(received[-1][1], inputs)
            finally:
                node.destroy_subscription(subscription)
                node.destroy_node()
                rclpy.shutdown()
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait(timeout=5)


if __name__ == '__main__':
    unittest.main()
