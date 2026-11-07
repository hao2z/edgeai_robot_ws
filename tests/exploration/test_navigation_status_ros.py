"""Exercise read-only diagnostics, including paused /clock, without a simulator."""
from pathlib import Path
import json
import os
import sys
import tempfile
import time
import unittest
import yaml
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts/ros'))
from navigation_diagnostics import NavigationDiagnostics


class StatusROS(unittest.TestCase):
    def test_depth_and_commands_keep_updating_when_clock_pauses(self):
        if os.environ.get('ROS_DOMAIN_ID','0') in ('0','86'):
            self.skipTest('Use an isolated ROS_DOMAIN_ID, e.g. 192')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            (path/'nav2-runtime.yaml').write_text(yaml.safe_dump(dict(local_costmap=dict(local_costmap=dict(
                ros__parameters=dict(plugins=['depth_layer'],depth_layer=dict(enabled=True)))))))
            rclpy.init(args=['--ros-args','-p','use_sim_time:=true','-p','log_directory:='+directory])
            diagnostics = NavigationDiagnostics()
            node = rclpy.create_node('status_test_source')
            executor = rclpy.executors.SingleThreadedExecutor()
            executor.add_node(diagnostics);executor.add_node(node)
            try:
                received=[]
                node.create_subscription(String,'/diagnostics/navigation_status',lambda message:received.append(message.data),1)
                clock_pub = node.create_publisher(Clock,'/clock',10)
                command_pub = node.create_publisher(Twist,'/cmd_vel',10)
                odom_pub = node.create_publisher(Odometry,'/odom',10)
                scan_pub = node.create_publisher(LaserScan,'/scan',10)
                depth_pub = node.create_publisher(PointCloud2,'/camera/obstacle_points',10)
                state_pub = node.create_publisher(String,'/navigation_authority/state',
                    QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
                authority = dict(mode='AUTO',phase='ACTIVE',generation=1,following_path=False,
                                 arrival_policy='position',distance_remaining=1.2)
                def send():
                    clock=Clock();clock.clock.sec=10
                    clock_pub.publish(clock)
                    for kind,publisher in [(Odometry,odom_pub),(LaserScan,scan_pub),(PointCloud2,depth_pub)]:
                        message=kind();message.header.stamp.sec=10;publisher.publish(message)
                    command=Twist();command.linear.x=.2;command_pub.publish(command)
                    state_pub.publish(String(data=json.dumps(authority)))
                def wait(predicate,timeout=8.):
                    deadline=time.monotonic()+timeout
                    while time.monotonic()<deadline:
                        send();executor.spin_once(timeout_sec=.05)
                        if received and predicate(received[-1]):return
                    self.fail('Diagnostic message absent or wrong: '+str(received[-1:] ))
                wait(lambda body:'已配置开启' in body and '0.20 m/s' in body)
                # Repeated equal /clock values do not advance simulation time;
                # steady-clock timer must still publish an honest pause status.
                wait(lambda body:'仿真时钟暂停或未推进' in body)
                self.assertIn('<!--DETAILS-->',received[-1])
                self.assertIn('<!--LEGEND-->',received[-1])
                rows=[json.loads(line) for line in (path/'alignment.jsonl').read_text().splitlines()]
                self.assertTrue(rows)
                self.assertIn('operational_status',rows[-1])
            finally:
                executor.shutdown()
                diagnostics.destroy_node();node.destroy_node();rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
