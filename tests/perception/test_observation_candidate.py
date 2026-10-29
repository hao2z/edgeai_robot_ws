"""ROS data gates for observation admission, without moving a robot."""
from pathlib import Path
import sys
import unittest
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import TransformStamped
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'src/edgeai_perception/scripts'))
from observation_scan import ObservationScan


class CandidateTests(unittest.TestCase):
    def test_fresh_open_space_history_and_stale_map(self):
        rclpy.init();node=ObservationScan()
        try:
            now=node.get_clock().now()
            msg=OccupancyGrid();msg.header.frame_id='odom';msg.header.stamp=now.to_msg()
            msg.info.width=msg.info.height=60;msg.info.resolution=.05
            msg.info.origin.position.x=msg.info.origin.position.y=-1.5
            msg.info.origin.orientation.w=1.;msg.data=[0]*3600
            odom=Odometry();odom.header.stamp=now.to_msg()
            node.costmap(msg);node.motion(odom)
            node.stationary_since=now.nanoseconds*1e-9-1
            for a,b in [('map','odom'),('odom','base_footprint')]:
                t=TransformStamped();t.header.frame_id=a;t.child_frame_id=b;t.transform.rotation.w=1.
                node.buffer.set_transform_static(t,'test')
            self.assertEqual(node.candidate(),(0.,0.))
            node.history.append((0.,0.));self.assertIsNone(node.candidate())
            self.assertIn('过近',node.gate_reason)
            node.history.clear();msg.data[30*60+35]=100
            self.assertIsNone(node.candidate());self.assertIn('障碍',node.gate_reason)
            msg.data[30*60+35]=0;msg.header.stamp.sec-=2
            self.assertIsNone(node.candidate());self.assertIn('过期',node.gate_reason)
        finally:
            node.close();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':unittest.main()
