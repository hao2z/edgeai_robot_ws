"""Truth benchmark TF is derived from time-paired poses, never latest fallback."""
from pathlib import Path
import sys,unittest
from unittest.mock import Mock
import math
import rclpy
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'src/edgeai_simulation/tools'))
from navigation_truth_localization import TruthLocalization
from truth_geometry import compose,planar


class Localization(unittest.TestCase):
    def test_paired_transform_reconstructs_world_pose_and_rejects_stale_truth(self):
        rclpy.init();node=TruthLocalization()
        try:
            now=node.get_clock().now();stamp=Time(nanoseconds=now.nanoseconds-50_000_000).to_msg()
            sample=TransformStamped();sample.header.stamp=stamp
            sample.transform.translation.x=3.;sample.transform.translation.y=2.
            sample.transform.rotation.z=math.sin(.7/2);sample.transform.rotation.w=math.cos(.7/2)
            odom=TransformStamped();odom.transform.translation.x=1.;odom.transform.translation.y=-.2
            odom.transform.rotation.z=math.sin(.2/2);odom.transform.rotation.w=math.cos(.2/2)
            node.buffer.lookup_transform=Mock(return_value=odom)
            node.broadcaster.sendTransform=Mock();node.pose_pub.publish=Mock()
            node.pending.append(sample)
            import time
            node.received=time.monotonic();node.tick()
            tf=node.broadcaster.sendTransform.call_args.args[0]
            world=compose(planar(tf.transform.translation,tf.transform.rotation),planar(odom.transform.translation,odom.transform.rotation))
            self.assertAlmostEqual(world[0],3.);self.assertAlmostEqual(world[1],2.);self.assertAlmostEqual(world[2],.7)
            requested=node.buffer.lookup_transform.call_args.args[2]
            self.assertEqual(requested.nanoseconds,Time.from_msg(stamp).nanoseconds)
            node.broadcaster.sendTransform.reset_mock();node.received=time.monotonic()-2.;node.tick()
            node.broadcaster.sendTransform.assert_not_called()
        finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':unittest.main()
