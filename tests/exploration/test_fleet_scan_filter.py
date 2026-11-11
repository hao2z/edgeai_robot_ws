"""Footprint masking preserves occlusion and unrelated obstacle measurements."""
import math
from pathlib import Path
import sys
import unittest
from sensor_msgs.msg import LaserScan

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'src/edgeai_navigation/scripts'))
from fleet_scan_filter import mask_scan, covariance_reliable


class FilterTests(unittest.TestCase):
    def test_covariance_requires_valid_bounded_uncertainty(self):
        c=[0.]*36;c[0]=c[7]=.01;c[35]=.01
        self.assertTrue(covariance_reliable(c,.15,.2))
        c[1]=c[6]=.02
        self.assertFalse(covariance_reliable(c,.15,.2))
        c[1]=c[6]=0.;c[35]=float('nan')
        self.assertFalse(covariance_reliable(c,.15,.2))
        c[35]=-.01
        self.assertFalse(covariance_reliable(c,.15,.2))

    def test_only_points_inside_follower_are_removed_without_mutating_input(self):
        scan=LaserScan();scan.range_min=.05;scan.range_max=10.
        scan.angle_min=0.;scan.angle_increment=math.pi/2;scan.ranges=[.6,2.,float('inf'),float('nan')]
        out,count=mask_scan(scan,[-.6,0.,0.],[-.3,.1,-.16,.16],.05)
        self.assertEqual(count,1);self.assertTrue(math.isnan(out.ranges[0]))
        self.assertAlmostEqual(scan.ranges[0],.6);self.assertEqual(out.ranges[1],2.)
        self.assertTrue(math.isinf(out.ranges[2]));self.assertTrue(math.isnan(out.ranges[3]))

    def test_rotation_and_margin(self):
        scan=LaserScan();scan.range_min=.05;scan.range_max=10.
        scan.angle_increment=1.;scan.ranges=[.6]
        out,count=mask_scan(scan,[0.,-.6,math.pi/2],[-.3,.1,-.16,.16],.05)
        self.assertEqual(count,1)
        _,count=mask_scan(scan,[0.,-.9,math.pi/2],[-.3,.1,-.16,.16],.05)
        self.assertEqual(count,0)


if __name__=='__main__':unittest.main()
