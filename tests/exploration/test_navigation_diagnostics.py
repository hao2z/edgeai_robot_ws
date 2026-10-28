"""Reference-frame and timestamp checks for the read-only visual diagnostics."""
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
import unittest

path = Path(__file__).resolve().parents[2]/'scripts/ros/navigation_diagnostics.py'
spec = importlib.util.spec_from_file_location('navigation_diagnostics', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class DiagnosticGeometryTest(unittest.TestCase):
    def test_rate_counts_simulation_time_and_expires(self):
        fake = SimpleNamespace(arrivals={'scan':[1.0,1.05,1.1]})
        self.assertAlmostEqual(module.NavigationDiagnostics.rate(fake,'scan',1.11),20.0)
        self.assertEqual(module.NavigationDiagnostics.rate(fake,'scan',2.0),0.0)
        self.assertEqual(module.NavigationDiagnostics.rate(fake,'missing',1.11),0.0)

    def test_timestamp_age_preserves_future_and_zero(self):
        self.assertIsNone(module.age_seconds(5_000_000_000, SimpleNamespace(sec=0,nanosec=0)))
        self.assertAlmostEqual(module.age_seconds(5_000_000_000, SimpleNamespace(sec=4,nanosec=800_000_000)), .2)
        self.assertAlmostEqual(module.age_seconds(5_000_000_000, SimpleNamespace(sec=5,nanosec=300_000_000)), -.3)

    def test_old_outline_is_reprojected_at_common_pose(self):
        body = [(-.3125,.165),(.1125,.165),(.1125,-.165),(-.3125,-.165)]
        # A producer published while the axle was at x=1, yaw=90deg.
        old = module.apply_pose((1,0,math.pi/2), body)
        recovered = module.apply_pose((0,1,-math.pi/2), old)
        aligned = module.apply_pose((2,3,0), recovered)
        expected = module.apply_pose((2,3,0), body)
        for a,b in zip(aligned,expected):
            self.assertAlmostEqual(a[0],b[0])
            self.assertAlmostEqual(a[1],b[1])

    def test_yaw_uses_quaternion_not_z_component(self):
        transform = SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=1.,y=2.),
            rotation=SimpleNamespace(x=0.,y=0.,z=math.sqrt(.5),w=math.sqrt(.5))))
        pose = module.planar(transform)
        point = module.apply_pose(pose,[(.1,.2)])[0]
        self.assertAlmostEqual(point[0],.8)
        self.assertAlmostEqual(point[1],2.1)


if __name__ == '__main__':
    unittest.main()
