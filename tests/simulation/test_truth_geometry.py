"""Verify the robot bridge with independently specified world/map geometry."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'src/edgeai_simulation/tools'))
from truth_geometry import compose, inverse


class TruthGeometryTest(unittest.TestCase):
    def test_north_facing_start_maps_forward_goal_to_world_north(self):
        # Map robot at origin facing +X; true robot starts facing world +Y.
        bridge = compose((.45, .55, math.pi/2), inverse((0, 0, 0)))
        goal = compose(bridge, (3.2, -.55, 0))
        self.assertAlmostEqual(goal[0], 1.0)
        self.assertAlmostEqual(goal[1], 3.75)
        self.assertAlmostEqual(goal[2], math.pi/2)

    def test_robot_bridge_preserves_relative_goal_after_map_correction(self):
        # Same robot and world goal, but a SLAM correction rotates/translates map.
        true_robot, world_goal = (1, 2, .7), (1.3, 2.4, -.2)
        for corrected_robot in [(0, 0, 0), (3.5, -.6, -.4)]:
            relative = compose(inverse(true_robot), world_goal)
            map_goal = compose(corrected_robot, relative)
            bridge = compose(true_robot, inverse(corrected_robot))
            recovered = compose(bridge, map_goal)
            for actual, expected in zip(recovered, world_goal):
                self.assertAlmostEqual(actual, expected)


if __name__ == '__main__':
    unittest.main()
