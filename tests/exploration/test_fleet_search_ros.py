"""Fleet search proposals must preserve independent visits and shared coverage."""
import copy
import json
import os
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import yaml

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'src/edgeai_mission'),str(ROOT/'src/edgeai_exploration')]
from edgeai_tasks.fleet_search import FleetSearchUnits, clone_candidates, commit_candidate, missing_targets
from edgeai_tasks.search_candidates import SpatialCandidates
from edgeai_tasks.exploration import AssignmentPolicy


class FleetSearchROS(unittest.TestCase):
    def test_constructs_real_ros_clients_and_search_subscriptions(self):
        if os.environ.get('ROS_DOMAIN_ID','0') in ('0','86'):self.skipTest('Use an isolated domain')
        import rclpy
        rclpy.init()
        try:
            with tempfile.TemporaryDirectory() as tmp:
                settings=yaml.safe_load((ROOT/'src/edgeai_navigation/config/midterm_single_rescue.yaml').read_text())
                (Path(tmp)/'rescue-config.yaml').write_text(yaml.safe_dump(settings))
                unit=FleetSearchUnits(dict(directory=tmp,required_targets=[dict(id=1,type='normal')]))
                try:
                    self.assertEqual(set(unit.nav),{'robot1','robot2'})
                    self.assertEqual(set(unit.spin),{'robot1','robot2'})
                    self.assertEqual(set(unit.heading),{'robot1','robot2'})
                finally:unit.node.destroy_node()
        finally:rclpy.shutdown()


if __name__=='__main__':unittest.main()
