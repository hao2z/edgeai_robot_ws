"""Each action keeps a fixed arrival rule through replans and recovery."""
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
import yaml

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/edgeai_navigation/scripts'))
from goal_policy import managed_trees, POLICIES, CONTROLLERS


class GoalTrees(unittest.TestCase):
    def test_fixed_plugins_preserve_recovery_tree(self):
        source=ROOT/'src/edgeai_navigation/config/behavior_trees/explore_recovery.xml'
        with tempfile.TemporaryDirectory() as tmp:
            actual='[[0.1125,0.165],[-0.3125,0.165],[-0.3125,-0.165],[0.1125,-0.165]]'
            (Path(tmp)/'nav2-runtime.yaml').write_text(yaml.safe_dump({'local_costmap':{'local_costmap':{'ros__parameters':{'footprint':actual}}}}))
            paths=managed_trees(source,tmp)
            for policy,(checker,planner) in POLICIES.items():
                tree=ET.parse(paths[policy])
                self.assertFalse(tree.findall('.//GoalCheckerSelector'))
                self.assertFalse(tree.findall('.//PlannerSelector'))
                self.assertFalse(tree.findall('.//ControllerSelector'))
                self.assertEqual(tree.find('.//FollowPath').get('goal_checker_id'),checker)
                self.assertEqual(tree.find('.//FollowPath').get('controller_id'),CONTROLLERS[policy])
                self.assertEqual(tree.find('.//ComputePathToPose').get('planner_id'),planner)
                self.assertEqual(tree.find('.//SelectRecovery').get('controller_id'),CONTROLLERS[policy])
                self.assertEqual(tree.find('.//SelectRecovery').get('actual_footprint'),actual)
                retry=tree.find('.//RetryUntilSuccessful')
                self.assertIsNotNone(retry)
                self.assertEqual(retry.get('num_attempts'),'8')
                self.assertIsNotNone(retry.find('.//SelectRecovery'))
                self.assertFalse(tree.findall('.//Wait'))
                planner_help=tree.findall('.//WouldAPlannerRecoveryHelp')
                self.assertEqual(len(planner_help),1 if policy=='rescue' else 2)
                self.assertIsNotNone(tree.find('.//PlannerStartOccupied'))
                if policy=='rescue':
                    self.assertEqual(tree.find('.//ScriptCondition').get('code'),'compute_path_error_code == 0')
                for tag in ('BackUp','Spin','RecoveryNode','ClearEntireCostmap'):
                    self.assertEqual(len(tree.findall('.//'+tag)),len(ET.parse(source).findall('.//'+tag)))


if __name__=='__main__':
    unittest.main()
