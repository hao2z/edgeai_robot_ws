"""Stage configuration derivation checks; no motion."""
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import yaml

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'src/edgeai_navigation/scripts'),str(ROOT/'src/edgeai_mission/scripts')]
from prepare_mission import mission_targets, prepare
from navigation_config import load_navigation

MIDTERM_TARGETS = ROOT/'src/edgeai_simulation/config/targets_midterm.yaml'


class MissionConfig(unittest.TestCase):
    def test_arena_rules_override_both_rescue_profiles(self):
        with tempfile.TemporaryDirectory() as temporary:
            d = Path(temporary)
            nav_dir = ROOT/'src/edgeai_navigation/config'
            (d/'nav2-runtime.yaml').write_text(yaml.safe_dump(load_navigation(nav_dir, 'rpp')))
            arena = yaml.safe_load((ROOT/'src/edgeai_simulation/config/targets_competition.yaml').read_text())
            arena['rescue_rules']['cooperative'].update(radius_m=.8, dwell_s=7.)
            targets = d/'targets-config.yaml'
            targets.write_text(yaml.safe_dump(arena))
            config = prepare(ROOT, d, nav_dir/'midterm_single_rescue.yaml', d/'map', 600, True, 'frontier',
                             targets_file=targets)
            normal = yaml.safe_load((d/'rescue-config.yaml').read_text())
            joint = yaml.safe_load((d/'cooperative-rescue-config.yaml').read_text())
            self.assertEqual(config['target_count'], 4)
            self.assertEqual(normal['normal_offset_m'], .7)
            self.assertEqual(normal['target_radius_m'], .8)
            self.assertTrue(normal['alignment_enabled'])
            self.assertEqual((joint['target_id'], joint['radius_m'], joint['dwell_s']), (10, .8, 7.))

    def test_competition_target_list_and_overrides(self):
        arena = ROOT/'src/edgeai_simulation/config/targets_competition.yaml'
        expected = [dict(id=i, type='normal') for i in (1, 2, 3)] + [dict(id=10, type='cooperative')]
        self.assertEqual(mission_targets(arena, True, count=4), expected)
        self.assertEqual(mission_targets(arena, True, '3:normal,10:cooperative', 2), expected[2:])
        with self.assertRaisesRegex(ValueError, 'Expected 3 targets'):
            mission_targets(arena, True, count=3)
        with self.assertRaisesRegex(ValueError, 'does not match'):
            mission_targets(arena, True, '1:cooperative,10:normal')
        with self.assertRaisesRegex(ValueError, 'must be unique'):
            mission_targets(arena, True, '1:normal,1:normal,10:cooperative')

    def test_joint_mapping_uses_assigned_s1_tree_and_separate_policy_overrides(self):
        with tempfile.TemporaryDirectory() as temporary, patch.dict('os.environ',EDGEAI_JOINT_MAPPING='true'):
            d=Path(temporary)
            (d/'nav2-runtime.yaml').write_text(yaml.safe_dump(load_navigation(ROOT/'src/edgeai_navigation/config','rpp')))
            config=prepare(ROOT,d,ROOT/'src/edgeai_navigation/config/midterm_single_rescue.yaml',d/'map',600,True,'frontier',targets_file=MIDTERM_TARGETS)
            self.assertEqual(Path(config['task_trees']['s1']).read_text(),
                             (ROOT/'src/edgeai_mission/config/tasks/s1.xml').read_text())
            self.assertEqual(config['single_rescue_assignment']['max_navigation_attempts'],1)
            self.assertEqual(config['exploration_assignment']['max_navigation_attempts'],2)

    def test_search_and_scan_geometry_derived_from_runtime(self):
        with tempfile.TemporaryDirectory() as temporary:
            d=Path(temporary)
            nav=load_navigation(ROOT/'src/edgeai_navigation/config','rpp')
            (d/'nav2-runtime.yaml').write_text(yaml.safe_dump(nav))
            c=prepare(ROOT,d,ROOT/'src/edgeai_navigation/config/midterm_single_rescue.yaml',d/'map',600,True,'frontier',targets_file=MIDTERM_TARGETS)
            rescue=yaml.safe_load((d/'rescue-config.yaml').read_text())
            self.assertEqual(rescue['allowed_target_ids'], [1])
            self.assertNotIn('target_id', rescue)
            geometry=nav['local_costmap']['local_costmap']['ros__parameters']
            radius=max(math.hypot(x,y) for x,y in json.loads(geometry['footprint']))
            self.assertGreaterEqual(rescue['search_clearance_m'],radius+.05-1e-8)
            self.assertEqual(c['initial_relative_pose'],[0.,-.6,0.])
            self.assertEqual(c['required_targets'],[dict(id=1,type='normal'),dict(id=2,type='cooperative')])
            self.assertEqual(c['mapping'],yaml.safe_load((ROOT/'src/edgeai_mission/config/mapping.yaml').read_text()))
            self.assertEqual(c['mapping']['min_known_area_m2'],18.)
            self.assertIn('find_targets',c['task_trees'])
            self.assertTrue(c['cooperative']);self.assertTrue(c['rescue_enabled'])
            self.assertEqual(c['fleet']['mapping_robots'],['robot1'])
            self.assertEqual(c['fleet']['localization_robots'],['robot1','robot2'])
            self.assertEqual(c['fleet']['task_robots']['c1'],['robot1','robot2'])
            self.assertEqual(c['task_attempts'],dict(explore=1,s1=3,c1=4))
            self.assertEqual(c['task_navigation_attempts'],dict(explore=1,s1=3,c1=3))
            self.assertEqual(c['task_wall_timeout_s']['explore'],600.)
            self.assertIn('explore',c['task_trees'])
            self.assertEqual(c['exploration_assignment'],yaml.safe_load((ROOT/'src/edgeai_mission/config/exploration.yaml').read_text()) or {})
            for path in [c['tree'],*c['task_trees'].values()]:
                self.assertTrue(Path(path).is_relative_to(d))
                self.assertTrue(Path(path).is_file())
            disabled=prepare(ROOT,d,ROOT/'src/edgeai_navigation/config/midterm_single_rescue.yaml',d/'map',600,True,'frontier',rescue_enabled=False,targets_file=MIDTERM_TARGETS)
            self.assertFalse(disabled['rescue_enabled'])


if __name__=='__main__':unittest.main()
