"""Stage configuration derivation and shell boundary checks; no motion."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
import yaml

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'src/edgeai_navigation/scripts'),str(ROOT/'src/edgeai_mission/scripts')]
from prepare_mission import prepare
from navigation_config import load_navigation


class MissionConfig(unittest.TestCase):
    def test_search_and_scan_geometry_derived_from_runtime(self):
        with tempfile.TemporaryDirectory() as temporary:
            d=Path(temporary)
            nav=load_navigation(ROOT/'src/edgeai_navigation/config','rpp')
            (d/'nav2-runtime.yaml').write_text(yaml.safe_dump(nav))
            c=prepare(ROOT,d,ROOT/'src/edgeai_navigation/config/midterm_single_rescue.yaml',d/'map',600,True,True,'frontier')
            rescue=yaml.safe_load((d/'rescue-config.yaml').read_text())
            scan=yaml.safe_load((d/'observation-scan-runtime.yaml').read_text())['observation_scan']['ros__parameters']
            self.assertGreaterEqual(rescue['search_clearance_m'],scan['rotation_radius_m']+.05-1e-8)
            self.assertEqual(c['initial_relative_pose'],[0.,-.6,0.])
            self.assertEqual(scan['output_directory'],str(d))
            self.assertEqual(c['required_targets'],[dict(id=1,type='normal'),dict(id=2,type='cooperative')])
            self.assertIn('find_targets',c['task_trees'])
            self.assertTrue(c['cooperative']);self.assertTrue(c['rescue_enabled'])
            self.assertEqual(c['task_attempts'],dict(s1=3,c1=4))
            self.assertEqual(c['task_navigation_attempts'],dict(s1=3,c1=3))
            for path in [c['tree'],*c['task_trees'].values()]:
                self.assertTrue(Path(path).is_relative_to(d))
                self.assertTrue(Path(path).is_file())
            disabled=prepare(ROOT,d,ROOT/'src/edgeai_navigation/config/midterm_single_rescue.yaml',d/'map',600,True,False,'frontier',rescue_enabled=False)
            self.assertFalse(disabled['rescue_enabled'])

    def test_shell_no_longer_schedules_rescue_stages(self):
        text=(ROOT/'scripts/simulation/run-open-source-exploration.sh').read_text()
        for legacy in ('normal_rescue.py','cooperative_rescue.py','SCAN_CONFIG','SEARCH_CONFIG','rescue-result.json'):
            self.assertNotIn(legacy,text)
        self.assertIn('mission_coordinator',text)
        self.assertIn('wait "$mission_pid"',text)


if __name__=='__main__':unittest.main()
