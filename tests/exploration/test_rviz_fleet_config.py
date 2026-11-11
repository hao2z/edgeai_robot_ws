"""Validate the actual launch-time RViz generator without starting a simulator."""
from pathlib import Path
import os
import subprocess
import sys
import tempfile
import unittest
import yaml

ROOT = Path(__file__).resolve().parents[2]


class FleetRVizConfig(unittest.TestCase):
    def test_both_controller_modes_keep_robot_topics_separate(self):
        source = (ROOT/'scripts/simulation/run-open-source-exploration.sh').read_text()
        generator = source.split("<<'TEST_RVIZ'\n", 1)[1].split('\nTEST_RVIZ', 1)[0]
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)/'fleet.rviz'
            for controller in ('rpp', 'dwb'):
                with self.subTest(controller=controller):
                    subprocess.run([sys.executable, '-c', generator,
                        str(ROOT/'src/edgeai_exploration/rviz/exploration.rviz'), str(output),
                        'false', controller, str(ROOT/'src/edgeai_navigation/config/nav2_exploration_rectangle.yaml')],
                        env=dict(os.environ, EDGEAI_COOPERATIVE_RESCUE='true'), check=True)
                    displays = yaml.safe_load(output.read_text())['Visualization Manager']['Displays']
                    groups = {item['Name']:item['Displays'] for item in displays if item['Class']=='rviz_common/Group'}
                    self.assertEqual(set(groups), {'Robot 1', 'Robot 2'})
                    def topics(items):
                        return {d.get('Topic', d.get('Description Topic', {})).get('Value') for d in items}
                    first, second = topics(groups['Robot 1']), topics(groups['Robot 2'])
                    local = '/lookahead_arc' if controller == 'rpp' else '/local_plan'
                    for topic in ('/plan', local, '/scan', '/diagnostics/navigation',
                                  '/navigation_authority/dispatched_goal', '/robot_description', '/truth/markers'):
                        self.assertIn(topic, first)
                        self.assertIn('/robot2'+topic, second)
                    self.assertFalse(any(t.startswith('/camera/') for t in second))
                    self.assertTrue(all(t.startswith('/robot2/') for t in second))
                    self.assertIn('/map', topics(displays))
                    self.assertIn('/targets/markers', topics(displays))


if __name__ == '__main__':
    unittest.main()
