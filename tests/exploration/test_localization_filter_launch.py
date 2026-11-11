"""Effective AMCL inputs and peer TF names; navigation obstacle inputs stay raw."""
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import yaml
from launch import LaunchContext

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/edgeai_navigation/scripts'))
from fleet_config import namespaced_config
from navigation_config import load_navigation
spec=importlib.util.spec_from_file_location('cooperative_launch',ROOT/'src/edgeai_navigation/launch/cooperative_navigation.launch.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


class LocalizationFilterLaunch(unittest.TestCase):
    def check(self,enabled):
        with tempfile.TemporaryDirectory() as temporary:
            d=Path(temporary);(d/'robot2').mkdir();share=d/'share';(share/'config').mkdir(parents=True)
            filtering=yaml.safe_load((ROOT/'src/edgeai_navigation/config/fleet_scan_filter.yaml').read_text())
            filtering['localization']['enabled']=enabled
            (share/'config/fleet_scan_filter.yaml').write_text(yaml.safe_dump(filtering))
            nav=load_navigation(ROOT/'src/edgeai_navigation/config','rpp')
            nav['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml']=str(ROOT/'src/edgeai_navigation/config/behavior_trees/explore_recovery.xml')
            originals=[]
            for ns in ('','robot2'):
                folder=d/ns;config=namespaced_config(nav,ns) if ns else nav
                text=yaml.safe_dump(config);originals.append(text);(folder/'nav2-runtime.yaml').write_text(text)
            context=LaunchContext();context.launch_configurations['directory']=str(d)
            with patch.object(module,'get_package_share_directory',return_value=str(share)):
                actions=module.setup(context)
            for i,ns in enumerate(('','robot2')):
                folder=d/ns
                amcl=yaml.safe_load((folder/'amcl-localization.yaml').read_text())['/**']['ros__parameters']
                self.assertEqual(amcl['scan_topic'],'scan_localization' if enabled else 'scan')
                self.assertEqual((folder/'nav2-runtime.yaml').read_text(),originals[i])
                if enabled:
                    params=yaml.safe_load((folder/'localization-scan-filter.yaml').read_text())['/**']['ros__parameters']
                    self.assertEqual(params['own_odom_frame'],ns+'/odom' if ns else 'odom')
                    self.assertEqual(params['other_base_frame'],'base_footprint' if ns else 'robot2/base_footprint')
                    self.assertEqual(params['other_pose_topic'],'/amcl_pose' if ns else '/robot2/amcl_pose')
                    self.assertEqual(params['margin_m'],.05)
                else:self.assertFalse((folder/'localization-scan-filter.yaml').exists())

    def test_two_amcl_inputs_use_independent_peer_masks(self):self.check(True)

    def test_disabled_filters_restore_raw_amcl_inputs(self):self.check(False)


if __name__=='__main__':unittest.main()
