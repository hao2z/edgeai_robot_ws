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
    def check(self,enabled,reuse=False,single=False):
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
            if reuse:context.launch_configurations['reuse_robot2_navigation']='true'
            if single:context.launch_configurations['enable_robot2']='false'
            with patch.object(module,'get_package_share_directory',return_value=str(share)), \
                    patch.object(module,'IncludeLaunchDescription',wraps=module.IncludeLaunchDescription) as navigation, \
                    patch.object(module,'ExecuteProcess',wraps=module.ExecuteProcess) as processes, \
                    patch.object(module,'Node',wraps=module.Node) as nodes:
                actions=module.setup(context)
            self.assertEqual(navigation.call_count,0 if reuse or single else 1)
            authorities=[c for c in processes.call_args_list if any('navigation_authority.py' in str(a) for a in c.kwargs['cmd'])]
            self.assertEqual(len(authorities),0 if reuse or single else 1)
            self.assertEqual([c.kwargs['namespace'] for c in nodes.call_args_list
                if c.kwargs.get('executable')=='amcl'],[''] if single else ['','robot2'])
            managers=[c.kwargs for c in nodes.call_args_list if c.kwargs.get('executable')=='lifecycle_manager']
            self.assertEqual(len(managers),1 if single else 2)
            self.assertEqual(managers[0]['parameters'][0]['node_names'],['map_server','amcl'])
            if not single:
                self.assertEqual(managers[1]['namespace'],'robot2')
                self.assertEqual(managers[1]['parameters'][0]['node_names'],['amcl'])
            for i,ns in enumerate(('',) if single else ('','robot2')):
                folder=d/ns
                amcl=yaml.safe_load((folder/'amcl-localization.yaml').read_text())['/**']['ros__parameters']
                self.assertEqual(amcl['scan_topic'],'scan_localization' if enabled and not single else 'scan')
                self.assertEqual((folder/'nav2-runtime.yaml').read_text(),originals[i])
                if enabled and not single:
                    params=yaml.safe_load((folder/'localization-scan-filter.yaml').read_text())['/**']['ros__parameters']
                    self.assertEqual(params['own_odom_frame'],ns+'/odom' if ns else 'odom')
                    self.assertEqual(params['other_base_frame'],'base_footprint' if ns else 'robot2/base_footprint')
                    self.assertEqual(params['other_pose_topic'],'/amcl_pose' if ns else '/robot2/amcl_pose')
                    self.assertEqual(params['margin_m'],.05)
                else:self.assertFalse((folder/'localization-scan-filter.yaml').exists())

    def test_two_amcl_inputs_use_independent_peer_masks(self):self.check(True)

    def test_disabled_filters_restore_raw_amcl_inputs(self):self.check(False)

    def test_reuse_navigation_keeps_both_amcl_and_peer_masks(self):self.check(True,reuse=True)

    def test_reuse_navigation_without_localization_masks(self):self.check(False,reuse=True)

    def test_single_robot_launch_never_waits_for_robot2(self):self.check(True,single=True)


if __name__=='__main__':unittest.main()
