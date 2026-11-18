"""Final-map priors and retries must not depend on an already stopped SLAM."""
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'src/edgeai_mission/scripts'),str(ROOT/'src/edgeai_mission'),
              str(ROOT/'src/edgeai_navigation')]
from mission_localization import prepare_localization
from edgeai_tasks.mission_fleet import fleet_layout,localization_priors,configured_fleet


class MissionLocalization(unittest.TestCase):
    def test_runtime_snapshot_before_fleet_schema_can_still_start(self):
        fleet=configured_fleet(dict(joint_mapping=True,cooperative=True),[0.,-.6,0.])
        self.assertEqual(fleet['mapping_robots'],['robot1','robot2'])
        self.assertEqual(fleet['localization_robots'],['robot1','robot2'])

    def exercise(self,joint=True,dual=True):
        with tempfile.TemporaryDirectory() as temporary:
            d=Path(temporary)
            (d/'cooperative-map.yaml').write_text('image: map.pgm\n')
            (d/'targets.json').write_text(json.dumps(dict(map_binding=dict(finalized=True))))
            (d/'mission-reference.json').write_text(json.dumps(dict(mapping_start_pose=[1.,2.,0.],initial_odom2=[0.,0.,0.])))
            def odom(x):return NS(pose=NS(pose=NS(position=NS(x=x,y=0.),orientation=NS(x=0.,y=0.,z=0.,w=1.))))
            node=NS(directory=d,joint_mapping=joint,c=dict(initial_relative_pose=[0.,-.6,0.]),
                localization_namespaces=('robot1','robot2') if dual else ('robot1',),map=object(),now=lambda:1.,
                odom={'robot1':odom(0.),'robot2':odom(0.)},finalize_targets=Mock(),stop_slam=Mock(),
                start_localization=Mock(),event=Mock())
            node.wait=lambda p,t:self.assertTrue(p())
            node.tf_pose=lambda frame:[1.,2.,0.] if frame=='robot1/base_footprint' else [3.,4.,0.]
            prepare_localization(node,dict(fleet=fleet_layout(joint,dual,[0.,-.6,0.])))
            self.assertEqual(node.start_localization.call_args.args,([1.,2.,0.],[3.,4.,0.] if joint else [1.,1.4,0.] if dual else None))
            # A retry after recovery uses local odometry displacement, not dead SLAM.
            node.odom['robot1']=odom(.1)
            node.map=None  # Previous map_server and SLAM no longer publish.
            node.tf_pose=Mock(side_effect=AssertionError('SLAM is stopped'))
            prepare_localization(node,dict(fleet=fleet_layout(joint,dual,[0.,-.6,0.])))
            self.assertEqual(node.finalize_targets.call_count,1)
            self.assertAlmostEqual(node.start_localization.call_args.args[0][0],1.1)
            self.assertTrue((d/'cooperative-targets.json').is_file())

    def test_joint_map_reuses_both_corrected_poses_on_retry(self):self.exercise()
    def test_single_mapper_uses_original_stationary_robot2_reference(self):self.exercise(joint=False)
    def test_single_robot_does_not_require_robot2_localization(self):self.exercise(joint=False,dual=False)

    def test_unmapped_robot_uses_fixed_initial_reference_and_its_own_odometry(self):
        fleet=fleet_layout(False,True,[0.,-.6,0.])
        reference=dict(mapping_start_pose=[1.,2.,0.],initial_odoms={'robot2':[0.,0.,0.]})
        priors=localization_priors(fleet,{'robot1':[9.,9.,1.]},{'robot2':[.2,0.,0.]},reference)
        self.assertEqual(priors['robot1'],[9.,9.,1.])
        self.assertEqual(priors['robot2'],[1.2,1.4,0.])
        with self.assertRaisesRegex(ValueError,'reference missing'):
            localization_priors(fleet,{'robot1':[9.,9.,1.]},{'robot2':[.2,0.,0.]})


class LocalizationTracking(unittest.TestCase):
    def test_forced_updates_only_during_stationary_convergence_and_one_pending(self):
        from concurrent.futures import Future
        from rclpy.time import Time
        from edgeai_tasks.localization_runtime import LocalizationRuntime
        velocity=NS(linear=NS(x=0.,y=0.),angular=NS(z=0.))
        odom=NS(header=NS(stamp=Time(seconds=10.).to_msg()),twist=NS(twist=velocity))
        client=Mock();pending=Future();client.call_async.return_value=pending
        n=NS(localizing=False,closing=False,nomotion={'robot1':client},nomotion_pending={},
             odom={'robot1':odom},now=lambda:10.,c={'max_data_age_s':.5})
        LocalizationRuntime.update_at_rest(n);client.call_async.assert_not_called()
        n.localizing=True
        LocalizationRuntime.update_at_rest(n)
        LocalizationRuntime.update_at_rest(n)
        self.assertEqual(client.call_async.call_count,1)
        pending.set_result(NS())
        velocity.linear.x=.1
        LocalizationRuntime.update_at_rest(n)
        self.assertEqual(client.call_async.call_count,1)
        velocity.linear.x=0.;n.now=lambda:11.
        LocalizationRuntime.update_at_rest(n)
        self.assertEqual(client.call_async.call_count,1)
        n.now=lambda:10.
        LocalizationRuntime.update_at_rest(n)
        self.assertEqual(client.call_async.call_count,2)

    def test_tracking_checks_live_tf_and_scan_without_forcing_amcl_pose_publication(self):
        from rclpy.time import Time
        from edgeai_tasks.localization_runtime import LocalizationRuntime
        cov=[0.]*36
        n=NS(localization={'robot1':NS(header=NS(stamp=Time(seconds=1.).to_msg()),
                                      pose=NS(covariance=cov))},localizing=False,
             now=lambda:10.,c={'max_data_age_s':.5,'max_std_xy_m':.15,
                              'max_std_yaw_rad':.2,'min_scan_match_ratio':.55},
             buffer=Mock(),laser_match=Mock(return_value=.9))
        n.buffer.lookup_transform.return_value=NS(header=NS(stamp=Time(seconds=9.9).to_msg()))
        self.assertTrue(LocalizationRuntime.localized(n,'robot1'))
        n.localizing=True
        self.assertFalse(LocalizationRuntime.localized(n,'robot1'))
        n.localizing=False;n.laser_match.return_value=.1
        self.assertFalse(LocalizationRuntime.localized(n,'robot1'))
        n.laser_match.return_value=.9
        n.buffer.lookup_transform.return_value=NS(header=NS(stamp=Time(seconds=9.).to_msg()))
        self.assertFalse(LocalizationRuntime.localized(n,'robot1'))
        with self.assertRaisesRegex(ValueError,'TF time unavailable'):
            LocalizationRuntime.localized(n,'robot1',require_data=True)
        n.buffer.lookup_transform.return_value=NS(header=NS(stamp=Time(seconds=10.01).to_msg()))
        with self.assertRaisesRegex(ValueError,'TF time unavailable'):
            LocalizationRuntime.localized(n,'robot1',require_data=True)
        n.buffer.lookup_transform.return_value=NS(header=NS(stamp=Time(seconds=9.9).to_msg()))
        cov[0]=1.
        self.assertFalse(LocalizationRuntime.localized(n,'robot1',require_data=True))

    def test_ready_owner_releases_initialization_subscriptions_and_action_clients(self):
        import rclpy
        import yaml
        from edgeai_tasks.localization_runtime import LocalizationRuntime
        rclpy.init()
        try:
            with tempfile.TemporaryDirectory() as temporary:
                config=yaml.safe_load((ROOT/'src/edgeai_navigation/config/cooperative_rescue.yaml').read_text())
                n=LocalizationRuntime(temporary,config)
                try:
                    self.assertEqual(len(n.initialization_subscriptions),7)
                    n.finish_initialization()
                    self.assertFalse(n.initialization_subscriptions)
                    self.assertTrue(n.nomotion_timer.is_canceled())
                    self.assertFalse(n.nav);self.assertFalse(n.nomotion)
                    topics={sub.topic_name for sub in n.subscriptions}
                    self.assertNotIn('/robot1/scan',topics)
                    self.assertNotIn('/map',topics)
                    self.assertIn('/robot1/amcl_pose',topics)
                    self.assertIn('/tf',topics)
                finally:n.destroy_node()
        finally:rclpy.shutdown()

    def test_c1_tree_reuses_prepared_localization(self):
        import xml.etree.ElementTree as ET
        tree=ET.parse(ROOT/'src/edgeai_mission/config/tasks/c1.xml')
        operations=[n.attrib['operation'] for n in tree.iter('TaskUnit')]
        self.assertIn('wait_navigation_ready',operations)
        self.assertNotIn('localize',operations)


if __name__=='__main__':unittest.main()
