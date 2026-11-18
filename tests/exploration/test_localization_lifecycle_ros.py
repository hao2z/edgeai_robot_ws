"""Real map/AMCL managers: dual-namespace bonds survive startup and reset.

No Gazebo, navigation goals or competition-domain participants are used.
"""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import unittest

import rclpy
from rclpy.time import Time
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ManageLifecycleNodes
from rosgraph_msgs.msg import Clock
import yaml

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'src/edgeai_mission'),str(ROOT/'src/edgeai_navigation'),
              str(ROOT/'src/edgeai_navigation/scripts')]
from edgeai_tasks.localization_runtime import LocalizationRuntime
from edgeai_nav.fleet_config import namespaced_config
from navigation_config import load_navigation


class LocalizationLifecycleROS(unittest.TestCase):
    def test_real_dual_amcl_bonds_and_restart(self):
        self.check_bonds(('robot1','robot2'))

    def test_single_robot_uses_same_lifecycle_layout(self):
        self.check_bonds(('robot1',))

    def check_bonds(self,namespaces):
        domain=169
        # Child ROS tools inherit both middleware and domain, matching the
        # parent context even when the shell has a competition-domain default.
        previous_domain=os.environ.get('ROS_DOMAIN_ID')
        os.environ['ROS_DOMAIN_ID']=str(domain)
        rclpy.init()
        clock_node=rclpy.create_node('localization_lifecycle_clock')
        clock=clock_node.create_publisher(Clock,'/clock',10)
        stop=threading.Event()
        def publish_clock():
            while not stop.is_set():
                clock.publish(Clock(clock=Time(seconds=time.monotonic()).to_msg()))
                stop.wait(.02)
        thread=threading.Thread(target=publish_clock);thread.start()
        node=None
        try:
            with tempfile.TemporaryDirectory(prefix='localization-bonds-') as temporary:
                directory=Path(temporary);(directory/'robot2').mkdir()
                (directory/'cooperative-map.pgm').write_bytes(b'P5\n20 20\n255\n'+bytes([254])*400)
                (directory/'cooperative-map.yaml').write_text(yaml.safe_dump(dict(
                    image='cooperative-map.pgm',resolution=.05,origin=[0.,0.,0.],
                    negate=0,occupied_thresh=.65,free_thresh=.196)))
                nav=load_navigation(ROOT/'src/edgeai_navigation/config','rpp')
                for ns in namespaces:
                    config=namespaced_config(nav,ns)
                    ((directory/'robot2' if ns=='robot2' else directory)/'nav2-runtime.yaml').write_text(yaml.safe_dump(config))
                config=yaml.safe_load((ROOT/'src/edgeai_navigation/config/cooperative_rescue.yaml').read_text())
                config['localization_namespaces']=list(namespaces)
                node=LocalizationRuntime(directory,config,node_name='localization_lifecycle_probe')
                node.deadline=time.monotonic()+70.
                with (directory/'launch.log').open('w+') as log:
                    node.process=subprocess.Popen(['ros2','launch','edgeai_navigation','cooperative_navigation.launch.py',
                        'directory:='+str(directory),'reuse_robot2_navigation:=true','localization_autostart:=false',
                        'enable_robot2:='+str(len(namespaces)>1).lower()],
                        env=dict(os.environ,ROS_DOMAIN_ID=str(domain)),stdout=log,stderr=log,start_new_session=True)
                    try:
                        # Require real service round trips, rather than only
                        # discovery, before sending a lifecycle mutation.
                        for name in tuple(ns+'/amcl' for ns in namespaces):
                            client=node.create_client(GetState,'/'+name+'/get_state')
                            try:node.service(client,GetState.Request(),10.)
                            finally:node.destroy_client(client)
                        for command in (ManageLifecycleNodes.Request.STARTUP,
                                        ManageLifecycleNodes.Request.RESET,ManageLifecycleNodes.Request.STARTUP):
                            node.localization_transition(command)
                            expected=(State.PRIMARY_STATE_UNCONFIGURED if command==ManageLifecycleNodes.Request.RESET
                                      else State.PRIMARY_STATE_ACTIVE)
                            for name in ('map_server',)+tuple(ns+'/amcl' for ns in namespaces):
                                client=node.create_client(GetState,'/'+name+'/get_state')
                                try:self.assertEqual(node.service(client,GetState.Request()).current_state.id,expected)
                                finally:node.destroy_client(client)
                        # Longer than the default 4s bond timeout: not just ACTIVE once.
                        end=time.monotonic()+5.
                        node.wait(lambda:time.monotonic()>=end,6.)
                        log.flush();log.seek(0);evidence=log.read()
                        self.assertNotIn('unable to be reached',evidence)
                        self.assertNotIn('Failed to bring up',evidence)
                        self.assertGreaterEqual(evidence.count('Server amcl connected with bond'),2*len(namespaces),evidence)
                    except Exception:
                        log.flush();log.seek(0);print(log.read())
                        raise
                    finally:node.close()
        finally:
            stop.set();thread.join(timeout=2.)
            if node:node.destroy_node()
            clock_node.destroy_node();rclpy.shutdown()
            if previous_domain is None:os.environ.pop('ROS_DOMAIN_ID',None)
            else:os.environ['ROS_DOMAIN_ID']=previous_domain
