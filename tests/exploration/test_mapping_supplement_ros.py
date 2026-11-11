"""Actual mapping stage adapters and authority/Nav2 protocol, without simulation."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import time
import unittest
import yaml
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.task import Future
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String, Int32
from std_srvs.srv import SetBool, Trigger
from rosgraph_msgs.msg import Clock
from tf2_ros import TransformBroadcaster

ROOT=Path(__file__).resolve().parents[2]


class MappingSupplementROS(unittest.TestCase):
    def exercise(self,aborted=False,no_points=False,timed_out=False):
        rclpy.init();peer=rclpy.create_node('mapping_adapter_peer')
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        state=dict(command_sequence=1,mode='STOPPED',phase='IDLE',explorer_paused=True)
        states=peer.create_publisher(String,'/navigation_authority/state',qos)
        maps=peer.create_publisher(OccupancyGrid,'/map',qos)
        odoms=peer.create_publisher(Odometry,'/odom',10)
        clocks=peer.create_publisher(Clock,'/clock',10)
        monitors=peer.create_publisher(Int32,'/explore/available_frontiers',1)
        pulses=[];cancellations=[]
        tf=TransformBroadcaster(peer);goals=[];modes=[]
        def mode(req,res):
            state.update(command_sequence=state['command_sequence']+1,
                         mode='MANUAL' if req.data else 'AUTO',explorer_paused=req.data)
            modes.append(state['mode']);res.success=True;return res
        def stop(req,res):
            state.update(command_sequence=state['command_sequence']+1,mode='STOPPED',explorer_paused=True)
            res.success=True;return res
        services=[peer.create_service(SetBool,'/navigation_authority/set_manual',mode),
                  peer.create_service(Trigger,'/navigation_authority/stop',stop)]
        async def execute(handle):
            goals.append(handle.request.pose)
            if timed_out:
                until=time.monotonic()+5.
                while not handle.is_cancel_requested and time.monotonic()<until:
                    pulse=Future();pulses.append(pulse);await pulse
                if handle.is_cancel_requested:
                    cancellations.append(True);handle.canceled();return NavigateToPose.Result()
            if aborted:handle.abort()
            else:handle.succeed()
            result=NavigateToPose.Result();result.error_code=208 if aborted else 0
            return result
        servers=[ActionServer(peer,NavigateToPose,topic,execute,cancel_callback=lambda _:CancelResponse.ACCEPT) for topic in
                 ('/navigate_to_pose','/navigation/rescue_to_pose')]
        def publish():
            stamp=peer.get_clock().now().to_msg();clocks.publish(Clock(clock=stamp))
            states.publish(String(data=json.dumps(state)))
            monitors.publish(Int32(data=0))
            for pulse in pulses:
                if not pulse.done():pulse.set_result(True)
            pulses.clear()
            grid=OccupancyGrid();grid.header.frame_id='map';grid.header.stamp=stamp
            grid.info.width=grid.info.height=40;grid.info.resolution=.1
            grid.info.origin.position.x=grid.info.origin.position.y=-2.
            grid.info.origin.orientation.w=1.;grid.data=[100 if no_points else 0]*1600
            grid.data[20*40+20]=0;maps.publish(grid)
            odom=Odometry();odom.header.stamp=stamp;odom.pose.pose.orientation.w=1.;odoms.publish(odom)
            t=TransformStamped();t.header.stamp=stamp;t.header.frame_id='map';t.child_frame_id='base_footprint'
            t.transform.rotation.w=1.;tf.sendTransform(t)
        timer=peer.create_timer(.05,publish);process=None
        try:
            with tempfile.TemporaryDirectory() as temporary:
                d=Path(temporary)
                rescue=yaml.safe_load((ROOT/'src/edgeai_navigation/config/midterm_single_rescue.yaml').read_text())
                rescue['search_clearance_m']=.4
                (d/'rescue-config.yaml').write_text(yaml.safe_dump(rescue))
                config=d/'config.json';config.write_text(json.dumps(dict(directory=str(d),project_root=str(ROOT),backend='frontier',mapping=dict(reposition_timeout_s=.2 if timed_out else 5.))))
                for step in ('supplement_mapping','resume_exploration'):
                    with (d/'worker.log').open('a+') as log:
                        process=subprocess.Popen(['python3',str(ROOT/'src/edgeai_mission/scripts/mission_stage.py'),
                            '--step',step,'--config',str(config)],stdout=log,stderr=log)
                        until=time.monotonic()+20.
                        while process.poll() is None and time.monotonic()<until:rclpy.spin_once(peer,timeout_sec=.02)
                        if process.poll() is None:process.terminate();process.wait(timeout=10.)
                        log.seek(0);output=log.read()
                    if no_points:
                        self.assertEqual(process.returncode,1,output)
                        result=json.loads((d/f'stage-{step}-result.json').read_text())
                        self.assertIn('no untried clear repositioning point',result['reason'])
                        self.assertFalse(goals);self.assertNotIn('AUTO',modes)
                        return
                    self.assertEqual(process.returncode,0,output)
                    self.assertTrue(json.loads((d/f'stage-{step}-result.json').read_text())['success'],output)
                self.assertEqual(len(goals),1)
                if timed_out:self.assertTrue(cancellations,'AUTO resumed before canceling supplemental goal')
                self.assertGreaterEqual((goals[0].pose.position.x**2+goals[0].pose.position.y**2)**.5,.6)
                self.assertEqual(len(json.loads((d/'mapping-supplement-positions.json').read_text())),1)
                self.assertEqual(modes,['MANUAL','AUTO']);self.assertEqual(state['mode'],'AUTO')
        finally:
            if process and process.poll() is None:process.kill();process.wait()
            for server in servers:server.destroy()
            peer.destroy_node();rclpy.shutdown()

    def test_reposition_then_resume_with_confirmed_authority(self):self.exercise()

    def test_nav2_aborted_reposition_still_allows_frontier_reevaluation(self):self.exercise(aborted=True)

    def test_no_clear_point_reports_incomplete_without_motion(self):self.exercise(no_points=True)

    def test_reposition_timeout_cancels_then_rechecks_frontiers(self):self.exercise(timed_out=True)


if __name__=='__main__':
    if os.environ.get('ROS_DOMAIN_ID','0') in ('0','86'):raise SystemExit('Use an isolated test domain')
    unittest.main()
