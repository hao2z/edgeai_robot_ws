"""Exercise the native behavior action loop on synthetic maps in domain 198."""
import json,math,os,subprocess,tempfile,time,unittest
from pathlib import Path
import rclpy,yaml
from rclpy.action import ActionClient
from geometry_msgs.msg import TransformStamped,Twist
from nav2_msgs.msg import Costmap
from nav2_msgs.action import DriveOnHeading
from lifecycle_msgs.srv import ChangeState
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile,DurabilityPolicy


class OccupiedEscapeROS(unittest.TestCase):
    def test_exit_padding_and_actual_refuse_unknown_and_stop_without_motion(self):
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'),'198')
        rclpy.init();node=rclpy.create_node('escape_test_source')
        proc=None
        try:
            broadcaster=TransformBroadcaster(node)
            pub=node.create_publisher(Costmap,'/local_costmap/costmap_raw',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            state={'x':.5,'v':0.,'mode':'padding','move':True,'nonzero':0}
            def command(m):
                state['v']=m.linear.x
                if abs(m.linear.x)>1e-5:state['nonzero']+=1
            sub=node.create_subscription(Twist,'/cmd_vel',command,10)
            last=time.monotonic()
            def tick():
                nonlocal last
                now=time.monotonic();dt=min(.05,now-last);last=now
                if state['move']:state['x']+=state['v']*dt
                t=TransformStamped();t.header.frame_id='odom';t.child_frame_id='base_footprint';t.header.stamp=node.get_clock().now().to_msg()
                t.transform.translation.x=state['x'];t.transform.translation.y=.5;t.transform.rotation.w=1.;broadcaster.sendTransform(t)
                m=Costmap();m.header.frame_id='odom';m.header.stamp=t.header.stamp
                m.metadata.resolution=.005;m.metadata.size_x=m.metadata.size_y=200;m.metadata.origin.orientation.w=1.
                data=[0]*40000;column=121 if state['mode']=='actual' else 123
                for y in range(200):data[y*200+column]=255 if state['mode']=='unknown' else 254
                if state['mode']=='partial':
                    for y in range(200):
                        for x in range(110,124):data[y*200+x]=254
                m.data=data;pub.publish(m)
            timer=node.create_timer(.02,tick)
            def wait(future,seconds=10):
                end=time.monotonic()+seconds
                while not future.done() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
                if not future.done():raise RuntimeError('escape ROS test timeout')
                return future.result()
            def spin(seconds):
                end=time.monotonic()+seconds
                while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
            with tempfile.TemporaryDirectory() as tmp:
                params={'behavior_server':{'ros__parameters':{'use_sim_time':False,'cycle_frequency':10.,
                    'local_frame':'odom','global_frame':'odom','robot_base_frame':'base_footprint','transform_tolerance':.1,
                    'enable_stamped_cmd_vel':False,'behavior_plugins':['escape_occupied'],
                    'escape_occupied':{'plugin':'edgeai_navigation::OccupiedEscape','footprint_padding':.01,
                    'actual_footprint':'[[0.1125,0.165],[-0.3125,0.165],[-0.3125,-0.165],[0.1125,-0.165]]'}}}}
                file=Path(tmp)/'params.yaml';file.write_text(yaml.safe_dump(params))
                with (Path(tmp)/'behavior.log').open('w') as log:
                    proc=subprocess.Popen(['ros2','run','nav2_behaviors','behavior_server','--ros-args','--params-file',str(file)],stdout=log,stderr=log)
                    lifecycle=node.create_client(ChangeState,'/behavior_server/change_state')
                    self.assertTrue(lifecycle.wait_for_service(timeout_sec=10))
                    for transition in (1,3):
                        req=ChangeState.Request();req.transition.id=transition
                        self.assertTrue(wait(lifecycle.call_async(req)).success)
                    action=ActionClient(node,DriveOnHeading,'/escape_occupied')
                    self.assertTrue(action.wait_for_server(timeout_sec=5));spin(.3)
                    def attempt(mode,move):
                        state.update(x=.5,v=0.,mode=mode,move=move,nonzero=0);spin(.2)
                        request=DriveOnHeading.Goal();request.target.x=-.05;request.speed=.03;request.time_allowance.sec=8
                        handle=wait(action.send_goal_async(request));self.assertTrue(handle.accepted)
                        result=wait(handle.get_result_async());spin(.1)
                        self.assertAlmostEqual(state['v'],0.,places=5)
                        return result
                    result=attempt('padding',True)
                    self.assertEqual(result.status,4,(Path(tmp)/'behavior.log').read_text())
                    self.assertGreater(state['nonzero'],0);self.assertLess(state['x'],.5)
                    self.assertGreater(state['x'],.44)
                    result=attempt('actual',True)
                    self.assertEqual(result.status,4,(Path(tmp)/'behavior.log').read_text())
                    self.assertGreater(state['nonzero'],0);self.assertLess(state['x'],.49)
                    result=attempt('unknown',True)
                    self.assertEqual(result.status,6);self.assertEqual(state['nonzero'],0)
                    result=attempt('partial',True)
                    self.assertEqual(result.status,4,(Path(tmp)/'behavior.log').read_text())
                    self.assertGreater(state['nonzero'],0);self.assertLess(state['x'],.46)
                    self.assertIn('OCCUPIED_ESCAPE_PROGRESS',(Path(tmp)/'behavior.log').read_text())
                    for mode in ('padding','actual'):
                        result=attempt(mode,False)
                        self.assertEqual(result.status,6);self.assertGreater(state['nonzero'],0)
        finally:
            if proc:
                proc.terminate()
                try:proc.wait(timeout=5)
                except subprocess.TimeoutExpired:proc.kill();proc.wait()
            node.destroy_node();rclpy.shutdown()


if __name__=='__main__':unittest.main()
