"""Feed isolated ROS topics; no simulator, navigation goals or robot commands."""
import json,os,subprocess,sys,tempfile,time,unittest
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy
from geometry_msgs.msg import Twist,PointStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import FollowPath
from action_msgs.msg import GoalStatusArray,GoalStatus
from std_msgs.msg import String
from rcl_interfaces.msg import Log

ROOT=Path(__file__).resolve().parents[2]


class ProbeROS(unittest.TestCase):
    def test_active_controller_feedback_and_velocity_chain(self):
        # The test must be invoked with its own domain, never the live robot's.
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'),'197')
        rclpy.init();node=rclpy.create_node('probe_test_source')
        proc=None
        try:
            pubs={name:node.create_publisher(Twist,'/'+name,10) for name in ('cmd_vel_nav','cmd_vel_smoothed','cmd_vel')}
            odom=node.create_publisher(Odometry,'/odom',10)
            state=node.create_publisher(String,'/navigation_authority/state',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            status=node.create_publisher(GoalStatusArray,'/navigation_backend/follow_path/_action/status',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            feedback=node.create_publisher(FollowPath.Impl.FeedbackMessage,'/navigation_backend/follow_path/_action/feedback',10)
            carrot=node.create_publisher(PointStamped,'/lookahead_point',10)
            logs=node.create_publisher(Log,'/rosout',10)
            with tempfile.TemporaryDirectory() as tmp:
                session=Path(tmp);(session/'session.json').write_text(json.dumps({'domain':197}))
                (session/'nav2-runtime.yaml').write_text('controller_server:\n  ros__parameters:\n    controller_plugins: [FollowPath]\n    FollowPath:\n      plugin: Shim\n      primary_controller: MatchedRPP\n')
                proc=subprocess.Popen([sys.executable,str(ROOT/'scripts/ros/controller_probe.py'),
                    '--session',str(session),'--duration','3','--hz','10'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
                deadline=time.monotonic()+12
                while proc.poll() is None and time.monotonic()<deadline:
                    for name,pub in pubs.items():
                        msg=Twist();msg.linear.x=-.2 if name=='cmd_vel_nav' else -.1;pub.publish(msg)
                    odom.publish(Odometry())
                    state.publish(String(data=json.dumps({'phase':'ACTIVE','arrival_policy':'position','following_path':True})))
                    item=GoalStatus();item.goal_info.goal_id.uuid=[1]*16;item.status=2
                    status.publish(GoalStatusArray(status_list=[item]))
                    msg=FollowPath.Impl.FeedbackMessage();msg.goal_id.uuid=[1]*16
                    msg.feedback.distance_to_goal=.4;msg.feedback.speed=.1;feedback.publish(msg)
                    point=PointStamped();point.header.frame_id='base_footprint';point.point.x=-.3;carrot.publish(point)
                    logs.publish(Log(name='controller_server',level=40,msg='Failed to make progress'))
                    rclpy.spin_once(node,timeout_sec=.05)
                if proc.poll() is None:proc.terminate()
                output=proc.communicate(timeout=5)[0]
                self.assertEqual(proc.returncode,0,output)
                report=json.loads(next((session/'controller-diagnostics').glob('*/summary.json')).read_text())
                view=report['local_controller']['latest_observation']
                self.assertEqual(view['controller_id'],'FollowPath')
                self.assertEqual(view['follow_path']['name'],'EXECUTING')
                self.assertAlmostEqual(view['feedback']['distance_to_goal_m'],.4,places=5)
                self.assertLess(view['lookahead']['x'],0)
                self.assertGreater(report['difference_samples']['smoother_difference'],0)
                self.assertGreater(report['difference_samples']['command_without_measured_motion'],0)
                self.assertGreater(report['warning_counts']['progress'],0)
                self.assertEqual(report['channels']['cmd_vel']['min_v'],-.1)
        finally:
            if proc and proc.poll() is None:proc.kill();proc.wait()
            node.destroy_node();rclpy.shutdown()


if __name__=='__main__':unittest.main()
