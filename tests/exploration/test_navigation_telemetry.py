"""ROS observations must preserve recovery evidence without commanding motion."""
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from action_msgs.msg import GoalStatus,GoalStatusArray
from nav_msgs.msg import Odometry

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'src/edgeai_navigation/scripts'))
from navigation_telemetry import NavigationTelemetry


class Telemetry(unittest.TestCase):
    def setUp(self):
        self.events=[];self.now=1_000_000_000
        self.node=SimpleNamespace(current=object(),distance=2.,
            create_subscription=lambda *args:None,
            get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=self.now)),
            event=lambda name,message,**data:self.events.append((name,data)))
        self.observer=NavigationTelemetry(self.node)

    def pose(self,x=0.,frame='odom'):
        m=Odometry();m.header.frame_id=frame;m.child_frame_id='base_footprint'
        m.header.stamp.sec=self.now//10**9;m.pose.pose.position.x=x;m.pose.pose.orientation.w=1.
        self.observer.odometry(m)

    def status(self,kind,status):
        self.observer.action_status(kind,GoalStatusArray(status_list=[GoalStatus(status=status)]))

    def test_recovery_requires_motion_then_navigation_progress(self):
        self.pose();self.status('backup',2)
        self.pose(.1);self.status('backup',4)
        self.assertTrue(self.observer.recovery.movement_verified)
        self.assertFalse(self.observer.snapshot()['recovered'])
        self.status('follow_path',2);self.pose(.2);self.node.distance=1.8
        self.observer.note_navigation()
        self.assertTrue(self.observer.snapshot()['recovered'])
        self.assertEqual([n for n,_ in self.events].count('NAVIGATION_RESTORED'),1)
        self.observer.note_navigation()
        self.assertEqual([n for n,_ in self.events].count('NAVIGATION_RESTORED'),1)

    def test_frame_change_invalidates_motion_and_reset_preserves_execution(self):
        self.pose();self.status('spin',2)
        self.pose(.2,'other_odom');self.status('spin',4)
        self.assertFalse(self.observer.recovery.movement_verified)
        current=self.node.current;self.observer.reset()
        self.assertIsNone(self.observer.snapshot());self.assertFalse(self.observer.following)
        self.assertIs(self.node.current,current)


if __name__=='__main__':unittest.main()
