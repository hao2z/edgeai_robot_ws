"""Callable peer-yield contracts, without Gazebo or live navigation servers."""
from concurrent.futures import Future
from pathlib import Path
import sys
import time
from types import SimpleNamespace as S
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'src/edgeai_navigation'))
from edgeai_nav.peer_yield import PeerYieldUnit


class PeerYieldTests(unittest.TestCase):
    def make_unit(self):
        grid = S(info=S(resolution=.1,width=50,height=40,
            origin=S(position=S(x=0.,y=0.),orientation=S(x=0.,y=0.,z=0.,w=1.))),
            data=[0 if x<18 or x>32 or 18<=y<=22 else 100 for y in range(40) for x in range(50)])
        settings=dict(enabled=True,yield_body_radius_m=.15,yield_max_cells=20000,
            yield_max_attempts=2,yield_sample_spacing_m=.2,yield_min_distance_m=1.,
            yield_max_distance_m=3.,yield_requester_clearance_m=1.,yield_route_clearance_m=.5,
            yield_max_candidates=20,yield_navigation_timeout_s=90.)
        poses={'':[.55,2.05,0.],'robot2':[2.55,2.05,0.]}
        twist=S(linear=S(x=0.,y=0.),angular=S(z=0.))
        odom=S(header=S(stamp=S(sec=1,nanosec=0)),twist=S(twist=twist))
        events=[]
        n=S(c=dict(yielding=settings,max_data_age_s=.5),map=grid,odom={r:odom for r in poses},
            states={r:dict(phase='IDLE') for r in poses},handles=[],footprint=[[-.1,-.1],[.1,-.1],[.1,.1],[-.1,.1]])
        n.now=lambda:1.;n.check_owner=lambda:None;n.begin_yield=lambda peer:None
        n.tf_pose=lambda frame:list(poses['robot2' if frame.startswith('robot2/') else ''])
        n.event=lambda phase,**data:events.append((phase,data))
        unit=PeerYieldUnit(n,{})
        return unit,n,poses,events

    def test_only_actual_peer_blockage_creates_yield_context(self):
        u,n,poses,_=self.make_unit()
        u.diagnose([dict(robot='',goal=[4.45,2.05,0.])])
        self.assertEqual(u.context['peer'],'robot2')
        poses['robot2']=[1.,1.,0.]
        with self.assertRaisesRegex(RuntimeError,'No confirmed'):u.diagnose([dict(robot='',goal=[4.45,2.05,0.])])
        self.assertIsNone(u.context)

    def test_active_peer_cannot_be_ordered_to_yield(self):
        u,n,_,_=self.make_unit();n.states['robot2']['phase']='NAVIGATING'
        with self.assertRaisesRegex(RuntimeError,'No confirmed'):u.diagnose([dict(robot='',goal=[4.45,2.05,0.])])

    def test_stopped_active_requester_can_request_idle_peer_yield(self):
        u,n,_,_=self.make_unit();n.states['']['phase']='ACTIVE'
        u.diagnose([dict(robot='',goal=[4.45,2.05,0.])])
        self.assertEqual(u.context['peer'],'robot2')

    def test_native_rejection_consumes_attempt_without_dispatch(self):
        u,n,_,_=self.make_unit();u.diagnose([dict(robot='',goal=[4.45,2.05,0.])])
        n.goal_pose=lambda p:p;n.plan={'robot2':object()}
        n.action=lambda *args:S(status=6,result=S(error_code=208,path=S(poses=[])))
        with self.assertRaisesRegex(RuntimeError,'Native planner'):u.select()
        self.assertEqual(u.attempts['robot2'],1)
        self.assertEqual(u.pending,{})
        self.assertEqual(n.handles,[])

    def completed(self,u,n,poses,after,status=4):
        future=Future();future.set_result(S(status=status,result=S(error_code=0)))
        handle=object();u.pending['robot2']=(handle,future,[1.,1.,0.]);n.handles.append(handle)
        poses['robot2']=after
        return u.poll()

    def test_action_success_without_movement_does_not_release_route(self):
        u,n,poses,_=self.make_unit();u.diagnose([dict(robot='',goal=[4.45,2.05,0.])])
        with self.assertRaisesRegex(RuntimeError,'did not restore'):self.completed(u,n,poses,poses['robot2'])
        self.assertEqual(u.pending,{})

    def test_measured_yield_releases_route_and_cleans_action(self):
        u,n,poses,events=self.make_unit();u.diagnose([dict(robot='',goal=[4.45,2.05,0.])])
        self.assertTrue(self.completed(u,n,poses,[1.2,1.,0.])['complete'])
        self.assertTrue(events[-1][1]['route_released'])
        self.assertEqual(n.handles,[])

    def test_external_cancel_is_fatal(self):
        u,n,poses,_=self.make_unit();u.diagnose([dict(robot='',goal=[4.45,2.05,0.])])
        with self.assertRaises(RuntimeError) as error:self.completed(u,n,poses,poses['robot2'],5)
        self.assertEqual(error.exception.category,'fatal')

    def test_yield_uses_position_navigation_and_registers_pending_action(self):
        u,n,poses,_=self.make_unit();u.diagnose([dict(robot='',goal=[4.45,2.05,0.])])
        u.context['yield_goal']=[1.2,1.,0.]
        result=Future();sent=Future()
        handle=S(accepted=True,get_result_async=lambda:result)
        sent.set_result(handle)
        requests=[]
        u.navigation['robot2']=S(server_is_ready=lambda:True,
            send_goal_async=lambda goal:(requests.append(goal) or sent))
        n.goal_pose=lambda p:__import__('geometry_msgs.msg',fromlist=['PoseStamped']).PoseStamped()
        n.wait=lambda predicate,*args:self.assertTrue(predicate())
        self.assertFalse(u.dispatch()['complete'])
        self.assertEqual(u.pending['robot2'][0],handle)
        self.assertEqual(n.handles,[handle])
        self.assertEqual(len(requests),1)

    def test_timeout_requires_confirmed_terminal_before_retry(self):
        u,n,poses,_=self.make_unit();u.diagnose([dict(robot='',goal=[4.45,2.05,0.])])
        u.context['deadline']=time.monotonic()-1
        result=Future();cancel=Future();cancel.set_result(S())
        def cancel_goal():
            result.set_result(S(status=5));return cancel
        handle=S(cancel_goal_async=cancel_goal)
        u.pending['robot2']=(handle,result,[1.,1.,0.]);n.handles.append(handle)
        n.wait=lambda predicate,*args:self.assertTrue(predicate())
        with self.assertRaisesRegex(RuntimeError,'timed out'):u.poll()
        self.assertEqual(u.pending,{})
        self.assertEqual(n.handles,[])

    def test_unknown_timeout_terminal_is_fatal_and_keeps_handle_for_cleanup(self):
        u,n,poses,_=self.make_unit();u.diagnose([dict(robot='',goal=[4.45,2.05,0.])])
        u.context['deadline']=time.monotonic()-1
        result=Future();handle=S(cancel_goal_async=lambda:Future())
        u.pending['robot2']=(handle,result,[1.,1.,0.]);n.handles.append(handle)
        def timeout(*args):raise RuntimeError('terminal unavailable')
        n.wait=timeout
        with self.assertRaises(RuntimeError) as error:u.poll()
        self.assertEqual(error.exception.category,'fatal')
        self.assertIn('robot2',u.pending)
        self.assertEqual(n.handles,[handle])


if __name__=='__main__':unittest.main()
