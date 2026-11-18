"""Bounded release to joint task ownership, for both one-robot and two-robot fleets."""
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/edgeai_mission/scripts'))
from mapping_supplement import resume_exploration


class ResumeProtocol(unittest.TestCase):
    def exercise(self,scenario='success',joint=True):
        import json
        now=[100.];requests=[];callback=[None];destroyed=[]
        initial=dict(command_sequence=7,mode='STOPPED',phase='IDLE',explorer_paused=True)
        node=SimpleNamespace(state=dict(initial),state_received=now[0])
        def request(name,req):
            self.assertFalse(hasattr(req,'data'))
            requests.append(name)
            success=not (name=='robot2' and scenario=='rejected')
            return SimpleNamespace(done=lambda:True,result=lambda:SimpleNamespace(success=success))
        def client(name):
            return SimpleNamespace(service_is_ready=lambda:True,call_async=lambda req:request(name,req))
        node.manual=client('robot1')
        node.stop=client('robot1')
        node.create_client=lambda *args:client('robot2')
        def subscribe(message,topic,receive,qos):
            self.assertEqual(topic,'/robot2/navigation_authority/state')
            callback[0]=receive;return 'subscription'
        node.create_subscription=subscribe
        node.destroy_subscription=lambda s:destroyed.append(s)
        node.destroy_client=lambda c:destroyed.append('client')
        def wait(predicate,timeout):
            until=now[0]+timeout
            while now[0]<until:
                now[0]+=.05
                state=dict(initial)
                if requests:
                    state.update(command_sequence=8,mode='STOPPED',
                        phase='IDLE',owner='',explorer_paused=True)
                node.state=dict(state);node.state_received=now[0]
                if callback[0] is not None:
                    if requests and scenario=='superseded':state['command_sequence']=9
                    if requests and scenario=='unknown':state['phase']='UNKNOWN'
                    if requests and scenario=='canceling':state.update(phase='CANCELING',cancel_requested=True)
                    if requests and scenario=='old_sequence':state['command_sequence']=7
                    # A retained baseline alone is never a task resume acknowledgement.
                    if not requests or scenario!='missing_ack':
                        callback[0](SimpleNamespace(data=json.dumps(state)))
                if predicate():return
            raise RuntimeError('Timed out waiting for task transition')
        node.wait=wait
        with patch('mapping_supplement.time.monotonic',side_effect=lambda:now[0]):
            try:
                result=resume_exploration(node,joint=joint,timeout_s=.8)
                self.assertEqual(result['status'],'ready_for_exploration_task')
            finally:
                self.assertLessEqual(now[0]-100.,.9)
                self.assertEqual(requests,['robot1','robot2'] if joint else ['robot1'])
                self.assertEqual(destroyed,['subscription','client'] if joint else [])

    def test_joint_release_confirms_both_stopped(self):self.exercise()

    def test_single_mode_only_resumes_primary(self):self.exercise(joint=False)

    def test_robot2_rejected_resume_fails(self):
        with self.assertRaisesRegex(RuntimeError,'robot2: authority rejected'):self.exercise('rejected')

    def test_operator_command_supersedes_resume(self):
        with self.assertRaisesRegex(RuntimeError,'robot2: authority changed'):self.exercise('superseded')

    def test_missing_or_invalid_acknowledgement_is_bounded(self):
        for scenario in ('missing_ack','old_sequence','unknown','canceling'):
            with self.subTest(scenario=scenario),self.assertRaisesRegex(RuntimeError,'Timed out'):
                self.exercise(scenario)


if __name__=='__main__':unittest.main()
