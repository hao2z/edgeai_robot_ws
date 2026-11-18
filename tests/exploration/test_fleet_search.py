"""Fleet search proposals must preserve independent visits and shared coverage."""
import copy
import json
import os
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import yaml

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'src/edgeai_mission'),str(ROOT/'src/edgeai_exploration')]
from edgeai_tasks.fleet_search import FleetSearchUnits, clone_candidates, commit_candidate, missing_targets
from edgeai_tasks.search_candidates import SpatialCandidates
from edgeai_tasks.exploration import AssignmentPolicy


class FleetSearchTests(unittest.TestCase):
    def test_unselected_proposals_do_not_consume_visits(self):
        base=SpatialCandidates([(1.,0.),(4.,0.),(7.,0.)])
        trial=clone_candidates(base)
        trial.next_view((0,0,0))
        before=clone_candidates(trial);trial.next_view((0,0,0))
        selected=commit_candidate(base,before,trial)
        self.assertEqual(base.attempted,set())
        self.assertEqual(selected.attempted,{1})
        self.assertEqual(selected.next_view((0,0,0))['x'],1.)

    def test_parallel_planner_commits_only_selected_suspect(self):
        base=SimpleNamespace(baseline=SpatialCandidates([]),used=[],used_positions=[],visits=0,active_id=None)
        before=clone_candidates(base);before.used=[('a',0,1,0)];before.used_positions=[(1,0)];before.visits=1
        after=clone_candidates(before);after.used.append(('b',0,-1,0));after.used_positions.append((4,0));after.visits=2;after.active_id='b'
        selected=commit_candidate(base,before,after)
        self.assertEqual(selected.used,[('b',0,-1,0)])
        self.assertEqual(selected.used_positions,[(4,0)])
        self.assertEqual(selected.visits,1)

    def test_targets_need_both_ids_positions_and_types(self):
        required=[dict(id=1,type='normal'),dict(id=2,type='cooperative')]
        registry=dict(frame_id='map',targets=[dict(id=1,type='normal',x=0.,y=0.,yaw=0.)])
        self.assertEqual(missing_targets(registry,required),required[1:])
        registry['targets'].append(dict(id=2,type='cooperative',x=1.,y=0.,yaw=0.))
        self.assertEqual(missing_targets(registry,required),[])
        registry['targets'][1]['yaw']=float('nan')
        self.assertEqual(missing_targets(registry,required),required[1:])

    def test_each_normal_target_is_required_before_search_completes(self):
        required = [dict(id=i, type='normal') for i in (1, 2, 3)] + [dict(id=10, type='cooperative')]
        registry = dict(frame_id='map', targets=[dict(t, x=0., y=0., yaw=0.) for t in required if t['id'] != 3])
        self.assertEqual(missing_targets(registry, required), required[2:3])
        registry['targets'].append(dict(id=3, type='normal', x=1., y=0., yaw=0.))
        self.assertEqual(missing_targets(registry, required), [])

    def make_unit(self):
        unit=object.__new__(FleetSearchUnits); unit.robots=('robot1','robot2')
        unit.check_owner=lambda:None;unit.condition_complete=False
        unit.map=SimpleNamespace(header=SimpleNamespace(frame_id='map'),info=SimpleNamespace(
            origin=SimpleNamespace(orientation=SimpleNamespace(x=0.,y=0.,z=0.))))
        unit.visited=[];unit.pending={};unit.waiting_sweeps={};unit.yield_context=None
        unit.states={r:dict(phase='IDLE') for r in ('robot1','robot2')}
        unit.settle_until={};unit.now_ns=lambda:100
        unit.pose=lambda r:dict(x=0. if r=='robot1' else 5.,y=0.,yaw=0.)
        unit.infos=dict(robot1=True,robot2=True);unit.adapter=lambda r:None
        unit.planners={r:SpatialCandidates([(1.,0.),(4.,0.)]) for r in unit.states}
        unit.search=dict(search_max_positions=6);unit.policy=AssignmentPolicy()
        unit.visits=dict(robot1=0,robot2=0);unit.retry_goals=dict(robot1=[],robot2=[])
        unit.navigation_client=lambda *a:SimpleNamespace(server_is_ready=lambda:True)
        sent=[];unit._send=lambda r,g,p,k:sent.append((r,g))
        return unit,sent

    def test_both_robots_receive_distinct_candidates(self):
        unit,sent=self.make_unit()
        unit.dispatch()
        self.assertEqual([(r,g['x']) for r,g in sent],[('robot1',1.),('robot2',4.)])
        self.assertEqual(unit.planners['robot1'].attempted,{0})
        self.assertEqual(unit.planners['robot2'].attempted,{1})

    def suspect_unit(self, positions):
        unit,sent=self.make_unit()
        unit.states['robot2']['phase']='BUSY'
        unit.search['search_suspects']=dict(min_view_spacing_m=.10)
        class BoundedCandidates:
            def __init__(self):
                self.baseline=SpatialCandidates([])
                self.used=[];self.used_positions=[];self.visits=0;self.active_id=None
            def next_view(self, pose):
                if self.visits>=2:return self.baseline.next_view(pose)
                for index,x in enumerate(positions):
                    if index in self.used:continue
                    self.used.append(index);self.used_positions.append((x,0.))
                    self.visits+=1;self.active_id='a'
                    return dict(x=x,y=0.,yaw=0.,strategy='suspected_panel')
                return None
        unit.planners['robot1']=BoundedCandidates()
        return unit,sent

    def test_completed_view_allows_nearby_angle_but_filters_duplicate(self):
        unit,sent=self.suspect_unit([1.05,1.20])
        unit.visited=[dict(x=1.,y=0.)]
        unit.dispatch()
        self.assertEqual([g['x'] for _,g in sent],[1.20])
        self.assertEqual(unit.planners['robot1'].visits,1)
        self.assertEqual(unit.planners['robot1'].used,[1])

    def test_rejected_proposals_do_not_exhaust_observation_budget(self):
        unit,sent=self.suspect_unit([1.01,1.02,1.03,1.25])
        unit.visited=[dict(x=1.,y=0.)]
        unit.dispatch()
        self.assertEqual([g['x'] for _,g in sent],[1.25])
        self.assertEqual(unit.planners['robot1'].visits,1)
        self.assertEqual(unit.planners['robot1'].used_positions,[(1.25,0.)])

    def test_active_and_sweeping_peer_still_reserve_full_spacing(self):
        for sweeping in (False,True):
            with self.subTest(sweeping=sweeping):
                unit,sent=self.suspect_unit([1.2,2.])
                goal=dict(x=1.,y=0.,strategy='suspected_panel')
                if sweeping:unit.waiting_sweeps['robot2']=goal
                else:unit.pending['robot2']=dict(goal=goal)
                unit.dispatch()
                self.assertEqual([g['x'] for _,g in sent],[2.])

    def test_retry_views_use_completed_spacing_too(self):
        unit,sent=self.suspect_unit([])
        unit.visited=[dict(x=1.,y=0.)]
        unit.retry_goals['robot1']=[dict(x=1.2,y=0.,strategy='suspected_panel')]
        unit.dispatch()
        self.assertEqual([g['x'] for _,g in sent],[1.2])

    def test_search_config_allows_closer_concurrent_views(self):
        settings=yaml.safe_load((ROOT/'src/edgeai_mission/config/search.yaml').read_text())
        policy=AssignmentPolicy(settings)
        goal=dict(x=1.5,y=0.)
        self.assertEqual(policy.allocate(dict(robot1=[goal]),dict(robot1=dict(x=0.,y=0.)),
                                        dict(robot2=dict(x=1.,y=0.)),0.),dict(robot1=goal))
        self.assertEqual(policy.allocate(dict(robot1=[dict(x=1.3,y=0.)]),
                                        dict(robot1=dict(x=0.,y=0.)),dict(robot2=dict(x=1.,y=0.)),0.),{})
        self.assertAlmostEqual(policy.history_radius,.10)



if __name__=='__main__':unittest.main()
