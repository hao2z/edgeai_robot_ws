from pathlib import Path
from types import SimpleNamespace as NS
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'src/edgeai_navigation/scripts'))
from suspect_search import normal_views,ParallelCandidates,without_known
from frustum_search import Camera

CAMERA=Camera(640,480,554.,554.,320.,240.,(.1,0.,.35),(0.,0.,1.,-1.,0.,0.,0.,-1.,0.))


class SearchHints(unittest.TestCase):
    def test_surface_hypothesis_first_then_opposite_side_before_nearby_angles(self):
        class Baseline:
            positions=[]
            def next_view(self,robot):return dict(strategy='frustum')
        grid=NS(data=[0]*1600,info=NS(width=40,height=40,resolution=.1,origin=NS(position=NS(x=0.,y=0.))))
        node=NS(c={},search_map=grid,read_target_registry=lambda **kw:{},
            search_suspects=dict(frame_id='map',stamp=5.,candidates=[dict(id=0,
                position=[2.,2.,.25],normal=[1.,0.],normal_kind='surface',normal_ambiguous=True,
                confidence=.8,last_seen=5.)]),get_clock=lambda:NS(now=lambda:NS(nanoseconds=5_000_000_000)))
        policy=ParallelCandidates(Baseline(),node,lambda:CAMERA)
        first=policy.next_view((1.,2.,0.))
        second=policy.next_view((first['x'],first['y'],first['yaw']))
        self.assertEqual(first['normal_basis'],'measured_surface')
        self.assertEqual(second['normal_basis'],'measured_surface')
        self.assertEqual(second['normal_sign'],-first['normal_sign'])
        # If the opposite side is blocked, remaining feasible angles still work.
        node.search_map.data=[0 if x<25 else 100 for y in range(40) for x in range(40)]
        policy=ParallelCandidates(Baseline(),node,lambda:CAMERA)
        first=policy.next_view((1.,2.,0.));second=policy.next_view((first['x'],first['y'],first['yaw']))
        self.assertEqual(first['normal_sign'],-1)
        self.assertEqual(second['normal_sign'],-1)
        self.assertNotEqual(first['view_angle_offset_deg'],second['view_angle_offset_deg'])

    def test_new_live_candidate_preempts_next_baseline_selection(self):
        class Baseline:
            positions=[]
            def next_view(self,robot):return dict(strategy='frustum',x=1.,y=1.,yaw=0.)
        grid=NS(data=[0]*1600,info=NS(width=40,height=40,resolution=.1,origin=NS(position=NS(x=0.,y=0.))))
        node=NS(c={},search_map=grid,search_suspects=None,read_target_registry=lambda **kw:{},
                get_clock=lambda:NS(now=lambda:NS(nanoseconds=5_000_000_000)))
        policy=ParallelCandidates(Baseline(),node,lambda:CAMERA)
        self.assertEqual(policy.next_view((1.,1.,0.))['strategy'],'frustum')
        node.search_suspects=dict(frame_id='map',stamp=5.,candidates=[dict(id=10,
            position=[2.,2.,.25],normal=[1.,0.],confidence=.8,last_seen=5.,observations=2)])
        view=policy.next_view((1.,1.,0.))
        self.assertEqual(view['strategy'],'suspected_panel')
        self.assertEqual(view['suspect_id'],10)

    def test_millimetre_edge_uses_perpendicular_face_hypothesis(self):
        hint=dict(id=0,position=[2.,2.,.25],normal=[.0655,.99785],confidence=.6,
                  width_m=.0061,normal_ambiguous=True)
        views=normal_views([hint],[0]*1600,40,40,.1,(0.,0.),(1.,1.,0.),camera=CAMERA,config={'view_half_angle_deg':0.})
        self.assertEqual(len(views),2)
        for view in views:
            dx,dy=view['x']-2.,view['y']-2.
            self.assertAlmostEqual(dx*hint['normal'][0]+dy*hint['normal'][1],0.,places=5)
            self.assertEqual(view['normal_axis'],1)
            self.assertEqual(view['normal_basis'],'inferred_face_from_edge')
        # A broad observed face keeps its measured normal.
        hint.update(width_m=.18,normal_ambiguous=False)
        views=normal_views([hint],[0]*1600,40,40,.1,(0.,0.),(1.,1.,0.),camera=CAMERA,config={'view_half_angle_deg':0.})
        self.assertTrue(views)
        self.assertTrue(all(v['normal_axis']==0 for v in views))

    def test_normals_and_clearance(self):
        hint=dict(id=2,position=[2.,2.,.25],normal=[1.,0.],confidence=.7,normal_ambiguous=False)
        data=[0]*1600
        args=(data,40,40,.1,(0.,0.),(1.,1.,0.))
        self.assertEqual(len(normal_views([hint],*args,camera=CAMERA,config={'view_half_angle_deg':0.})),2)
        hint['normal_ambiguous']=True
        views=normal_views([hint],*args,camera=CAMERA,config={'view_half_angle_deg':0.})
        self.assertEqual(len(views),4)
        self.assertTrue(all(v['require_heading'] for v in views))
        for y in range(40):data[y*40+25]=100
        # East endpoint lies across an occupied barrier: reject it.
        views=normal_views([hint],*args,camera=CAMERA,config={'view_half_angle_deg':0.})
        self.assertTrue(all(v['x']<2.5 for v in views))
        self.assertFalse(normal_views([hint],[-1]*1600,*args[1:],camera=CAMERA,config={'view_half_angle_deg':0.}))

    def test_complete_suspect_round_before_baseline_and_expire(self):
        class Baseline:
            positions=[(1.,1.)]
            def next_view(self,robot):return dict(x=1.,y=1.,yaw=0.,strategy='frustum')
            def record_sweep(self,actual):pass
        grid=NS(data=[0]*1600,info=NS(width=40,height=40,resolution=.1,origin=NS(position=NS(x=0.,y=0.))))
        now=10.
        node=NS(c={'search_suspects':{'max_positions':2}},search_map=grid,
            read_target_registry=lambda **kwargs:{},
            search_suspects=dict(frame_id='map',stamp=10.,candidates=[dict(id=1,position=[2.,2.,.25],
                normal=[1.,0.],confidence=.8,last_seen=10.)]),
            get_clock=lambda:NS(now=lambda:NS(nanoseconds=int(now*1e9))))
        policy=ParallelCandidates(Baseline(),node,lambda:CAMERA)
        self.assertEqual(policy.next_view((1.,1.,0.))['strategy'],'suspected_panel')
        self.assertEqual(policy.next_view((1.,1.,0.))['strategy'],'suspected_panel')
        self.assertEqual(policy.next_view((1.,1.,0.))['strategy'],'frustum')
        now=20.
        policy=ParallelCandidates(Baseline(),node,lambda:CAMERA)
        self.assertEqual(policy.next_view((1.,1.,0.))['strategy'],'frustum')
        # Old mapping observation is usable only if multi-frame memory is
        # explicitly retained and the reprojected publication is fresh.
        node.search_suspects['stamp']=now
        retained=node.search_suspects['candidates'][0]
        retained.update(retained=True,observations=3,last_seen=1.)
        policy=ParallelCandidates(Baseline(),node,lambda:CAMERA)
        self.assertEqual(policy.next_view((1.,1.,0.))['strategy'],'suspected_panel')
        node.search_map.data=[-1]*1600
        policy=ParallelCandidates(Baseline(),node,lambda:CAMERA)
        self.assertEqual(policy.next_view((1.,1.,0.))['strategy'],'frustum')

    def test_exclude_all_localized_ids_including_stale_targets(self):
        hints=[dict(position=[1.,2.,.25]),dict(position=[3.,2.,.25])]
        known=dict(targets=[dict(id=2,type='cooperative',state='stale',position=[1.05,2.,.25])])
        self.assertEqual(without_known(hints,known),hints[1:])
        self.assertEqual(without_known(hints,dict(targets=[dict(id=1,position=None)])),hints)

    def test_full_frame_adapts_distance_and_rejects_edge_occlusion(self):
        hint=dict(id=1,position=[2.,2.,.25],normal=[1.,0.],confidence=.8)
        data=[0]*1600
        args=(data,40,40,.1,(0.,0.),(1.,1.,0.))
        narrow=Camera(640,480,1600.,1600.,320.,240.,(.1,0.,.25),CAMERA.rotation)
        views=normal_views([hint],*args,standoff=.55,camera=narrow,config={'view_half_angle_deg':0.})
        self.assertEqual(len(views),2)
        self.assertTrue(all(v['standoff_m']>.55 for v in views))
        self.assertFalse(normal_views([hint],*args,camera=CAMERA,
            config={'min_panel_side_px':1000.}))
        # Block an edge ray while leaving the centre sightline free.
        data[18*40+18]=100
        data[19*40+18]=100
        views=normal_views([hint],*args,camera=CAMERA,config={'view_half_angle_deg':0.})
        self.assertTrue(all(v['x']>2. for v in views))
        # Unknown space also blocks viewing; cannot be treated as transparent.
        data[18*40+18]=-1
        data[19*40+18]=-1
        self.assertTrue(all(v['x']>2. for v in normal_views([hint],*args,camera=CAMERA,config={'view_half_angle_deg':0.})))



    def test_fan_reaches_70_degrees_and_avoids_blocked_straight_view(self):
        hint=dict(id=1,position=[2.,2.,.25],normal=[1.,0.],confidence=.8)
        data=[0]*1600
        views=normal_views([hint],data,40,40,.1,(0.,0.),(1.,1.,0.),camera=CAMERA)
        self.assertTrue(any(abs(v['view_angle_offset_deg'])==70 for v in views))
        self.assertTrue(all(abs(v['view_angle_offset_deg'])<=70 for v in views))
        centre=next(v for v in views if v['normal_sign']==-1 and v['view_angle_offset_deg']==0)
        side=next(v for v in views if v['normal_sign']==-1 and v['view_angle_offset_deg']==70)
        self.assertLess(side['projected_width_px'],centre['projected_width_px'])
        self.assertGreaterEqual(side['projected_width_px'],24.)
        for y in (19,20,21):data[y*40+18]=100
        views=normal_views([hint],data,40,40,.1,(0.,0.),(1.,1.,0.),camera=CAMERA)
        self.assertFalse(any(v['normal_sign']==-1 and v['view_angle_offset_deg']==0 for v in views))
        self.assertTrue(any(v['normal_sign']==-1 and abs(v['view_angle_offset_deg'])>0 for v in views))

if __name__=='__main__':unittest.main()
