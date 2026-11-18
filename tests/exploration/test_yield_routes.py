"""Peer blockage evidence, clearance, and bounded diagnostic failures."""
from pathlib import Path
import sys
from types import SimpleNamespace as S
import unittest
sys.path[:0]=[str(Path(__file__).resolve().parents[2]/'src/edgeai_navigation')]
from edgeai_nav.yield_routes import GridRoutes


def arena():
    data=[0 if (x<18 or x>32 or 18<=y<=22) else 100
          for y in range(40) for x in range(50)]
    return S(info=S(resolution=.1,width=50,height=40,
        origin=S(position=S(x=0.,y=0.),orientation=S(x=0.,y=0.,z=0.,w=1.))),data=data)


class YieldRoutesTests(unittest.TestCase):
    def test_idle_peer_seals_corridor_and_can_retreat_off_route(self):
        routes=GridRoutes(arena(),.15)
        start,goal,peer=[.55,2.05],[4.45,2.05],[2.55,2.05]
        result=routes.diagnose(start,goal,peer)
        self.assertEqual(result['classification'],'peer_blocks_static_route')
        settings=dict(yield_sample_spacing_m=.2,yield_min_distance_m=1.,yield_max_distance_m=3.,
            yield_requester_clearance_m=1.,yield_route_clearance_m=.5,yield_max_candidates=20)
        pose=routes.yielding_pose(peer,start,result['path'],settings)
        self.assertIsNotNone(pose)
        self.assertEqual(routes.diagnose(start,goal,[pose['x'],pose['y']])['classification'],'route_available')

    def test_static_failure_is_not_blame_assigned_to_peer(self):
        grid=arena()
        for y in range(40):grid.data[y*50+25]=100
        self.assertEqual(GridRoutes(grid,.15).diagnose([.55,2.05],[4.45,2.05],[2.55,2.05])
                         ['classification'],'inconclusive')

    def test_search_budget_and_unknown_cells_are_not_approved(self):
        self.assertEqual(GridRoutes(arena(),.15,max_cells=2).diagnose([.55,2.05],[4.45,2.05],[2.55,2.05])
                         ['classification'],'inconclusive')
        grid=arena();grid.data=[-1]*len(grid.data)
        self.assertEqual(GridRoutes(grid,.15).route([.55,2.05],[4.45,2.05])['status'],'unreachable')


if __name__=='__main__':unittest.main()
