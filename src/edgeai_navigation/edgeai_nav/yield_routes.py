"""Read-only grid diagnosis and bounded selection of idle-robot yielding poses.

The comparison is evidence about the supplied static map, not proof that every
other Nav2 layer is clear. Execution always uses the unchanged Nav2 costmap.
"""
from collections import deque
import math


class GridRoutes:
    def __init__(self, grid, body_radius, max_cells=20000):
        self.grid, self.radius, self.limit = grid, body_radius, int(max_cells)
        self.res = grid.info.resolution
        self.w, self.h = grid.info.width, grid.info.height
        q = grid.info.origin.orientation
        self.yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.c, self.s = math.cos(self.yaw), math.sin(self.yaw)
        self.x, self.y = grid.info.origin.position.x, grid.info.origin.position.y
        n = math.ceil(body_radius/self.res+.7072)
        self.offsets = [(x,y) for x in range(-n,n+1) for y in range(-n,n+1)
                        if math.hypot(x,y)*self.res <= body_radius+self.res*.7072]
        self.cache = {}

    def cell(self, pose):
        dx,dy = pose[0]-self.x, pose[1]-self.y
        return (math.floor((self.c*dx+self.s*dy)/self.res),
                math.floor((-self.s*dx+self.c*dy)/self.res))

    def world(self, p):
        x,y = (p[0]+.5)*self.res, (p[1]+.5)*self.res
        return [self.x+self.c*x-self.s*y, self.y+self.s*x+self.c*y]

    def clear(self, p):
        if p not in self.cache:
            self.cache[p] = all(0 <= p[0]+dx < self.w and 0 <= p[1]+dy < self.h
                and 0 <= self.grid.data[(p[1]+dy)*self.w+p[0]+dx] < 50
                for dx,dy in self.offsets)
        return self.cache[p]

    def route(self, start, goal, peer=None):
        a,b = self.cell(start), self.cell(goal)
        queue, parent = deque([a]), {a:None}
        while queue:
            p = queue.popleft()
            if not self.clear(p) or (peer is not None and
                    math.dist(self.world(p),peer[:2]) <= 2*self.radius):
                continue
            if p == b:
                path=[]
                while p is not None:
                    path.append(self.world(p));p=parent[p]
                return dict(status='reachable',path=path[::-1])
            for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
                q = p[0]+dx,p[1]+dy
                if q not in parent and 0 <= q[0] < self.w and 0 <= q[1] < self.h:
                    parent[q]=p;queue.append(q)
            if len(parent) >= self.limit:
                return dict(status='budget_exhausted',path=[])
        return dict(status='unreachable',path=[])

    def diagnose(self, start, goal, peer):
        baseline=self.route(start,goal)
        if baseline['status'] != 'reachable':
            return dict(classification='inconclusive',baseline=baseline['status'])
        occupied=self.route(start,goal,peer)
        return dict(classification='peer_blocks_static_route' if occupied['status']=='unreachable'
                    else 'inconclusive' if occupied['status']=='budget_exhausted' else 'route_available',
                    baseline=baseline['status'],with_peer=occupied['status'],path=baseline['path'])

    def yielding_pose(self, peer, requester, path, settings):
        # Coarse bounded sampling; exact native planning verifies the winner.
        stride=max(1,round(settings['yield_sample_spacing_m']/self.res))
        candidates=[]
        for y in range(0,self.h,stride):
            for x in range(0,self.w,stride):
                p=self.world((x,y));travel=math.dist(p,peer[:2])
                if not settings['yield_min_distance_m'] <= travel <= settings['yield_max_distance_m']:
                    continue
                if math.dist(p,requester[:2]) < settings['yield_requester_clearance_m']:
                    continue
                if any(math.dist(p,q) < settings['yield_route_clearance_m'] for q in path):
                    continue
                if self.clear((x,y)):
                    candidates.append((travel,p))
        for _,p in sorted(candidates)[:int(settings['yield_max_candidates'])]:
            if self.route(peer,p,requester)['status']=='reachable':
                return dict(x=p[0],y=p[1],size=0.)
        return None
