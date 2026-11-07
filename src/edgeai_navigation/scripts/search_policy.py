"""Clear observation endpoints in known free space; Nav2 validates each approach."""
from collections import deque
import math


def observation_positions(data, width, height, resolution, origin, robot,
                          clearance=.40, spacing=1.0, limit=6):
    if resolution <= 0 or width*height != len(data):
        raise ValueError('Invalid search map')
    ox, oy = origin
    sx, sy = (int(math.floor((robot[0]-ox)/resolution)),
              int(math.floor((robot[1]-oy)/resolution)))
    radius = math.ceil(clearance/resolution)
    # Include intersecting cell corners in the clearance test.
    offsets = [(dx, dy) for dy in range(-radius, radius+1)
               for dx in range(-radius, radius+1)
               if math.hypot(dx, dy)*resolution <= clearance+resolution/math.sqrt(2)]
    cache = {}
    def free(x, y):
        return 0 <= x < width and 0 <= y < height and data[y*width+x] == 0

    def safe(x, y):
        key = (x, y)
        if key not in cache:
            cache[key] = all(0 <= x+dx < width and 0 <= y+dy < height and
                             data[(y+dy)*width+x+dx] == 0 for dx, dy in offsets)
        return cache[key]
    # Circumscribed-circle clearance is required at observation endpoints,
    # not at every transit cell. Nav2 checks the actual footprint and route.
    if not free(sx, sy):
        return []
    reached, queue = {(sx, sy)}, deque([(sx, sy)])
    while queue:
        x, y = queue.popleft()
        for point in ((x-1,y), (x+1,y), (x,y-1), (x,y+1)):
            if point not in reached and free(*point):
                reached.add(point); queue.append(point)
    candidates = [(ox+(x+.5)*resolution, oy+(y+.5)*resolution)
                  for x,y in sorted(reached) if safe(x, y)]
    if not candidates:
        return []
    selected = [min(candidates, key=lambda p: math.dist(p, robot))]
    while len(selected) < limit:
        point = max(candidates, key=lambda p: min(math.dist(p, q) for q in selected))
        if min(math.dist(point, q) for q in selected) < spacing:
            break
        selected.append(point)
    # Visit spatially distributed views in nearest-neighbour order.
    ordered, current = [], robot
    while selected:
        point = min(selected, key=lambda p: math.dist(p, current))
        selected.remove(point); ordered.append(point); current = point
    return ordered
