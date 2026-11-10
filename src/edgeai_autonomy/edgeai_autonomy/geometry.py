"""Geometry shared by perception, exploration and tests; no simulator truth."""
import math
import heapq
import numpy as np
from scipy import ndimage
from scipy.spatial.transform import Rotation


def matrix(transform):
    q, t = transform.rotation, transform.translation
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    result[:3, 3] = [t.x, t.y, t.z]
    return result


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def rescue_goal(target, standoff):
    position = np.array(target['position'])
    normal = np.array(target['normal'][:2], dtype=float)
    length = np.linalg.norm(normal)
    if length < 0.7:
        raise ValueError('Marker is not sufficiently vertical for the practice rescue geometry')
    normal /= length
    goal = position[:2] + normal*standoff
    return float(goal[0]), float(goal[1]), math.atan2(-normal[1], -normal[0])


def clearance_at(data, resolution, origin, position):
    """Known-free clearance at the current pose; unknown is not safe space."""
    if position is None:
        return 0.0
    row = math.floor((position[1]-origin[1])/resolution)
    col = math.floor((position[0]-origin[0])/resolution)
    grid = np.asarray(data)
    if not (0 <= row < grid.shape[0] and 0 <= col < grid.shape[1]):
        return 0.0
    free = (grid >= 0) & (grid <= 25)
    return float(ndimage.distance_transform_edt(free)[row,col]*resolution)


def frontier_goals(data, resolution, origin, robot, visited, clearance=0.40):
    """Cluster free/unknown boundaries; choose reachable, cleared observation cells.

    All inputs are live occupancy-grid data. There are no arena coordinates or
    saved reference maps. Unknown cells are excluded from the travel space.
    """
    grid = np.asarray(data)
    free, unknown = (grid >= 0) & (grid <= 25), grid < 0
    safe = ndimage.distance_transform_edt(free)*resolution >= clearance
    components, _ = ndimage.label(safe, np.ones((3, 3)))
    yy, xx = np.where(safe)
    if not len(xx):
        return []
    wx, wy = origin[0]+(xx+0.5)*resolution, origin[1]+(yy+0.5)*resolution
    nearest = np.argmin((wx-robot[0])**2+(wy-robot[1])**2)
    seed = (int(yy[nearest]), int(xx[nearest]))
    reachable = components[yy, xx] == components[yy[nearest], xx[nearest]]
    xx, yy, wx, wy = xx[reachable], yy[reachable], wx[reachable], wy[reachable]
    frontiers = free & ndimage.binary_dilation(unknown)
    labels, count = ndimage.label(frontiers, np.ones((3, 3)))
    candidates = []
    for label in range(1, count+1):
        fy, fx = np.where(labels == label)
        if len(fx) < 4:
            continue
        # A single frontier may wrap around a room: its centroid can be in
        # already explored space. Split it spatially before choosing viewpoints.
        bins = np.c_[fx//max(1, round(0.6/resolution)), fy//max(1, round(0.6/resolution))]
        for patch in np.unique(bins, axis=0):
            selected = np.all(bins == patch, axis=1)
            if selected.sum() < 3:
                continue
            cx = origin[0]+(fx[selected].mean()+0.5)*resolution
            cy = origin[1]+(fy[selected].mean()+0.5)*resolution
            distance = (wx-cx)**2+(wy-cy)**2
            index = int(np.argmin(distance))
            x, y = float(wx[index]), float(wy[index])
            travel = math.hypot(x-robot[0], y-robot[1])
            if travel < 0.45 or distance[index] > 0.9**2 or any(math.hypot(x-v[0], y-v[1]) < 0.55 for v in visited):
                continue
            score = int(selected.sum())*resolution/(0.5+travel)
            candidates.append({'x': x, 'y': y, 'yaw': math.atan2(cy-y, cx-x),
                               'score': score, 'frontier_cells': int(selected.sum())})
    # A visited transit cell is still a valid route to an unvisited frontier.
    # Candidate destinations above are filtered; filtering the short waypoint
    # again would strand exploration behind a shared corridor.
    return incremental_goals(safe, seed, resolution, origin, robot, [], candidates)


def observation_goals(data, resolution, origin, robot, visited, hints, clearance=0.40):
    """Seek a better view of image-derived, unconfirmed marker hypotheses.

    Hints cannot certify targets. Snap proposed front-facing viewpoints to
    known cleared space, and use the same bounded safe paths as exploration.
    """
    grid = np.asarray(data)
    safe = ndimage.distance_transform_edt((grid >= 0) & (grid <= 25))*resolution >= clearance
    yy, xx = np.where(safe)
    if not len(xx):
        return []
    wx, wy = origin[0]+(xx+0.5)*resolution, origin[1]+(yy+0.5)*resolution
    nearest = np.argmin((wx-robot[0])**2+(wy-robot[1])**2)
    seed = (int(yy[nearest]), int(xx[nearest]))
    candidates = []
    for hint in hints:
        try:
            x, y, heading = rescue_goal(hint, 1.0)
        except ValueError:
            continue
        distance = (wx-x)**2+(wy-y)**2
        index = int(np.argmin(distance))
        if distance[index] > 0.6**2:
            continue
        x, y = float(wx[index]), float(wy[index])
        if any(math.hypot(x-v[0], y-v[1]) < 0.45 for v in visited):
            continue
        candidates.append({'x': x, 'y': y, 'yaw': math.atan2(hint['position'][1]-y, hint['position'][0]-x),
                           'score': 1/(0.5+math.hypot(x-robot[0], y-robot[1])),
                           'hint_id': hint['id']})
    # Revisiting a transit segment is allowed when approaching a new viewpoint.
    return incremental_goals(safe, seed, resolution, origin, robot, [], candidates)


def coverage_goals(data, resolution, origin, robot, visited, clearance=0.40):
    """Observe mapped surfaces when lidar frontiers no longer cover camera needs."""
    grid = np.asarray(data)
    safe = ndimage.distance_transform_edt((grid >= 0) & (grid <= 25))*resolution >= clearance
    yy, xx = np.where(safe)
    if not len(xx) or not np.any(grid >= 65):
        return []
    wx, wy = origin[0]+(xx+0.5)*resolution, origin[1]+(yy+0.5)*resolution
    nearest = np.argmin((wx-robot[0])**2+(wy-robot[1])**2)
    seed = (int(yy[nearest]), int(xx[nearest]))
    to_surface = ndimage.distance_transform_edt(grid < 65)*resolution
    candidates = []
    bins = np.c_[xx//max(1, round(0.5/resolution)), yy//max(1, round(0.5/resolution))]
    for patch in np.unique(bins, axis=0):
        selected = np.where(np.all(bins == patch, axis=1) & (to_surface[yy,xx] <= 1.2))[0]
        if not len(selected):
            continue
        index = selected[np.argmin((wx[selected]-wx[selected].mean())**2+(wy[selected]-wy[selected].mean())**2)]
        x, y = float(wx[index]), float(wy[index])
        travel = math.hypot(x-robot[0], y-robot[1])
        if travel < 0.6 or any(math.hypot(x-v[0], y-v[1]) < 0.65 for v in visited):
            continue
        candidates.append({'x': x, 'y': y, 'yaw': math.atan2(y-robot[1], x-robot[0]),
                           'score': 1/(0.5+travel), 'source': 'mapped_surface_coverage'})
    return incremental_goals(safe, seed, resolution, origin, robot, [], candidates)


def incremental_goals(safe, seed, resolution, origin, robot, visited, candidates):
    # Follow a cleared grid path for at most 0.9 m before observing/replanning.
    # Long goals beside unknown shadows can become invalid as SLAM updates.
    distances, parents, queue = {seed: 0.0}, {}, [(0.0, seed)]
    while queue:
        cost, cell = heapq.heappop(queue)
        if cost > distances[cell]:
            continue
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if not (dx or dy):
                    continue
                row, col = cell[0]+dy, cell[1]+dx
                if not (0 <= row < safe.shape[0] and 0 <= col < safe.shape[1] and safe[row,col]):
                    continue
                if dx and dy and not (safe[cell[0],col] and safe[row,cell[1]]):
                    continue  # Never cut diagonally through an obstacle corner.
                neighbor = (row,col)
                updated = cost+math.hypot(dx,dy)*resolution
                if updated < distances.get(neighbor, math.inf):
                    distances[neighbor], parents[neighbor] = updated, cell
                    heapq.heappush(queue, (updated,neighbor))
    result = []
    for candidate in sorted(candidates, key=lambda item: item['score'], reverse=True):
        cell = (int((candidate['y']-origin[1])/resolution), int((candidate['x']-origin[0])/resolution))
        if cell not in distances:
            continue
        path = [cell]
        while cell != seed:
            cell = parents[cell]
            path.append(cell)
        path.reverse()
        selected = max(i for i, cell in enumerate(path) if distances[cell] <= 0.9)
        row,col = path[selected]
        x,y = origin[0]+(col+0.5)*resolution, origin[1]+(row+0.5)*resolution
        if math.hypot(x-robot[0],y-robot[1]) < 0.45 or any(math.hypot(x-v[0],y-v[1]) < 0.45 for v in visited):
            continue
        candidate = dict(candidate, frontier_x=candidate['x'], frontier_y=candidate['y'], x=x, y=y)
        if selected+1 < len(path):
            after = path[selected+1]
            candidate['yaw'] = math.atan2(after[0]-row, after[1]-col)
        result.append(candidate)
    return result
