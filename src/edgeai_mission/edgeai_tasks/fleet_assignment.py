"""Shared deterministic allocation, reservations and persistent retry history."""
import math


ROBOTS = ('robot1', 'robot2')
DEFAULTS = dict(reservation_radius_m=.7, completion_hold_s=2.,
                max_candidate_age_s=1.5, max_navigation_attempts=2,
                failure_cooldown_s=10., max_retry_rounds=2,
                navigation_timeout_s=90., acceptance_timeout_s=5.,
                cancel_timeout_s=8., authority_max_age_s=2., settle_s=.5,
                assignment_timeout_s=15.,
                arrival_cooldown_s=3., max_candidate_arrivals=3,
                frontier_match_radius_m=.6, frontier_update_hold_s=2.,
                frontier_min_navigation_s=5.)


def distance(a, b):
    return math.hypot(a['x']-b['x'], a['y']-b['y'])


class AssignmentPolicy:
    """Nearest available edge first, with stable robot/coordinate tie breaks.

    History is per robot and anchored to the first failed endpoint. Attempts
    never reset: each cooldown unlocks one bounded retry round, then the
    neighborhood becomes exhausted for this task.
    """
    def __init__(self, settings=None):
        self.c = dict(DEFAULTS, **(settings or {}))
        for key, value in self.c.items():
            if (key in DEFAULTS or key == 'history_match_radius_m') and (not math.isfinite(float(value)) or float(value) <= 0):
                raise ValueError('fleet_assignment.'+key+' must be positive')
        for key in ('max_navigation_attempts', 'max_retry_rounds', 'max_candidate_arrivals'):
            if int(self.c[key]) != self.c[key]:
                raise ValueError(key+' must be an integer')
        self.history = {r: [] for r in ROBOTS}
        self.arrivals = {r: [] for r in ROBOTS}

    @property
    def radius(self):
        return self.c['reservation_radius_m']

    @property
    def history_radius(self):
        return self.c.get('history_match_radius_m', self.radius)

    def history_for(self, robot, goal):
        if 'target_id' in goal:
            return next((h for h in self.history[robot] if h['goal'].get('target_id') == goal['target_id']), None)
        matches = [h for h in self.history[robot] if distance(h['goal'], goal) < self.history_radius]
        return min(matches, key=lambda h: (distance(h['goal'], goal), h['goal']['x'], h['goal']['y'])) if matches else None

    def eligibility(self, robot, goal, wall):
        arrivals = [a for a in self.arrivals[robot] if distance(a['goal'], goal) < self.history_radius]
        if len(arrivals) >= self.c['max_candidate_arrivals']:
            return 'exhausted'
        if arrivals and wall < max(a['wall'] for a in arrivals)+self.c['arrival_cooldown_s']:
            return 'suppressed'
        h = self.history_for(robot, goal)
        if h is None:
            return 'eligible'
        if h['attempts'] >= self.c['max_navigation_attempts']*self.c['max_retry_rounds']:
            return 'exhausted'
        return 'suppressed' if wall < h['retry_after'] else 'eligible'

    def reached(self, robot, goal, wall):
        # A persistent frontier after arrival is not exploration completion.
        # Allow map updates before retrying, and bound repeated zero-distance goals.
        self.arrivals[robot].append(dict(goal=dict(goal), wall=wall))

    def failed(self, robot, goal, wall, reason):
        h = self.history_for(robot, goal)
        if h is None:
            h = dict(goal=dict(goal), attempts=0, retry_after=0., reasons=[])
            self.history[robot].append(h)
        h['attempts'] += 1
        h['reasons'].append(reason)
        if h['attempts'] % self.c['max_navigation_attempts'] == 0:
            h['retry_after'] = wall+self.c['failure_cooldown_s']
        return h

    def allocate(self, candidates, poses, reserved, wall, limit=None):
        if limit is not None and (isinstance(limit, bool) or not isinstance(limit, int) or limit < 1):
            raise ValueError('Assignment limit must be a positive integer')
        chosen = {}
        occupied = list(reserved.values())
        edges = sorted(((distance(pose, goal), robot, goal['x'], goal['y'], goal)
                       for robot, pose in poses.items() if robot not in reserved
                       for goal in candidates.get(robot, [])
                       if self.eligibility(robot, goal, wall) == 'eligible'),
                       key=lambda edge: edge[:4])
        for _, robot, _, _, goal in edges:
            if robot in chosen or any(distance(goal, p) < self.radius for p in occupied):
                continue
            chosen[robot] = dict(goal)
            occupied.append(goal)
            if limit is not None and len(chosen) >= limit:
                break
        return chosen

    def allocate_rescues(self, candidates, poses, reserved, wall, compatible):
        """Maximize concurrent distinct rescues, then minimize the longest approach."""
        occupied = list(reserved.values())
        robots = sorted(poses.keys() - reserved.keys())
        options = {robot: sorted(
            ((distance(poses[robot], goal), goal) for goal in candidates.get(robot, [])
             if self.eligibility(robot, goal, wall) == 'eligible'),
            key=lambda item: (item[0], item[1]['target_id'], item[1]['x'], item[1]['y']))
            for robot in robots}
        best = {}
        best_score = (0, 0., 0.)

        def visit(index, selected, costs):
            nonlocal best, best_score
            if index == len(robots):
                score = (-len(selected), max(costs, default=0.), sum(costs))
                if score < best_score:
                    best, best_score = dict(selected), score
                return
            robot = robots[index]
            for cost, goal in options[robot]:
                if all(goal['target_id'] != other['target_id'] and compatible(goal, other) for other in occupied):
                    selected[robot] = goal
                    occupied.append(goal)
                    visit(index + 1, selected, costs + [cost])
                    occupied.pop()
                    del selected[robot]
            visit(index + 1, selected, costs)

        visit(0, {}, [])
        return best
