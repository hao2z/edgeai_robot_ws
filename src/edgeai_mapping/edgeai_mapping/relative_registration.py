"""Bounded, conservative SE(2) registration. No ROS, map, or truth dependencies.

Poses are [x metres, y metres, yaw radians], target_from_source. Covariance is
row-major 3x3 in target axes (x,y,yaw), a local fit estimate, not ground truth.
"""
from dataclasses import dataclass
import math
import numpy as np

try:
    from scipy.spatial import cKDTree
except ImportError:  # Both dimensions are capped at 180 before querying.
    cKDTree = None


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return np.array([a[0]+c*b[0]-s*b[1], a[1]+s*b[0]+c*b[1],
                     math.atan2(math.sin(a[2]+b[2]), math.cos(a[2]+b[2]))])


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return np.array([-c*a[0]-s*a[1], s*a[0]-c*a[1], -a[2]])


def transform(points, pose):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return np.asarray(points) @ np.array([[c, s], [-s, c]]) + pose[:2]


def bounded_points(points, limit=180):
    p = np.asarray(points, dtype=float).reshape(-1, 2)
    p = p[np.isfinite(p).all(axis=1)]
    if len(p) > limit:
        p = p[np.linspace(0, len(p)-1, limit, dtype=int)]
    return p


def scan_points(ranges, angle_min, angle_increment, range_min, range_max, limit=180):
    r = np.asarray(ranges, dtype=float)
    a = angle_min + np.arange(len(r))*angle_increment
    valid = np.isfinite(r) & (r >= range_min) & (r <= range_max) & (r > 0)
    return bounded_points(np.column_stack((r[valid]*np.cos(a[valid]),
                                          r[valid]*np.sin(a[valid]))), limit)


def remove_peer(points, peer_from_points, bounds, margin):
    """Discard actual returns in the predicted peer; never synthesize free rays."""
    q = transform(points, peer_from_points)
    xmin, xmax, ymin, ymax = bounds
    inside = ((q[:, 0] >= xmin-margin) & (q[:, 0] <= xmax+margin)
              & (q[:, 1] >= ymin-margin) & (q[:, 1] <= ymax+margin))
    return np.asarray(points)[~inside]


class Nearest:
    def __init__(self, points):
        self.points = points
        self.tree = cKDTree(points) if cKDTree is not None else None

    def query(self, points, k=1):
        if self.tree is not None:
            return self.tree.query(points, k=k)
        d = np.sum((np.asarray(points)[:, None]-self.points[None, :])**2, axis=2)
        idx = np.argsort(d, axis=1)[:, :k]
        dist = np.sqrt(np.take_along_axis(d, idx, axis=1))
        return (dist[:, 0], idx[:, 0]) if k == 1 else (dist, idx)


@dataclass(frozen=True)
class RegistrationConfig:
    max_points: int = 180
    min_points: int = 30
    iterations: int = 25
    max_correspondence_m: float = .25
    trim_fraction: float = .8
    min_overlap: float = .45
    max_residual_m: float = .065
    min_spread_m: float = .20
    min_information_ratio: float = .025
    max_correction_m: float = .40
    max_correction_yaw_rad: float = .30
    coarse_xy_m: float = .20
    coarse_yaw_rad: float = .15
    distinct_translation_m: float = .16
    distinct_yaw_rad: float = .12
    min_distinctiveness: float = .10
    residual_floor_m: float = .005
    normal_neighbors: int = 7
    surface_planarity_ratio: float = .20
    max_seeds: int = 7
    max_refinement_step_m: float = .10
    max_refinement_step_yaw_rad: float = .10
    convergence_m: float = .00001
    convergence_yaw_rad: float = .00001

    def __post_init__(self):
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError('Invalid registration setting: '+name)
        if (not 7 <= self.min_points <= self.max_points <= 180
                or int(self.max_points) != self.max_points
                or int(self.min_points) != self.min_points
                or int(self.iterations) != self.iterations or self.iterations > 100
                or int(self.normal_neighbors) != self.normal_neighbors
                or not 3 <= self.normal_neighbors <= self.min_points
                or int(self.max_seeds) != self.max_seeds or self.max_seeds > 27
                or self.surface_planarity_ratio >= 1
                or not 0 < self.trim_fraction <= 1 or self.min_overlap > 1
                or self.min_information_ratio >= 1 or self.min_distinctiveness >= 1
                or self.coarse_xy_m > self.max_correction_m
                or self.coarse_yaw_rad > self.max_correction_yaw_rad):
            raise ValueError('Inconsistent registration bounds')


def _pairs(target, source, pose, tree, cfg):
    moved = transform(source, pose)
    d, j = tree.query(moved)
    _, back = Nearest(moved).query(target)
    ids = np.flatnonzero((d <= cfg.max_correspondence_m)
                         & (back[j] == np.arange(len(source))))
    overlap = len(ids)/max(len(target), len(source))
    keep = max(3, int(math.ceil(len(ids)*cfg.trim_fraction)))
    ids = ids[np.argsort(d[ids])[:keep]]
    return moved, ids, j[ids], overlap


def _cost(tree, source, pose, cfg):
    d, _ = tree.query(transform(source, pose))
    return float(np.mean(np.minimum(d, cfg.max_correspondence_m)**2))


def register_scans(target, source, prior, config=None):
    """Return a JSON-safe candidate; accepted here still needs temporal confirmation."""
    cfg = config or RegistrationConfig()
    target, source = (bounded_points(p, cfg.max_points) for p in (target, source))
    prior = np.asarray(prior, dtype=float)
    if prior.shape != (3,) or not np.isfinite(prior).all():
        raise ValueError('prior must be finite SE(2)')
    result = dict(accepted=False, reason='insufficient_points', odom1_from_odom2=None,
                  residual=None, overlap=0., covariance=None, distinctiveness=None,
                  information_ratio=None, spread_m=None,
                  point_counts=[len(target), len(source)])
    if min(len(target), len(source)) < cfg.min_points:
        return result
    tree = Nearest(target)
    _, neighborhood = tree.query(target, k=cfg.normal_neighbors)
    local = target[neighborhood]
    local -= local.mean(axis=1, keepdims=True)
    local_eig, local_vec = np.linalg.eigh(np.einsum('nki,nkj->nij', local, local))
    surface = local_eig[:, 0] < cfg.surface_planarity_ratio*np.maximum(local_eig[:, 1], 1e-12)
    surface_normals = local_vec[:, :, 0]
    seeds = [prior+np.array([x, y, a])
             for x in (-cfg.coarse_xy_m, 0., cfg.coarse_xy_m)
             for y in (-cfg.coarse_xy_m, 0., cfg.coarse_xy_m)
             for a in (-cfg.coarse_yaw_rad, 0., cfg.coarse_yaw_rad)]
    solutions = []
    # Sparse walls can lock reciprocal matches in a wrong local basin. Refine
    # several bounded seeds instead of trusting the single coarse winner.
    for seed in sorted(seeds, key=lambda p: _cost(tree, source, p, cfg))[:cfg.max_seeds]:
        pose = seed.copy()
        for _ in range(cfg.iterations):
            moved, i, j, overlap = _pairs(target, source, pose, tree, cfg)
            if len(i) < cfg.min_points*cfg.trim_fraction:
                break
            a, b = moved[i], target[j]
            ac, bc = a.mean(axis=0), b.mean(axis=0)
            u, _, vt = np.linalg.svd((a-ac).T @ (b-bc))
            rotation = vt.T @ u.T
            if np.linalg.det(rotation) < 0:
                vt[-1] *= -1
                rotation = vt.T @ u.T
            delta = np.r_[bc-rotation@ac, math.atan2(rotation[1, 0], rotation[0, 0])]
            pose = compose(delta, pose)
            if (np.linalg.norm(delta[:2]) < cfg.convergence_m
                    and abs(delta[2]) < cfg.convergence_yaw_rad):
                break
        # Finish against local surfaces: beam sampling along a wall must not
        # pin a correspondence to a particular discrete endpoint.
        for _ in range(cfg.iterations):
            moved, i, j, _ = _pairs(target, source, pose, tree, cfg)
            valid = surface[j]
            if np.count_nonzero(valid) < cfg.min_points*cfg.trim_fraction:
                break
            a, b, n = moved[i][valid], target[j][valid], surface_normals[j][valid]
            h = np.column_stack((n, -n[:, 0]*a[:, 1]+n[:, 1]*a[:, 0]))
            step, _, rank, _ = np.linalg.lstsq(h, np.sum(n*(b-a), axis=1), rcond=None)
            if rank < 3:
                break
            step[:2] *= min(1., cfg.max_refinement_step_m/max(np.linalg.norm(step[:2]), 1e-12))
            step[2] = np.clip(step[2], -cfg.max_refinement_step_yaw_rad,
                              cfg.max_refinement_step_yaw_rad)
            pose = compose(step, pose)
            if (np.linalg.norm(step[:2]) < cfg.convergence_m
                    and abs(step[2]) < cfg.convergence_yaw_rad):
                break
        solutions.append(pose)
    pose = min(solutions, key=lambda p: _cost(tree, source, p, cfg))
    moved, i, j, overlap = _pairs(target, source, pose, tree, cfg)
    if len(i) < cfg.min_points*cfg.trim_fraction:
        result.update(reason='insufficient_overlap', overlap=overlap)
        return result
    residual = float(np.sqrt(np.mean(np.sum((moved[i]-target[j])**2, axis=1))))
    centered = target[j]-target[j].mean(axis=0)
    spread = float(np.sqrt(np.linalg.eigvalsh(centered.T@centered/len(i))[0]))
    # Local surface normals expose sliding along parallel corridor walls, unlike
    # point-to-point Hessians, which falsely assume every beam is a landmark.
    _, neighbors = tree.query(target[j], k=cfg.normal_neighbors)
    patches = target[neighbors]
    patches = patches-patches.mean(axis=1, keepdims=True)
    eig, vec = np.linalg.eigh(np.einsum('nki,nkj->nij', patches, patches))
    normals = vec[:, :, 0]
    planar = eig[:, 0] < cfg.surface_planarity_ratio*np.maximum(eig[:, 1], 1e-12)
    q = moved[i]-moved[i].mean(axis=0)
    radius = max(float(np.sqrt(np.mean(np.sum(q*q, axis=1)))), .1)
    h = np.column_stack((normals, (-normals[:, 0]*q[:, 1]+normals[:, 1]*q[:, 0])/radius))
    info = h[planar].T@h[planar]
    ev = np.linalg.eigvalsh(info)
    ratio = float(ev[0]/max(ev[-1], 1e-12))
    best = _cost(tree, source, pose, cfg)
    competitors = [compose([x, y, a], pose)
                   for x, y, a in ((cfg.distinct_translation_m, 0, 0),
                                   (-cfg.distinct_translation_m, 0, 0),
                                   (0, cfg.distinct_translation_m, 0),
                                   (0, -cfg.distinct_translation_m, 0),
                                   (0, 0, cfg.distinct_yaw_rad),
                                   (0, 0, -cfg.distinct_yaw_rad))]
    for solution in solutions:
        delta = compose(inverse(pose), solution)
        if (np.linalg.norm(delta[:2]) >= cfg.distinct_translation_m
                or abs(delta[2]) >= cfg.distinct_yaw_rad):
            competitors.append(solution)
    other = min(_cost(tree, source, p, cfg) for p in competitors)
    # A perfect discrete fit must not imply infinite measurement precision.
    # Alternatives below the configured noise floor remain indistinguishable.
    distinct = float((other-best)/max(other, cfg.residual_floor_m**2))
    correction = compose(inverse(prior), pose)
    # Scale back yaw column; residual floor prevents zero covariance for exact data.
    scale = np.diag([1., 1., 1./radius])
    covariance = scale@np.linalg.pinv(info)@scale*max(residual**2, cfg.residual_floor_m**2)
    reason = 'candidate'
    if overlap < cfg.min_overlap: reason = 'insufficient_overlap'
    elif residual > cfg.max_residual_m: reason = 'high_residual'
    elif spread < cfg.min_spread_m or ratio < cfg.min_information_ratio: reason = 'degenerate_geometry'
    elif distinct < cfg.min_distinctiveness: reason = 'ambiguous_geometry'
    elif (np.linalg.norm(correction[:2]) > cfg.max_correction_m
          or abs(correction[2]) > cfg.max_correction_yaw_rad): reason = 'correction_limit'
    result.update(accepted=reason == 'candidate', reason=reason,
                  odom1_from_odom2=pose.tolist(), residual=residual, overlap=overlap,
                  covariance=covariance.reshape(-1).tolist(), distinctiveness=distinct,
                  information_ratio=ratio, spread_m=spread)
    return result


class TemporalConfirmation:
    """Consecutive independent scan pairs; frozen after confirmation, fail closed."""
    def __init__(self, required=3, min_interval_s=.4, translation_m=.08, yaw_rad=.06):
        if (int(required) != required or required < 2 or not all(math.isfinite(v) and v > 0
                for v in (min_interval_s, translation_m, yaw_rad))):
            raise ValueError('Invalid confirmation settings')
        self.required, self.interval = required, min_interval_s
        self.translation, self.yaw = translation_m, yaw_rad
        self.samples = []
        self.sample_stamps = []
        self.last_stamps = None
        self.confirmed = None

    def observe(self, candidate, stamps):
        if self.confirmed is not None:
            return self.confirmed
        if (len(stamps) != 2 or not all(math.isfinite(s) and s > 0 for s in stamps)):
            raise ValueError('Two positive scan timestamps required')
        if self.last_stamps is not None and any(
                s-old < self.interval-1e-9 for s, old in zip(stamps, self.last_stamps)):
            return None
        self.last_stamps = tuple(stamps)
        if not candidate['accepted']:
            self.samples.clear()
            self.sample_stamps.clear()
            return None
        pose = candidate['odom1_from_odom2']
        if len(pose) != 3 or not all(math.isfinite(v) for v in pose):
            raise ValueError('Invalid candidate pose')
        if any(np.linalg.norm(compose(inverse(p), pose)[:2]) > self.translation
               or abs(compose(inverse(p), pose)[2]) > self.yaw for p in self.samples):
            self.samples.clear()
            self.sample_stamps.clear()
        self.samples.append(pose)
        self.sample_stamps.append(tuple(stamps))
        if len(self.samples) >= self.required:
            # Retain an actually validated estimate; do not average away fit checks.
            self.confirmed = dict(candidate, accepted=True, reason='confirmed',
                                  confirmations=len(self.samples))
        return self.confirmed
