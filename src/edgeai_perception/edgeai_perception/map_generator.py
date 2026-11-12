#!/usr/bin/env python3
"""
Parametric & Procedural Gazebo Arena + OccupancyGrid Map Generator (Module 2)
KTH EdgeAI Robotics Challenge

Features:
- Zero third-party Python dependencies required (runs natively on macOS & Linux).
- Supports angled (non-orthogonal) walls, rotated box obstacles, and cylindrical
  barrel obstacles (<cylinder> in SDF and rasterized circles in PGM).
- Configurable minimum passable gap (guarantees >= 1 valid collision-free route via
  inflated 8-connected A* verification) AND deceptive narrow "trap gaps" (0.28-0.40m).
- Room partitioning (BSP + graph doorway connection) & T-junction spur walls.
- Exports 5 artifacts per map:
    1) <name>.sdf           : Gazebo Harmonic 3D world with ArUco stands (S1 & C1)
    2) <name>_clean.pgm/.yaml: Ideal 2D OccupancyGrid aligned with robot spawn origin
    3) <name>_noisy.pgm/.yaml: Degraded OccupancyGrid (edge jitter, speckles, unknown
                               gray shadows, slight rotation/offset) for robustness tests
    4) <name>_gt.json       : Ground-truth metadata (walls, obstacles, trap gaps,
                               target poses, C1 dual-robot docking slots, A* paths)
    5) <name>_preview.svg   : Vector visualization of the arena, obstacles, trap gaps,
                               rescue zones, and verified A* paths (viewable on Mac/PC)
"""

import argparse
import copy
import heapq
import json
import math
import os
import random
import shutil
import sys
from typing import Any, Dict, List, Optional, Tuple


# ============================================================================
# Default Configuration & Lightweight YAML/JSON Loader
# ============================================================================

DEFAULT_CONFIG: Dict[str, Any] = {
    'arena': {
        'width': 5.0,
        'height': 5.0,
        'wall_thickness': 0.10,
        'wall_height': 0.50,
        'resolution': 0.05,
        'map_padding_m': 0.25,
        'start_zone': [0.0, 0.0, 1.4, 1.0],
        'spawn_pose': [0.7, 0.5, 0.0],
        'spawn_pose_robot2': [1.15, 0.45, 0.0],
    },
    'rooms_and_walls': {
        'num_rooms': 3,
        'min_room_size': 1.45,
        'allow_tilted_walls': True,
        'wall_tilt_prob': 0.55,
        'max_wall_tilt_deg': 18.0,
        'add_t_spur_wall': True,
        'spur_length_range': [0.40, 0.65],
    },
    'passages_and_traps': {
        'min_passable_gap': 0.88,
        'max_passable_gap': 1.18,
        'enable_trap_gaps': True,
        'trap_gap_range': [0.28, 0.40],
        'robot_inflation_radius': 0.23,
    },
    'obstacles': {
        'num_boxes': 2,
        'num_cylinders': 2,
        'allow_rotated_boxes': True,
        'box_width_range': [0.45, 0.80],
        'box_length_range': [0.50, 0.95],
        'cylinder_radius_range': [0.16, 0.26],
        'obstacle_height': 0.50,
        'min_clearance_to_passages': 0.78,
    },
    'targets': {
        'marker_size': 0.16,
        'S1': {
            'id': 1,
            'type': 'normal',
            'rescue_radius': 0.50,
            'hold_duration_sec': 10.0,
        },
        'C1': {
            'id': 2,
            'type': 'coop',
            'rescue_radius': 0.70,
            'hold_duration_sec': 10.0,
            'docking_slot_dist': 0.44,
            'min_inter_robot_dist': 0.52,
        },
    },
    'noise_test': {
        'export_noisy_map': True,
        'edge_jitter_prob': 0.16,
        'random_speckle_count': 6,
        'unknown_shadow_count': 3,
        'global_offset_m': [0.05, -0.05],
        'global_rotation_deg': 1.5,
    },
}


def _parse_scalar(val_str: str) -> Any:
    val_str = val_str.strip()
    if not val_str:
        return ''
    if val_str.startswith('[') and val_str.endswith(']'):
        inner = val_str[1:-1].strip()
        if not inner:
            return []
        return [_parse_scalar(x) for x in inner.split(',')]
    low = val_str.lower()
    if low == 'true':
        return True
    if low == 'false':
        return False
    try:
        if '.' in val_str or 'e' in low:
            return float(val_str)
        return int(val_str)
    except ValueError:
        return val_str.strip('\'"')


def load_config_file(path: Optional[str]) -> Dict[str, Any]:
    """Load YAML or JSON configuration, merging onto DEFAULT_CONFIG."""
    cfg = copy.deepcopy(DEFAULT_CONFIG)
    if not path or not os.path.isfile(path):
        return cfg

    with open(path, 'r', encoding='utf-8') as f:
        raw = f.read()

    parsed: Optional[Dict[str, Any]] = None
    if path.endswith('.json'):
        parsed = json.loads(raw)
    else:
        try:
            import yaml  # type: ignore
            parsed = yaml.safe_load(raw)
        except Exception:
            # Fallback indentation-based YAML parser for zero-dependency execution
            parsed = {}
            stack: List[Tuple[int, Dict[str, Any]]] = [(-1, parsed)]
            for line in raw.splitlines():
                code = line.split('#', 1)[0].rstrip()
                if not code.strip():
                    continue
                indent = len(code) - len(code.lstrip(' '))
                key_part, _, val_part = code.strip().partition(':')
                key = key_part.strip()
                val_str = val_part.strip()
                while len(stack) > 1 and indent <= stack[-1][0]:
                    stack.pop()
                parent = stack[-1][1]
                if not val_str:
                    new_dict: Dict[str, Any] = {}
                    parent[key] = new_dict
                    stack.append((indent, new_dict))
                else:
                    parent[key] = _parse_scalar(val_str)

    if isinstance(parsed, dict):
        def _deep_update(dst: Dict[str, Any], src: Dict[str, Any]):
            for k, v in src.items():
                if isinstance(v, dict) and isinstance(dst.get(k), dict):
                    _deep_update(dst[k], v)
                else:
                    dst[k] = v
        _deep_update(cfg, parsed)
    return cfg


# ============================================================================
# Geometry Primitives & 2D Grid Rasterization / A* Path Verification
# ============================================================================

def dist2d(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


def point_to_segment_dist(
    px: float, py: float, x1: float, y1: float, x2: float, y2: float
) -> float:
    dx = x2 - x1
    dy = y2 - y1
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq < 1e-9:
        return math.hypot(px - x1, py - y1)
    t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / seg_len_sq))
    proj_x = x1 + t * dx
    proj_y = y1 + t * dy
    return math.hypot(px - proj_x, py - proj_y)


def point_in_rotated_box(
    px: float, py: float, cx: float, cy: float, sx: float, sy: float, yaw: float,
    margin: float = 0.0
) -> bool:
    dx = px - cx
    dy = py - cy
    cos_y = math.cos(-yaw)
    sin_y = math.sin(-yaw)
    lx = dx * cos_y - dy * sin_y
    ly = dx * sin_y + dy * cos_y
    return abs(lx) <= (sx * 0.5 + margin) and abs(ly) <= (sy * 0.5 + margin)


class ArenaLayout:
    """Holds all geometric elements of a generated 5x5m arena."""

    def __init__(self, name: str, width: float, height: float, cfg: Dict[str, Any]):
        self.name = name
        self.width = width
        self.height = height
        self.cfg = cfg
        self.walls: List[Dict[str, Any]] = []
        self.boxes: List[Dict[str, Any]] = []
        self.cylinders: List[Dict[str, Any]] = []
        self.targets: Dict[str, Dict[str, Any]] = {}
        self.rooms: List[Dict[str, Any]] = []
        self.doorways: List[Dict[str, Any]] = []
        self.trap_gaps: List[Dict[str, Any]] = []
        self.verified_paths: Dict[str, List[Tuple[float, float]]] = {}

    def add_perimeter_walls(self, thickness: float = 0.10, height: float = 0.50):
        w, h, t = self.width, self.height, thickness
        # South, North, West, East perimeter walls matching midterm_arena.sdf convention
        self.walls.extend([
            {
                'name': 'south_wall',
                'x1': -t, 'y1': -t * 0.5, 'x2': w + t, 'y2': -t * 0.5,
                'cx': w * 0.5, 'cy': -t * 0.5,
                'length': w + 2 * t, 'thickness': t, 'height': height,
                'yaw': 0.0, 'is_perimeter': True,
            },
            {
                'name': 'north_wall',
                'x1': -t, 'y1': h + t * 0.5, 'x2': w + t, 'y2': h + t * 0.5,
                'cx': w * 0.5, 'cy': h + t * 0.5,
                'length': w + 2 * t, 'thickness': t, 'height': height,
                'yaw': 0.0, 'is_perimeter': True,
            },
            {
                'name': 'west_wall',
                'x1': -t * 0.5, 'y1': -t, 'x2': -t * 0.5, 'y2': h + t,
                'cx': -t * 0.5, 'cy': h * 0.5,
                'length': h + 2 * t, 'thickness': t, 'height': height,
                'yaw': math.pi * 0.5, 'is_perimeter': True,
            },
            {
                'name': 'east_wall',
                'x1': w + t * 0.5, 'y1': -t, 'x2': w + t * 0.5, 'y2': h + t,
                'cx': w + t * 0.5, 'cy': h * 0.5,
                'length': h + 2 * t, 'thickness': t, 'height': height,
                'yaw': math.pi * 0.5, 'is_perimeter': True,
            },
        ])

    def add_wall_segment(
        self, name: str, x1: float, y1: float, x2: float, y2: float,
        thickness: float = 0.10, height: float = 0.50
    ):
        length = math.hypot(x2 - x1, y2 - y1)
        if length < 0.12:
            return
        cx = 0.5 * (x1 + x2)
        cy = 0.5 * (y1 + y2)
        yaw = math.atan2(y2 - y1, x2 - x1)
        self.walls.append({
            'name': name,
            'x1': round(x1, 4), 'y1': round(y1, 4),
            'x2': round(x2, 4), 'y2': round(y2, 4),
            'cx': round(cx, 4), 'cy': round(cy, 4),
            'length': round(length, 4),
            'thickness': round(thickness, 4),
            'height': round(height, 4),
            'yaw': round(yaw, 5),
            'is_perimeter': False,
        })

    def is_point_occupied(
        self, px: float, py: float, clearance: float = 0.0,
        include_target_stands: bool = True
    ) -> bool:
        """Check if point (px, py) collides with any wall, obstacle, or target stand."""
        if px < clearance or px > self.width - clearance:
            return True
        if py < clearance or py > self.height - clearance:
            return True

        for w in self.walls:
            if w['is_perimeter']:
                continue
            if point_in_rotated_box(
                px, py, w['cx'], w['cy'], w['length'], w['thickness'], w['yaw'],
                margin=clearance
            ):
                return True

        for b in self.boxes:
            if point_in_rotated_box(
                px, py, b['cx'], b['cy'], b['size_x'], b['size_y'], b['yaw'],
                margin=clearance
            ):
                return True

        for c in self.cylinders:
            if math.hypot(px - c['cx'], py - c['cy']) <= (c['radius'] + clearance):
                return True

        if include_target_stands:
            for t in self.targets.values():
                # Physical stand base has radius 0.12m
                if math.hypot(px - t['x'], py - t['y']) <= (0.12 + clearance):
                    return True

        return False

    def has_line_of_sight(
        self, x1: float, y1: float, x2: float, y2: float, clearance: float = 0.05
    ) -> bool:
        """Raymarch check between two points (excluding target stands)."""
        d = math.hypot(x2 - x1, y2 - y1)
        steps = max(4, int(d / 0.04))
        for i in range(1, steps):
            u = i / float(steps)
            px = x1 + u * (x2 - x1)
            py = y1 + u * (y2 - y1)
            if self.is_point_occupied(px, py, clearance=clearance, include_target_stands=False):
                return False
        return True


def rasterize_layout(
    layout: ArenaLayout,
    include_target_stands: bool = False
) -> Tuple[List[List[int]], Dict[str, Any]]:
    """
    Rasterize ArenaLayout into a 2D OccupancyGrid image matrix.
    Pixel conventions (ROS nav_msgs/map_saver standard):
      254 = Free space (0)
      0   = Occupied wall/obstacle (100)
      205 = Unknown (-1)
    Row 0 is the TOP of the image (highest Y), Row H-1 is the BOTTOM (lowest Y).
    """
    acfg = layout.cfg['arena']
    res = float(acfg['resolution'])
    pad = float(acfg['map_padding_m'])

    min_x = -pad
    min_y = -pad
    max_x = layout.width + pad
    max_y = layout.height + pad

    grid_w = int(round((max_x - min_x) / res))
    grid_h = int(round((max_y - min_y) / res))

    # Initialize outside arena bounds as unknown (205), inside as free (254)
    grid: List[List[int]] = [[205 for _ in range(grid_w)] for _ in range(grid_h)]

    for r in range(grid_h):
        # World Y coordinate of row center (row 0 is top -> max_y)
        wy = max_y - (r + 0.5) * res
        for c in range(grid_w):
            wx = min_x + (c + 0.5) * res
            if 0.0 <= wx <= layout.width and 0.0 <= wy <= layout.height:
                grid[r][c] = 254

            # Check perimeter + internal walls
            occupied = False
            for w in layout.walls:
                if point_in_rotated_box(
                    wx, wy, w['cx'], w['cy'], w['length'], w['thickness'], w['yaw'],
                    margin=res * 0.35
                ):
                    occupied = True
                    break

            if not occupied:
                for b in layout.boxes:
                    if point_in_rotated_box(
                        wx, wy, b['cx'], b['cy'], b['size_x'], b['size_y'], b['yaw'],
                        margin=res * 0.25
                    ):
                        occupied = True
                        break

            if not occupied:
                for cyl in layout.cylinders:
                    if math.hypot(wx - cyl['cx'], wy - cyl['cy']) <= (cyl['radius'] + res * 0.25):
                        occupied = True
                        break

            if not occupied and include_target_stands:
                for t in layout.targets.values():
                    if math.hypot(wx - t['x'], wy - t['y']) <= 0.13:
                        occupied = True
                        break

            if occupied:
                grid[r][c] = 0

    spawn_x, spawn_y, _ = acfg['spawn_pose']
    # In odom/map frame where robot spawns at (0,0) corresponding to arena (spawn_x, spawn_y),
    # the map lower-left pixel (min_x, min_y) is at (min_x - spawn_x, min_y - spawn_y)
    meta = {
        'resolution': res,
        'width_px': grid_w,
        'height_px': grid_h,
        'arena_min_x': min_x,
        'arena_min_y': min_y,
        'arena_max_x': max_x,
        'arena_max_y': max_y,
        'origin_odom': [round(min_x - spawn_x, 4), round(min_y - spawn_y, 4), 0.0],
        'origin_arena': [round(min_x, 4), round(min_y, 4), 0.0],
    }
    return grid, meta


def inflate_grid(grid: List[List[int]], inflation_radius_m: float, res: float) -> List[List[int]]:
    """Inflate occupied (0) cells by inflation_radius_m for collision-free A* planning."""
    h = len(grid)
    w = len(grid[0])
    rad_px = int(math.ceil(inflation_radius_m / res))
    out = [row[:] for row in grid]

    offsets: List[Tuple[int, int]] = []
    for dr in range(-rad_px, rad_px + 1):
        for dc in range(-rad_px, rad_px + 1):
            if math.hypot(dr * res, dc * res) <= inflation_radius_m:
                offsets.append((dr, dc))

    for r in range(h):
        for c in range(w):
            if grid[r][c] == 0:
                for dr, dc in offsets:
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < h and 0 <= nc < w:
                        out[nr][nc] = 0
    return out


def astar_search(
    inflated_grid: List[List[int]],
    meta: Dict[str, Any],
    start_xy: Tuple[float, float],
    goal_xy: Tuple[float, float]
) -> Optional[List[Tuple[float, float]]]:
    """8-connected A* search in Arena coordinates on an inflated grid."""
    res = meta['resolution']
    min_x = meta['arena_min_x']
    max_y = meta['arena_max_y']
    h = meta['height_px']
    w = meta['width_px']

    def world_to_rc(wx: float, wy: float) -> Tuple[int, int]:
        c = int((wx - min_x) / res)
        r = int((max_y - wy) / res)
        return max(0, min(h - 1, r)), max(0, min(w - 1, c))

    def rc_to_world(r: int, c: int) -> Tuple[float, float]:
        wx = min_x + (c + 0.5) * res
        wy = max_y - (r + 0.5) * res
        return round(wx, 3), round(wy, 3)

    sr, sc = world_to_rc(start_xy[0], start_xy[1])
    gr, gc = world_to_rc(goal_xy[0], goal_xy[1])

    if inflated_grid[sr][sc] != 254 or inflated_grid[gr][gc] != 254:
        return None

    neighbors = [
        (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
        (-1, -1, 1.4142), (-1, 1, 1.4142), (1, -1, 1.4142), (1, 1, 1.4142),
    ]

    open_heap: List[Tuple[float, int, int]] = []
    heapq.heappush(open_heap, (0.0, sr, sc))
    g_score: Dict[Tuple[int, int], float] = {(sr, sc): 0.0}
    came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}

    while open_heap:
        _, cr, cc = heapq.heappop(open_heap)
        if (cr, cc) == (gr, gc):
            path_rc = [(cr, cc)]
            while (cr, cc) in came_from:
                cr, cc = came_from[(cr, cc)]
                path_rc.append((cr, cc))
            path_rc.reverse()
            # Subsample path waypoints for clean visualization & tracking
            world_pts = [rc_to_world(r, c) for r, c in path_rc[::3]]
            if world_pts[-1] != rc_to_world(gr, gc):
                world_pts.append(rc_to_world(gr, gc))
            return world_pts

        for dr, dc, step_cost in neighbors:
            nr, nc = cr + dr, cc + dc
            if not (0 <= nr < h and 0 <= nc < w):
                continue
            if inflated_grid[nr][nc] != 254:
                continue
            tentative_g = g_score[(cr, cc)] + step_cost
            if tentative_g < g_score.get((nr, nc), 1e9):
                came_from[(nr, nc)] = (cr, cc)
                g_score[(nr, nc)] = tentative_g
                heur = math.hypot(nr - gr, nc - gc)
                heapq.heappush(open_heap, (tentative_g + heur, nr, nc))

    return None


# ============================================================================
# Preset 1: Official Midterm Check Arena (Exact Specification)
# ============================================================================

def build_midterm_preset(cfg: Dict[str, Any]) -> ArenaLayout:
    """Construct the exact 5.0m x 5.0m Midterm Check arena from Figure 1."""
    acfg = cfg['arena']
    layout = ArenaLayout(
        name='midterm_arena',
        width=float(acfg['width']),
        height=float(acfg['height']),
        cfg=cfg,
    )
    layout.add_perimeter_walls(
        thickness=float(acfg['wall_thickness']),
        height=float(acfg['wall_height']),
    )

    # Internal walls W1, W1b, W2 per Midterm Task Description Section 3
    layout.add_wall_segment('wall_W1', 0.0, 3.1, 2.2, 3.1, thickness=0.10, height=0.50)
    layout.add_wall_segment('wall_W1b', 2.2, 3.1, 2.2, 3.6, thickness=0.10, height=0.50)
    layout.add_wall_segment('wall_W2', 3.4, 0.0, 3.4, 2.8, thickness=0.10, height=0.50)

    # Obstacles O2 and O3
    layout.boxes.append({
        'name': 'obstacle_O2',
        'cx': 2.2, 'cy': 2.0,
        'size_x': 0.6, 'size_y': 1.0, 'size_z': 0.5,
        'yaw': 0.0,
    })
    layout.boxes.append({
        'name': 'obstacle_O3',
        'cx': 4.4, 'cy': 3.6,
        'size_x': 0.8, 'size_y': 0.6, 'size_z': 0.5,
        'yaw': 0.0,
    })

    # Logical room centroids for documentation
    layout.rooms = [
        {'id': 0, 'name': 'Start & West Room', 'cx': 1.0, 'cy': 1.5, 'bounds': [0.0, 0.0, 2.2, 3.1]},
        {'id': 1, 'name': 'North-West S1 Room', 'cx': 1.1, 'cy': 4.1, 'bounds': [0.0, 3.1, 2.2, 5.0]},
        {'id': 2, 'name': 'Central Corridor', 'cx': 2.8, 'cy': 2.2, 'bounds': [2.2, 0.0, 3.4, 5.0]},
        {'id': 3, 'name': 'East C1 Room', 'cx': 4.2, 'cy': 1.8, 'bounds': [3.4, 0.0, 5.0, 5.0]},
    ]

    # Targets S1 and C1
    layout.targets['S1'] = {
        'name': 'S1',
        'id': 1,
        'type': 'normal',
        'x': 1.00,
        'y': 4.10,
        'yaw': -0.35,
        'rescue_radius': 0.50,
        'hold_duration_sec': 10.0,
        'viewpoint': [1.80, 3.80, 2.79],
        'docking_slots': [[1.32, 3.88, 2.55]],
    }
    layout.targets['C1'] = {
        'name': 'C1',
        'id': 2,
        'type': 'coop',
        'x': 4.10,
        'y': 1.00,
        'yaw': 1.5708,
        'rescue_radius': 0.70,
        'hold_duration_sec': 10.0,
        'viewpoint': [4.10, 2.00, -1.5708],
        'docking_slots': [
            [3.88, 1.42, -1.09],    # Slot 1: North-West of C1 (dist = 0.47m <= 0.70m, 0.43m clear of W2)
            [4.42, 1.30, -2.39],    # Slot 2: North-East of C1 (dist = 0.44m <= 0.70m, robot_sep = 0.55m)
        ],
    }

    # Verify paths with A*
    grid, meta = rasterize_layout(layout, include_target_stands=True)
    inf_grid = inflate_grid(
        grid,
        float(cfg['passages_and_traps']['robot_inflation_radius']),
        meta['resolution'],
    )
    r1_start = (float(acfg['spawn_pose'][0]), float(acfg['spawn_pose'][1]))
    r2_start = (float(acfg['spawn_pose_robot2'][0]), float(acfg['spawn_pose_robot2'][1]))

    p_s1 = astar_search(inf_grid, meta, r1_start, (1.32, 3.88))
    p_c1_r1 = astar_search(inf_grid, meta, (1.32, 3.88), (3.88, 1.42))
    p_c1_r2 = astar_search(inf_grid, meta, r2_start, (4.42, 1.30))
    if p_s1:
        layout.verified_paths['robot1_to_S1'] = p_s1
    if p_c1_r1:
        layout.verified_paths['S1_to_C1_slot1'] = p_c1_r1
    if p_c1_r2:
        layout.verified_paths['robot2_to_C1_slot2'] = p_c1_r2

    return layout


# ============================================================================
# Preset 2: Procedural Random Arena Generator (BSP + Nodes + Traps + Cylinders)
# ============================================================================

def _find_c1_docking_slots(
    layout: ArenaLayout, cx: float, cy: float, slot_dist: float, min_sep: float, clearance: float
) -> Optional[List[List[float]]]:
    """Find 2 collision-free docking poses around C1 within rescue_radius and >= min_sep apart."""
    candidates: List[Tuple[float, float, float]] = []
    for deg in range(0, 360, 15):
        ang = math.radians(deg)
        sx = cx + slot_dist * math.cos(ang)
        sy = cy + slot_dist * math.sin(ang)
        if not layout.is_point_occupied(sx, sy, clearance=clearance, include_target_stands=True):
            # Face toward the marker stand
            face_yaw = math.atan2(cy - sy, cx - sx)
            candidates.append((round(sx, 3), round(sy, 3), round(face_yaw, 4)))

    for i in range(len(candidates)):
        for j in range(i + 1, len(candidates)):
            c1 = candidates[i]
            c2 = candidates[j]
            if math.hypot(c1[0] - c2[0], c1[1] - c2[1]) >= min_sep:
                return [list(c1), list(c2)]
    return None


def build_random_arena(
    cfg: Dict[str, Any], seed: int, map_name: Optional[str] = None
) -> ArenaLayout:
    """
    Generate a procedural 5x5m arena using BSP room splitting, node doorway linking,
    optional tilted walls, deceptive trap gaps, rotated boxes, and cylinder barrels.
    Automatically retries sub-seeds until 100% A* traversability is verified.
    """
    acfg = cfg['arena']
    rcfg = cfg['rooms_and_walls']
    pcfg = cfg['passages_and_traps']
    ocfg = cfg['obstacles']
    tcfg = cfg['targets']

    w = float(acfg['width'])
    h = float(acfg['height'])
    wall_t = float(acfg['wall_thickness'])
    wall_h = float(acfg['wall_height'])
    min_room = float(rcfg['min_room_size'])
    num_rooms = max(2, min(4, int(rcfg['num_rooms'])))
    min_gap = float(pcfg['min_passable_gap'])
    max_gap = float(pcfg['max_passable_gap'])
    trap_min, trap_max = [float(v) for v in pcfg['trap_gap_range']]
    inf_rad = float(pcfg['robot_inflation_radius'])

    r1_start = (float(acfg['spawn_pose'][0]), float(acfg['spawn_pose'][1]))
    r2_start = (float(acfg['spawn_pose_robot2'][0]), float(acfg['spawn_pose_robot2'][1]))
    start_box = [float(v) for v in acfg['start_zone']]

    name = map_name or f'random_arena_s{seed}'

    for attempt in range(60):
        rng = random.Random(seed * 1009 + attempt * 37)
        layout = ArenaLayout(name=name, width=w, height=h, cfg=cfg)
        layout.add_perimeter_walls(thickness=wall_t, height=wall_h)

        # --------------------------------------------------------------------
        # Step 1: BSP Room Partitioning (1 wall splits 1 space into 2 spaces)
        # --------------------------------------------------------------------
        rects: List[List[float]] = [[0.0, 0.0, w, h]]
        splits: List[Dict[str, Any]] = []

        for split_idx in range(num_rooms - 1):
            # Pick largest room that can be split while preserving min_room_size
            splittable = []
            for idx, r in enumerate(rects):
                rw = r[2] - r[0]
                rh = r[3] - r[1]
                if rw >= 2 * min_room + 0.2 or rh >= 2 * min_room + 0.2:
                    splittable.append((rw * rh, idx))
            if not splittable:
                break
            splittable.sort(reverse=True)
            _, chosen_idx = splittable[0]
            x0, y0, x1, y1 = rects.pop(chosen_idx)
            rw, rh = x1 - x0, y1 - y0

            # Decide split orientation
            if rw >= 2 * min_room + 0.2 and (rh < 2 * min_room + 0.2 or rw > rh * 1.15):
                # Vertical wall at x = sx
                sx = rng.uniform(x0 + min_room, x1 - min_room)
                # Ensure we don't slice right through the robot start zone
                if y0 < start_box[3] + 0.2 and sx < start_box[2] + 0.35:
                    sx = max(sx, start_box[2] + 0.45)
                rects.append([x0, y0, sx, y1])
                rects.append([sx, y0, x1, y1])
                splits.append({
                    'id': split_idx + 1,
                    'orient': 'vertical',
                    'x1': sx, 'y1': y0, 'x2': sx, 'y2': y1,
                    'bounds': [x0, y0, x1, y1],
                })
            else:
                # Horizontal wall at y = sy
                sy = rng.uniform(y0 + min_room, y1 - min_room)
                if x0 < start_box[2] + 0.2 and sy < start_box[3] + 0.35:
                    sy = max(sy, start_box[3] + 0.45)
                rects.append([x0, y0, x1, sy])
                rects.append([x0, sy, x1, y1])
                splits.append({
                    'id': split_idx + 1,
                    'orient': 'horizontal',
                    'x1': x0, 'y1': sy, 'x2': x1, 'y2': sy,
                    'bounds': [x0, y0, x1, y1],
                })

        for r_idx, r in enumerate(rects):
            layout.rooms.append({
                'id': r_idx,
                'name': f'Room_{r_idx + 1}',
                'cx': round(0.5 * (r[0] + r[2]), 3),
                'cy': round(0.5 * (r[1] + r[3]), 3),
                'bounds': [round(v, 3) for v in r],
            })

        # --------------------------------------------------------------------
        # Step 2: Connect Room Nodes with True Passages + Optional Trap Gaps & Tilt
        # --------------------------------------------------------------------
        for sp in splits:
            wx1, wy1, wx2, wy2 = sp['x1'], sp['y1'], sp['x2'], sp['y2']
            bx0, by0, bx1, by1 = sp['bounds']

            # Apply optional wall tilt around midpoint while clamping to parent bounds
            if rcfg.get('allow_tilted_walls', True) and rng.random() < float(rcfg.get('wall_tilt_prob', 0.5)):
                max_tilt = math.radians(float(rcfg.get('max_wall_tilt_deg', 18.0)))
                tilt = rng.uniform(-max_tilt, max_tilt)
                if abs(tilt) > math.radians(4.0):
                    mx, my = 0.5 * (wx1 + wx2), 0.5 * (wy1 + wy2)
                    half_len = 0.5 * math.hypot(wx2 - wx1, wy2 - wy1) * 0.92
                    base_ang = math.atan2(wy2 - wy1, wx2 - wx1) + tilt
                    nx1 = max(bx0 + 0.05, min(bx1 - 0.05, mx - half_len * math.cos(base_ang)))
                    ny1 = max(by0 + 0.05, min(by1 - 0.05, my - half_len * math.sin(base_ang)))
                    nx2 = max(bx0 + 0.05, min(bx1 - 0.05, mx + half_len * math.cos(base_ang)))
                    ny2 = max(by0 + 0.05, min(by1 - 0.05, my + half_len * math.sin(base_ang)))
                    wx1, wy1, wx2, wy2 = nx1, ny1, nx2, ny2

            total_len = math.hypot(wx2 - wx1, wy2 - wy1)
            ux = (wx2 - wx1) / max(1e-6, total_len)
            uy = (wy2 - wy1) / max(1e-6, total_len)

            # Carve a guaranteed passable doorway along the wall
            door_w = rng.uniform(min_gap, max_gap)
            # Pick doorway center biased toward one side or middle
            margin = max(0.35, door_w * 0.5 + 0.15)
            if total_len <= door_w + 0.45:
                continue
            door_pos = rng.uniform(margin, total_len - margin)
            d_start = max(0.0, door_pos - door_w * 0.5)
            d_end = min(total_len, door_pos + door_w * 0.5)

            door_cx = wx1 + ux * door_pos
            door_cy = wy1 + uy * door_pos
            layout.doorways.append({
                'wall_id': sp['id'],
                'cx': round(door_cx, 3),
                'cy': round(door_cy, 3),
                'width': round(d_end - d_start, 3),
            })

            # Segment A: from 0 to d_start
            seg_a_start = 0.0
            if pcfg.get('enable_trap_gaps', True) and d_start > 1.05 and sp['id'] == 1:
                # Carve a deceptive narrow trap gap at the start of the wall
                trap_w = rng.uniform(trap_min, trap_max)
                seg_a_start = trap_w
                layout.trap_gaps.append({
                    'name': f"trap_gap_W{sp['id']}",
                    'x1': round(wx1, 3), 'y1': round(wy1, 3),
                    'x2': round(wx1 + ux * trap_w, 3), 'y2': round(wy1 + uy * trap_w, 3),
                    'width': round(trap_w, 3),
                })

            if d_start - seg_a_start >= 0.25:
                layout.add_wall_segment(
                    f"wall_W{sp['id']}a",
                    wx1 + ux * seg_a_start, wy1 + uy * seg_a_start,
                    wx1 + ux * d_start, wy1 + uy * d_start,
                    thickness=wall_t, height=wall_h,
                )

            # Segment B: from d_end to total_len
            seg_b_end = total_len
            if (
                pcfg.get('enable_trap_gaps', True)
                and (total_len - d_end) > 1.05
                and len(layout.trap_gaps) == 0
            ):
                trap_w = rng.uniform(trap_min, trap_max)
                seg_b_end = total_len - trap_w
                layout.trap_gaps.append({
                    'name': f"trap_gap_W{sp['id']}b",
                    'x1': round(wx1 + ux * seg_b_end, 3), 'y1': round(wy1 + uy * seg_b_end, 3),
                    'x2': round(wx2, 3), 'y2': round(wy2, 3),
                    'width': round(trap_w, 3),
                })

            if seg_b_end - d_end >= 0.25:
                layout.add_wall_segment(
                    f"wall_W{sp['id']}b",
                    wx1 + ux * d_end, wy1 + uy * d_end,
                    wx1 + ux * seg_b_end, wy1 + uy * seg_b_end,
                    thickness=wall_t, height=wall_h,
                )

            # Optional perpendicular T-junction spur wall at one doorway tip (like W1b)
            if rcfg.get('add_t_spur_wall', True) and sp['id'] == 1:
                spur_len = rng.uniform(*[float(v) for v in rcfg.get('spur_length_range', [0.40, 0.60])])
                tip_x = wx1 + ux * d_start if d_start >= (total_len - d_end) else wx1 + ux * d_end
                tip_y = wy1 + uy * d_start if d_start >= (total_len - d_end) else wy1 + uy * d_end
                px, py = -uy, ux
                sx2 = tip_x + px * spur_len
                sy2 = tip_y + py * spur_len
                if 0.45 < sx2 < w - 0.45 and 0.45 < sy2 < h - 0.45:
                    layout.add_wall_segment(
                        f"wall_W{sp['id']}_spur",
                        tip_x, tip_y, sx2, sy2,
                        thickness=wall_t, height=wall_h,
                    )

        # --------------------------------------------------------------------
        # Step 3: Place S1 (Single Rescue) & C1 (Cooperative Rescue) Targets
        # --------------------------------------------------------------------
        # Sort rooms by distance from start zone so targets are placed in non-start rooms
        candidate_rooms = sorted(
            layout.rooms,
            key=lambda rm: math.hypot(rm['cx'] - r1_start[0], rm['cy'] - r1_start[1]),
            reverse=True,
        )

        # Place S1
        s1_placed = False
        for rm in candidate_rooms:
            rx0, ry0, rx1, ry1 = rm['bounds']
            for _ in range(25):
                tx = rng.uniform(max(0.65, rx0 + 0.55), min(w - 0.65, rx1 - 0.55))
                ty = rng.uniform(max(0.65, ry0 + 0.55), min(h - 0.65, ry1 - 0.55))
                if tx < start_box[2] + 0.5 and ty < start_box[3] + 0.5:
                    continue
                if layout.is_point_occupied(tx, ty, clearance=0.52, include_target_stands=False):
                    continue
                # Find clear approach direction (viewpoint at 0.85m and rescue hold at 0.36m)
                for deg in rng.sample(list(range(0, 360, 20)), 18):
                    ang = math.radians(deg)
                    vx = tx + 0.85 * math.cos(ang)
                    vy = ty + 0.85 * math.sin(ang)
                    hx = tx + 0.36 * math.cos(ang)
                    hy = ty + 0.36 * math.sin(ang)
                    if (
                        not layout.is_point_occupied(vx, vy, clearance=0.30, include_target_stands=False)
                        and not layout.is_point_occupied(hx, hy, clearance=0.30, include_target_stands=False)
                        and layout.has_line_of_sight(tx, ty, vx, vy, clearance=0.15)
                    ):
                        layout.targets['S1'] = {
                            'name': 'S1',
                            'id': int(tcfg['S1']['id']),
                            'type': 'normal',
                            'x': round(tx, 3),
                            'y': round(ty, 3),
                            'yaw': round(ang, 4),
                            'rescue_radius': float(tcfg['S1']['rescue_radius']),
                            'hold_duration_sec': float(tcfg['S1']['hold_duration_sec']),
                            'viewpoint': [round(vx, 3), round(vy, 3), round(math.atan2(ty - vy, tx - vx), 4)],
                            'docking_slots': [[round(hx, 3), round(hy, 3), round(math.atan2(ty - hy, tx - hx), 4)]],
                        }
                        s1_placed = True
                        break
                if s1_placed:
                    break
            if s1_placed:
                break

        if not s1_placed:
            continue

        # Place C1 at least 2.0m away from S1 with space for 2 robots
        c1_placed = False
        s1_xy = (layout.targets['S1']['x'], layout.targets['S1']['y'])
        slot_dist = float(tcfg['C1'].get('docking_slot_dist', 0.44))
        min_sep = float(tcfg['C1'].get('min_inter_robot_dist', 0.52))

        for rm in reversed(candidate_rooms):
            rx0, ry0, rx1, ry1 = rm['bounds']
            for _ in range(30):
                tx = rng.uniform(max(0.80, rx0 + 0.60), min(w - 0.80, rx1 - 0.60))
                ty = rng.uniform(max(0.80, ry0 + 0.60), min(h - 0.80, ry1 - 0.60))
                if tx < start_box[2] + 0.5 and ty < start_box[3] + 0.5:
                    continue
                if math.hypot(tx - s1_xy[0], ty - s1_xy[1]) < 1.95:
                    continue
                if layout.is_point_occupied(tx, ty, clearance=0.60, include_target_stands=True):
                    continue

                slots = _find_c1_docking_slots(layout, tx, ty, slot_dist, min_sep, clearance=0.28)
                if not slots:
                    continue

                for deg in rng.sample(list(range(0, 360, 20)), 18):
                    ang = math.radians(deg)
                    vx = tx + 0.90 * math.cos(ang)
                    vy = ty + 0.90 * math.sin(ang)
                    if (
                        not layout.is_point_occupied(vx, vy, clearance=0.30, include_target_stands=True)
                        and layout.has_line_of_sight(tx, ty, vx, vy, clearance=0.15)
                    ):
                        layout.targets['C1'] = {
                            'name': 'C1',
                            'id': int(tcfg['C1']['id']),
                            'type': 'coop',
                            'x': round(tx, 3),
                            'y': round(ty, 3),
                            'yaw': round(ang, 4),
                            'rescue_radius': float(tcfg['C1']['rescue_radius']),
                            'hold_duration_sec': float(tcfg['C1']['hold_duration_sec']),
                            'viewpoint': [round(vx, 3), round(vy, 3), round(math.atan2(ty - vy, tx - vx), 4)],
                            'docking_slots': slots,
                        }
                        c1_placed = True
                        break
                if c1_placed:
                    break
            if c1_placed:
                break

        if not c1_placed:
            continue

        # --------------------------------------------------------------------
        # Step 4: Place Rotated Box Obstacles & Cylindrical Barrel Obstacles
        # --------------------------------------------------------------------
        obs_h = float(ocfg.get('obstacle_height', 0.50))
        min_clear = float(ocfg.get('min_clearance_to_passages', 0.78))

        def _is_safe_for_obstacle(ox: float, oy: float, rad: float) -> bool:
            if ox < start_box[2] + rad + 0.35 and oy < start_box[3] + rad + 0.35:
                return False
            for d in layout.doorways:
                if math.hypot(ox - d['cx'], oy - d['cy']) < (rad + min_clear * 0.75):
                    return False
            for t in layout.targets.values():
                if math.hypot(ox - t['x'], oy - t['y']) < (rad + t['rescue_radius'] + 0.25):
                    return False
                vp = t['viewpoint']
                if math.hypot(ox - vp[0], oy - vp[1]) < (rad + 0.45):
                    return False
            return not layout.is_point_occupied(ox, oy, clearance=rad + 0.38, include_target_stands=True)

        # Add rectangular boxes (rotated or axis-aligned)
        for b_idx in range(int(ocfg.get('num_boxes', 2))):
            bw = rng.uniform(*[float(v) for v in ocfg.get('box_width_range', [0.45, 0.80])])
            bl = rng.uniform(*[float(v) for v in ocfg.get('box_length_range', [0.50, 0.95])])
            byaw = rng.uniform(-math.pi * 0.35, math.pi * 0.35) if ocfg.get('allow_rotated_boxes', True) else 0.0
            brad = 0.5 * math.hypot(bw, bl)
            for _ in range(35):
                bx = rng.uniform(brad + 0.45, w - brad - 0.45)
                by = rng.uniform(brad + 0.45, h - brad - 0.45)
                if _is_safe_for_obstacle(bx, by, brad):
                    layout.boxes.append({
                        'name': f'obstacle_box_{b_idx + 1}',
                        'cx': round(bx, 3), 'cy': round(by, 3),
                        'size_x': round(bw, 3), 'size_y': round(bl, 3), 'size_z': round(obs_h, 3),
                        'yaw': round(byaw, 4),
                    })
                    break

        # Add cylindrical barrel obstacles (<cylinder> in SDF)
        for c_idx in range(int(ocfg.get('num_cylinders', 2))):
            crad = rng.uniform(*[float(v) for v in ocfg.get('cylinder_radius_range', [0.16, 0.26])])
            for _ in range(35):
                cx = rng.uniform(crad + 0.45, w - crad - 0.45)
                cy = rng.uniform(crad + 0.45, h - crad - 0.45)
                if _is_safe_for_obstacle(cx, cy, crad):
                    layout.cylinders.append({
                        'name': f'obstacle_barrel_{c_idx + 1}',
                        'cx': round(cx, 3), 'cy': round(cy, 3),
                        'radius': round(crad, 3), 'height': round(obs_h, 3),
                    })
                    break

        # --------------------------------------------------------------------
        # Step 5: Inflated Grid A* Verification (Guarantees >= 1 True Route)
        # --------------------------------------------------------------------
        grid, meta = rasterize_layout(layout, include_target_stands=True)
        inf_grid = inflate_grid(grid, inf_rad, meta['resolution'])

        s1_slot = tuple(layout.targets['S1']['docking_slots'][0][:2])
        c1_slot1 = tuple(layout.targets['C1']['docking_slots'][0][:2])
        c1_slot2 = tuple(layout.targets['C1']['docking_slots'][1][:2])

        p_r1_s1 = astar_search(inf_grid, meta, r1_start, s1_slot)
        p_s1_c1 = astar_search(inf_grid, meta, s1_slot, c1_slot1)
        p_r2_c1 = astar_search(inf_grid, meta, r2_start, c1_slot2)

        if p_r1_s1 and p_s1_c1 and p_r2_c1:
            layout.verified_paths['robot1_to_S1'] = p_r1_s1
            layout.verified_paths['S1_to_C1_slot1'] = p_s1_c1
            layout.verified_paths['robot2_to_C1_slot2'] = p_r2_c1
            return layout

    raise RuntimeError(f'Failed to generate a connected arena after 60 attempts for seed={seed}')


# ============================================================================
# Noisy OccupancyGrid Generator (Robustness Testing for Navigation & SLAM)
# ============================================================================

def generate_noisy_grid(
    clean_grid: List[List[int]],
    meta: Dict[str, Any],
    layout: ArenaLayout,
    seed: int = 101
) -> Tuple[List[List[int]], Dict[str, Any]]:
    """
    Create a realistic degraded/noisy OccupancyGrid from clean_grid:
      1) Slight global angular rotation (simulating IMU/odom heading drift)
      2) 1-pixel LiDAR boundary roughness (edge jitter)
      3) Random isolated occupied speckles (salt & pepper sensor noise)
      4) Unexplored gray (205 / -1) shadow patches behind obstacles/corners
      5) Slight global origin translation offset in YAML metadata
    """
    ncfg = layout.cfg['noise_test']
    rng = random.Random(seed + 777)
    h = len(clean_grid)
    w = len(clean_grid[0])

    # 1) Global slight rotation around grid center
    rot_deg = float(ncfg.get('global_rotation_deg', 1.5))
    rot_rad = math.radians(rot_deg)
    cos_r = math.cos(-rot_rad)
    sin_r = math.sin(-rot_rad)
    cr_mid = h * 0.5
    cc_mid = w * 0.5

    rotated = [[205 for _ in range(w)] for _ in range(h)]
    for r in range(h):
        for c in range(w):
            dr = r - cr_mid
            dc = c - cc_mid
            sr = int(round(cr_mid + dr * cos_r - dc * sin_r))
            sc = int(round(cc_mid + dr * sin_r + dc * cos_r))
            if 0 <= sr < h and 0 <= sc < w:
                rotated[r][c] = clean_grid[sr][sc]

    noisy = [row[:] for row in rotated]

    # 2) LiDAR boundary edge jitter
    jitter_prob = float(ncfg.get('edge_jitter_prob', 0.16))
    for r in range(1, h - 1):
        for c in range(1, w - 1):
            if rotated[r][c] == 0:
                # Check if adjacent to free space
                adj_free = [
                    (r + dr, c + dc)
                    for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]
                    if rotated[r + dr][c + dc] == 254
                ]
                if adj_free and rng.random() < jitter_prob:
                    if rng.random() < 0.65:
                        # Bump out by 1 pixel
                        nr, nc = rng.choice(adj_free)
                        noisy[nr][nc] = 0
                    else:
                        # Erode by 1 pixel
                        noisy[r][c] = 254

    # 3) Unknown shadow patches (205) near corners or behind obstacles
    shadow_count = int(ncfg.get('unknown_shadow_count', 3))
    for _ in range(shadow_count):
        sr = rng.randint(6, h - 10)
        sc = rng.randint(6, w - 10)
        rad_r = rng.randint(2, 4)
        rad_c = rng.randint(2, 4)
        for rr in range(sr - rad_r, sr + rad_r + 1):
            for cc in range(sc - rad_c, sc + rad_c + 1):
                if 0 <= rr < h and 0 <= cc < w and noisy[rr][cc] == 254:
                    noisy[rr][cc] = 205

    # 4) Isolated occupied speckles in free space (away from robot start & target slots)
    speckle_count = int(ncfg.get('random_speckle_count', 6))
    res = meta['resolution']
    min_x = meta['arena_min_x']
    max_y = meta['arena_max_y']
    r1_x, r1_y, _ = layout.cfg['arena']['spawn_pose']

    placed_speckles = 0
    for _ in range(speckle_count * 10):
        if placed_speckles >= speckle_count:
            break
        rr = rng.randint(8, h - 9)
        cc = rng.randint(8, w - 9)
        if noisy[rr][cc] != 254:
            continue
        wx = min_x + (cc + 0.5) * res
        wy = max_y - (rr + 0.5) * res
        if math.hypot(wx - r1_x, wy - r1_y) < 0.8:
            continue
        noisy[rr][cc] = 0
        placed_speckles += 1

    # 5) Global translation offset in metadata
    noisy_meta = copy.deepcopy(meta)
    dx, dy = [float(v) for v in ncfg.get('global_offset_m', [0.05, -0.05])]
    noisy_meta['origin_odom'] = [
        round(meta['origin_odom'][0] + dx, 4),
        round(meta['origin_odom'][1] + dy, 4),
        0.0,
    ]
    noisy_meta['injected_noise'] = {
        'edge_jitter_prob': jitter_prob,
        'speckles': placed_speckles,
        'unknown_shadows': shadow_count,
        'global_offset_m': [dx, dy],
        'global_rotation_deg': rot_deg,
    }
    return noisy, noisy_meta


# ============================================================================
# Exporters: SDF, PGM, YAML, Ground-Truth JSON, and Preview SVG
# ============================================================================

def export_sdf(layout: ArenaLayout, sdf_path: str):
    """Export ArenaLayout to a complete Gazebo Harmonic .sdf world file."""
    lines = [
        '<?xml version="1.0" ?>',
        f'<!-- Auto-generated by edgeai_perception/map_generator.py: {layout.name} -->',
        '<sdf version="1.9">',
        f'  <world name="{layout.name}">',
        '',
        '    <physics name="physics" type="ignored">',
        '      <max_step_size>0.001</max_step_size>',
        '      <real_time_factor>1.0</real_time_factor>',
        '    </physics>',
        '',
        '    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>',
        '    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>',
        '    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>',
        '    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">',
        '      <render_engine>ogre2</render_engine>',
        '    </plugin>',
        '',
        '    <model name="ground_plane">',
        '      <static>true</static>',
        '      <link name="link">',
        '        <collision name="collision">',
        '          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>',
        '        </collision>',
        '        <visual name="visual">',
        '          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>',
        '          <material>',
        '            <ambient>0.85 0.85 0.85 1</ambient>',
        '            <diffuse>0.85 0.85 0.85 1</diffuse>',
        '            <specular>0.1 0.1 0.1 1</specular>',
        '          </material>',
        '        </visual>',
        '      </link>',
        '    </model>',
        '',
        '    <!-- Perimeter & Internal Walls (including angled walls) -->',
    ]

    for w in layout.walls:
        z_center = w['height'] * 0.5
        amb = '0.3 0.3 0.3 1' if w['is_perimeter'] else '0.35 0.35 0.35 1'
        dif = '0.4 0.4 0.4 1' if w['is_perimeter'] else '0.45 0.45 0.45 1'
        lines.extend([
            f'    <model name="{w["name"]}">',
            '      <static>true</static>',
            f'      <pose>{w["cx"]} {w["cy"]} {z_center:.3f} 0 0 {w["yaw"]}</pose>',
            '      <link name="link">',
            '        <collision name="collision">',
            f'          <geometry><box><size>{w["length"]} {w["thickness"]} {w["height"]}</size></box></geometry>',
            '        </collision>',
            '        <visual name="visual">',
            f'          <geometry><box><size>{w["length"]} {w["thickness"]} {w["height"]}</size></box></geometry>',
            f'          <material><ambient>{amb}</ambient><diffuse>{dif}</diffuse></material>',
            '        </visual>',
            '      </link>',
            '    </model>',
        ])

    lines.append('')
    lines.append('    <!-- Rectangular Box Obstacles (supports yaw rotation) -->')
    for b in layout.boxes:
        zc = b['size_z'] * 0.5
        lines.extend([
            f'    <model name="{b["name"]}">',
            '      <static>true</static>',
            f'      <pose>{b["cx"]} {b["cy"]} {zc:.3f} 0 0 {b["yaw"]}</pose>',
            '      <link name="link">',
            '        <collision name="collision">',
            f'          <geometry><box><size>{b["size_x"]} {b["size_y"]} {b["size_z"]}</size></box></geometry>',
            '        </collision>',
            '        <visual name="visual">',
            f'          <geometry><box><size>{b["size_x"]} {b["size_y"]} {b["size_z"]}</size></box></geometry>',
            '          <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.55 0.55 0.55 1</diffuse></material>',
            '        </visual>',
            '      </link>',
            '    </model>',
        ])

    lines.append('')
    lines.append('    <!-- Cylindrical Barrel Obstacles -->')
    for cyl in layout.cylinders:
        zc = cyl['height'] * 0.5
        lines.extend([
            f'    <model name="{cyl["name"]}">',
            '      <static>true</static>',
            f'      <pose>{cyl["cx"]} {cyl["cy"]} {zc:.3f} 0 0 0</pose>',
            '      <link name="link">',
            '        <collision name="collision">',
            f'          <geometry><cylinder><radius>{cyl["radius"]}</radius><length>{cyl["height"]}</length></cylinder></geometry>',
            '        </collision>',
            '        <visual name="visual">',
            f'          <geometry><cylinder><radius>{cyl["radius"]}</radius><length>{cyl["height"]}</length></cylinder></geometry>',
            '          <material><ambient>0.75 0.42 0.15 1</ambient><diffuse>0.85 0.48 0.18 1</diffuse></material>',
            '        </visual>',
            '      </link>',
            '    </model>',
        ])

    lines.append('')
    lines.append('    <!-- Freestanding ArUco Target Stands (S1 & C1) -->')
    for t_key, t in layout.targets.items():
        tex_file = f'textures/marker_{t["id"]}.png'
        lines.extend([
            f'    <model name="target_{t_key}">',
            '      <static>true</static>',
            f'      <pose>{t["x"]} {t["y"]} 0 0 0 {t["yaw"]}</pose>',
            '      <link name="link">',
            '        <visual name="base_visual">',
            '          <pose>0 0 0.01 0 0 0</pose>',
            '          <geometry><cylinder><radius>0.12</radius><length>0.02</length></cylinder></geometry>',
            '          <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.3 0.3 0.3 1</diffuse></material>',
            '        </visual>',
            '        <collision name="base_collision">',
            '          <pose>0 0 0.01 0 0 0</pose>',
            '          <geometry><cylinder><radius>0.12</radius><length>0.02</length></cylinder></geometry>',
            '        </collision>',
            '        <visual name="pole_visual">',
            '          <pose>0 0 0.10 0 0 0</pose>',
            '          <geometry><cylinder><radius>0.015</radius><length>0.16</length></cylinder></geometry>',
            '          <material><ambient>0.6 0.6 0.6 1</ambient><diffuse>0.7 0.7 0.7 1</diffuse></material>',
            '        </visual>',
            '        <collision name="pole_collision">',
            '          <pose>0 0 0.10 0 0 0</pose>',
            '          <geometry><cylinder><radius>0.015</radius><length>0.16</length></cylinder></geometry>',
            '        </collision>',
            '        <visual name="marker_visual">',
            '          <pose>0 0 0.28 0 0 0</pose>',
            '          <geometry><box><size>0.01 0.20 0.20</size></box></geometry>',
            '          <material>',
            '            <ambient>1 1 1 1</ambient>',
            '            <diffuse>1 1 1 1</diffuse>',
            f'            <pbr><metal><albedo_map>{tex_file}</albedo_map></metal></pbr>',
            '          </material>',
            '        </visual>',
            '        <collision name="marker_collision">',
            '          <pose>0 0 0.28 0 0 0</pose>',
            '          <geometry><box><size>0.01 0.20 0.20</size></box></geometry>',
            '        </collision>',
            '      </link>',
            '    </model>',
        ])

    lines.extend([
        '',
        '    <light type="directional" name="sun">',
        '      <cast_shadows>true</cast_shadows>',
        f'      <pose>{layout.width * 0.5} {layout.height * 0.5} 10 0 0 0</pose>',
        '      <diffuse>0.8 0.8 0.8 1</diffuse>',
        '      <specular>0.1 0.1 0.1 1</specular>',
        '      <direction>-0.3 0.2 -0.9</direction>',
        '    </light>',
        '',
        '    <scene>',
        '      <ambient>0.5 0.5 0.5 1</ambient>',
        '      <background>0.15 0.15 0.15 1</background>',
        '      <shadows>true</shadows>',
        '    </scene>',
        '',
        '  </world>',
        '</sdf>',
        '',
    ])

    os.makedirs(os.path.dirname(os.path.abspath(sdf_path)), exist_ok=True)
    with open(sdf_path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines))


def export_pgm_and_yaml(grid: List[List[int]], meta: Dict[str, Any], base_path: str):
    """Export binary PGM (P5) and Nav2-compatible YAML file."""
    pgm_path = f'{base_path}.pgm'
    yaml_path = f'{base_path}.yaml'
    h = len(grid)
    w = len(grid[0])

    header = f'P5\n{w} {h}\n255\n'.encode('ascii')
    body = bytearray()
    for row in grid:
        body.extend(row)

    with open(pgm_path, 'wb') as f:
        f.write(header)
        f.write(body)

    ox, oy, oyaw = meta['origin_odom']
    yaml_lines = [
        f'image: {os.path.basename(pgm_path)}',
        'mode: trinary',
        f'resolution: {meta["resolution"]:.4f}',
        f'origin: [{ox:.4f}, {oy:.4f}, {oyaw:.4f}]',
        'negate: 0',
        'occupied_thresh: 0.65',
        'free_thresh: 0.25',
        '',
    ]
    with open(yaml_path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(yaml_lines))


def export_preview_svg(layout: ArenaLayout, svg_path: str):
    """Render an interactive SVG top-down diagram of the generated arena and A* paths."""
    scale = 110.0  # pixels per meter
    pad = 0.55     # meters margin around arena
    total_w = int((layout.width + 2 * pad) * scale)
    total_h = int((layout.height + 2 * pad) * scale)

    def to_px(wx: float, wy: float) -> Tuple[float, float]:
        px = (wx + pad) * scale
        py = (layout.height + pad - wy) * scale
        return px, py

    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{total_w}" height="{total_h}" '
        f'viewBox="0 0 {total_w} {total_h}" style="background:#1e1e24;font-family:monospace;">',
        f'  <text x="18" y="28" fill="#ffffff" font-size="16" font-weight="bold">'
        f'Arena Preview: {layout.name} ({layout.width}m x {layout.height}m)</text>',
    ]

    # Arena floor
    fx, fy = to_px(0.0, layout.height)
    svg.append(
        f'  <rect x="{fx:.1f}" y="{fy:.1f}" width="{layout.width * scale:.1f}" '
        f'height="{layout.height * scale:.1f}" fill="#f4f4f6" stroke="#888" stroke-width="1"/>'
    )

    # 1m grid lines
    for m in range(1, int(layout.width)):
        gx, gy1 = to_px(m, 0.0)
        _, gy2 = to_px(m, layout.height)
        svg.append(
            f'  <line x1="{gx:.1f}" y1="{gy1:.1f}" x2="{gx:.1f}" y2="{gy2:.1f}" '
            f'stroke="#ddd" stroke-dasharray="4,4"/>'
        )
    for m in range(1, int(layout.height)):
        gx1, gy = to_px(0.0, m)
        gx2, _ = to_px(layout.width, m)
        svg.append(
            f'  <line x1="{gx1:.1f}" y1="{gy:.1f}" x2="{gx2:.1f}" y2="{gy:.1f}" '
            f'stroke="#ddd" stroke-dasharray="4,4"/>'
        )

    # Start zone
    sz = layout.cfg['arena']['start_zone']
    sx, sy = to_px(sz[0], sz[3])
    sw = (sz[2] - sz[0]) * scale
    sh = (sz[3] - sz[1]) * scale
    svg.append(
        f'  <rect x="{sx:.1f}" y="{sy:.1f}" width="{sw:.1f}" height="{sh:.1f}" '
        f'fill="#c8f7c5" fill-opacity="0.55" stroke="#2ecc71" stroke-width="2" stroke-dasharray="6,4"/>'
    )
    svg.append(f'  <text x="{sx + 8:.1f}" y="{sy + sh - 10:.1f}" fill="#1e824c" font-size="12">Start Area</text>')

    # Verified A* paths
    path_colors = {
        'robot1_to_S1': '#2980b9',
        'S1_to_C1_slot1': '#8e44ad',
        'robot2_to_C1_slot2': '#16a085',
    }
    for p_name, pts in layout.verified_paths.items():
        if len(pts) >= 2:
            poly = ' '.join(f'{to_px(x, y)[0]:.1f},{to_px(x, y)[1]:.1f}' for x, y in pts)
            col = path_colors.get(p_name, '#2980b9')
            svg.append(
                f'  <polyline points="{poly}" fill="none" stroke="{col}" '
                f'stroke-width="3" stroke-dasharray="5,3" opacity="0.85"/>'
            )

    # Trap gaps (highlighted in warning red/orange)
    for tg in layout.trap_gaps:
        tx1, ty1 = to_px(tg['x1'], tg['y1'])
        tx2, ty2 = to_px(tg['x2'], tg['y2'])
        svg.append(
            f'  <line x1="{tx1:.1f}" y1="{ty1:.1f}" x2="{tx2:.1f}" y2="{ty2:.1f}" '
            f'stroke="#e74c3c" stroke-width="8" stroke-dasharray="4,3"/>'
        )
        mx, my = 0.5 * (tx1 + tx2), 0.5 * (ty1 + ty2)
        svg.append(
            f'  <text x="{mx + 6:.1f}" y="{my - 6:.1f}" fill="#c0392b" font-size="11" font-weight="bold">'
            f'TRAP ({tg["width"]:.2f}m)</text>'
        )

    # Walls
    for w in layout.walls:
        cx, cy = to_px(w['cx'], w['cy'])
        lw = w['length'] * scale
        lt = w['thickness'] * scale
        deg = -math.degrees(w['yaw'])
        svg.append(
            f'  <g transform="translate({cx:.1f},{cy:.1f}) rotate({deg:.2f})">'
            f'<rect x="{-lw * 0.5:.1f}" y="{-lt * 0.5:.1f}" width="{lw:.1f}" height="{lt:.1f}" '
            f'fill="#2c3e50"/></g>'
        )

    # Box obstacles
    for b in layout.boxes:
        cx, cy = to_px(b['cx'], b['cy'])
        bw = b['size_x'] * scale
        bh = b['size_y'] * scale
        deg = -math.degrees(b['yaw'])
        svg.append(
            f'  <g transform="translate({cx:.1f},{cy:.1f}) rotate({deg:.2f})">'
            f'<rect x="{-bw * 0.5:.1f}" y="{-bh * 0.5:.1f}" width="{bw:.1f}" height="{bh:.1f}" '
            f'fill="#7f8c8d" stroke="#34495e" stroke-width="2"/>'
            f'<text x="-18" y="4" fill="#fff" font-size="10">{b["name"].replace("obstacle_", "")}</text></g>'
        )

    # Cylinder barrel obstacles
    for cyl in layout.cylinders:
        cx, cy = to_px(cyl['cx'], cyl['cy'])
        cr = cyl['radius'] * scale
        svg.append(
            f'  <circle cx="{cx:.1f}" cy="{cy:.1f}" r="{cr:.1f}" '
            f'fill="#e67e22" stroke="#d35400" stroke-width="2"/>'
        )
        svg.append(f'  <text x="{cx - 14:.1f}" y="{cy + 4:.1f}" fill="#fff" font-size="9">Barrel</text>')

    # Targets S1 & C1 + Rescue Zones + Docking Slots
    for t_key, t in layout.targets.items():
        tx, ty = to_px(t['x'], t['y'])
        rr = t['rescue_radius'] * scale
        col = '#27ae60' if t['type'] == 'normal' else '#0097e6'
        svg.append(
            f'  <circle cx="{tx:.1f}" cy="{ty:.1f}" r="{rr:.1f}" '
            f'fill="{col}" fill-opacity="0.18" stroke="{col}" stroke-width="2" stroke-dasharray="6,4"/>'
        )
        svg.append(f'  <circle cx="{tx:.1f}" cy="{ty:.1f}" r="{0.12 * scale:.1f}" fill="{col}"/>')
        # Facing arrow
        ax, ay = to_px(t['x'] + 0.32 * math.cos(t['yaw']), t['y'] + 0.32 * math.sin(t['yaw']))
        svg.append(f'  <line x1="{tx:.1f}" y1="{ty:.1f}" x2="{ax:.1f}" y2="{ay:.1f}" stroke="{col}" stroke-width="3"/>')
        svg.append(
            f'  <text x="{tx - 24:.1f}" y="{ty - 16:.1f}" fill="#111" font-size="12" font-weight="bold">'
            f'{t_key} (r={t["rescue_radius"]}m)</text>'
        )
        for s_idx, slot in enumerate(t.get('docking_slots', [])):
            sx, sy = to_px(slot[0], slot[1])
            svg.append(
                f'  <circle cx="{sx:.1f}" cy="{sy:.1f}" r="7" fill="none" stroke="{col}" stroke-width="2"/>'
            )
            svg.append(f'  <text x="{sx + 8:.1f}" y="{sy + 4:.1f}" fill="{col}" font-size="10">Slot{s_idx+1}</text>')

    # Robot 1 & Robot 2 start poses
    for r_label, pose_key, r_col in [('R1', 'spawn_pose', '#2980b9'), ('R2', 'spawn_pose_robot2', '#16a085')]:
        rx, ry, _ = layout.cfg['arena'][pose_key]
        px, py = to_px(rx, ry)
        svg.append(f'  <circle cx="{px:.1f}" cy="{py:.1f}" r="{0.18 * scale:.1f}" fill="{r_col}" fill-opacity="0.8"/>')
        svg.append(f'  <text x="{px - 8:.1f}" y="{py + 4:.1f}" fill="#fff" font-size="11" font-weight="bold">{r_label}</text>')

    svg.append('</svg>')
    with open(svg_path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(svg))


def save_all_artifacts(
    layout: ArenaLayout, out_dir: str, seed: int = 0, install_to_sim: bool = False
) -> Dict[str, str]:
    """Export all 5 artifacts (.sdf, _clean.pgm/.yaml, _noisy.pgm/.yaml, _gt.json, _preview.svg)."""
    os.makedirs(out_dir, exist_ok=True)
    base = os.path.join(out_dir, layout.name)

    # Ensure textures/ exist next to exported .sdf
    ws_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
    src_tex_dir = os.path.join(ws_root, 'src', 'edgeai_perception', 'worlds', 'textures')
    dst_tex_dir = os.path.join(out_dir, 'textures')
    if os.path.isdir(src_tex_dir) and os.path.abspath(src_tex_dir) != os.path.abspath(dst_tex_dir):
        os.makedirs(dst_tex_dir, exist_ok=True)
        for fn in os.listdir(src_tex_dir):
            if fn.endswith('.png'):
                shutil.copy2(os.path.join(src_tex_dir, fn), os.path.join(dst_tex_dir, fn))

    # 1) Export Gazebo .sdf
    sdf_path = f'{base}.sdf'
    export_sdf(layout, sdf_path)

    # 2) Export Clean OccupancyGrid (.pgm + .yaml)
    clean_grid, clean_meta = rasterize_layout(layout, include_target_stands=False)
    export_pgm_and_yaml(clean_grid, clean_meta, f'{base}_clean')

    # 3) Export Noisy OccupancyGrid (.pgm + .yaml)
    noisy_paths = {}
    if layout.cfg['noise_test'].get('export_noisy_map', True):
        noisy_grid, noisy_meta = generate_noisy_grid(clean_grid, clean_meta, layout, seed=seed)
        export_pgm_and_yaml(noisy_grid, noisy_meta, f'{base}_noisy')
        noisy_paths = {
            'noisy_pgm': f'{base}_noisy.pgm',
            'noisy_yaml': f'{base}_noisy.yaml',
        }

    # 4) Export Ground-Truth JSON
    gt_path = f'{base}_gt.json'
    gt_payload = {
        'arena_name': layout.name,
        'seed': seed,
        'dimensions_m': [layout.width, layout.height],
        'start_zone': layout.cfg['arena']['start_zone'],
        'spawn_pose_robot1': layout.cfg['arena']['spawn_pose'],
        'spawn_pose_robot2': layout.cfg['arena']['spawn_pose_robot2'],
        'map_metadata': clean_meta,
        'rooms': layout.rooms,
        'walls': layout.walls,
        'boxes': layout.boxes,
        'cylinders': layout.cylinders,
        'doorways': layout.doorways,
        'trap_gaps': layout.trap_gaps,
        'targets': layout.targets,
        'verified_astar_waypoint_counts': {
            k: len(v) for k, v in layout.verified_paths.items()
        },
        'verified_astar_paths': {
            k: [[p[0], p[1]] for p in v] for k, v in layout.verified_paths.items()
        },
    }
    with open(gt_path, 'w', encoding='utf-8') as f:
        json.dump(gt_payload, f, indent=2)

    # 5) Export SVG Preview
    svg_path = f'{base}_preview.svg'
    export_preview_svg(layout, svg_path)

    # Optional: copy .sdf into edgeai_robot_description/worlds/ for immediate simulation launch
    if install_to_sim:
        sim_worlds_dir = os.path.join(ws_root, 'src', 'edgeai_robot_description', 'worlds')
        if os.path.isdir(sim_worlds_dir):
            shutil.copy2(sdf_path, os.path.join(sim_worlds_dir, f'{layout.name}.sdf'))

    result = {
        'sdf': sdf_path,
        'clean_pgm': f'{base}_clean.pgm',
        'clean_yaml': f'{base}_clean.yaml',
        'gt_json': gt_path,
        'preview_svg': svg_path,
    }
    result.update(noisy_paths)
    return result


# ============================================================================
# CLI Entry Point
# ============================================================================

def main(argv: Optional[List[str]] = None):
    parser = argparse.ArgumentParser(
        description='Generate Gazebo .sdf worlds, Clean/Noisy .pgm maps, and Ground-Truth JSON.'
    )
    parser.add_argument(
        '--config',
        type=str,
        default=None,
        help='Path to generator_config.yaml (defaults to package config/generator_config.yaml)',
    )
    parser.add_argument(
        '--preset',
        choices=['midterm', 'random'],
        default='midterm',
        help='Build exact "midterm" preset or procedural "random" arena',
    )
    parser.add_argument('--seed', type=int, default=42, help='Random seed for procedural generation')
    parser.add_argument(
        '--batch',
        type=int,
        default=1,
        help='Number of maps to generate when --preset random is selected',
    )
    parser.add_argument(
        '--out-dir',
        type=str,
        default='generated_maps',
        help='Output directory for generated worlds, maps, and ground truth',
    )
    parser.add_argument(
        '--name',
        type=str,
        default=None,
        help='Custom base name for generated files',
    )
    parser.add_argument(
        '--install-to-sim',
        action='store_true',
        help='Also copy generated .sdf into src/edgeai_robot_description/worlds/',
    )
    args = parser.parse_args(argv)

    default_cfg_path = os.path.abspath(
        os.path.join(os.path.dirname(__file__), '..', 'config', 'generator_config.yaml')
    )
    cfg_path = args.config or default_cfg_path
    cfg = load_config_file(cfg_path)

    if args.preset == 'midterm':
        layout = build_midterm_preset(cfg)
        if args.name:
            layout.name = args.name
        files = save_all_artifacts(
            layout, args.out_dir, seed=0, install_to_sim=args.install_to_sim
        )
        print(f'[OK] Generated Midterm preset "{layout.name}" in {args.out_dir}/:')
        for k, v in files.items():
            print(f'  - {k:12s}: {v}')
    else:
        for i in range(args.batch):
            cur_seed = args.seed + i
            m_name = args.name if (args.name and args.batch == 1) else f'random_arena_s{cur_seed}'
            layout = build_random_arena(cfg, seed=cur_seed, map_name=m_name)
            files = save_all_artifacts(
                layout, args.out_dir, seed=cur_seed, install_to_sim=args.install_to_sim
            )
            print(
                f'[OK] Generated Random Arena "{layout.name}" (seed={cur_seed}, '
                f'rooms={len(layout.rooms)}, walls={len(layout.walls)}, '
                f'boxes={len(layout.boxes)}, cylinders={len(layout.cylinders)}, '
                f'trap_gaps={len(layout.trap_gaps)}) -> {files["preview_svg"]}'
            )


if __name__ == '__main__':
    main()
