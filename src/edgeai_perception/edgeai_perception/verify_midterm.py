#!/usr/bin/env python3
"""
Automated Arena/Navigation Test Suite & Midterm Check Result Verifier (Module 2)
KTH EdgeAI Robotics Challenge

Capabilities:
1. Automated Map & Navigation Robustness Suite (--test-maps):
   - Tests Clean & Noisy (.pgm) maps against Ground-Truth (_gt.json).
   - Verifies that deceptive "trap gaps" (0.28-0.40m) are automatically blocked
     when proper robot obstacle inflation (>= 0.20m) is applied.
   - Tests morphological despeckling + A* path planning on noisy/drifted maps
     (with boundary jitter, isolated lidar speckles, and unknown gray shadows).
   - Validates Gazebo .sdf XML integrity (angled walls, rotated boxes, cylinders, targets).

2. Referee 2-Minute Result Verification Dashboard (--verify-run):
   - Evaluates Part 1 SLAM map coverage & wall accuracy against Ground Truth.
   - Evaluates Part 1 ArUco S1 (ID 1) & C1 (ID 2) localization error (m).
   - Evaluates Part 2 S1 single-robot 10s continuous hold within 0.50m radius.
   - Evaluates Part 2 C1 two-robot simultaneous 10s hold within 0.70m radius + collision check.
   - Exports a visual summary dashboard SVG (evidence/verification_dashboard.svg).
"""

import argparse
import glob
import json
import math
import os
import sys
import xml.etree.ElementTree as ET
from typing import Any, Dict, List, Optional, Tuple

# Allow direct execution on macOS/Linux without requiring colcon install first
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from edgeai_perception.map_generator import astar_search, inflate_grid  # noqa: E402


# ============================================================================
# PGM Loader & Despeckle Filter (For Noisy Map Robustness Testing)
# ============================================================================

def load_pgm(pgm_path: str) -> List[List[int]]:
    """Read binary P5 .pgm file into a 2D list of ints (0..255)."""
    with open(pgm_path, 'rb') as f:
        data = f.read()

    idx = 0
    tokens: List[bytes] = []
    while len(tokens) < 4 and idx < len(data):
        while idx < len(data) and chr(data[idx]).isspace():
            idx += 1
        if idx < len(data) and chr(data[idx]) == '#':
            while idx < len(data) and chr(data[idx]) != '\n':
                idx += 1
            continue
        start = idx
        while idx < len(data) and not chr(data[idx]).isspace():
            idx += 1
        if start < idx:
            tokens.append(data[start:idx])

    if len(tokens) < 4 or tokens[0] != b'P5':
        raise ValueError(f'Unsupported PGM format in {pgm_path}')

    w = int(tokens[1])
    h = int(tokens[2])
    # Skip single whitespace byte after maxval
    idx += 1
    pixels = data[idx:idx + w * h]

    grid: List[List[int]] = []
    for r in range(h):
        row = list(pixels[r * w:(r + 1) * w])
        grid.append(row)
    return grid


def despeckle_grid(grid: List[List[int]], treat_unknown_as_free: bool = True) -> List[List[int]]:
    """
    Filter isolated 1-pixel LiDAR salt-and-pepper noise from a noisy OccupancyGrid
    before obstacle inflation, while preserving real walls and obstacles.
    """
    h = len(grid)
    w = len(grid[0])
    cleaned = [row[:] for row in grid]

    for r in range(1, h - 1):
        for c in range(1, w - 1):
            val = grid[r][c]
            if val == 0:
                occ_neighbors = 0
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        if dr == 0 and dc == 0:
                            continue
                        if grid[r + dr][c + dc] == 0:
                            occ_neighbors += 1
                if occ_neighbors == 0:
                    cleaned[r][c] = 254
            elif val == 205 and treat_unknown_as_free:
                cleaned[r][c] = 254
    return cleaned


# ============================================================================
# Part A: Automated Map & Navigation Robustness Test Suite
# ============================================================================

def run_map_test_suite(maps_dir: str) -> Tuple[int, int, List[Dict[str, Any]]]:
    """Run automated verification tests on all *_gt.json maps in maps_dir."""
    gt_files = sorted(glob.glob(os.path.join(maps_dir, '*_gt.json')))
    if not gt_files:
        print(f'[WARN] No *_gt.json files found in {maps_dir}. Run map_generator.py first.')
        return 0, 0, []

    passed_count = 0
    total_count = 0
    reports: List[Dict[str, Any]] = []

    print('=' * 78)
    print(f'RUNNING AUTOMATED ARENA & NAVIGATION TEST SUITE ({len(gt_files)} maps in {maps_dir})')
    print('=' * 78)

    for gt_path in gt_files:
        with open(gt_path, 'r', encoding='utf-8') as f:
            gt = json.load(f)

        name = gt['arena_name']
        base = gt_path[:-len('_gt.json')]
        sdf_path = f'{base}.sdf'
        clean_pgm = f'{base}_clean.pgm'
        noisy_pgm = f'{base}_noisy.pgm'
        meta = gt['map_metadata']
        res = float(meta['resolution'])

        checks: List[Tuple[str, bool, str]] = []

        # 1) SDF XML validity & model check
        sdf_ok = False
        sdf_detail = ''
        if os.path.isfile(sdf_path):
            try:
                tree = ET.parse(sdf_path)
                root = tree.getroot()
                models = root.findall('.//model')
                model_names = {m.attrib.get('name', '') for m in models}
                has_targets = 'target_S1' in model_names and 'target_C1' in model_names
                sdf_ok = has_targets and len(models) >= 7
                sdf_detail = (
                    f'{len(models)} SDF models (walls={len(gt["walls"])}, '
                    f'boxes={len(gt["boxes"])}, barrels={len(gt["cylinders"])})'
                )
            except Exception as ex:
                sdf_detail = f'XML error: {ex}'
        else:
            sdf_detail = 'Missing .sdf file'
        checks.append(('SDF 3D World Integrity', sdf_ok, sdf_detail))

        # 2) Clean Map Inflated A* Traversability
        clean_grid = load_pgm(clean_pgm)
        inf_clean = inflate_grid(clean_grid, 0.22, res)

        r1 = tuple(gt['spawn_pose_robot1'][:2])
        r2 = tuple(gt['spawn_pose_robot2'][:2])
        s1_slot = tuple(gt['targets']['S1']['docking_slots'][0][:2])
        c1_slot1 = tuple(gt['targets']['C1']['docking_slots'][0][:2])
        c1_slot2 = tuple(gt['targets']['C1']['docking_slots'][1][:2])

        p1 = astar_search(inf_clean, meta, r1, s1_slot)
        p2 = astar_search(inf_clean, meta, s1_slot, c1_slot1)
        p3 = astar_search(inf_clean, meta, r2, c1_slot2)
        clean_nav_ok = bool(p1 and p2 and p3)
        checks.append((
            'Clean Map A* Multi-Robot Paths',
            clean_nav_ok,
            f'R1->S1 ({len(p1 or [])} pts), S1->C1_1 ({len(p2 or [])} pts), R2->C1_2 ({len(p3 or [])} pts)',
        ))

        # 3) C1 Cooperative Docking Slot Geometry Check
        c1_x, c1_y = gt['targets']['C1']['x'], gt['targets']['C1']['y']
        c1_rad = float(gt['targets']['C1']['rescue_radius'])
        d_s1 = math.hypot(c1_slot1[0] - c1_x, c1_slot1[1] - c1_y)
        d_s2 = math.hypot(c1_slot2[0] - c1_x, c1_slot2[1] - c1_y)
        inter_d = math.hypot(c1_slot1[0] - c1_slot2[0], c1_slot1[1] - c1_slot2[1])
        coop_geom_ok = (d_s1 <= c1_rad) and (d_s2 <= c1_rad) and (inter_d >= 0.48)
        checks.append((
            'C1 Dual-Robot Docking Geometry',
            coop_geom_ok,
            f'd1={d_s1:.2f}m, d2={d_s2:.2f}m (<= {c1_rad}m), robot_sep={inter_d:.2f}m (>= 0.48m)',
        ))

        # 4) Trap Gap Rejection Check (Deceptive narrow passages blocked by inflation)
        trap_gaps = gt.get('trap_gaps', [])
        if trap_gaps:
            all_traps_blocked = True
            trap_notes = []
            for tg in trap_gaps:
                mx = 0.5 * (tg['x1'] + tg['x2'])
                my = 0.5 * (tg['y1'] + tg['y2'])
                cc = int((mx - meta['arena_min_x']) / res)
                rr = int((meta['arena_max_y'] - my) / res)
                raw_free = (clean_grid[rr][cc] == 254)
                inf_blocked = (inf_clean[rr][cc] == 0)
                if not inf_blocked:
                    all_traps_blocked = False
                trap_notes.append(
                    f'{tg["name"]}({tg["width"]:.2f}m: raw_free={raw_free}, inflated_blocked={inf_blocked})'
                )
            checks.append(('Trap Gap Inflation Rejection', all_traps_blocked, '; '.join(trap_notes)))

        # 5) Noisy Map Despeckle + A* Robustness Check
        if os.path.isfile(noisy_pgm):
            noisy_grid = load_pgm(noisy_pgm)
            filtered_grid = despeckle_grid(noisy_grid, treat_unknown_as_free=True)
            inf_noisy = inflate_grid(filtered_grid, 0.20, res)
            np1 = astar_search(inf_noisy, meta, r1, s1_slot)
            np3 = astar_search(inf_noisy, meta, r2, c1_slot2)
            noisy_ok = bool(np1 and np3)
            checks.append((
                'Noisy Map (Jitter+Speckles) A* Robustness',
                noisy_ok,
                f'Despeckled A* R1->S1 ({len(np1 or [])} pts), R2->C1_2 ({len(np3 or [])} pts)',
            ))

        map_passed = all(ok for _, ok, _ in checks)
        print(f'\nMap: {name}  -->  {"[PASS]" if map_passed else "[FAIL]"}')
        for title, ok, detail in checks:
            total_count += 1
            if ok:
                passed_count += 1
            status = 'PASS' if ok else 'FAIL'
            print(f'  [{status}] {title:42s} | {detail}')

        reports.append({
            'arena_name': name,
            'passed': map_passed,
            'checks': [{'name': t, 'passed': ok, 'detail': d} for t, ok, d in checks],
        })

    print('\n' + '=' * 78)
    print(f'TEST SUITE SUMMARY: {passed_count}/{total_count} checks passed across {len(gt_files)} maps.')
    print('=' * 78)
    return passed_count, total_count, reports


# ============================================================================
# Part B: Referee 2-Minute Result Verification & Mock Evidence Generator
# ============================================================================

def generate_mock_evidence(evidence_dir: str, maps_dir: str, gt_path: str):
    """
    Generate realistic sample Part 1 & Part 2 evidence files on macOS so the
    verification pipeline and dashboard can be tested before running Gazebo on Linux.
    """
    os.makedirs(evidence_dir, exist_ok=True)
    os.makedirs(maps_dir, exist_ok=True)

    with open(gt_path, 'r', encoding='utf-8') as f:
        gt = json.load(f)

    src_pgm = gt_path.replace('_gt.json', '_noisy.pgm')
    if not os.path.isfile(src_pgm):
        src_pgm = gt_path.replace('_gt.json', '_clean.pgm')
    dst_pgm = os.path.join(maps_dir, 'midterm_slam_map.pgm')
    if os.path.isfile(src_pgm) and not os.path.isfile(dst_pgm):
        with open(src_pgm, 'rb') as fsrc, open(dst_pgm, 'wb') as fdst:
            fdst.write(fsrc.read())

    s1_gt = gt['targets']['S1']
    c1_gt = gt['targets']['C1']

    part1_summary = {
        'S1': {
            'id': 1,
            'name': 'S1',
            'type': 'normal',
            'frame_id': 'map',
            'odom_map_pose': {
                'x': round(s1_gt['x'] - 0.7 + 0.032, 3),
                'y': round(s1_gt['y'] - 0.5 - 0.025, 3),
                'yaw': s1_gt['yaw'],
            },
            'arena_pose': {
                'x': round(s1_gt['x'] + 0.032, 3),
                'y': round(s1_gt['y'] - 0.025, 3),
                'yaw': s1_gt['yaw'],
            },
            'relative_to_robot_base': {'forward_m': 0.84, 'left_m': 0.05, 'distance_m': 0.842},
            'first_detected_sim_time_sec': 94.2,
            'last_detected_sim_time_sec': 112.6,
            'observations': 86,
        },
        'C1': {
            'id': 2,
            'name': 'C1',
            'type': 'coop',
            'frame_id': 'map',
            'odom_map_pose': {
                'x': round(c1_gt['x'] - 0.7 - 0.028, 3),
                'y': round(c1_gt['y'] - 0.5 + 0.035, 3),
                'yaw': c1_gt['yaw'],
            },
            'arena_pose': {
                'x': round(c1_gt['x'] - 0.028, 3),
                'y': round(c1_gt['y'] + 0.035, 3),
                'yaw': c1_gt['yaw'],
            },
            'relative_to_robot_base': {'forward_m': 0.91, 'left_m': -0.03, 'distance_m': 0.911},
            'first_detected_sim_time_sec': 168.5,
            'last_detected_sim_time_sec': 185.0,
            'observations': 92,
        },
    }
    with open(os.path.join(evidence_dir, 'part1_detection_summary.json'), 'w', encoding='utf-8') as f:
        json.dump(part1_summary, f, indent=2)

    s1_verify = {
        'target': 'S1',
        'target_id': 1,
        'target_type': 'normal',
        'target_arena_center': [s1_gt['x'], s1_gt['y']],
        'rescue_radius_m': 0.50,
        'required_hold_duration_sec': 10.0,
        'achieved_continuous_hold_sec': 10.05,
        'hold_start_sim_time_sec': 42.10,
        'hold_end_sim_time_sec': 52.15,
        'min_distance_m': 0.348,
        'max_distance_m': 0.362,
        'mean_distance_m': 0.354,
        'within_radius_entire_hold': True,
        'collision_count': 0,
        'status': 'COMPLETED',
    }
    with open(os.path.join(evidence_dir, 'S1_rescue_verification.json'), 'w', encoding='utf-8') as f:
        json.dump(s1_verify, f, indent=2)

    c1_verify = {
        'target': 'C1',
        'target_id': 2,
        'target_type': 'coop',
        'target_arena_center': [c1_gt['x'], c1_gt['y']],
        'rescue_radius_m': 0.70,
        'required_hold_duration_sec': 10.0,
        'achieved_simultaneous_hold_sec': 10.10,
        'hold_start_sim_time_sec': 88.40,
        'hold_end_sim_time_sec': 98.50,
        'robot1_mean_distance_m': 0.438,
        'robot1_max_distance_m': 0.452,
        'robot2_mean_distance_m': 0.445,
        'robot2_max_distance_m': 0.461,
        'min_inter_robot_distance_m': 0.528,
        'collision_count': 0,
        'status': 'COMPLETED',
    }
    with open(os.path.join(evidence_dir, 'C1_coop_verification.json'), 'w', encoding='utf-8') as f:
        json.dump(c1_verify, f, indent=2)


def verify_competition_run(
    evidence_dir: str, maps_dir: str, gt_path: str
) -> Dict[str, Any]:
    """
    Evaluate Part 1 and Part 2 outputs against Midterm Check Section 6 requirements,
    print a referee verification table, and export evidence/verification_dashboard.svg.
    """
    with open(gt_path, 'r', encoding='utf-8') as f:
        gt = json.load(f)

    items: List[Dict[str, Any]] = []

    # 1) Part 1 SLAM Map Coverage & Recognizable Features
    slam_pgm = os.path.join(maps_dir, 'midterm_slam_map.pgm')
    if not os.path.isfile(slam_pgm):
        alt_pgm = os.path.join('src', 'edgeai_robot_description', 'maps', 'midterm_arena.pgm')
        if os.path.isfile(alt_pgm):
            slam_pgm = alt_pgm

    if os.path.isfile(slam_pgm):
        s_grid = load_pgm(slam_pgm)
        known_cells = sum(1 for row in s_grid for v in row if v in (0, 254))
        wall_cells = sum(1 for row in s_grid for v in row if v == 0)
        res = float(gt['map_metadata']['resolution'])
        known_area_m2 = known_cells * (res * res)
        coverage_pct = min(100.0, (known_area_m2 / 25.0) * 100.0)
        map_ok = coverage_pct >= 80.0 and wall_cells >= 150
        items.append({
            'id': 'Part 1.A',
            'title': 'Autonomous SLAM Occupancy Map',
            'passed': map_ok,
            'metric': f'Mapped Area: {known_area_m2:.1f} m^2 ({coverage_pct:.1f}%), Wall cells: {wall_cells}',
            'file': slam_pgm,
        })
    else:
        items.append({
            'id': 'Part 1.A',
            'title': 'Autonomous SLAM Occupancy Map',
            'passed': False,
            'metric': 'Missing maps/midterm_slam_map.pgm',
            'file': slam_pgm,
        })

    # 2) Part 1 ArUco Target Detection (S1 & C1)
    det_json = os.path.join(evidence_dir, 'part1_detection_summary.json')
    if os.path.isfile(det_json):
        with open(det_json, 'r', encoding='utf-8') as f:
            det = json.load(f)
        for t_key in ('S1', 'C1'):
            gt_t = gt['targets'][t_key]
            if t_key in det:
                d_entry = det[t_key]
                ap = d_entry.get('arena_pose', {})
                err_m = math.hypot(
                    float(ap.get('x', 999)) - gt_t['x'],
                    float(ap.get('y', 999)) - gt_t['y'],
                )
                rel = d_entry.get('relative_to_robot_base', {})
                ok = (
                    int(d_entry.get('id', -1)) == int(gt_t['id'])
                    and d_entry.get('type') == gt_t['type']
                    and err_m <= 0.25
                )
                items.append({
                    'id': f'Part 1.{t_key}',
                    'title': f'ArUco Target {t_key} (ID {gt_t["id"]}, {gt_t["type"]})',
                    'passed': ok,
                    'metric': (
                        f'Est=({ap.get("x"):.2f}, {ap.get("y"):.2f}) vs GT=({gt_t["x"]:.2f}, {gt_t["y"]:.2f}), '
                        f'Err={err_m:.3f}m, RelDist={rel.get("distance_m", 0):.2f}m, Obs={d_entry.get("observations")}'
                    ),
                    'file': det_json,
                })
            else:
                items.append({
                    'id': f'Part 1.{t_key}',
                    'title': f'ArUco Target {t_key} Detection',
                    'passed': False,
                    'metric': f'{t_key} not found in detection summary',
                    'file': det_json,
                })
    else:
        items.append({
            'id': 'Part 1.B',
            'title': 'ArUco S1 & C1 Detection Summary',
            'passed': False,
            'metric': 'Missing evidence/part1_detection_summary.json',
            'file': det_json,
        })

    # 3) Part 2 Single Rescue (S1 - 10s continuous within 0.50m)
    s1_json = os.path.join(evidence_dir, 'S1_rescue_verification.json')
    if os.path.isfile(s1_json):
        with open(s1_json, 'r', encoding='utf-8') as f:
            s1_rec = json.load(f)
        hold_s = float(s1_rec.get('achieved_continuous_hold_sec', 0.0))
        max_d = float(s1_rec.get('max_distance_m', 99.0))
        s1_ok = (
            s1_rec.get('status') == 'COMPLETED'
            and hold_s >= 10.0
            and max_d <= 0.50
        )
        items.append({
            'id': 'Part 2.S1',
            'title': 'Single Target S1 Rescue (10s <= 0.50m)',
            'passed': s1_ok,
            'metric': f'Hold={hold_s:.2f}s (>=10.0s), MaxDist={max_d:.3f}m (<=0.50m), Status={s1_rec.get("status")}',
            'file': s1_json,
        })
    else:
        items.append({
            'id': 'Part 2.S1',
            'title': 'Single Target S1 Rescue (10s <= 0.50m)',
            'passed': False,
            'metric': 'Missing evidence/S1_rescue_verification.json',
            'file': s1_json,
        })

    # 4) Part 2 Cooperative Rescue (C1 - 2 Robots Simultaneous 10s <= 0.70m)
    c1_json = os.path.join(evidence_dir, 'C1_coop_verification.json')
    if os.path.isfile(c1_json):
        with open(c1_json, 'r', encoding='utf-8') as f:
            c1_rec = json.load(f)
        hold_s = float(c1_rec.get('achieved_simultaneous_hold_sec', 0.0))
        r1_max = float(c1_rec.get('robot1_max_distance_m', 99.0))
        r2_max = float(c1_rec.get('robot2_max_distance_m', 99.0))
        sep_min = float(c1_rec.get('min_inter_robot_distance_m', 0.0))
        c1_ok = (
            c1_rec.get('status') == 'COMPLETED'
            and hold_s >= 10.0
            and r1_max <= 0.70
            and r2_max <= 0.70
            and sep_min >= 0.36
        )
        items.append({
            'id': 'Part 2.C1',
            'title': 'Coop Target C1 Rescue (2 Robots 10s <= 0.70m)',
            'passed': c1_ok,
            'metric': (
                f'SimulHold={hold_s:.2f}s, R1_max={r1_max:.2f}m, R2_max={r2_max:.2f}m (<=0.70m), '
                f'MinRobotSep={sep_min:.2f}m (Collision-Free)'
            ),
            'file': c1_json,
        })
    else:
        items.append({
            'id': 'Part 2.C1',
            'title': 'Coop Target C1 Rescue (2 Robots 10s <= 0.70m)',
            'passed': False,
            'metric': 'Missing evidence/C1_coop_verification.json',
            'file': c1_json,
        })

    all_passed = all(it['passed'] for it in items)

    print('\n' + '=' * 86)
    print('KTH EDGEAI CHALLENGE — MIDTERM CHECK REFEREE VERIFICATION DASHBOARD')
    print('=' * 86)
    for it in items:
        badge = 'PASS' if it['passed'] else 'FAIL'
        print(f'[{badge}] {it["id"]:10s} | {it["title"]:42s}')
        print(f'                  -> {it["metric"]}')
    print('-' * 86)
    print(f'OVERALL VERIFICATION STATUS: {"ALL ITEMS PASSED" if all_passed else "INCOMPLETE / CHECK ITEMS ABOVE"}')
    print('=' * 86)

    os.makedirs(evidence_dir, exist_ok=True)
    dash_svg = os.path.join(evidence_dir, 'verification_dashboard.svg')
    _export_dashboard_svg(items, all_passed, dash_svg)

    report = {'overall_passed': all_passed, 'items': items, 'dashboard_svg': dash_svg}
    with open(os.path.join(evidence_dir, 'verification_report.json'), 'w', encoding='utf-8') as f:
        json.dump(report, f, indent=2)
    return report


def _export_dashboard_svg(items: List[Dict[str, Any]], all_passed: bool, svg_path: str):
    w = 900
    h = 130 + len(items) * 78
    header_col = '#27ae60' if all_passed else '#e67e22'
    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" '
        f'viewBox="0 0 {w} {h}" style="background:#16161d;font-family:monospace;">',
        f'  <rect x="20" y="20" width="{w - 40}" height="64" rx="8" fill="{header_col}"/>',
        '  <text x="40" y="58" fill="#ffffff" font-size="20" font-weight="bold">'
        'KTH EdgeAI Robotics Challenge — Midterm Verification Report</text>',
    ]
    for idx, it in enumerate(items):
        y = 104 + idx * 74
        col = '#2ecc71' if it['passed'] else '#e74c3c'
        status = 'PASS' if it['passed'] else 'FAIL'
        lines.extend([
            f'  <rect x="20" y="{y}" width="{w - 40}" height="62" rx="6" fill="#23232e" stroke="{col}" stroke-width="2"/>',
            f'  <rect x="34" y="{y + 15}" width="68" height="32" rx="4" fill="{col}"/>',
            f'  <text x="48" y="{y + 36}" fill="#fff" font-size="15" font-weight="bold">{status}</text>',
            f'  <text x="118" y="{y + 26}" fill="#f1f2f6" font-size="15" font-weight="bold">'
            f'{it["id"]}: {it["title"]}</text>',
            f'  <text x="118" y="{y + 48}" fill="#a4b0be" font-size="12">{it["metric"]}</text>',
        ])
    lines.append('</svg>')
    with open(svg_path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines))


def main(argv: Optional[List[str]] = None):
    parser = argparse.ArgumentParser(
        description='Run automated map/navigation robustness tests and Midterm referee verification.'
    )
    parser.add_argument(
        '--mode',
        choices=['all', 'test-maps', 'verify-run'],
        default='all',
        help='Which verification suite to run',
    )
    parser.add_argument('--maps-dir', type=str, default='generated_maps', help='Directory of generated test maps')
    parser.add_argument('--evidence-dir', type=str, default='evidence', help='Directory of runtime evidence logs')
    parser.add_argument(
        '--gt-file',
        type=str,
        default='generated_maps/midterm_arena_gt.json',
        help='Ground-truth JSON file for referee verification',
    )
    parser.add_argument(
        '--generate-mock-evidence',
        action='store_true',
        help='Generate realistic mock Part 1 & Part 2 evidence logs to test verification on macOS',
    )
    args = parser.parse_args(argv)

    if args.mode in ('all', 'test-maps'):
        run_map_test_suite(args.maps_dir)

    if args.mode in ('all', 'verify-run'):
        if args.generate_mock_evidence and os.path.isfile(args.gt_file):
            generate_mock_evidence(args.evidence_dir, 'maps', args.gt_file)
        if os.path.isfile(args.gt_file):
            verify_competition_run(args.evidence_dir, 'maps', args.gt_file)


if __name__ == '__main__':
    main()
