"""Prepare stage configuration only; no ROS calls or task dispatch."""
import argparse
import json
import math
import os
import shutil
from pathlib import Path
import yaml


def mission_targets(targets_file, cooperative, selection='', count=0):
    arena = yaml.safe_load(Path(targets_file).read_text())['targets']
    known = {row['id']: row['type'] for row in arena}
    if selection:
        pairs = [item.strip().split(':') for item in selection.split(',')]
        if any(len(pair) != 2 or not pair[0].isdigit() for pair in pairs):
            raise ValueError('Use comma-separated ID:type pairs (e.g. 1:normal,10:cooperative)')
        targets = [dict(id=int(marker), type=kind) for marker, kind in pairs]
    else:
        targets = [dict(id=marker, type=kind) for marker, kind in known.items()
                   if kind == 'normal' or cooperative]
    if len({t['id'] for t in targets}) != len(targets):
        raise ValueError('Target IDs must be unique')
    for target in targets:
        marker, kind = target['id'], target['type']
        if kind not in ('normal', 'cooperative') or known.get(marker) != kind:
            raise ValueError(f'Target {marker}:{kind} does not match {Path(targets_file).name}')
    if count and count != len(targets):
        raise ValueError(f'Expected {count} targets, got {len(targets)}')
    if not any(t['type'] == 'normal' for t in targets):
        raise ValueError('The integrated rescue mission requires a normal target')
    cooperative_count = sum(t['type'] == 'cooperative' for t in targets)
    if cooperative and cooperative_count != 1:
        raise ValueError('The current C1 task requires exactly one cooperative target')
    return targets


def prepare(root, directory, rescue, map_basename, duration, cooperative, backend, rescue_enabled=True,
            part='integrated', robots_file=None, *, targets_file, target_selection='', target_count=0):
    if part not in ('integrated','part1'):raise ValueError('Unsupported mapping mission part')
    root, directory = Path(root).resolve(), Path(directory).resolve()
    nav = yaml.safe_load((directory/'nav2-runtime.yaml').read_text())
    radii = []
    for name in ('local_costmap', 'global_costmap'):
        geometry = nav[name][name]['ros__parameters']
        pad = geometry.get('footprint_padding', 0.)
        radii.append(max(math.hypot(x+math.copysign(pad,x), y+math.copysign(pad,y))
                         for x,y in json.loads(geometry['footprint']))
                     if 'footprint' in geometry else geometry['robot_radius']+pad)
    required_targets = mission_targets(targets_file, cooperative, target_selection, target_count)
    arena = yaml.safe_load(Path(targets_file).read_text())
    rescue_rules = arena.get('rescue_rules', {})
    c = yaml.safe_load(Path(rescue).read_text())
    if 'normal' in rescue_rules:
        # The arena chooses the distance reference. A template may have a
        # marker-centred radius that does not apply to this arena.
        c.pop('target_radius_m', None)
        c.update(rescue_rules['normal'])
    if c.get('goal_source') == 'detected_target':
        c.pop('target_id', None)
        c['allowed_target_ids'] = [target['id'] for target in required_targets if target['type'] == 'normal']
    c['search_clearance_m'] = max(c.get('search_clearance_m', .4), max(radii)+.05)
    (directory/'rescue-config.yaml').write_text(yaml.safe_dump(c, sort_keys=False))
    coop = yaml.safe_load((root/'src/edgeai_navigation/config/cooperative_rescue.yaml').read_text())
    coop.update(rescue_rules.get('cooperative', {}))
    cooperative_target = next((t for t in required_targets if t['type'] == 'cooperative'), None)
    if cooperative_target is not None:
        coop['target_id'] = cooperative_target['id']
    if cooperative:
        for key in ('radius_m', 'dwell_s', 'yaw_tolerance_rad'):
            if not math.isfinite(coop[key]) or coop[key] <= 0:
                raise ValueError(f'Invalid cooperative rescue {key}')
        if not any(0 < r <= coop['radius_m'] - coop['position_margin_m'] + 1e-6
                   for r in coop['parking_radii_m']):
            raise ValueError('No cooperative parking radius fits the rescue radius and margin')
    (directory/'cooperative-rescue-config.yaml').write_text(yaml.safe_dump(coop, sort_keys=False))
    # The adapters currently supply robot1 and robot2; membership is independent
    # of the number of mapping sources and can be consumed without mode branches.
    import sys
    sys.path.insert(0,str(root/'src/edgeai_mission'))
    from edgeai_tasks.mission_fleet import fleet_layout,validate_fleet
    joint_mapping = os.environ.get('EDGEAI_JOINT_MAPPING') == 'true'
    relative_pose = coop['initial_relative_pose']
    if (joint_mapping or cooperative) and robots_file:
        from edgeai_nav.geometry import relative
        robots = yaml.safe_load(Path(robots_file).read_text())['robots'][:2]
        if [robot['name'] for robot in robots] != ['robot1', 'robot2']:
            raise ValueError('A two-robot mission requires robot1 and robot2 in the spawn configuration')
        poses = [robot['pose'] for robot in robots]
        if any(len(pose) != 6 or not all(math.isfinite(v) for v in pose) for pose in poses):
            raise ValueError('Robot spawn poses must contain six finite values')
        relative_pose = relative([*poses[0][:2], poses[0][5]], [*poses[1][:2], poses[1][5]])
    fleet=fleet_layout(joint_mapping,cooperative,relative_pose)
    validate_fleet(fleet)
    trees=directory/'mission-trees';trees.mkdir(exist_ok=True)
    source=root/'src/edgeai_mission/config'
    mapping=yaml.safe_load((source/'mapping.yaml').read_text())
    localization=yaml.safe_load((source/'localization.yaml').read_text())
    if (not isinstance(localization['startup_attempts'],int) or localization['startup_attempts']<1
            or not math.isfinite(localization['startup_timeout_s']) or localization['startup_timeout_s']<=0):
        raise ValueError('Invalid localization startup settings')
    if (not math.isfinite(mapping['min_known_area_m2']) or mapping['min_known_area_m2'] <= 0
            or mapping['supplement_attempts'] < 0 or mapping['reposition_timeout_s'] <= 0):
        raise ValueError('Invalid mapping completion settings')
    for name in ('explore','find_targets','s1','c1'):shutil.copyfile(source/'tasks'/f'{name}.xml',trees/f'{name}.xml')
    assignment=yaml.safe_load((source/'exploration.yaml').read_text()) or {}
    shutil.copyfile(source/'integrated.xml',trees/'integrated.xml')
    if part=='part1':
        shutil.copyfile(source/'midterm_part1.xml',trees/'midterm_part1.xml')
    settings = dict(project_root=str(root), directory=str(directory), map_basename=str(Path(map_basename).resolve()),
                    joint_mapping=os.environ.get('EDGEAI_JOINT_MAPPING')=='true',
                    fleet=fleet,
                    post_mapping_localization=part=='integrated',
                    localization_worker_script=str(root/'src/edgeai_mission/scripts/mission_localization.py'),
                    localization=localization,
                    initial_pose_seed=os.environ.get('EDGEAI_JOINT_MAPPING')!='true',
                    exploration_timeout_s=float(duration), rescue_enabled=rescue_enabled and bool(c.get('enabled')),
                    cooperative=cooperative, backend=backend,
                    mapping=mapping,
                    exploration_assignment=assignment,
                    search_assignment=yaml.safe_load((source/'search.yaml').read_text()) or {},
                    single_rescue_assignment=yaml.safe_load((source/'single_rescue_assignment.yaml').read_text()) or {},
                    initial_relative_pose=relative_pose,
                    # Preserve existing stage budgets in this migration. A competition-wide timer is still pending.
                    wall_timeout_s=float(duration)+2*float(c['wall_timeout_s'])+(float(coop['wall_timeout_s']) if cooperative else 0.)
                        +(localization['startup_attempts']*localization['startup_timeout_s'] if part=='integrated' else 0.),
                    task_wall_timeout_s=dict(explore=float(duration),find_targets=float(c['wall_timeout_s']),s1=float(c['wall_timeout_s']),c1=float(coop['wall_timeout_s'])),
                    task_attempts=dict(explore=1,s1=int(c.get('rescue_max_retries',2))+1,c1=int(coop.get('rescue_repair_rounds',3))+1),
                    task_navigation_attempts=dict(explore=1,s1=int(c.get('navigation_max_retries',2))+1,c1=int(coop.get('navigation_retries',2))+1),
                    worker_script=str(root/'src/edgeai_mission/scripts/mission_stage.py'),
                    unit_worker_script=str(root/'src/edgeai_mission/scripts/task_units.py'),
                    target_count=len(required_targets), required_targets=required_targets,
                    target_observation_require_confirmation=c.get('require_confirmation',False),
                    task_trees={task:str(trees/f'{task}.xml') for task in ('explore','find_targets','s1','c1')},
                    tree=str(trees/('midterm_part1.xml' if part=='part1' else 'integrated.xml')),
                    demonstration_part=part)
    (directory/'mission-runtime.json').write_text(json.dumps(settings, indent=2))
    return settings


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('root','directory','rescue','map','backend'):p.add_argument('--'+name,required=True)
    p.add_argument('--duration',type=float,required=True)
    p.add_argument('--cooperative',choices=('true','false'),required=True)
    p.add_argument('--rescue-enabled',choices=('true','false'),default='true')
    p.add_argument('--part',choices=('integrated','part1'),default='integrated')
    p.add_argument('--robots-file', type=Path)
    p.add_argument('--targets-file', type=Path, required=True)
    p.add_argument('--targets', default=os.environ.get('EDGEAI_TARGETS', ''))
    p.add_argument('--target-count', type=int, default=os.environ.get('EDGEAI_TARGET_COUNT', '0'))
    a=p.parse_args()
    prepare(a.root,a.directory,a.rescue,a.map,a.duration,a.cooperative=='true',a.backend,a.rescue_enabled=='true',
            a.part,a.robots_file,targets_file=a.targets_file,target_selection=a.targets,target_count=a.target_count)
