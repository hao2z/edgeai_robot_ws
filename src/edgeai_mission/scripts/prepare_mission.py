"""Prepare stage configuration only; no ROS calls or task dispatch."""
import argparse
import json
import math
import shutil
from pathlib import Path
import yaml


def prepare(root, directory, rescue, map_basename, duration, cooperative, observation, backend, rescue_enabled=True):
    root, directory = Path(root).resolve(), Path(directory).resolve()
    nav = yaml.safe_load((directory/'nav2-runtime.yaml').read_text())
    radii = []
    for name in ('local_costmap', 'global_costmap'):
        geometry = nav[name][name]['ros__parameters']
        pad = geometry.get('footprint_padding', 0.)
        radii.append(max(math.hypot(x+math.copysign(pad,x), y+math.copysign(pad,y))
                         for x,y in json.loads(geometry['footprint']))
                     if 'footprint' in geometry else geometry['robot_radius']+pad)
    c = yaml.safe_load(Path(rescue).read_text())
    c['search_clearance_m'] = max(c.get('search_clearance_m', .4), max(radii)+.05)
    (directory/'rescue-config.yaml').write_text(yaml.safe_dump(c, sort_keys=False))
    scan = yaml.safe_load((root/'src/edgeai_perception/config/observation_scan.yaml').read_text())
    scan['observation_scan']['ros__parameters'].update(rotation_radius_m=max(radii), output_directory=str(directory))
    (directory/'observation-scan-runtime.yaml').write_text(yaml.safe_dump(scan, sort_keys=False))
    coop = yaml.safe_load((root/'src/edgeai_navigation/config/cooperative_rescue.yaml').read_text())
    trees=directory/'mission-trees';trees.mkdir(exist_ok=True)
    source=root/'src/edgeai_mission/config'
    for name in ('find_targets','s1','c1'):shutil.copyfile(source/'tasks'/f'{name}.xml',trees/f'{name}.xml')
    shutil.copyfile(source/'integrated.xml',trees/'integrated.xml')
    settings = dict(project_root=str(root), directory=str(directory), map_basename=str(Path(map_basename).resolve()),
                    exploration_timeout_s=float(duration), rescue_enabled=rescue_enabled and bool(c.get('enabled')),
                    cooperative=cooperative, observation=observation, backend=backend,
                    initial_relative_pose=coop['initial_relative_pose'],
                    # Preserve existing stage budgets in this migration. A competition-wide timer is still pending.
                    wall_timeout_s=float(duration)+2*float(c['wall_timeout_s'])+(float(coop['wall_timeout_s']) if cooperative else 0.),
                    task_wall_timeout_s=dict(find_targets=float(c['wall_timeout_s']),s1=float(c['wall_timeout_s']),c1=float(coop['wall_timeout_s'])),
                    task_attempts=dict(s1=int(c.get('rescue_max_retries',2))+1,c1=int(coop.get('rescue_repair_rounds',3))+1),
                    task_navigation_attempts=dict(s1=int(c.get('navigation_max_retries',2))+1,c1=int(coop.get('navigation_retries',2))+1),
                    worker_script=str(root/'src/edgeai_mission/scripts/mission_stage.py'),
                    unit_worker_script=str(root/'src/edgeai_mission/scripts/task_units.py'),
                    required_targets=[dict(id=c['target_id'],type='normal')]+([dict(id=coop['target_id'],type='cooperative')] if cooperative else []),
                    task_trees={task:str(trees/f'{task}.xml') for task in ('find_targets','s1','c1')},
                    tree=str(trees/'integrated.xml'))
    (directory/'mission-runtime.json').write_text(json.dumps(settings, indent=2))
    return settings


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('root','directory','rescue','map','backend'):p.add_argument('--'+name,required=True)
    p.add_argument('--duration',type=float,required=True)
    p.add_argument('--cooperative',choices=('true','false'),required=True)
    p.add_argument('--observation',choices=('true','false'),required=True)
    p.add_argument('--rescue-enabled',choices=('true','false'),default='true')
    a=p.parse_args()
    prepare(a.root,a.directory,a.rescue,a.map,a.duration,a.cooperative=='true',a.observation=='true',a.backend,a.rescue_enabled=='true')
