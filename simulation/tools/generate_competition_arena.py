"""Development arena inspired by kickoff slide 4, not an official arena replica."""
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

root = Path(__file__).resolve().parents[2]
share = root / 'src/edgeai_robot_description'
tree = ET.parse(share / 'worlds/empty.sdf')
world = tree.getroot().find('world')
for model in list(world.findall('model')):
    if model.get('name', '').startswith('obstacle_'):
        world.remove(model)

# x, y, length, width: 0.72-0.82 m door openings for a ~0.5 m swept footprint.
boxes = [
    ('upper_left_partition', -1.0, 1.95, .10, 1.10),
    ('upper_right_partition', 1.05, 1.90, .10, 1.20),
    ('left_room_bottom', -1.75, .50, 1.50, .10),
    ('right_room_bottom', 1.80, .50, 1.40, .10),
    ('lower_right_top', 1.80, -.10, 1.40, .10),
    ('lower_right_side', 1.15, -1.70, .10, 1.60),
    ('left_room_box', -2.05, .95, .40, .35),
    ('right_room_box', 2.05, .95, .40, .35),
    ('central_box', -.30, .00, .70, .25),
    ('lower_left_box', -1.90, -.70, .40, .40),
    ('lower_right_box', 1.85, -2.00, .50, .35),
]
for name, x, y, sx, sy in boxes:
    m = ET.SubElement(world, 'model', name=name)
    ET.SubElement(m, 'static').text = 'true'
    ET.SubElement(m, 'pose').text = f'{x} {y} .25 0 0 0'
    link = ET.SubElement(m, 'link', name='body')
    for tag in ('collision', 'visual'):
        obj = ET.SubElement(link, tag, name=tag)
        box = ET.SubElement(ET.SubElement(obj, 'geometry'), 'box')
        ET.SubElement(box, 'size').text = f'{sx} {sy} .5'
ET.indent(tree, space='  ')
tree.write(share / 'worlds/competition_rooms.sdf', encoding='utf-8', xml_declaration=True)
targets = yaml.safe_load((share / 'config/targets.yaml').read_text())
poses = [[-1.75, 2.42, .24, 0, 0, -1.57079632679],
         [1.75, 2.42, .24, 0, 0, -1.57079632679],
         [2.42, -1.15, .24, 0, 0, 3.14159265359],
         [0, 2.42, .24, 0, 0, -1.57079632679]]
for target, pose in zip(targets['targets'], poses):
    target['pose'] = pose
(share / 'config/targets_competition.yaml').write_text(
    '# Development fixtures inspired by kickoff slide 4; not official target locations.\n' +
    yaml.safe_dump(targets, sort_keys=False), encoding='utf-8')
print('Created competition_rooms.sdf and targets_competition.yaml')
