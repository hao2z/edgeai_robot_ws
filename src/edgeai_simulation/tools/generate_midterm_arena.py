#!/usr/bin/env python3
"""Build the midterm Figure 1 arena in its original 0..5 metre coordinates."""
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


def box(world, name, x, y, z, sx, sy, sz, color='0.16 0.16 0.18 1'):
    model = ET.SubElement(world, 'model', name=name)
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = f'{x} {y} {z} 0 0 0'
    link = ET.SubElement(model, 'link', name='body')
    for tag in ('collision', 'visual'):
        obj = ET.SubElement(link, tag, name=tag)
        ET.SubElement(ET.SubElement(ET.SubElement(obj, 'geometry'), 'box'), 'size').text = f'{sx} {sy} {sz}'
        if tag == 'visual':
            material = ET.SubElement(obj, 'material')
            for key in ('ambient', 'diffuse'):
                ET.SubElement(material, key).text = color
    return model


def generate(share):
    tree = ET.parse(share / 'worlds/empty.sdf')
    world = tree.getroot().find('world')
    world.set('name', 'midterm')
    for model in list(world.findall('model')):
        if model.get('name') != 'ground_plane':
            world.remove(model)
    # Boundary reference lines are x/y=0 and 5; wall centrelines follow them.
    # Wall height is a team choice (0.50 m); plan dimensions follow Section 3.
    walls = [('south_wall', 2.5, 0, 5.1, .1),
             ('north_wall', 2.5, 5, 5.1, .1),
             ('west_wall', 0, 2.5, .1, 5),
             ('east_wall', 5, 2.5, .1, 5),
             ('W1', 1.1, 3.1, 2.2, .1),
             ('W1b', 2.2, 3.35, .1, .5),
             ('W2', 3.4, 1.4, .1, 2.8)]
    for name, x, y, sx, sy in walls:
        box(world, name, x, y, .25, sx, sy, .5)
    box(world, 'O2', 2.2, 2.0, .25, .6, 1.0, .5, '0.45 0.45 0.45 1')
    box(world, 'O3', 4.4, 3.6, .25, .8, .6, .5, '0.45 0.45 0.45 1')
    config = yaml.safe_load((share / 'config/targets_midterm.yaml').read_text())
    for target in config['targets']:
        x, y, height, *_ = target['pose']
        name = target['name']
        # Separate ground-level base/pole with real collision geometry. The
        # Marker centre height comes from the target config; the pole meets its lower edge.
        box(world, name + '_stand_base', x, y, .0125, .11, .11, .025, '0.3 0.3 0.32 1')
        pole_top = height - config['panel_size_m'] / 2
        box(world, name + '_stand_pole', x, y, pole_top/2, .02, .02, pole_top, '0.3 0.3 0.32 1')
    # Start area is visual only; it cannot block lidar or robot motion.
    area = box(world, 'starting_area', .7, .5, .001, 1.4, 1, .002, '0.35 0.65 0.9 1')
    link = area.find('link')
    link.remove(link.find('collision'))
    gui = ET.SubElement(world, 'gui', fullscreen='false')
    scene = ET.SubElement(gui, 'plugin', filename='MinimalScene', name='3D View')
    for tag, value in [('engine', 'ogre2'), ('scene', 'scene'),
                       ('ambient_light', '0.5 0.5 0.5'),
                       ('background_color', '0.8 0.8 0.8'),
                       ('camera_pose', '2.5 -3.8 7 0 0.85 1.57079632679')]:
        ET.SubElement(scene, tag).text = value
    for filename in ('GzSceneManager', 'InteractiveViewControl', 'EntityContextMenuPlugin',
                     'EntityTree', 'ComponentInspector'):
        ET.SubElement(gui, 'plugin', filename=filename, name=filename)
    control = ET.SubElement(gui, 'plugin', filename='WorldControl', name='World control')
    ET.SubElement(control, 'start_paused').text = 'false'
    ET.indent(tree, space='  ')
    output = share / 'worlds/midterm.sdf'
    tree.write(output, encoding='utf-8', xml_declaration=True)
    print(output)


if __name__ == '__main__':
    generate(Path(__file__).resolve().parents[1])
