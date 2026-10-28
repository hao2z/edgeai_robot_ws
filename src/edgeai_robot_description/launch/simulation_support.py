"""Shared simulation configuration; no target truth is published on ROS topics."""
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import yaml


def load_yaml(path):
    with open(path, encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def validate_targets(config):
    size = config['marker_size_m']
    panel = config['panel_size_m']
    if not 0 < size < panel:
        raise ValueError('marker_size_m must be positive and smaller than panel_size_m')
    ids, names = set(), set()
    for target in config['targets']:
        if target['id'] in ids or target['name'] in names:
            raise ValueError('Duplicate target ID or name')
        ids.add(target['id'])
        names.add(target['name'])
        if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', target['name']):
            raise ValueError('Invalid target name')
        if target['type'] not in ('normal', 'cooperative'):
            raise ValueError('Unknown target type')
        if len(target['pose']) != 6 or not all(math.isfinite(v) for v in target['pose']):
            raise ValueError('Target pose must contain six finite values')
        if abs(target['pose'][3]) > 1e-9 or abs(target['pose'][4]) > 1e-9:
            raise ValueError('Development rescue geometry supports upright markers only')


def rescue_zones(target, config):
    x, y, _, _, _, yaw = target['pose']
    geometry = config['development_rescue_geometry']
    offsets = [0.0] if target['type'] == 'normal' else [
        -geometry['cooperative_lateral_offset_m'], geometry['cooperative_lateral_offset_m']]
    return [[x + geometry['normal_offset_m'] * math.cos(yaw) - offset * math.sin(yaw),
             y + geometry['normal_offset_m'] * math.sin(yaw) + offset * math.cos(yaw)]
            for offset in offsets]


def make_world(package_share, targets_path, output_path, arena_path=None):
    """Compose the unchanged arena and packaged textured target models."""
    share = Path(package_share)
    config = load_yaml(targets_path)
    validate_targets(config)
    manifest = load_yaml(share / 'models/manifest.yaml')
    if any(manifest[key] != config[key] for key in ('dictionary', 'marker_size_m', 'panel_size_m')):
        raise ValueError('Dictionary or marker size changed: regenerate target assets')
    tree = ET.parse(arena_path or share / 'worlds' / 'empty.sdf')
    world = tree.getroot().find('world')
    for target in config['targets']:
        model_path = share / 'models' / ('aruco_' + str(target['id'])) / 'model.sdf'
        if not model_path.is_file():
            raise ValueError(f'Missing marker asset {model_path}; run generate_targets.py')
        include = ET.SubElement(world, 'include')
        ET.SubElement(include, 'uri').text = model_path.as_uri()
        ET.SubElement(include, 'name').text = target['name']
        ET.SubElement(include, 'pose').text = ' '.join(map(str, target['pose']))
        # Visual-only rescue disks do not affect lidar, collision, or autonomy.
        for index, (x, y) in enumerate(rescue_zones(target, config)):
            zone = ET.SubElement(world, 'model', name=f"zone_{target['name']}_{index}")
            ET.SubElement(zone, 'static').text = 'true'
            ET.SubElement(zone, 'pose').text = f'{x} {y} 0.003 0 0 0'
            link = ET.SubElement(zone, 'link', name='zone')
            visual = ET.SubElement(link, 'visual', name='development_zone')
            cylinder = ET.SubElement(ET.SubElement(visual, 'geometry'), 'cylinder')
            ET.SubElement(cylinder, 'radius').text = str(config['development_rescue_geometry']['radius_m'])
            ET.SubElement(cylinder, 'length').text = '0.002'
            material = ET.SubElement(visual, 'material')
            color = '0.1 0.7 0.2 0.5' if target['type'] == 'normal' else '0.1 0.3 0.9 0.5'
            ET.SubElement(material, 'ambient').text = color
            ET.SubElement(material, 'diffuse').text = color
            ET.SubElement(visual, 'transparency').text = '0.5'
    ET.indent(tree, space='  ')
    tree.write(output_path, encoding='utf-8', xml_declaration=True)
    return config


def prepare_robot(xml, namespace, imu_config):
    """Prefix URDF frames AND Gazebo topics; ROS namespaces alone are insufficient."""
    root = ET.fromstring(xml)
    prefix = namespace + '/' if namespace else ''
    topic_prefix = '/' + namespace if namespace else ''
    root.set('name', namespace or 'edgeai_robot_real')
    for link in root.findall('link'):
        link.set('name', prefix + link.get('name'))
    for joint in root.findall('joint'):
        joint.set('name', prefix + joint.get('name'))
        for tag in ('parent', 'child'):
            element = joint.find(tag)
            element.set('link', prefix + element.get('link'))
    for gazebo in root.findall('gazebo'):
        if gazebo.get('reference'):
            gazebo.set('reference', prefix + gazebo.get('reference'))
    for tag in ('left_joint', 'right_joint', 'frame_id', 'child_frame_id', 'gz_frame_id', 'optical_frame_id'):
        for element in root.iter(tag):
            element.text = prefix + element.text
    for tag in ('topic', 'odom_topic', 'tf_topic'):
        for element in root.iter(tag):
            element.text = topic_prefix + '/' + element.text.lstrip('/')
    # Put optical_frame_id inside <camera>, as required by gz-sensors.
    for sensor in root.iter('sensor'):
        if sensor.get('type') == 'rgbd_camera':
            optical = sensor.find('optical_frame_id')
            if optical is not None:
                sensor.remove(optical)
                sensor.find('camera').append(optical)
        if sensor.get('type') == 'imu':
            sensor.find('update_rate').text = str(imu_config['update_rate'])
            imu = sensor.find('imu')
            imu.find('enable_orientation').text = str(imu_config['enable_orientation']).lower()
            for kind in ('angular_velocity', 'linear_acceleration'):
                for axis in ('x', 'y', 'z'):
                    noise = ET.SubElement(ET.SubElement(imu.find(kind), axis), 'noise', type='gaussian')
                    ET.SubElement(noise, 'mean').text = '0.0'
                    for key, value in imu_config[kind].items():
                        ET.SubElement(noise, key).text = str(value)
    return ET.tostring(root, encoding='unicode')
