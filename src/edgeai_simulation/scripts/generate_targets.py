#!/usr/bin/env python3
"""Generate actual ArUco previews and renderer-independent binary panel models."""
import argparse
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import yaml


def generate(package_dir, config_path):
    spec = importlib.util.spec_from_file_location('support', package_dir / 'launch/simulation_support.py')
    support = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(support)
    config = support.load_yaml(config_path)
    support.validate_targets(config)
    dictionary_name = config['dictionary']
    if not hasattr(cv2.aruco, dictionary_name):
        raise ValueError('Unknown OpenCV ArUco dictionary: ' + dictionary_name)
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
    # Six modules for DICT_4X4 plus black border; enough pixels for exact texture.
    panel_px = 720
    marker_px = round(panel_px * config['marker_size_m'] / config['panel_size_m'])
    if (panel_px - marker_px) % 2:
        raise ValueError('Choose sizes giving an even pixel margin')
    margin = (panel_px - marker_px) // 2
    manifest = {'dictionary': dictionary_name, 'marker_size_m': config['marker_size_m'],
                'panel_size_m': config['panel_size_m'], 'ids': []}
    for target in config['targets']:
        marker_id = target['id']
        if not isinstance(marker_id, int) or not 0 <= marker_id < len(dictionary.bytesList):
            raise ValueError(f'Marker ID outside dictionary: {marker_id}')
        directory = package_dir / 'models' / f'aruco_{marker_id}'
        directory.mkdir(parents=True, exist_ok=True)
        marker = cv2.aruco.generateImageMarker(dictionary, marker_id, marker_px)
        panel = np.full((panel_px, panel_px), 255, dtype=np.uint8)
        panel[margin:margin + marker_px, margin:margin + marker_px] = marker
        if not cv2.imwrite(str(directory / 'marker.png'), panel):
            raise RuntimeError('Could not write marker texture')
        size = config['panel_size_m']
        # Render binary modules as flat geometry. This avoids renderer-specific
        # PBR/mesh UV issues while preserving the exact generated dictionary bits.
        cells = cv2.aruco.generateImageMarker(dictionary, marker_id, dictionary.markerSize + 2)
        cell_size = config['marker_size_m'] / cells.shape[0]
        visuals = []
        for row in range(cells.shape[0]):
            for col in range(cells.shape[1]):
                if cells[row, col] != 0:
                    continue
                y = (col + 0.5 - cells.shape[1] / 2) * cell_size
                z = (cells.shape[0] / 2 - row - 0.5) * cell_size
                visuals.append(f'<visual name="cell_{row}_{col}"><pose>0 {y} {z} 0 0 0</pose>'
                    f'<cast_shadows>false</cast_shadows><geometry><box><size>0.001 {cell_size} {cell_size}</size></box></geometry>'
                    '<material><ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse><specular>0 0 0 1</specular></material></visual>')
        model = f'''<?xml version="1.0"?>
<sdf version="1.9"><model name="aruco_{marker_id}">
  <static>true</static><link name="panel">
    <collision name="backing"><pose>-0.006 0 0 0 0 0</pose><geometry><box><size>0.01 {size} {size}</size></box></geometry></collision>
    <visual name="white_panel"><pose>-0.006 0 0 0 0 0</pose><cast_shadows>false</cast_shadows>
      <geometry><box><size>0.01 {size} {size}</size></box></geometry>
      <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material>
    </visual>
    {''.join(visuals)}
  </link>
</model></sdf>'''
        (directory / 'model.sdf').write_text(model)
        (directory / 'model.config').write_text(f'<?xml version="1.0"?><model><name>aruco_{marker_id}</name><version>1.0</version><sdf version="1.9">model.sdf</sdf></model>')
        # Round-trip validation rejects mirrored, invalid or incorrectly encoded assets.
        corners, ids, _ = cv2.aruco.ArucoDetector(dictionary).detectMarkers(panel)
        if ids is None or ids.flatten().tolist() != [marker_id]:
            raise RuntimeError(f'Generated marker {marker_id} did not decode')
        ET.fromstring(model)
        manifest['ids'].append(marker_id)
    (package_dir / 'models' / 'manifest.yaml').write_text(yaml.safe_dump(manifest))
    print(yaml.safe_dump(manifest), end='')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package-dir', type=Path)
    parser.add_argument('--config', type=Path)
    args = parser.parse_args()
    if args.package_dir:
        package = args.package_dir.resolve()
    else:
        from ament_index_python.packages import get_package_share_directory
        package = Path(get_package_share_directory('edgeai_robot_description'))
    generate(package, args.config or package / 'config/targets.yaml')
