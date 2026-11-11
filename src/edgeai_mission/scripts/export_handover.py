#!/usr/bin/env python3
"""Freeze Task 1 outputs with map metadata and unique localized target rows."""
import csv
import json
from pathlib import Path
import sys
import yaml


def export_preview(map_yaml):
    """Write a lossless PNG alongside the ROS map, without changing its YAML."""
    from PIL import Image
    map_yaml = Path(map_yaml)
    metadata = yaml.safe_load(map_yaml.read_text())
    preview = map_yaml.with_suffix('.png')
    with Image.open(map_yaml.parent/metadata['image']) as image:
        image.save(preview, format='PNG')
    return preview


def export(directory, map_yaml):
    directory, map_yaml = Path(directory), Path(map_yaml)
    source = directory/'targets.json'
    registry = json.loads(source.read_text()) if source.exists() else dict(frame_id='map', targets=[])
    frozen = directory/'handover-targets.json'
    frozen.write_text(json.dumps(registry, indent=2, ensure_ascii=False))
    with (directory/'target-list.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['id', 'type', 'x', 'y', 'yaw'])
        writer.writeheader()
        for row in sorted(registry['targets'], key=lambda r: r['id']):
            if row.get('yaw') is not None:
                writer.writerow({key: row[key] for key in writer.fieldnames})
    metadata = yaml.safe_load(map_yaml.read_text())
    result = dict(schema_version=1, frame_id='map', map_yaml=str(map_yaml.resolve()),
                  resolution=metadata['resolution'], origin=metadata['origin'],
                  image=str((map_yaml.parent/metadata['image']).resolve()),
                  target_list='target-list.csv', target_details='handover-targets.json',
                  target_stamp_ns=registry.get('stamp_ns'),
                  note='Development ID/type mapping and geometry; not final official parameters')
    (directory/'task1-output.json').write_text(json.dumps(result, indent=2))
    # A convenience image must not turn successful map handover into failure.
    try:
        preview = export_preview(map_yaml)
        print(f'地图预览已保存：{preview}', flush=True)
    except (OSError, ValueError, ImportError) as error:
        print(f'地图预览生成失败（原始地图已保留）：{error}', file=sys.stderr, flush=True)


if __name__ == '__main__':
    export(*sys.argv[1:])
