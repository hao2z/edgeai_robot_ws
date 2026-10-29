"""Add native Nav2 overlays and the explicitly time-aligned diagnostic legend."""
from pathlib import Path
import sys
import yaml


def topic(value, durable=False):
    return {'Value': value, 'Depth': 1, 'Reliability Policy': 'Reliable',
            'Durability Policy': 'Transient Local' if durable else 'Volatile'}


def configure(path):
    path = Path(path)
    config = yaml.safe_load(path.read_text())
    # Limit saved multi-monitor window geometry; Qt/WSLg can otherwise restore
    # a window larger than the Windows working area and cover its taskbar.
    geometry = config.setdefault('Window Geometry', {})
    geometry.update(Width=1280, Height=840, X=20, Y=20)
    manager = config['Visualization Manager']
    config['Panels'] = [p for p in config['Panels'] if p.get('Class') not in (
        'edgeai_navigation_rviz/NavigationPanel', 'rviz_common/Views')]
    config['Panels'].append({'Class':'edgeai_navigation_rviz/NavigationPanel', 'Name':'导航图例与时间状态'})
    for panel in config['Panels']:
        if panel.get('Class') == 'rviz_common/Displays':
            panel['Property Tree Widget']['Expanded'] = ['/Status1']
    manager['Global Options']['Frame Rate'] = 60
    displays = manager['Displays']
    # Re-running preparation is idempotent.
    names = {'Global navigation costs', 'Local live obstacle costs', 'Global path (green)',
             'Local trajectory (pink)', 'Aligned footprints and timing legend',
             'Raw global footprint (message time)', 'Raw local footprint (message time)', 'Navigation TF'}
    displays[:] = [d for d in displays if d.get('Name') not in names and d.get('Name') != 'Robot footprint']
    # RViz Image displays can reopen their dock during initialization even when
    # Enabled is false. Keep depth/point cloud as manually added diagnostic views.
    displays[:] = [d for d in displays if d.get('Class') != 'rviz_default_plugins/PointCloud2'
                   and not (d.get('Class') == 'rviz_default_plugins/Image'
                            and 'depth' in d.get('Topic', {}).get('Value', ''))]
    for display in displays:
        if display.get('Class') == 'rviz_default_plugins/MarkerArray' and 'Marker Topic' in display:
            display['Topic'] = display.pop('Marker Topic')
        if display.get('Class') == 'rviz_default_plugins/Map':
            display['Name'] = 'SLAM map (occupancy)'
            display['Alpha'] = .65
        if display.get('Class') == 'rviz_default_plugins/PointCloud2':
            display['Enabled'] = False
        if display.get('Class') == 'rviz_default_plugins/Image':
            depth = 'depth' in display.get('Topic', {}).get('Value', '')
            display.update(Name='Depth camera (optional)' if depth else 'RGB camera',
                           Enabled=not depth, Value=not depth)
    for name, value, alpha in (
            ('Global navigation costs','/global_costmap/costmap',.20),
            ('Local live obstacle costs','/local_costmap/costmap',.40)):
        enabled = value.startswith('/local_')
        displays.append({'Class':'rviz_default_plugins/Map', 'Name':name, 'Enabled':enabled,
                         'Value':enabled, 'Alpha':alpha, 'Color Scheme':'costmap',
                         'Draw Behind':False, 'Use Timestamp':True, 'Topic':topic(value, True)})
    for name, value, color in (
            ('Global path (green)','/plan','80; 255; 80'),
            ('Local trajectory (pink)','/local_plan','255; 80; 170')):
        displays.append({'Class':'rviz_default_plugins/Path', 'Name':name, 'Enabled':True,
                         'Value':True, 'Color':color, 'Alpha':1.0, 'Line Style':'Lines',
                         'Line Width':.025, 'Buffer Length':1, 'Topic':topic(value)})
    for name, value, color in (
            ('Raw global footprint (message time)','/global_costmap/published_footprint','0; 255; 255'),
            ('Raw local footprint (message time)','/local_costmap/published_footprint','255; 220; 0')):
        displays.append({'Class':'rviz_default_plugins/Polygon', 'Name':name, 'Enabled':False,
                         'Value':False, 'Alpha':1.0, 'Color':color, 'Topic':topic(value)})
    displays.append({'Class':'rviz_default_plugins/MarkerArray', 'Enabled':True,
                     'Name':'Aligned footprints and timing legend', 'Value':True,
                     'Topic':topic('/diagnostics/navigation')})
    displays.append({'Class':'rviz_default_plugins/MarkerArray', 'Enabled':True,
                     'Name':'ArUco targets (ID + position)', 'Value':True,
                     'Topic':topic('/targets/markers')})
    displays.append({'Class':'rviz_default_plugins/TF', 'Enabled':False, 'Name':'Navigation TF',
                     'Show Names':True, 'Marker Scale':.2,
                     'Frames':{'All Enabled':False, 'map':{'Value':True},
                               'odom':{'Value':True}, 'base_footprint':{'Value':True}}})
    # Leave room above the arena for the meaning/age legend.
    manager['Views']['Current'].update(Distance=12.0)
    path.write_text(yaml.safe_dump(config, sort_keys=False), encoding='utf-8')


if __name__ == '__main__':
    configure(sys.argv[1])
