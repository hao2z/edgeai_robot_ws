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
    manager = config['Visualization Manager']
    config['Panels'] = [p for p in config['Panels'] if p.get('Class') not in (
        'edgeai_navigation_rviz/NavigationPanel', 'rviz_common/Views')]
    config['Panels'].append({'Class':'edgeai_navigation_rviz/NavigationPanel', 'Name':'导航图例与时间状态'})
    manager['Global Options']['Frame Rate'] = 60
    displays = manager['Displays']
    # Re-running preparation is idempotent.
    names = {'Global navigation costs', 'Local live obstacle costs', 'Global path (green)',
             'Local trajectory (pink)', 'Aligned footprints and timing legend',
             'Raw global footprint (message time)', 'Raw local footprint (message time)', 'Navigation TF'}
    displays[:] = [d for d in displays if d.get('Name') not in names and d.get('Name') != 'Robot footprint']
    for display in displays:
        if display.get('Class') == 'rviz_default_plugins/MarkerArray' and 'Marker Topic' in display:
            display['Topic'] = display.pop('Marker Topic')
        if display.get('Class') == 'rviz_default_plugins/Map':
            display['Name'] = 'SLAM map (occupancy)'
            display['Alpha'] = .65
        if display.get('Class') == 'rviz_default_plugins/PointCloud2':
            display['Enabled'] = False
    for name, value, alpha in (
            ('Global navigation costs','/global_costmap/costmap',.20),
            ('Local live obstacle costs','/local_costmap/costmap',.40)):
        displays.append({'Class':'rviz_default_plugins/Map', 'Name':name, 'Enabled':True,
                         'Value':True, 'Alpha':alpha, 'Color Scheme':'costmap',
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
    displays.append({'Class':'rviz_default_plugins/TF', 'Enabled':False, 'Name':'Navigation TF',
                     'Show Names':True, 'Marker Scale':.2,
                     'Frames':{'All Enabled':False, 'map':{'Value':True},
                               'odom':{'Value':True}, 'base_footprint':{'Value':True}}})
    # Leave room above the arena for the meaning/age legend.
    manager['Views']['Current'].update(Distance=10.0)
    path.write_text(yaml.safe_dump(config, sort_keys=False), encoding='utf-8')


if __name__ == '__main__':
    configure(sys.argv[1])
