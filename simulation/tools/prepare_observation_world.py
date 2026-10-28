"""Reuse installed Gazebo GUI plugins with an arena overview camera."""
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

source, output, gui_config = map(Path, sys.argv[1:])
tree = ET.parse(source)
world = tree.getroot().find('world')
old_gui = world.find('gui')
if old_gui is not None:
    world.remove(old_gui)
gui = ET.SubElement(world, 'gui', fullscreen='false')
# The official GUI config contains several top-level elements.
text = gui_config.read_text().replace('<?xml version="1.0"?>', '')
defaults = ET.fromstring('<root>' + text + '</root>')
for plugin in defaults.findall('plugin'):
    if plugin.get('filename') in ('ComponentInspector', 'EntityTree'):
        continue
    if plugin.get('filename') == 'MinimalScene':
        plugin.find('camera_pose').text = '3.8 -4.35 6 0 0.80 2.288'
    gui.append(plugin)
ET.indent(tree, space='  ')
tree.write(output, encoding='utf-8', xml_declaration=True)
