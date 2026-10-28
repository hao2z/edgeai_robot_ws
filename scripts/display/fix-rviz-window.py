import ctypes
import re
import subprocess
from pathlib import Path

path = Path(__file__).resolve().parents[2] / 'src/edgeai_robot_description/rviz/realistic_robot.rviz'
text = path.read_text()
backup = (Path(__file__).resolve().parents[2] / 'tests/results/backups/realistic_robot-original.rviz')
if not backup.exists():
    backup.write_text(text)
before, geometry = text.split('Window Geometry:\n', 1)
for key, value in [('X', 40), ('Y', 40), ('Width', 1280), ('Height', 720)]:
    geometry = re.sub(rf'^  {key}: .*$', f'  {key}: {value}', geometry, flags=re.M)
path.write_text(before + 'Window Geometry:\n' + geometry)
tree = subprocess.check_output(['xwininfo', '-root', '-tree'], text=True)
line = next(line for line in tree.splitlines() if ' - RViz"' in line)
window_id = int(line.strip().split()[0], 16)
x11 = ctypes.CDLL('libX11.so.6')
x11.XOpenDisplay.restype = ctypes.c_void_p
x11.XMoveResizeWindow.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_int, ctypes.c_uint, ctypes.c_uint]
x11.XRaiseWindow.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
x11.XFlush.argtypes = [ctypes.c_void_p]
x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
display = x11.XOpenDisplay(None)
assert display
x11.XMoveResizeWindow(display, window_id, 40, 40, 1280, 720)
x11.XRaiseWindow(display, window_id)
x11.XFlush(display)
x11.XCloseDisplay(display)
print('Moved RViz onto the visible desktop; saved geometry for future launches.')
