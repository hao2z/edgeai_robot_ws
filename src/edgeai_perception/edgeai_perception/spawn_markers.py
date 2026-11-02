#!/usr/bin/env python3
import os
import sys
import subprocess


def spawn_marker(name: str, model_path: str, x: float, y: float, z: float, yaw: float, world_name: str = 'midterm_arena'):
    cmd = [
        'ros2', 'run', 'ros_gz_sim', 'create',
        '-world', world_name,
        '-file', model_path,
        '-name', name,
        '-x', str(x),
        '-y', str(y),
        '-z', str(z),
        '-Y', str(yaw)
    ]
    print(f"Spawning {name} at ({x}, {y}, {z}) yaw={yaw} in world '{world_name}'...")
    subprocess.run(cmd)


def main():
    world_name = sys.argv[1] if len(sys.argv) > 1 else 'midterm_arena'
    models_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'models'))

    # Official Midterm Specification (Section 3 & Figure 1):
    # S1 (Single Rescue Target): Center (1.0, 4.1), ID 1, facing South (-Y, yaw = -pi/2)
    # C1 (Cooperative Rescue Target): Center (4.1, 1.0), ID 2, facing North (+Y, yaw = +pi/2)
    markers = [
        ('target_S1', os.path.join(models_dir, 'aruco_marker_1', 'model.sdf'), 1.0, 4.1, 0.0, -1.570796),
        ('target_C1', os.path.join(models_dir, 'aruco_marker_2', 'model.sdf'), 4.1, 1.0, 0.0, 1.570796),
    ]

    for name, sdf_path, x, y, z, yaw in markers:
        spawn_marker(name, sdf_path, x, y, z, yaw, world_name)


if __name__ == '__main__':
    main()
