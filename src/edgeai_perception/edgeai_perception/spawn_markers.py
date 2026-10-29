#!/usr/bin/env python3
import os
import subprocess


def spawn_marker(name: str, model_path: str, x: float, y: float, z: float, yaw: float):
    cmd = [
        'ros2', 'run', 'ros_gz_sim', 'create',
        '-world', 'empty',
        '-file', model_path,
        '-name', name,
        '-x', str(x),
        '-y', str(y),
        '-z', str(z),
        '-Y', str(yaw)
    ]
    print(f"Spawning {name} at ({x}, {y}, {z}) yaw={yaw}...")
    subprocess.run(cmd)


def main():
    models_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'models'))

    # Place 3 markers around the outside of the 1m x 1m square patrol loop
    # Patrol corners: (-1.5, -1.5) -> (-0.5, -1.5) -> (-0.5, -0.5) -> (-1.5, -0.5)
    markers = [
        # Side 1 (+X): Marker 0 ahead of (-0.5, -1.5), facing -X (yaw = pi)
        ('target_0', os.path.join(models_dir, 'aruco_marker_0', 'model.sdf'), 0.1, -1.5, 0.25, 3.14159),
        # Side 2 (+Y): Marker 1 ahead of (-0.5, -0.5), facing -Y (yaw = -pi/2)
        ('target_1', os.path.join(models_dir, 'aruco_marker_1', 'model.sdf'), -0.5, 0.1, 0.25, -1.5708),
        # Side 3 (-X): Marker 2 ahead of (-1.5, -0.5), facing +X (yaw = 0)
        ('target_2', os.path.join(models_dir, 'aruco_marker_2', 'model.sdf'), -2.1, -0.5, 0.25, 0.0),
    ]

    for name, sdf_path, x, y, z, yaw in markers:
        spawn_marker(name, sdf_path, x, y, z, yaw)


if __name__ == '__main__':
    main()
