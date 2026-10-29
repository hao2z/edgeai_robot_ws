# EdgeAI Perception (`edgeai_perception`)

ROS 2 Jazzy Python package for ArUco target detection, 3D pose estimation, and persistent target mapping for the KTH EdgeAI Robotics Challenge (Workstream 2).

---

## Capabilities

1. **ArUco Detection & PnP 3D Pose Estimation (`aruco_detector`)**:
   - Subscribes to `/camera/image` and `/camera/camera_info`.
   - Uses OpenCV 4.12 (`DICT_4X4_50`) to detect marker IDs and pixel corners.
   - Solves Perspective-n-Point (PnP) using camera intrinsics to compute 3D translation $(x, y, z)$ and rotation vectors in `camera_optical_frame`.
   - Uses `tf2_ros` to transform marker coordinates into the global `map` (or `odom`) coordinate frame.

2. **Persistent Target Memory & De-duplication**:
   - Maintains a running average of $(x, y, z)$ and heading angle `yaw` for every discovered target.
   - Target IDs are mapped to target types (`normal` vs `coop`).
   - Targets remain stored in memory even after the robot turns away or drives to another room.

3. **Autonomous Closed-Loop Sweep (`square_patrol`)**:
   - Autonomous patrol node that moves 1.0 m forward and turns left 90° in a closed-loop using `/odom`.
   - Completes 2 full loops (8 sides) and automatically halts.

4. **Simulation Assets**:
   - `generate_markers.py`: Generates ArUco marker textures with white borders and builds Gazebo Harmonic 3D SDF models.
   - `spawn_markers.py`: Spawns test targets into the running Gazebo arena.

---

## Published Interfaces

### 1. `/detected_targets` (`std_msgs/msg/String`)
JSON-formatted array of all discovered targets, published at 2 Hz:

```json
[
  {
    "id": 0,
    "type": "normal",
    "x": 1.983,
    "y": 0.096,
    "yaw": -1.577,
    "frame_id": "odom",
    "observations": 4369
  },
  {
    "id": 2,
    "type": "coop",
    "x": -0.937,
    "y": 0.853,
    "yaw": 1.652,
    "frame_id": "odom",
    "observations": 745
  }
]
```

*Consumed by: Workstream 4 (Coordinator) and Workstream 1 (Navigation).*

### 2. `/target_markers` (`visualization_msgs/msg/MarkerArray`)
3D visual representations for RViz:
- **Cylinders**: Green for Normal Target, Cyan for Cooperative Target.
- **Arrows**: Pointing along `yaw` (the direction the marker face points).
- **Text Labels**: Floating label displaying `ID: <id> (<TYPE>)`.

---

## Running the Perception Pipeline

From the workspace root:

```bash
# 1. Build workspace
colcon build --packages-select edgeai_perception
source install/setup.bash

# 2. Launch ArUco Detector & Target Map
ros2 run edgeai_perception aruco_detector

# 3. (Optional) Run the 2-circle square patrol
ros2 run edgeai_perception square_patrol

# 4. Echo detected targets
ros2 topic echo /detected_targets
```

