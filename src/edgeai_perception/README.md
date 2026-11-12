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

4. **Simulation Assets & Parametric Map Generator (`map_generator`)**:
   - `config/generator_config.yaml`: Configurable parameters for arena dimensions, room counts (BSP splitting), tilted/angled walls, passable doorway widths vs. deceptive narrow "trap gaps", rotated boxes, cylindrical barrels, and OccupancyGrid noise injection.
   - `map_generator.py`: Zero-dependency generator that simultaneously exports:
     1. `<name>.sdf`: Gazebo Harmonic 3D world with ArUco target stands (`S1` & `C1`).
     2. `<name>_clean.pgm/.yaml`: Ideal 2D OccupancyGrid aligned with robot spawn origin.
     3. `<name>_noisy.pgm/.yaml`: Degraded OccupancyGrid (1-px edge jitter, isolated speckles, unknown gray shadows, slight rotation/offset) for testing navigation robustness.
     4. `<name>_gt.json`: Ground-truth coordinates, C1 dual-robot docking slots, trap gap locations, and A* verified paths.
     5. `<name>_preview.svg`: Top-down vector preview diagram (walls, barrels, trap gaps, rescue zones, A* paths).
   - `verify_midterm.py`: Automated test suite for Clean/Noisy A* navigation, trap-gap inflation rejection, and the 2-minute Referee Verification Dashboard (`evidence/verification_dashboard.svg`).

---

## Quickstart: Parametric Map Generator & Automated Test Suite (Works on macOS & Linux)

```bash
# 1. Generate the exact Midterm Check arena (SDF + Clean/Noisy PGM + GT JSON + Preview SVG)
python3 src/edgeai_perception/edgeai_perception/map_generator.py \
  --preset midterm --out-dir generated_maps

# 2. Generate 5 procedural random test arenas (angled walls, barrels, trap gaps, A* verified)
python3 src/edgeai_perception/edgeai_perception/map_generator.py \
  --preset random --seed 42 --batch 5 --out-dir generated_maps

# 3. Generate a random arena and install its .sdf directly into simulation worlds/
python3 src/edgeai_perception/edgeai_perception/map_generator.py \
  --preset random --seed 42 --name test_arena_1 --install-to-sim

# 4. Run the automated map & navigation robustness test suite + Referee Verification Dashboard
python3 src/edgeai_perception/edgeai_perception/verify_midterm.py \
  --mode all --generate-mock-evidence
```

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

