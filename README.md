# EdgeAI Robot Workspace

A ROS 2 workspace for a realistic differential-drive robot in Gazebo, with RViz visualization, LiDAR, an RGB-D camera, and SLAM toolbox mapping.

## Quickstart: Midterm Check Part 1 — Autonomous Arena Exploration, SLAM & ArUco Detection

This section provides quick instructions for running the `5.0 m × 5.0 m` Midterm Arena simulation (`midterm_arena.sdf`), live SLAM Toolbox mapping, ArUco target detection (`S1` ID 1 & `C1` ID 2), and autonomous full-arena exploration.

### Terminal 1: Launch Gazebo, SLAM Toolbox & RViz Simulation
```bash
./start_sim.sh
```
- Starts `midterm_arena.sdf` with walls (`W1`, `W1b`, `W2`), obstacles (`O2`, `O3`), and freestanding ArUco target stands (`S1` at `(1.0, 4.1)` and `C1` at `(4.1, 1.0)`).
- Automatically launches `slam_toolbox` (`online_async_launch.py`) to render the live 2D occupancy grid map (`/map`) in RViz.
- Opens RViz with `/map`, `/target_markers`, `/exploration_markers`, and `/camera/aruco_debug` pre-enabled.

### Terminal 2: Run ArUco Detector & Evidence Logger
```bash
pixi shell
source install/setup.bash
ros2 run edgeai_perception aruco_detector
```
- Subscribes to `/camera/image` and `/camera/camera_info`.
- Uses OpenCV PnP (`0.16 m` marker size) + `tf2_ros` to compute relative (`base_footprint`) and global (`map`/`odom` and Arena) target poses.
- Publishes:
  - `/camera/aruco_debug` (`sensor_msgs/msg/Image`): Live annotated camera HUD with bounding boxes, 3D axes, and distance.
  - `/detected_targets` (`std_msgs/msg/String`): JSON list of discovered targets (`S1` normal, `C1` coop).
  - `/target_markers` (`visualization_msgs/msg/MarkerArray`): 3D pins, labels, and rescue radius rings (`0.5 m` / `0.7 m`) in RViz.
- Automatically saves Part 1 verification snapshots and JSON logs to `evidence/`.

### Terminal 3: Run Autonomous Full-Arena Explorer or Single Target Rescue
```bash
pixi shell
source install/setup.bash
# 1) Full-arena SLAM exploration & target mapping:
ros2 run edgeai_perception arena_explorer

# 2) Single-target (S1) rescue (Start Area -> S1 hold 10s -> Start Area):
ros2 run edgeai_perception single_rescue
```
- `arena_explorer`: Autonomously navigates through all 5 zones of the `5.0 m × 5.0 m` Midterm Arena using SLAM-corrected TF (`map -> base_footprint`), `/odom`, and 360° LiDAR (`/scan`) obstacle avoidance; renders the complete 2D SLAM map in RViz, visually inspects both `S1` and `C1`, and saves `maps/midterm_slam_map.yaml` and `maps/midterm_slam_map.pgm`.
- `single_rescue`: Navigates from the robot's current pose to the Start Area `(0.70, 0.50)`, drives to Single Target `S1` `(1.0, 4.1)`, holds inside the `0.5 m` rescue radius for **10 consecutive seconds** of simulation time (saving `evidence/S1_rescue_verification.json`), and returns to the Start Area.
- *(Optional simple patrol)*: `ros2 run edgeai_perception square_patrol`

### Terminal 4: Inspect Live Detected Target List
```bash
pixi shell
source install/setup.bash
ros2 topic echo /detected_targets
```

---

## Quickstart: Parametric Map Generator, Previews & Automated Verifier

This section covers generating custom or random test arenas (`map_generator.py`), previewing generated 2D/3D maps, and running the automated robustness and referee verifier (`verify_midterm.py`).

> **Build note:** In this workspace's Pixi environment (`setuptools 84.0.0`), `edgeai_perception` does not accept `--symlink-install`. Use `pixi run colcon build` for full workspace builds; you can still use `--symlink-install` when rebuilding only `edgeai_robot_description`.

### 1. Generate Maps (Midterm Preset or New Random Seeds)
```bash
# Generate the standard Midterm preset arena
pixi run python3 src/edgeai_perception/edgeai_perception/map_generator.py \
  --preset midterm --out-dir generated_maps

# Generate a single random arena with a chosen seed (example: seed 100 -> random_arena_s100.*)
pixi run python3 src/edgeai_perception/edgeai_perception/map_generator.py \
  --preset random --seed 100 --out-dir generated_maps --install-to-sim

# Generate a batch of 5 consecutive seeds (seed 100..104 -> random_arena_s100..s104)
pixi run python3 src/edgeai_perception/edgeai_perception/map_generator.py \
  --preset random --seed 100 --batch 5 --out-dir generated_maps --install-to-sim
```
- Optional flags: `--name <custom_name>` (for single map output), `--config <path>` (defaults to `src/edgeai_perception/config/generator_config.yaml`).
- Each map outputs:
  - `<name>.sdf` (3D Gazebo world; copied to `src/edgeai_robot_description/worlds/` when `--install-to-sim` is passed)
  - `<name>_clean.pgm/.yaml` & `<name>_noisy.pgm/.yaml` (ideal and noise-degraded 2D occupancy maps)
  - `<name>_gt.json` (ground-truth obstacles, docking slots, trap gaps, and verified A* paths)
  - `<name>_preview.svg` (color-coded top-down SVG preview)

### 2. View Generated Maps (2D & 3D)
**2D previews & dashboard:**
```bash
xdg-open generated_maps/random_arena_s100_preview.svg
xdg-open generated_maps/midterm_arena_preview.svg
xdg-open evidence/verification_dashboard.svg
```

**3D Gazebo Harmonic (+ optional RViz):**
Whenever a new `.sdf` is copied into `src/edgeai_robot_description/worlds/` with `--install-to-sim`, rebuild `edgeai_robot_description` before launching:
```bash
pixi run colcon build --packages-select edgeai_robot_description --symlink-install
pixi run bash -c "source install/setup.bash && ros2 launch edgeai_robot_description simulation_real.launch.py world:=random_arena_s100.sdf"
```
To open RViz alongside Gazebo (`simulation_real.launch.py` alone starts Gazebo + SLAM Toolbox + bridges without RViz):
```bash
pixi run rviz2 -d src/edgeai_robot_description/rviz/realistic_robot.rviz
```

### 3. Run Automated Map Tests & Competition Verifier
```bash
# Offline test suite (SDF integrity, Clean/Noisy A* navigation, C1 docking geometry, trap-gap rejection)
pixi run python3 src/edgeai_perception/edgeai_perception/verify_midterm.py \
  --mode test-maps --maps-dir generated_maps

# Live simulation verification (checks SLAM map, Part 1 detections, and Part 2 evidence logs)
pixi run python3 src/edgeai_perception/edgeai_perception/verify_midterm.py --mode verify-run
```
- `--mode verify-run` writes `evidence/verification_report.json` and `evidence/verification_dashboard.svg`.

---

## Environment

- Ubuntu 24.04 (x86-64).
- Gazebo Harmonic.
- ROS 2 Jazzy managed by **Pixi**, using the RoboStack Jazzy and conda-forge channels in `pixi.toml`.
- Pixi installed and available on `PATH`; a graphical desktop for Gazebo and RViz.

The manifest and lockfile define the workspace dependencies. Use the Pixi environment for ROS commands and builds; the instructions below do not require sourcing `/opt/ros/jazzy/setup.bash`.

## Install and build

```bash
git clone https://github.com/dongjw-hit/edgeai_robot_ws.git
cd edgeai_robot_ws
pixi install
pixi run colcon build --symlink-install
```

Run these commands from the repository root. Rebuild after changing package resources or adding configuration files.

## Start the simulation

From the repository root:

```bash
./start_sim.sh
```

The script resolves the workspace relative to its own location, activates Pixi, sources `install/setup.bash`, launches `simulation_real.launch.py`, and opens RViz with `realistic_robot.rviz`. It requires a completed build and does not depend on a fixed home-directory path. Press **Ctrl+C** in its terminal to stop the launched session.

The launch file loads `edgeai_robot_real.urdf.xacro`, starts the world in `worlds/empty.sdf`, spawns `edgeai_robot_real`, and starts the ROS–Gazebo bridge and robot transforms. The old simple model and its launch script have been removed; `start_sim.sh` is the current entry point.

The portable realistic startup has been tested successfully. AMCL localization remains a local experiment; its launch and parameter files are not included in this repository. Installed AMCL dependencies do **not** mean that localization or autonomous navigation is complete.

## ROS commands in another terminal

In each additional terminal, enter the cloned repository and activate its environment:

```bash
cd /path/to/edgeai_robot_ws
pixi shell
source install/setup.bash
```

Replace `/path/to/edgeai_robot_ws` with your clone location. The following commands assume this setup and a running simulation.

### Main topics

| Topic | ROS message type | Purpose |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Robot velocity commands |
| `/odom` | `nav_msgs/msg/Odometry` | Gazebo DiffDrive odometry |
| `/tf` | `tf2_msgs/msg/TFMessage` | Dynamic transforms, including `odom` → `base_footprint` |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Fixed robot and sensor transforms |
| `/joint_states` | `sensor_msgs/msg/JointState` | Simulated joint states |
| `/scan` | `sensor_msgs/msg/LaserScan` | LiDAR scan |
| `/camera/image` | `sensor_msgs/msg/Image` | RGB image |
| `/camera/depth_image` | `sensor_msgs/msg/Image` | Depth image |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Camera calibration |
| `/camera/points` | `sensor_msgs/msg/PointCloud2` | RGB-D point cloud |
| `/clock` | `rosgraph_msgs/msg/Clock` | Simulation time |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Occupancy map when SLAM toolbox is running |

The saved RViz configuration uses `odom` as its fixed frame. To inspect available topics:

```bash
ros2 topic list
ros2 topic echo /odom --once
```

### Move and stop

In clear space, publish a slow forward command:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.1}, angular: {z: 0.0}}'
```

Stop the publisher with **Ctrl+C**, then explicitly send a zero velocity command:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.0}}'
```

Stopping the publisher alone should not be treated as a robot stop command.

## Mapping with SLAM toolbox

Keep the simulation running. In another prepared terminal at the repository root:

```bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:="$PWD/src/edgeai_robot_description/config/slam_toolbox.yaml" \
  use_sim_time:=true
```

The repository configuration uses `/scan`, `base_footprint`, `odom`, and `map`, with mapping mode enabled and a map resolution of 0.05 m. Simulation time comes from the bridged `/clock` topic. The simulation startup script does not start SLAM automatically.

In RViz, set **Fixed Frame** to `map` after SLAM starts and add a **Map** display for `/map`. Drive slowly through free space to collect scans and revisit previously observed areas. Avoid sustained contact with walls or obstacles.

### Save the occupancy map

While SLAM is running and `/map` is available, use another prepared terminal at the repository root:

```bash
mkdir -p maps
ros2 run nav2_map_server map_saver_cli -f "$PWD/maps/my_map" \
  --ros-args -p use_sim_time:=true
```

This writes `maps/my_map.yaml` and its associated map image. Choose a new basename for each map you want to preserve. These files are occupancy-map exports, not a serialized SLAM toolbox pose graph for resuming a mapping session.

The package also contains `src/edgeai_robot_description/maps/edgeai_arena.yaml` and its map image. Saving `maps/my_map` does not overwrite the packaged map.

## Known limitation: odometry drift during collisions

In the reported movement tests, normal straight driving stayed synchronized between Gazebo and RViz, and an explicit stop command stopped the robot. When the robot contacted a wall or obstacle, its physical motion in Gazebo stopped while `/odom` continued changing and the robot in RViz continued moving.

The current `/odom` source is the Gazebo DiffDrive plugin. Treat this odometry as an estimate rather than collision-corrected ground truth: wheel motion under contact or slip can diverge from actual displacement. The observed collision drift remains unresolved. Stop the robot when it becomes blocked, avoid driving into obstacles during mapping, and do not interpret continued RViz movement as physical motion in Gazebo. SLAM is not a guarantee that this behavior is corrected.

## Repository layout

```text
pixi.toml / pixi.lock                 Pixi environment and dependency lock
start_sim.sh                         Simulation + RViz entry point
src/edgeai_robot_description/
  launch/simulation_real.launch.py   Realistic simulation and bridges
  urdf/edgeai_robot_real.urdf.xacro   Current robot model
  worlds/empty.sdf                   Simulation world
  rviz/realistic_robot.rviz           RViz configuration
  config/slam_toolbox.yaml           Mapping parameters
  maps/                             Packaged map resources
```
