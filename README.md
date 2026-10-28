# EdgeAI Robot Workspace

The canonical development workspace is this repository, on branch
[`codex/Junnan-dev`](https://github.com/hao2z/edgeai_robot_ws/tree/codex/Junnan-dev).

| Directory | Purpose |
|---|---|
| `src/` | ROS package, models, worlds, launch files and runtime configuration |
| `simulation/` | Competition arena generation and visualization tools |
| `algorithms/exploration/` | Pinned open-source explorer/Nav2 integration |
| `tests/` | Simulation and exploration checks, observation documentation |
| `scripts/` | Build, environment, simulation, display and capture tools |
| `docs/competition/` | Competition source documents |
| `runtime/`, `tests/results/` | Local state and per-run logs/screenshots; excluded from Git |

Windows/WSL entry points, run from this directory:

```powershell
powershell -ExecutionPolicy Bypass -File .\Start-Simulation.ps1 -SLAM
powershell -ExecutionPolicy Bypass -File .\Start-Autonomous-Exploration.ps1 -Arena competition -Duration 300
```

Test restart closes previous project processes and terminal tabs. Gazebo opens
maximized with an inclined arena overview. Raw logs, readable key events and
startup/progress/final screenshots are retained per run. See the
[observation workflow](tests/docs/test-observation.md) and
[exploration results and limitations](AUTONOMOUS_EXPLORATION.md).
Current axle-centred footprint, smaller curve primitives and the distinction
between goal tolerance and measured accuracy are documented in
[geometry and accuracy](docs/geometry-and-accuracy.md).

A ROS 2 workspace for a realistic differential-drive robot in Gazebo, with RViz visualization, LiDAR, an RGB-D camera, and SLAM toolbox mapping.

The simulator now includes a noisy raw IMU, four development ArUco targets,
configurable rescue-zone visuals and optional 1–4 robot namespaces. See
[SIMULATION.md](SIMULATION.md) for configuration, frame conventions and tests.
Independent per-robot SLAM is supported; map fusion and rescue autonomy are
separate integration stages.

## Environment

- Ubuntu 24.04 (x86-64).
- Gazebo Harmonic.
- ROS 2 Jazzy managed by **Pixi**, using the RoboStack Jazzy and conda-forge channels in `pixi.toml`.
- Pixi installed and available on `PATH`; a graphical desktop for Gazebo and RViz.

The manifest and lockfile define the workspace dependencies. Use the Pixi environment for ROS commands and builds; the instructions below do not require sourcing `/opt/ros/jazzy/setup.bash`.

## Install and build

```bash
git clone --branch codex/Junnan-dev https://github.com/hao2z/edgeai_robot_ws.git
cd edgeai_robot_ws
bash scripts/setup/install-workspace.sh
```

Run these commands from the repository root. Build output is placed in
`~/.cache/edgeai-robot-build`, overridable with `EDGEAI_BUILD_ROOT`. The build
uses this checkout's `src`; no separate editable WSL copy is required.
Rebuild with `bash scripts/setup/build-workspace.sh` after package changes.
For autonomous navigation, prepare the pinned environment and explorer first:

```bash
bash scripts/setup/prepare-open-source.sh
```

## Start the simulation

From the repository root:

```bash
bash start_sim.sh
```

The script resolves the workspace relative to its own location, activates Pixi,
sources the build cache's `install/local_setup.bash`, launches
`simulation_real.launch.py`, and opens RViz with `realistic_robot.rviz`.
Press **Ctrl+C** in its terminal to stop the launched session.

The launch file loads `edgeai_robot_real.urdf.xacro`, starts the world in `worlds/empty.sdf`, spawns `edgeai_robot_real`, and starts the ROS–Gazebo bridge and robot transforms. The old simple model and its launch script have been removed; `start_sim.sh` is the current entry point.

The portable realistic startup has been tested successfully. AMCL localization remains a local experiment; its launch and parameter files are not included in this repository. Installed AMCL dependencies do **not** mean that localization or autonomous navigation is complete.

## ROS commands in another terminal

In each additional terminal, enter the cloned repository and activate its environment:

```bash
cd /path/to/edgeai_robot_ws
pixi shell
source "${EDGEAI_BUILD_ROOT:-$HOME/.cache/edgeai-robot-build}/install/local_setup.bash"
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
