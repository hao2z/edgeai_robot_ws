# EdgeAI Robot Workspace

The canonical development workspace is this repository, on branch
[`codex/Junnan-dev`](https://github.com/hao2z/edgeai_robot_ws/tree/codex/Junnan-dev).

| Directory | Purpose |
|---|---|
| `src/edgeai_robot_description/` | Robot URDF and basic RViz configuration |
| `src/edgeai_simulation/` | Gazebo launch files, worlds, targets and arena tools |
| `src/edgeai_navigation/` | Nav2 configuration, control authority and generators |
| `src/edgeai_perception/` | ArUco detection and target localization during mapping |
| `src/edgeai_exploration/` | Frontier configuration, upstream adapters and exploration visualization |
| `src/edgeai_navigation_rviz/` | RViz navigation panel plugin |
| `tests/` | Simulation and exploration checks, observation documentation |
| `scripts/` | Build, environment, simulation, display and capture tools |
| `docs/competition/` | Competition source documents |
| `runtime/`, `tests/results/` | Local state, retired-script archives and per-run evidence; excluded from Git |

本次建图＋目标寻找版本的结果、截图和限制见
[2026-10-30 开发基线](docs/mapping-target-search-baseline-2026-10-30.md)。

当前可运行基线、常调参数位置、最近测试与已知限制见
[AUTONOMOUS_EXPLORATION.md](AUTONOMOUS_EXPLORATION.md)。

Windows/WSL entry points, run from this directory:

```powershell
powershell -ExecutionPolicy Bypass -File .\Start-Simulation.ps1 -SLAM
powershell -ExecutionPolicy Bypass -File .\Start-Autonomous-Exploration.ps1 -Controller dwb -Arena competition -Duration 300
```

Alternatively, run directly in the Ubuntu-24.04 WSL terminal:

```bash
cd /mnt/d/KTH/Competition/edgeai_robot_ws
bash scripts/simulation/start-autonomous-exploration.sh --controller dwb --arena competition --duration 300
```

Both entries default to the native apt Jazzy environment. No manual `source`
or Pixi activation is required. Use Linux `--backend native` / Windows
`-Backend native` to override a saved backend. Linux `--check` / Windows
`-CheckOnly` validates the environment and installed project packages without
starting or stopping a session. Duration is a wall-clock budget in seconds.

This opens Gazebo/RViz, stops registered previous project sessions and streams
events in the current terminal. Per-run logs and the final map are saved under
`tests/results/runs/`; completed history is pruned after the test, with a visible
cleanup message. Windows monitor windows, automatic maximization and screenshot
automation are supplied by the PowerShell launcher. With the direct WSL entry,
close any old project terminal tabs yourself; Ctrl+C stops the active test.

With the PowerShell launcher, restart closes previous project processes and terminal tabs. Gazebo opens
maximized with an inclined arena overview. Raw logs, readable key events and
startup/progress/final screenshots are retained per run. See the
[observation workflow](tests/docs/test-observation.md) and
[exploration results and limitations](AUTONOMOUS_EXPLORATION.md).
Current axle-centred footprint, smaller curve primitives and the distinction
between goal tolerance and measured accuracy are documented in
[geometry and accuracy](docs/geometry-and-accuracy.md).
RViz also shows global/local costs and paths, same-time footprint comparisons,
SLAM correction and message ages; see [visualization and time alignment](docs/rviz-navigation-observability.md).
Large heading changes use Nav2 Rotation Shim before DWB tracking; ordinary
exploration gain updates no longer repeatedly cancel the active goal. See
[turning and goal coordination diagnosis](docs/navigation-turn-diagnosis.md).

A ROS 2 workspace for a realistic differential-drive robot in Gazebo, with RViz visualization, LiDAR, an RGB-D camera, and SLAM toolbox mapping.

The simulator now includes a noisy raw IMU, four development ArUco targets,
configurable rescue-zone visuals and optional 1–4 robot namespaces. See
[SIMULATION.md](SIMULATION.md) for configuration, frame conventions and tests.
Independent per-robot SLAM is supported; map fusion and rescue autonomy are
separate integration stages.

## Environment

- Ubuntu 24.04 (x86-64).
- Gazebo Harmonic.
- ROS 2 Jazzy from the official Ubuntu apt repository for the native backend.
- A graphical desktop for Gazebo and RViz; Windows launchers use WSLg by default.

Environment activation is centralized in `scripts/ros/with-ros-env.sh`.
Select native ROS with `EDGEAI_ROS_BACKEND=native`, or save `native` in the
machine-local `runtime/ros-backend` file after migration validation. Native ROS
uses `/opt/ros/jazzy/setup.bash`. The retained Pixi backend and its lockfiles
remain available for regression comparisons; start native ROS outside a
Pixi/Conda shell.

## Install and build

```bash
git clone --branch codex/Junnan-dev https://github.com/hao2z/edgeai_robot_ws.git
cd edgeai_robot_ws
sudo bash scripts/setup/install-native-ros.sh
EDGEAI_ROS_BACKEND=native bash scripts/setup/build-workspace.sh
```

Run these commands from the repository root. Native build output is placed in
`~/.cache/edgeai-native-build`, overridable with `EDGEAI_BUILD_ROOT`. The build
uses this checkout's `src`; no separate editable WSL copy is required.
Rebuild with `bash scripts/setup/build-workspace.sh` after package changes.
During migration, the native frontier overlay is built separately from the
pinned source cache with:

```bash
EDGEAI_ROS_BACKEND=native bash scripts/setup/build-frontier-runtime.sh
```

The legacy Pixi installation entry is `scripts/setup/install-workspace.sh`;
its dependency manifests remain in the repository. For the Pixi navigation
backend, prepare its pinned environment and explorer with:

```bash
EDGEAI_ROS_BACKEND=pixi bash scripts/setup/prepare-open-source.sh
```

## Start the simulation

From the repository root:

```bash
bash start_sim.sh
```

The script resolves the workspace relative to its own location, activates the selected ROS backend,
sources the build cache's `install/local_setup.bash`, launches
`simulation_real.launch.py`, and opens RViz with `realistic_robot.rviz`.
Press **Ctrl+C** in its terminal to stop the launched session.

The launch file loads `edgeai_robot_real.urdf.xacro`, starts the world in `worlds/empty.sdf`, spawns `edgeai_robot_real`, and starts the ROS–Gazebo bridge and robot transforms. The old simple model and its launch script have been removed; `start_sim.sh` is the current entry point.

The portable realistic startup has been tested successfully. AMCL localization remains a local experiment; its launch and parameter files are not included in this repository. Installed AMCL dependencies do **not** mean that localization or autonomous navigation is complete.

## ROS commands in another terminal

In each additional terminal, enter the cloned repository and activate its environment:

```bash
cd /path/to/edgeai_robot_ws
EDGEAI_ROS_BACKEND=native bash scripts/ros/open-ros-terminal.sh
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
  slam_params_file:="$PWD/src/edgeai_simulation/config/slam_toolbox.yaml" \
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

The package also contains `src/edgeai_simulation/maps/edgeai_arena.yaml` and its map image. Saving `maps/my_map` does not overwrite the packaged map.

## Known limitation: odometry drift during collisions

In the reported movement tests, normal straight driving stayed synchronized between Gazebo and RViz, and an explicit stop command stopped the robot. When the robot contacted a wall or obstacle, its physical motion in Gazebo stopped while `/odom` continued changing and the robot in RViz continued moving.

The current `/odom` source is the Gazebo DiffDrive plugin. Treat this odometry as an estimate rather than collision-corrected ground truth: wheel motion under contact or slip can diverge from actual displacement. The observed collision drift remains unresolved. Stop the robot when it becomes blocked, avoid driving into obstacles during mapping, and do not interpret continued RViz movement as physical motion in Gazebo. SLAM is not a guarantee that this behavior is corrected.

## Repository layout

See [workspace layout](docs/workspace-layout.md) for package boundaries,
build locations and the archive of retired scripts. The root `start_sim.sh`
remains the simulation entry point. The former root `algorithms/` and
`simulation/` directories have been consolidated into the packages under `src/`.
# 原生 Jazzy 环境

原生 apt 环境的准备、测试结果和回退方法见 [迁移记录](docs/native-ros-migration.md)。

建图时默认同步识别 ArUco 并记录目标地图坐标，配置和当前限制见
[src/edgeai_perception/README.md](src/edgeai_perception/README.md)。
