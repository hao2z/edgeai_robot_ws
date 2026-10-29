# IMU, rescue targets and multiple robots

The simulator retains the original arena and single-robot interfaces. The
additional targets and rescue geometry are development fixtures, not official
competition parameters. No target coordinates are published as detections.

## Start

From a built Pixi workspace on Linux:

```bash
bash start_sim.sh
bash start_sim.sh multi_robot:=true robot_count:=2 slam:=true
```

Do not run both simultaneously. Direct ROS launch (without RViz by default):

```bash
ros2 launch edgeai_simulation multi_robot.launch.py robot_count:=4
ros2 launch edgeai_simulation multi_robot.launch.py slam:=true rviz:=true
```

`gui:=false` runs only the Gazebo server. `robots_file:=/absolute/path.yaml`,
`targets_file:=/absolute/path.yaml` and `imu_file:=/absolute/path.yaml` override
the packaged defaults. In multi-robot mode even a fleet of one robot uses its
namespace. Legacy single mode uses the original spawn pose (-1.5,-1.5,0.01),
heading 0; multi mode uses all six pose values from robots.yaml.

Windows entry point (WSL Ubuntu-24.04, Linux copy built):

```powershell
powershell -ExecutionPolicy Bypass -File D:\KTH\Competition\edgeai_robot_ws\Start-Simulation.ps1 -Robots 2 -SLAM -DisplayServer WSLg
powershell -ExecutionPolicy Bypass -File D:\KTH\Competition\edgeai_robot_ws\Stop-Simulation.ps1
```

WSLg is the default. `-DisplayServer VcXsrv` explicitly selects the fallback.
`-Distro` can override the registered
distribution name. Windows source and `/home/ypfree/edgeai_robot_ws` are separate
copies: rebuild/synchronize the Linux copy after editing Windows source.

## Interfaces and frame ownership

| Legacy single robot | Multi robot (N=1..4) | Type |
| --- | --- | --- |
| /cmd_vel | /robotN/cmd_vel | geometry_msgs/msg/Twist |
| /odom | /robotN/odom | nav_msgs/msg/Odometry |
| /scan | /robotN/scan | sensor_msgs/msg/LaserScan |
| /imu/data_raw | /robotN/imu/data_raw | sensor_msgs/msg/Imu |
| /camera/image | /robotN/camera/image | sensor_msgs/msg/Image |
| /camera/depth_image | /robotN/camera/depth_image | sensor_msgs/msg/Image |
| /camera/camera_info | /robotN/camera/camera_info | sensor_msgs/msg/CameraInfo |
| /camera/points | /robotN/camera/points | sensor_msgs/msg/PointCloud2 |
| /map (optional SLAM) | /robotN/map (optional SLAM) | nav_msgs/msg/OccupancyGrid |

One shared /clock; one shared /tf and /tf_static bus, with unique `robotN/`
frame IDs. The URDF, Gazebo sensor topics, joint names and DiffDrive frames are
all prefixed, not just ROS node namespaces. DiffDrive owns odom->base_footprint;
SLAM owns map->odom. Each robot's map and odom are independent. No static
world->odom transform is invented from Gazebo spawn poses. This is fleet
simulation and independent SLAM support, not map merging, task allocation or
multi-robot navigation. If an EKF is added later, disable DiffDrive's TF output
before assigning that TF to the EKF.

## IMU noise

`config/imu.yaml` defines a 100 Hz raw inertial sensor, fixed 0.02 m above
base_link. Axes follow the body (x forward, y left, z up). Acceleration includes
gravity: a stationary level sensor reports approximately +9.8 m/s^2 in z.

| Per-axis parameter | Gyroscope (rad/s) | Accelerometer (m/s^2) |
| --- | --- | --- |
| White-noise standard deviation | 0.002 | 0.02 |
| Initial bias standard deviation | 0.0005 | 0.01 |
| Dynamic bias driving parameter | 0.00002 | 0.0001 |
| Dynamic bias correlation time | 300 s | 300 s |

White-noise values are per sample, not spectral noise densities. Dynamic bias
uses Gazebo's correlated bias process. These are plausible development values,
not a fitted Phidget calibration. Temperature, magnetic disturbances, hardware
latency and actual chassis vibration are not reproduced by this configuration.
Ideal simulator orientation is disabled. Do not fuse the raw message's
orientation or assume default covariance is a calibrated uncertainty; configure
an estimator for the angular-velocity/acceleration fields and provide suitable
covariances. No EKF or IMU integration into SLAM is enabled by this change.

## Target configuration and geometry

`config/targets.yaml` is arena ground truth for placing targets and evaluating
results. Its poses are in Gazebo **world**, not SLAM map coordinates.

| Target | ArUco ID | Type | World x,y,z (m) | Yaw (rad) |
| --- | --- | --- | --- | --- |
| T1 | 1 | normal | -2.42,-1.50,0.24 | 0 |
| T2 | 2 | normal | -2.42,-0.60,0.24 | 0 |
| T3 | 3 | normal | 2.42,1.60,0.24 | pi |
| TC1 | 10 | cooperative | -0.30,2.42,0.24 | -pi/2 |

The dictionary is DICT_4X4_50. The outer black marker square is 0.20 m, with a
white margin making the entire panel 0.24 m. Pass **0.20**, not 0.24, to PnP.
Each target frame has +x outward from its printed front, +y image-right when
viewed from its front, +z up. OpenCV's square-marker axes are target +y,+z,+x.
The generator renders the exact dictionary bits as thin black geometry over a
white backing, avoiding renderer-dependent UV/material failures. PNG previews
are also generated and round-trip decoded with OpenCV.

For upright targets the development rescue centres are calculated by rotating
target-local planar offsets into world coordinates:

* Normal: (normal_offset_m, 0).
* Cooperative: (normal_offset_m, +/-cooperative_lateral_offset_m).

Defaults are 0.70 m outward, +/-0.40 m lateral, zone radius 0.18 m, normal dwell
3 s and cooperative dwell 5 s. Ground disks show these zones visually and have
no collision geometry. There is no rescue scoring/timer node yet. Navigation
must still obtain marker pose from camera observations. Official geometry,
dictionary, dimensions and dwell times must replace these defaults when issued.

Changing only poses takes effect at the next launch. After changing marker IDs,
dictionary or sizes, regenerate source assets and rebuild:

```bash
pixi run python src/edgeai_simulation/scripts/generate_targets.py \
  --package-dir src/edgeai_robot_description
pixi run colcon build --symlink-install
```

Generated panels and PNG previews are package resources, so normal launch
does not require OpenCV. The generator requires an OpenCV build with `aruco`;
the current Pixi environment has OpenCV 4.12 with this module. Configured markers
are round-trip decoded during generation. The generated world uses absolute
asset paths resolved from the installed package and is kept under a temporary
`edgeai_world_*` directory for launch diagnostics.

## Verification

`D:\KTH\Competition\edgeai_robot_ws\tests\simulation\verify-simulation-extension.sh` runs a
bounded Gazebo check with no GUI and cleans up the launched process group.
It samples actual ROS messages, checks namespaced frames and independent maps,
decodes rendered camera images, measures IMU sampling/noise and optionally turns
only robot1 before explicitly stopping every robot. Reports and images are in
`tests/results/simulation-extension/`. These checks establish interface and
sensor operation, not autonomous rescue performance or live GUI appearance.

## Local validation on 2026-10-27

* Two robots: sensor and TF checks, two independent occupancy maps, rendered
  marker IDs 1/2, command isolation, gyro response and explicit stop passed.
* Four robots: sensor/TF/map checks passed for all four; their camera images
  decoded IDs 1,2,10,3 respectively. IMU sample timestamp spacing was 0.01 s;
  measured stationary gyro-z standard deviations were 0.00188–0.00215 rad/s.
* Initial validation used VcXsrv because WSLg's X11 socket was unavailable.
  Subsequent WSLg diagnosis reproduced the shared-buffer failure and gfxredir=0.
  Restarting only Weston restored gfxredir=1. Native WSLg Gazebo/two RViz
  windows and the dual-robot sensor/map/marker checks then passed. See
  tests/docs/wslg-platform-notes.md; obsolete diagnostics were cleaned up.
  No global WSL shutdown, version change or driver change was performed.
  A later root-cause repair upgraded WSL 2.0.14 to stable 3.0.3 (WSLg 1.0.79)
  after a verified full VHDX backup. Three unassisted starts/restarts within the
  same VM then retained gfxredir=1, and native GUI/sensor checks passed again.
  Historical summary: tests/docs/wslg-platform-notes.md.
* Gazebo's bridge leaves raw IMU covariance arrays zero (unknown), even with
  orientation output disabled. Consumers must ignore orientation and supply
  appropriate inertial measurement covariances when setting up an estimator.

Successful reports are `result-motion-2.json` and `result-multi-4.json` under
the evidence directory. Obsolete failed/debug logs have been cleaned up; current runs keep their own logs.
