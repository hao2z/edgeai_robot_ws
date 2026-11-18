# ArUco target search during mapping

The normal autonomous-exploration runner starts `aruco_targets.py` alongside
SLAM/Nav2. It never sends navigation goals or velocity commands. Set Linux env
`EDGEAI_TARGET_SEARCH=false` to disable; target search requires the RGB-D camera.

Configuration: `config/aruco_targets.yaml`. Development assumptions are
DICT_4X4_50 and a 0.20 m black marker edge, NOT the 0.24 m panel. Update these
when selecting another arena. The midterm runner overrides the edge to 0.16 m.
The mission launcher supplies the ID-to-type mapping from the selected target list.
Manual launches can set the `target_types` ROS parameter with comma-separated
`ID:type` pairs. The detector receives no target truth poses.

Inputs: `/camera/image`, `/camera/camera_info`, and image-time TF from
`camera_optical_frame` to `odom`. OpenCV 4.6 legacy and newer detector APIs are
supported. Square PnP candidates must have positive depth and pass size, range
and reprojection checks. The detector checks for new frames at 30 Hz. Simulation
cameras default to 15 Hz and accept an EDGEAI_CAMERA_RATE override.
In the integrated mission, each suspect detector shares the latest RGB frame
and camera calibration with ArUco. Depth and laser subscriptions stay per robot.

Outputs:
- `/targets/observations`: `std_msgs/String` JSON containing ID, timestamp,
  camera-frame translation/rotation vector, odom-frame position and pixel error.
- `/targets/estimates`: transient-local `std_msgs/String` JSON registry of map
  positions, candidate/confirmed/stale state, observation count and spread.
- `/targets/markers`: transient-local MarkerArray with spheres and ID labels.
- Per-run `perception.log`, `target-observations.jsonl`, and atomic `targets.json`.
  FOUND/CONFIRMED events also appear in events.log and the live event monitor.

Marker centres and fused orientations are now published; the post-mapping normal
rescue pipeline is documented in `docs/normal-rescue.md`. Automatic observations
remain opportunistic and do not guarantee that all markers will be seen. Three accepted frames confirm an ID. Confirmation is
not an accuracy guarantee: spread is sample consistency, not total uncertainty.
Observations bind to nearby SLAM keyframes and follow each corrected map pose. Before static-map handover, the final graph and target registry are frozen together; old observations then remain fixed in map. See [map anchoring](../../docs/target-map-anchoring.md). Raw observations and compact anchor-change records remain available for diagnosis.
Depth fusion is not enabled in this release.

Build: `bash scripts/setup/build-workspace.sh --packages-select edgeai_perception edgeai_navigation edgeai_exploration`

Manual mapping execution requires the project anchored_slam_toolbox node. With a static map, add the ROS parameter map_anchoring:=map. After environment setup:

```bash
bash scripts/ros/with-ros-env.sh ros2 run edgeai_perception aruco_targets.py \
  --ros-args --params-file src/edgeai_perception/config/aruco_targets.yaml
```

Validation:
`bash scripts/ros/with-ros-env.sh env ROS_DOMAIN_ID=96 python3 tests/perception/test_aruco_targets.py`

2026-10-29 GUI smoke: run `20261029-232800-domain86-82910` completed with exit0.
A temporary ID3 panel was inserted only in that simulator instance; the detector
actually confirmed the scene's existing ID10 with three observations. Its final
map centre was approximately (1.478, 3.964, 0.246) m. A fixed initial registration
comparison gives about 4.9 cm error for this one target; this is not a general
accuracy claim. RViz GUI/display configuration was visually inspected before
confirmation; the later capture after confirmation found the window already
closed, so the saved screenshot does not establish the final ID label rendering.

References: OpenCV ArUco tutorial and solvePnP/IPPE documentation.

## Startup 360-degree observation task

Windows: append `-Scan360` to `Start-Autonomous-Mission.ps1`.
Linux: append `--scan360` to `scripts/simulation/start-autonomous-exploration.sh`.

This is an optional startup task, not a periodic scheduler. Authority starts in
STOPPED mode, perception remains active, and no exploration goal is dispatched
until a Nav2 Spin action terminates. Requested yaw is 2π radians with a 30 s
simulation-time allowance. Spin speed/acceleration come from behavior_server
and velocity_smoother. Native collision checks remain active. Success requires
both action success and feedback angular travel of at least 2π−0.15 rad.
A known terminal failure is recorded as incomplete and exploration continues;
unknown action state or unexpected authority-mode change stops this launch.
`scan360.log` and `scan360-result.json` contain results; key events appear in
events.log. Acceptance/cancel/operational watchdogs use wall monotonic time,
while Nav2 motion allowance uses simulation time. The scan has a separate
bounded startup budget; `Duration` remains the subsequent exploration budget.

The task does not restore a previously canceled mid-run navigation goal. It
runs before the explorer starts, avoiding competition for velocity control.
It cannot be combined with isolated motion/recovery tests. A single initial
turn covers only visible targets from the starting pose; it does not guarantee
coverage of other rooms or replace future viewpoint planning.

Startup-scan validation on 2026-10-29: run `20261029-234031-domain86-86681`
reported Spin status SUCCEEDED, error_code=0, feedback travel 6.2872 rad;
exploration goals were sent only after the scan result. RViz capture showed
in-place rotation while navigation admission was held. Collision Monitor later
exited with -11 during shutdown; scan success is not a clean-shutdown claim.

## Automatic observation at open positions

Observation turns are dispatched by the mission's fleet search task, using
`edgeai_mission/config/tasks/find_targets.xml` and `edgeai_tasks/fleet_search.py`.
The retired AUTO observer and its pause/resume services are no longer installed.
`scan360.py` remains a bounded startup motion test; it leaves navigation STOPPED
and the task coordinator decides what runs next.
