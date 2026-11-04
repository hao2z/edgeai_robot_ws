# ArUco target search during mapping

The normal autonomous-exploration runner starts `aruco_targets.py` alongside
SLAM/Nav2. It never sends navigation goals or velocity commands. Set Linux env
`EDGEAI_TARGET_SEARCH=false` to disable; target search requires the RGB-D camera.

Configuration: `config/aruco_targets.yaml`. Development assumptions are
DICT_4X4_50 and a 0.20 m black marker edge, NOT the 0.24 m panel. Update these
when official technical specifications are available. No target truth pose or
fixture YAML is loaded by the detector.

Inputs: `/camera/image`, `/camera/camera_info`, and image-time TF from
`camera_optical_frame` to `odom`. OpenCV 4.6 legacy and newer detector APIs are
supported. Square PnP candidates must have positive depth and pass size, range
and reprojection checks. Only the latest image is processed, at 5 Hz.

Outputs:
- `/targets/observations`: `std_msgs/String` JSON containing ID, timestamp,
  camera-frame translation/rotation vector, odom-frame position and pixel error.
- `/targets/estimates`: transient-local `std_msgs/String` JSON registry of map
  positions, candidate/confirmed/stale state, observation count and spread.
- `/targets/markers`: transient-local MarkerArray with spheres and ID labels.
- Per-run `perception.log`, `target-observations.jsonl`, and atomic `targets.json`.
  FOUND/CONFIRMED events also appear in events.log and the live event monitor.

The first release localizes marker centres. It does not publish fused marker
orientation, infer rescue zones, actively sweep viewpoints, or guarantee that
all markers will be seen. Three accepted frames confirm an ID. Confirmation is
not an accuracy guarantee: spread is sample consistency, not total uncertainty.
The odom-frame sliding window is reprojected using map→odom at publication time;
this follows rigid SLAM corrections but cannot fully undo historical nonrigid
pose-graph deformation. Raw observations are retained for future refinement.
Depth fusion is not enabled in this release.

Build: `bash scripts/setup/build-workspace.sh --packages-select edgeai_perception edgeai_exploration`

Manual execution after environment setup:

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

Windows: append `-Scan360` to `Start-Autonomous-Exploration.ps1`.
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

Normal exploration now also launches `observation_scan.py`. Configure
`config/observation_scan.yaml`; Linux `EDGEAI_OBSERVATION_SCAN=false` disables
this scheduler without disabling passive target detection.

A candidate must meet ALL conditions:
- AUTO navigation is IDLE after at least one successful goal; no recovery or
  active/pending navigation is canceled for scanning.
- Robot odometry is fresh and below 0.03 m/s and 0.05 rad/s for 0.4 simulation s.
- Local costmap and robot TF are fresh (0.5 s); rotated costmap origins are
  rejected. Map edge, unknown cells and lethal obstacle cells reject the disk.
- Required clear disk radius is max(0.50 m, full-body circumscribed radius +
  0.08 m). The runner derives body radius from BOTH actual runtime footprints,
  including padding and the offset of the rotation axis. Costmap cells
  intersecting the disk boundary are included. Non-lethal inflation is not
  treated as a physical wall. Nav2 Spin independently collision-checks motion.
- At least 30 simulation s since the last scan, at least 1.2 m from ALL previous
  scan positions; at most eight attempts per run. Failed attempts also occupy
  their scan location, preventing repeated scans of the same blocked place.

The authority's `/navigation_authority/observation_hold` checks AUTO/IDLE
atomically before entering STOPPED. A busy request is deferred, not canceled.
The scheduler waits for explorer pause acknowledgement, checks geometry again,
then runs the same scan360 task. After a known terminal result it resumes AUTO.
Manual/stop commands change a request sequence; an intervening command cancels
the scan and prevents automatic resume. Uncertain motion terminal state leaves
control held. This is cooperative control admission, not a hard real-time lock.

Per-run `observation-scan-runtime.yaml` captures geometry and thresholds;
`observation.log`, `observation-scan-N.log/json` show admission and results.
Timer policy uses simulation time; control handshake deadlines use wall
monotonic time. Position spacing is in the current SLAM map and remains an
approximation if the map undergoes a large correction. The scheduler does not
yet explicitly classify rooms or estimate camera line-of-sight information gain.

Open-position smoke verification: `20261029-235033-domain86-90424` selected
map position approximately (1.094, 0.361) after the first navigation success.
Computed body rotation radius was 0.3669 m; the 0.50 m disk gate passed. Scan
reported status SUCCEEDED, error_code 0, angular travel 6.28785 rad, and resumed
exploration. RViz capture after scan completion showed exploration had resumed.
ID2 was confirmed later in the run, but this single run does not establish a
causal improvement in target recall. Earlier test 234848 deferred all scans;
the candidate TF lookup was corrected to use the fresh odometry timestamp
rather than requesting an unavailable future map→base transform.
The same open-position test recorded a Gazebo exit-code-1 warning during
shutdown despite the wrapper's exit-code file being 0; motion scan completion
and shutdown health are reported separately.
