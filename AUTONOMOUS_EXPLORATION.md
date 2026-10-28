# Single-robot autonomous exploration: development results

The default integration uses SLAM Toolbox, Nav2 DWB and upstream
`frontier_exploration_ros2` 1.6.1, pinned to
`11864e4ac6789dc81fd71bad51e43339c01deca2`. Its 141 upstream tests passed.
Explore Lite is retained only for comparisons, selected with
`EDGEAI_EXPLORER=lite`; its pinned revision is
`326cf8a0b487c34246bb8f3326afbcd69576dc60`.
The isolated environment is under `~/.cache/edgeai-open-source`; its manifest
and lockfile are in `algorithms/exploration/nav2-env`. The original Pixi
environment is separate. No preloaded arena map is given to exploration.

## Windows launch

```powershell
powershell -ExecutionPolicy Bypass -File D:\KTH\Competition\edgeai_robot_ws\Start-Autonomous-Exploration.ps1 -Arena competition -Duration 300
```

This opens Gazebo, RViz, a ROS shell and a ROS monitor on domain 86. Benchmark
runs use domain 87 and a distinct Gazebo partition. Ctrl+C in the exploration
window stops the session. Normal termination exports a map under
`tests/results/runs/<timestamp>-domain<id>/`. Raw logs and readable key events are retained. See [observation workflow](tests/docs/test-observation.md).

The competition arena is a development interpretation of kickoff slide 4:
5 x 5 m, upper rooms, a lower-right room, doorways and static obstacles. It is
not an official arena or an exact dimensional reconstruction. The original
world remains selectable with `-Arena simple`. Target placements are fixtures.

## Measurements on 2026-10-28

Each row is one run, not a statistically established ranking. Observation
coverage counts known map cells over ground-truth free area; it does not mean
the robot physically visited every cell. Evaluation uses a fixed initial
map/world registration, so SLAM drift affects the measurements.

| Arena / configuration | Budget | Observed coverage | Outcome |
|---|---:|---:|---|
| Simple / Explore Lite + MPPI | 45 s | 14.69% | Very slow reverse commands |
| Simple / Explore Lite + DWB | 45 s | 86.85% | Continued exploration |
| Simple / Explore Lite + DWB | 180 s | 98.64% | Stopped after about 162 s: all frontiers tried, including blacklisted goals |
| Competition / Explore Lite + DWB | 180 s | 83.22% | Time limit; planning and recovery failures near doorway |
| Competition / larger footprint + inflated-costmap frontiers | 180 s | 14.73% | Stopped after about 12 s: no frontiers detected; rejected as an improvement |
| Competition / frontier_exploration_ros2 + same DWB | 180 s | 83.65% | Upstream completion event at about 73 s; substantial regions still unknown |

Competition baseline per-room observation: upper left 43.36%, upper middle
99.84%, upper right 84.30%, lower right 45.40%, main area 97.70%. Geometry
validation found one connected component for a 0.25 m clearance. Thus the low
coverage is not evidence of a deliberately disconnected room.

## Open-source capability audit and limitations

Connected: NavigateToPose actions, NavFn global planning, DWB local control,
local/global costmaps and inflation, velocity smoothing, collision monitoring,
Nav2 behavior-tree recovery, Explore Lite frontier markers, pause/resume and
structured status publishers. Live logs confirmed Nav2 cancellation when
paused, resumption, and recovery attempts. The status publisher was inspected;
the CLI retained-message audit timed out and is not claimed as passed.

Explore Lite's completion status includes the case where every remaining
frontier is blacklisted. The runner distinguishes this from no detected
frontiers and from a time limit; none alone proves complete physical coverage.
`return_to_init` is deliberately false. ArUco detection, target-list generation,
multi-robot map merging and rescue execution are not implemented here.

The complex arena has not passed full exploration with either explorer.
The replacement reached multiple goals, skipped targets blocked in local
costmaps and published a completion event received by the reliable,
transient-local completion watcher. That event triggers map export, but does
not prove full physical coverage. Per-room observation for the replacement:
upper left 74.38%, upper middle 99.24%, upper right 58.82%, lower right 51.74%,
main area 95.56%. These are single-run results and do not establish superiority.

The replacement's native `/control_exploration` service was exercised in a
separate GUI run: STOP and START both returned `accepted=True`. The default
configuration enables runtime control, DP-based MRTSP ordering, blocked-goal
checks, goal preemption and temporary suppression. It waits when all frontiers
are temporarily suppressed instead of mapping that condition to completion.
Graphical runs default to Gazebo and RViz with the map and frontier markers
enabled. `EDGEAI_GUI=false EDGEAI_RVIZ=false` explicitly selects headless mode.

## Rectangular collision model

The default Windows launch now uses the `rectangle` navigation profile:
front +0.1125 m, rear -0.3125 m, left/right +/-0.165 m relative to
`base_footprint`. Its nominal size is 0.425 x 0.330 m; 0.01 m padding on each
side makes the effective envelope 0.445 x 0.350 m. The rectangle contains the
drive wheels and rear caster swivel sweep. This is a model-derived envelope,
not a physical-robot calibration.

Both costmaps use the polygon with no `robot_radius` setting. Smac Lattice
performs SE2 global planning with 5 cm / 0.20 m curve-radius differential-drive
primitives generated by the pinned upstream tool; DWB uses `ObstacleFootprint` instead of `BaseObstacle`.
Collision Monitor uses the local costmap's published polygon. The rectangle
outline is included in the RViz configuration. Circular profiles are retained
as explicit comparison configurations, enlarged to enclose the axle-centred model.

`tests/simulation/validate_rectangle_footprint.py` in this repository
checks collision-envelope samples, matching costmaps and the oriented
geometry of a narrow passage. That geometric check is not a physical passage
trial and does not by itself prove better exploration coverage.

The navigation reference is now the drive axle midpoint, 8.75 cm ahead of the
chassis centre. Measurements above predate this correction. Exploration goal
tolerance is now 0.15 m; a separate, unintegrated rescue candidate uses 0.05 m.
Neither is a measured accuracy guarantee. See [geometry and accuracy](docs/geometry-and-accuracy.md).
