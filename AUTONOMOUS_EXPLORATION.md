# Single-robot autonomous exploration: development results

The default integration uses SLAM Toolbox, Nav2 DWB and upstream
`frontier_exploration_ros2` 1.6.1, pinned to
`11864e4ac6789dc81fd71bad51e43339c01deca2`. Its 141 upstream tests passed.
Explore Lite is retained only for comparisons, selected with
`EDGEAI_EXPLORER=lite`; its pinned revision is
`326cf8a0b487c34246bb8f3326afbcd69576dc60`.
The default environment is native apt ROS Jazzy. Project builds are under
`~/.cache/edgeai-native-build`; upstream overlays are under `~/.cache/edgeai-native`.
The old Pixi environment under `~/.cache/edgeai-open-source` is an explicit
comparison backend. No preloaded arena map is given to exploration.

## 当前开发基线（2026-10-29）

推荐使用 native ROS Jazzy + NavFn Dijkstra + DWB/Rotation Shim + SLAM Toolbox。
DWB 与速度平滑器已统一：前进 0.6 m/s、倒车 0.3 m/s、角速度 ±0.9 rad/s，
线加速度 ±0.4 m/s²、角加速度 ±0.8 rad/s²。DWB 允许倒车，不能保证长路径一定先掉头。
MPPI 是独立比较配置，不能把 DWB 的参数与测试结论直接套用到 MPPI。

| 经常调整的内容 | 配置位置 |
|---|---|
| DWB 速度、评分、进展检查、碰撞模型 | `src/edgeai_navigation/config/nav2_exploration_rectangle.yaml` |
| MPPI 独立比较配置 | `src/edgeai_navigation/config/nav2_exploration_rectangle_mppi.yaml` |
| 重规划周期、恢复顺序、重试次数 | `src/edgeai_navigation/config/behavior_trees/explore_recovery.xml` |
| 目标评估、抢占、失败区域抑制 | `src/edgeai_exploration/config/frontier_exploration.yaml` |

当前 DWB 栅格 5 cm，局部/全局膨胀半径均 0.38 m，衰减因子分别 5.0/3.0；
矩形轮廓的局部/全局边距分别 0.005/0.01 m。全局仅启用 static + inflation，
局部保留实时雷达障碍；动态障碍依赖局部避障，全局路线可能穿过动态障碍。
MPPI 使用半径 0.37 m 的圆形模型，几何约束与 DWB 不同。

全局重规划由 XML `RateController.hz` 控制，当前 1 Hz；
`expected_planner_frequency` 只设置性能预期。前沿处理当前 2 Hz。
DWB 位置到达容差与 NavFn 目标搜索容差均为 0.10 m。
进展检查是 7.0 秒仿真时间内平移不足 0.10 m；原地转向不能满足该进展门槛。
清图后优先后退 0.20 m（请求速度 0.20 m/s），后退失败则转 1 rad，
后续恢复轮次尝试反向转 1 rad、等待 1 秒。保留原任务，仍执行碰撞检查。

前沿失败阈值为 2 次、冷却 30 秒仿真时间，区域初始尺寸 0.30 m、扩大量 0.10 m。
外层无进展超时仍为 10 秒，可能提前取消导航；尚未关闭。
成功到达不会自动删除前沿，失败抑制也不能处理“成功但无新增信息”的重复访问。

参数单位见行内中文注释。ROS double 类型的整秒值必须写 `7.0`，不能写 `7`。
修改 YAML/XML 后重启测试；每轮 `nav2-runtime.yaml` 是该轮实际启动快照。
配置生成工具用于从上游模板重建配置，不是当前手调配置的备份；重新生成会覆盖手调值，
需先查看差异。普通运行不需要执行这些生成工具。

## 最近验证与已知问题

- `20261029-224053-916-domain86`：探索器发布完成事件；早期 NavFn 存在势场路径提取失败，
  执行两轮后退/旋转。完成事件只代表算法无更多前沿，不证明完整覆盖。
- `20261029-224535-858-domain86`：用户观察运动较顺利；早期没有 NavFn 提取失败，
  后段仍记录无进展恢复、任务取消、重复目标和全部前沿被抑制后的等待。
  本次整理时没有最终地图评估，不能给出最终覆盖率或无故障结论。
- 仍需处理：重复低收益前沿、探索器外层超时与 Nav2 时序，以及偶发 NavFn 路径提取失败。
- Gazebo 偶发 D3D12/OpenGL 上下文失败，与导航参数错误区分；没有证据认定每次都是旧进程残留。
- 历史 Smac Lattice 基元和 rectangle_5cm 比较配置已删除，后文旧实验仅作历史记录。

原始日志、地图、截图保留在本机 `tests/results/runs/`，不随 Git 提交。
以上是运行改善的开发基线，尚未证明所有目标可达或比赛精度达标。

## WSL / Linux launch

```bash
cd /mnt/d/KTH/Competition/edgeai_robot_ws
bash scripts/simulation/start-autonomous-exploration.sh --controller dwb --arena competition --duration 300
```

This runs the test in the current terminal and opens Gazebo/RViz through WSLg.
Add `--check` to check the environment without starting or stopping anything.
No manual ROS sourcing or Pixi activation is required. `--help` lists the
named options. `--duration` is wall-clock seconds, not simulated seconds.

## Windows launch

```powershell
powershell -ExecutionPolicy Bypass -File D:\KTH\Competition\edgeai_robot_ws\Start-Autonomous-Exploration.ps1 -Controller dwb -Arena competition -Duration 300
```

This opens the test terminal, a live event monitor and a screenshot observer;
it delegates to the same Linux entry, which opens Gazebo/RViz on domain 86.
`-InCurrentWindow` keeps the test worker in the current PowerShell window;
`-CheckOnly` checks runtime packages without closing old terminals or starting a test.
Use `-Backend native` (Linux `--backend native`) to explicitly select native ROS.
Windows environment variables are passed explicitly by the launcher, so no
`WSLENV` editing is needed for backend selection. Benchmark
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

Both costmaps use the polygon with no `robot_radius` setting. The rectangular
profiles use NavFn (Dijkstra) for geometric global paths, with a 0.15 m goal
tolerance and final approach orientation. NavFn does not perform oriented
rectangular footprint checks in global search; DWB `ObstacleFootprint` and
MPPI footprint critics check local motion. Rotation Shim uses path positions;
MPPI disables path inversion enforcement and uses geometric path headings
(`PathAlignCritic.use_path_orientations=false`, `PathAngleCritic.mode=0`).
Robot dimensions, inflation, speed limits and critic weights are unchanged.
The lattice assets are retained for future comparisons.
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

## Recovery behavior tree

`config/behavior_trees/explore_recovery.xml` in `edgeai_navigation` is an unchanged
copy of the native Jazzy Nav2 `navigate_to_pose_w_replanning_and_recovery.xml`.
It replans at 1 Hz and uses contextual costmap clearing plus the default recovery
round robin: clear both costmaps, spin 1.57 rad, wait 5 s, back up 0.30 m at
0.15 m/s. The navigation authority and runtime configuration reference the same
file. Robot behavior-server limits and collision checks still apply. Existing
sessions must be restarted to load the restored tree.

MPPI circular comparison (2026-10-29): `--controller mppi` now uses a 0.37 m
radius in both costmaps and `CostCritic.consider_footprint=false`. Inflation is
0.40 m to cover that radius. The historical `nav2_exploration_rectangle_mppi.yaml`
filename is retained for launcher compatibility. DWB retains the rectangle.
Collision Monitor consumes the costmap's published circular outline.
