# 当前进度与提交范围

更新：2026-11-07。当前开发分支：`codex/Junnan-dev`。

## 已跑通的任务

默认中期流程已从单机器人建图推进到双机器人协作救援：

robot1 自主探索、SLAM 与视觉目标定位 → S1 单机救援 → 保存地图和目标 → 停止 SLAM → 共享静态地图、两机 AMCL → C1 两个停车位导航 → 双机共同连续停留。

robot1 使用根命名空间，robot2 使用 `/robot2`；地图与时钟共享，各自保留导航、控制权与原始激光避障。robot2 建图期间等待，其车身回波仅从 SLAM 输入中过滤，仍进入导航障碍层。初始相对位姿是可配置估计，不以 Gazebo 真值为运行时定位输入。

## 已保存的成功证据

最新完整轮次：本机 `tests/results/runs/20261107-031658-652-domain86/`，两个结果文件均为成功：

| 结果文件 | 记录 |
|---|---|
| `rescue-result.json` | S1 `success: true`，估计目标距离 0.408 m，连续停留 10 s，完成时刻 102.769 s |
| `cooperative-result.json` | C1 `success: true`，两机估计距离 0.598/0.602 m，共同连续停留 10 s |
| `cooperative-events.jsonl` | 共同计时递增，在仿真时间 143.854 s 写入成功结果 |

这些数值来自任务定位/感知估计。成功事件不能单独证明真实几何距离、无碰撞或比赛总时限达标。此前 `20261107-025941-823-domain86` 也完成 S1/C1（0.355 m、0.649/0.639 m，各10秒）；期间 `030721` 出现无法生成净空分离停车位，`031532` 被中断。因此已有两次完整成功记录，尚不能称为连续稳定成功。最新运行退出码0，关闭期间仍有仲裁/扫描过滤进程异常告警，需与任务执行成功区分。

原始日志、地图和截图保留在本机，不随 Git 提交。仓库保留上述结果摘要；完整协作设计和历史修复见 [协作救援](cooperative-rescue.md)。

## 当前入口与配置

```powershell
# 默认：native Jazzy、中期场地、RPP、Smac Lattice、双机完整任务
.\Start-Autonomous-Exploration.ps1
# 单机对照
.\Start-Autonomous-Exploration.ps1 -SingleRobot
# 仅查看双机仿真环境
.\Start-Simulation.ps1 -SimulationOnly -Robots 2
```

```bash
# WSL；显式指定控制器以对应 Windows 默认配置
bash scripts/simulation/start-autonomous-exploration.sh --controller rpp --planner lattice --arena midterm --duration 3600
# 单机对照：在同一命令前设置 EDGEAI_COOPERATIVE_RESCUE=false
```

`Start-Simulation.ps1` 未带 `-SimulationOnly` 时也转入默认任务。低层 `start_sim.sh` 仅启动仿真。`Start-Cooperative-Test.ps1` 使用真值地图/定位做专项诊断，不代替自主建图与 AMCL 验收。

| 调整内容 | 配置/实现 |
|---|---|
| S1 半径、停留、补充搜寻和重试 | `src/edgeai_navigation/config/midterm_single_rescue.yaml` |
| C1 相对初值、定位门槛、共同停留、停车余量 | `src/edgeai_navigation/config/cooperative_rescue.yaml` |
| 控制器、全局规划 | `rpp_controller.yaml`、`dwb_controller.yaml`、`mppi_controller.yaml`、`smac_lattice.yaml` |
| 公共地图、恢复、速度平滑 | `nav2_exploration_rectangle.yaml`、`behavior_trees/explore_recovery.xml` |
| 深度局部障碍 | `depth_obstacles.yaml`，当前默认关闭；入口可显式开启 |
| 阶段编排 | `scripts/simulation/run-open-source-exploration.sh`、`normal_rescue.py`、`cooperative_rescue.py` |

除标明完整路径外，配置均在 `src/edgeai_navigation/config/`；任务 Python 文件在该包 `scripts/`。实际参数以每轮 `nav2-runtime.yaml` 为准。

## 剩余边界

- 使用一张由 robot1 建立的共享地图，不是两机同时 SLAM 或地图融合。
- 没有窄通道预约或联合路径调度；停车位可达性与重复运行稳定性仍有失败记录。
- 尚未统一实现比赛从探索开始计入切换的 13 分钟总预算；C1 自身预算从 S1 后开始。
- 仍需独立核对真值距离、碰撞、初值扰动下 AMCL 收敛，以及完整干净启动的重复成功证据。比赛核验要求见 [中期清单](competition/midterm-checklist.md)。

## 本次整理

- 移除旧整套 Nav2 配置和两份旧生成器，使用公共配置与独立控制器组合；不改动本次手调 YAML。
- 删除无调用引用的 `diagnose_saved_pose.py` 和固定 20 秒的 `observe_nav2.py`；保留 `capture_navigation_state.py`、`controller_probe.py` 和对应测试作为当前诊断入口。
- 保留启动器仍引用的 MPPI 数学/指令探针、官方 demo 对照、Pixi 回退和恢复回归脚本；这些不是孤立废文件。
- 本次不启动仿真，不删除本机成功轮次证据。

## 本次提交前检查

- 工作区路径/Python与Shell语法检查通过；PowerShell 15个脚本语法通过。
- 原生Jazzy构建navigation、navigation_rviz、perception、simulation四个包通过；导航C++测试3项通过。清理生成器后也移除了CMake对空tools目录的安装要求，避免新克隆缺目录。
- 协作策略与真实ROS接口13项通过；速度限制4项、Lattice配置3项、日志6项、保留策略2项通过；感知/真值几何7项通过。
- 救援ROS接口12项在独立域单独复测通过。首次合并发现测试时出现一次末端修正取消确认异常；单独复测未复现，不据此宣称消除所有时序风险。
- `validate_rectangle_footprint.py` 仍在DWB倒车速度相等断言处失败：DWB请求下限-0.3m/s，公共平滑器下限-0.2m/s。该项不影响本轮RPP成功证据，但属于DWB对照配置的已知不一致，本次保持用户调参不变。
- 未执行新的图形仿真测试；ROS接口测试使用模拟后端，不能替代完整运行。
