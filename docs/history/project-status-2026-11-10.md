> 历史快照：本文描述当时版本；当前实现见 [项目状态](../project-status.md) 与 [任务流程](../task-workflow.md)。

# 当前版本与验证记录

更新：2026-11-10。分支：codex/Junnan-dev。

## 默认任务

robot1 自主建图并定位所有必需目标 → S1 单机救援 → 固定最终地图与目标 → 双机 AMCL 定位 → C1 并发导航和共同连续停留。

第一任务的 MappingAndTargetSearch 子树位于 src/edgeai_mission/config/integrated.xml。
前沿探索结束后暂停探索派发，调用 tasks/find_targets.xml 检查目标清单；缺少 S1 或 C1 时复用疑似目标优先、视锥观察及转圈搜寻。全部必需目标有效定位后，才保存输出并进入救援。默认双机必需目标为 ID1/normal、ID2/cooperative；单机对照只要求 S1。

| 层级 | 负责内容 |
|---|---|
| mission_coordinator | 固定规则调度、任务 Action 调用、健康检查与总预算 |
| task_executor + 任务树 | find_targets、S1、C1 的单位顺序、检查、有界重试 |
| task_units.py | 复用几何、感知、搜寻、定位算法，调用 Nav2 |
| Nav2 行为树 | 规划、控制、无进展与碰撞风险下的恢复 |
| Shell | 参数准备、进程、日志和清理 |

导航到点不等于救援成功。S1/C1 另行检查任务距离、新鲜数据、停车和连续停留；C1两机共同计时，任一不满足会重置。目标观测绑定SLAM关键帧，最终地图冻结后不随当前AMCL刚体变换重投影。

## 已核实的运行证据

原始记录仅保留本机 tests/results/runs/，不随Git上传。

| 会话 | 结果 |
|---|---|
| 20261110-010835-542-domain86 | 整体成功，168.717仿真秒汇总；S1估计距离0.420m，C1 0.544/0.600m，各停留10秒；含一次C1双停车位重选 |
| 20261110-013749-839-domain86 | mission-result.json记录成功，161.624仿真秒汇总 |
| 20261110-014714-076-domain86 | S1成功；C1就绪和定位通过，缺失ID2却直接进入prepare_target失败；本版本新增第一任务补搜，尚待用户完整仿真验证 |

成功结果来自感知/定位估计，不自动证明真实距离、无碰撞或比赛验收。不能从以上选择的轮次计算整体成功率。较早记录见 [11月7日状态快照](project-status-2026-11-07.md)。

## 启动与调整

~~~powershell
.\Start-Autonomous-Exploration.ps1
.\Start-Autonomous-Exploration.ps1 -SingleRobot
.\Start-Simulation.ps1 -SimulationOnly -Robots 2
~~~

~~~bash
bash edgeai.sh build
bash edgeai.sh check
bash edgeai.sh mission
bash edgeai.sh navigation
~~~

Linux安装与手动选点入口见 [队员上手](../linux-quickstart.md)。配置入口：

| 内容 | 文件（导航配置相对 src/edgeai_navigation/config） |
|---|---|
| 第一任务和救援顺序 | src/edgeai_mission/config/integrated.xml、tasks/*.xml |
| 必需目标、任务预算 | src/edgeai_mission/scripts/prepare_mission.py、会话mission-runtime.json |
| 补搜视锥/疑似点、S1条件 | midterm_single_rescue.yaml |
| C1定位、双位余量、共同计时 | cooperative_rescue.yaml |
| RPP/Shim、Lattice | rpp_controller.yaml、smac_lattice.yaml |
| 地图、进展判定、恢复 | nav2_exploration_rectangle.yaml、behavior_trees/explore_recovery.xml |

每轮实际配置以 nav2-runtime.yaml、mission-runtime.json 和 mission-trees/ 为准。
[任务流程](../task-workflow.md)、[重构记录](mission-refactor-status-2026-11-10.md)、[目标地图绑定](../target-map-anchoring.md)分别说明职责、异常处理和坐标关系。

## 剩余事项

- 首任务补搜的完整用户仿真回归；持续保存成功、失败和中断轮次。
- 独立Part2从规定起点完成S1+C1，及统一13分钟总预算；当前cooperative专项只覆盖C1。
- 独立核对真值距离、碰撞、连续停留以及粗略初值扰动下的AMCL收敛。
- 停车位检查考虑两车最终轮廓与各自规划，但没有预留另一车未来停车占据、联合路径或窄通道预约。
- 前沿、搜寻内部候选/扫描和途中观察未全部拆成任务树单位；当前迁移覆盖任务调度与救援编排。
- DWB对照配置倒车下限-0.3m/s与公共平滑器-0.2m/s不一致，矩形验证相关断言仍失败；默认RPP配置与该对照问题分开记录。
- 标记中心目前按用户要求降至0.15m，区别于文档约0.35m的推荐值，演示前应确认现场条件。

## 整理与验证边界

保留此前任务树、地图锚定、C1生命周期就绪修复；本轮额外补齐第一任务全目标搜寻门控。
删除已被协调器取代、无调用引用的探索完成监听脚本；过时长篇说明归档，原比赛文档与本机运行证据保留。
不修改本轮导航速度、代价或控制参数。验证使用构建、配置及隔离ROS接口测试；本轮不启动Gazebo/RViz，完整仿真由用户执行。

## 提交前验证

- 五包构建通过：mission、navigation、perception、exploration、simulation。
- 任务调度/真实任务树20个场景通过（18项整套通过，另2项修正任务筛选断言后定向复测通过）。
- 真实Action→XML→Python单位服务4项通过，包含第一任务已有S1、缺失C1的补搜。
- 单位适配17项、任务配置2项、救援策略10项、单机ROS12项、协作ROS12项、日志7项通过。
- 布局/Python/Shell检查、中期场地几何和15个PowerShell入口语法通过；当前指南本地链接有效。
- 矩形车身验证仍在DWB/速度平滑器倒车下限相等断言处失败，未修改这组对照参数。
- 本轮未启动图形仿真；上述接口测试使用非运动测试后端，不能替代用户完整仿真验收。
