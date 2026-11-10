# 任务流程入口

更新：2026-11-10。默认流程为robot1建图并找到全部必需目标→S1→地图交接→双机定位/C1。按固定规则调度；Shell负责启动和清理，不决定下一救援阶段。

| 层级 | 定义位置 | 职责 |
|---|---|---|
| 总任务 | `src/edgeai_mission/config/integrated.xml` | 执行第一任务、S1成功后启动C1 |
| 第一任务 | integrated.xml中的MappingAndTargetSearch、tasks/find_targets.xml | 前沿完成、停止探索、逐个检查ID/类型、补搜缺失目标、保存输出 |
| 单机救援 | `src/edgeai_mission/config/tasks/s1.xml` | 选目标/必要搜寻、选位、导航、条件检查、修正、有界重试 |
| 双机救援 | `src/edgeai_mission/config/tasks/c1.xml` | 交接、就绪/定位有界重试、双位选择、并发导航、共同停留、重选 |
| 任务接口 | `src/edgeai_mission/action/ExecuteTask.action`、`src/task_executor.cpp` | 可反馈、可取消的S1/C1任务执行 |
| 单位适配 | `src/edgeai_mission/scripts/task_units.py` | 计算、派发、轮询、一次判定；复用navigation算法 |
| 每次导航 | `src/edgeai_navigation/config/behavior_trees/explore_recovery.xml` | 规划、控制与恢复；两机及各到位策略保留共同恢复入口 |
| 启动 | `scripts/simulation/run-open-source-exploration.sh` | 参数、进程、日志和清理，不决定业务顺序 |

协调器通过 `/mission/tasks/s1/execute`、`/mission/tasks/c1/execute` 调用任务，接收Feedback和Result。任务树通过内部服务调用单位算法，再调用各机器人的Nav2 Action执行运动。候选计算和试规划都不移动机器人。

任务树负责救援判定，协调器负责调度和汇总。导航成功只表示到达导航要求，必须另行通过救援距离、有效数据、停车与连续停留检查。

S1配置为 `src/edgeai_navigation/config/midterm_single_rescue.yaml`，C1为 `cooperative_rescue.yaml`。导航失败次数与救援重做次数分别控制；运行时写入 `mission-runtime.json` 的 `task_navigation_attempts`、`task_attempts`，表示包含首次的尝试次数。重试不重置总预算；Nav2内部恢复次数独立。

运行树快照在会话的 `mission-trees/`。`mission.log` / `mission-events.jsonl`记录调度与反馈；`task-unit-events.jsonl`记录单位操作结果；最终查看 `mission-result.json`、`rescue-result.json`、`cooperative-result.json`。

停止总任务：

```bash
ros2 service call /mission/stop std_srvs/srv/Trigger '{}'
```

响应只表示收到请求，最终结果在清理确认后发布。参数/树修改后重新启动会话。

实现边界、验证与剩余拆分工作见 [重构记录](mission-refactor-status.md)。完整比赛模式、单位行为与异常清单见 [设计文档](mission-behavior-tree-design.md)，其中规划项不等于已实现功能。

目标随SLAM局部修正及静态地图交接机制见 [目标地图绑定](target-map-anchoring.md)。交接前先固定最终地图与关键帧目标，再保存地图并启动AMCL。

第一任务将前沿建图完成与视觉目标齐备分开检查：前沿耗尽不代表S1/C1都已找到。find_targets任务通过现有搜寻模块补齐缺失目标，全部通过check_targets后才允许进入S1。C1树只检查交接后的目标有效性，不承担首次搜寻。默认双机required_targets为ID1/normal、ID2/cooperative；单机对照只要求S1。任务输出target-search-result.json，不覆盖S1救援结果。
