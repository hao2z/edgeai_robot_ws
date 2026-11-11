# 任务流程入口

代码归属见[模块边界](module-boundaries.md)。救援与搜寻编排安装为`edgeai_tasks`，候选计算安装为`edgeai_search`；布局变更后需重新构建。

更新：2026-11-10。默认流程为robot1建图并找到全部必需目标→S1→地图交接→双机定位/C1。按固定规则调度；Shell负责启动和清理，不决定下一救援阶段。

| 层级 | 定义位置 | 职责 |
|---|---|---|
| 总任务 | `src/edgeai_mission/config/integrated.xml` | 执行第一任务、S1成功后启动C1 |
| 第一任务 | integrated.xml中的MappingAndTargetSearch、tasks/find_targets.xml | 无前沿后检查已知面积、不足则换位重启探索、逐个补搜缺失目标、保存输出 |
| 单机救援 | `src/edgeai_mission/config/tasks/s1.xml` | 选目标/必要搜寻、选位、导航、条件检查、修正、有界重试 |
| 双机救援 | `src/edgeai_mission/config/tasks/c1.xml` | 交接、就绪/定位有界重试、双位选择、并发导航、共同停留、重选 |
| 任务接口 | `src/edgeai_mission/action/ExecuteTask.action`、`src/task_executor.cpp` | 可反馈、可取消的S1/C1任务执行 |
| 单位适配 | `src/edgeai_mission/scripts/task_units.py` | 计算、派发、轮询、一次判定；复用navigation算法 |
| 每次导航 | `src/edgeai_navigation/config/behavior_trees/explore_recovery.xml` | 规划、控制与恢复；两机及各到位策略保留共同恢复入口 |
| 启动 | `scripts/simulation/run-open-source-exploration.sh` | 参数、进程、日志和清理，不决定业务顺序 |

协调器通过 `/mission/tasks/s1/execute`、`/mission/tasks/c1/execute` 调用任务，接收Feedback和Result。任务树通过内部服务调用单位算法，再调用各机器人的Nav2 Action执行运动。候选计算和试规划都不移动机器人。

任务树负责救援判定，协调器负责调度和汇总。导航成功只表示到达导航要求，必须另行通过救援距离、有效数据、停车与连续停留检查。

S1/C1通过任务判定后，`rescue_success_log.py`读取当前会话的Gazebo真值历史，输出每台机器人的真值距离、朝向误差、距离/朝向估计误差和目标定位误差，并核验停留时段的采样距离、朝向与位姿差分速度。C1要求两车覆盖共同停留时段。终端和`events.log`用OK/ERROR/WARN分别显示真值核验通过、未通过、无法确认；结构化明细写入对应救援结果的`success_evidence.truth_verification`及`robots[].truth`。真值记录不再依赖导航TF成功，启动时保存`truth-targets.json`以保留目标真值。核验只用于仿真评估，不向导航提供真值，也不改变任务树的估计位姿判定。

前沿探索的`/exploration/navigate_to_pose`使用frontier策略：输入朝向作为候选排序/等代价时的偏好，`GridBased`比较多朝向路径；局部采用`FollowPath`和位置检查器`general_goal_checker`，到位只检查位置，不执行最终转向。普通位置点、补充换位等共用这一多朝向规划策略；显式朝向点及最终救援仍使用`GridBasedHeading`原生固定朝向规划。当前前沿朝向由探索器的出发位置到目标的方向生成，不等同于精确的未知区域表面法线。

多朝向入口为`smac_lattice.yaml`的`goal_heading_mode`、`goal_heading_count`、`heading_search_budget_s`、`heading_reuse`。实现参考[最新上游Smac Lattice](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/smac/smac_lattice/configuring_smac_lattice/)的DEFAULT/BIDIRECTIONAL/ALL_DIRECTION模式。当前安装的Jazzy 1.3.13没有原生多目标朝向搜索，因此在`XYLatticePlanner`中作兼容：ALL_DIRECTION按`goal_heading_count`在全周分散选取原语朝向（默认4，0表示全部，当前最多16），不自动扩增候选数，各候选仍由原生Smac Lattice生成、碰撞检查；在共享2秒墙钟预算内比较返回路径，综合路程、地图代价、旋转和倒车评分。此评分参考上游analytic-expansion的路径积分，不是原生A*内部g值，也不保证全局最优；日志记录实际尝试数、可行数、分数和耗时。取消检查按原生搜索迭代周期执行，预算是协作式时间限制。

默认复用同一目标已成功的朝向，失效时再比较其他方向；关闭`heading_reuse`可每次重算全部候选。已移除原来NavFn建议朝向及“一次只试一个方向”的逻辑。朝向不合法会在同一次规划内尝试其余方向；起点占据、越界、TF错误及外部取消仍直接交给既有处理，不隐藏错误。2026-11-10真实Nav2无运动测试：窄槽固定朝向失败，16候选找到1条可行路径；开阔区域16候选中12条可行，选择沿路径方向的低分路线；重复规划只尝试1次缓存朝向。尚未进行新的Gazebo导航测试。

导航入口职责：`goal_policy.py`定义每种请求的规划器/控制器/到位检查器及路由；`navigation_requests.py`接入Action与RViz选点和手动朝向开关；`navigation_telemetry.py`只观测里程计、路径跟踪和恢复证据，不派发或取消动作。`navigation_authority.py`保留执行槽互斥、确认原Action终态、取消和模式交接、状态不明时禁止重复派发。任务检查、换位/探索切换、救援重试规则仍在协调器和任务行为树中；这些组件运行在同一个导航接入进程，不新增ROS节点。

S1配置为 `src/edgeai_navigation/config/midterm_single_rescue.yaml`，C1为 `cooperative_rescue.yaml`。导航失败次数与救援重做次数分别控制；运行时写入 `mission-runtime.json` 的 `task_navigation_attempts`、`task_attempts`，表示包含首次的尝试次数。重试不重置总预算；Nav2内部恢复次数独立。

运行树快照在会话的 `mission-trees/`。`mission.log` / `mission-events.jsonl`记录调度与反馈；`task-unit-events.jsonl`记录单位操作结果；最终查看 `mission-result.json`、`rescue-result.json`、`cooperative-result.json`。

停止总任务：

```bash
ros2 service call /mission/stop std_srvs/srv/Trigger '{}'
```

响应只表示收到请求，最终结果在清理确认后发布。参数/树修改后重新启动会话。

实现边界、验证与剩余拆分工作见 [重构记录](history/mission-refactor-status-2026-11-10.md)。完整比赛模式、单位行为与异常清单见 [设计文档](history/mission-behavior-tree-design-2026-11-10.md)，其中规划项不等于已实现功能。

目标随SLAM局部修正及静态地图交接机制见 [目标地图绑定](target-map-anchoring.md)。交接前先固定最终地图与关键帧目标，再保存地图并启动AMCL。

第一任务将前沿建图完成与视觉目标齐备分开检查：前沿耗尽不代表S1/C1都已找到。find_targets任务通过现有搜寻模块补齐缺失目标，全部通过check_targets后才允许进入S1。C1树只检查交接后的目标有效性，不承担首次搜寻。默认双机required_targets为ID1/normal、ID2/cooperative；单机对照只要求S1。任务输出target-search-result.json，不覆盖S1救援结果。

建图期间，`MappingAndTargetSearch`发现本轮所需目标均有地图位置和朝向后，关闭途中观测转圈的派发；已开始的观测正常结束，前沿探索继续。若交接时仍缺目标，`find_targets`照常补搜。

建图门槛在 `src/edgeai_mission/config/mapping.yaml`：默认已知面积至少 **18 m²**。协调器订阅原始 `/map`，按所有有效已知格（空闲及障碍，0～100）数量乘分辨率平方统计；不统计未知格，不使用外接矩形面积。无前沿只是重新检查门槛的信号，不直接授权任务切换。面积达标后才进入缺失目标搜寻，达到面积也不会提前打断仍在进行的前沿探索。

面积不足时，树暂停探索导航派发，并行执行`AwaitFrontier`检测和有净空的分散观察位置换位导航。`paused_map_processing_enabled: true`使探索器继续以2Hz接收地图、处理代价地图和计算前沿，但不抢占导航槽；可用候选数通过`/explore/available_frontiers`发布（-1表示数据不齐）。发现前沿后由协调器树终止换位分支，等待取消清理，再恢复AUTO派发前沿导航。任务切换规则在树中，权限节点仅执行控制槽交接。

2026-11-10验证：协调器27项、换位适配4项及真实探索器暂停/检测/恢复测试通过，未运行新Gazebo仿真。探索器上游抢占套件仍有3项旧断言失败（涉及低收益取消及settle），在移除此轮监测改动的独立基线中同样复现；不能将完整上游回归记为全通过。

最多补充3轮，换位超时由`mapping.yaml`的`reposition_timeout_s`控制（当前60秒），不重置探索总预算；换位沿用普通位置到达策略和Nav2恢复。换位超时会先取消并确认结束，再恢复前沿计算；外部取消、控制权被夺取、缺少可用观察位置、前沿监测失联、恢复AUTO失败或次数耗尽均报告失败，不进入救援。`MAPPING_AREA_READY/INSUFFICIENT`记录面积和轮次；`MAPPING_FRONTIER_AVAILABLE`记录树切换触发。换位动作记录在`mission-stage.log`、`rescue-events.jsonl`，位置保存在`mapping-supplement-positions.json`。无前沿时仅记录一次`FRONTIER_EMPTY_DIAGNOSTICS`，分层对比原图、决策图和代价筛选后的簇大小，并在`frontier-diagnostics/`保存当时的地图/代价地图/位姿，供离线重放。18 m²是当前场景的最低面积门槛，不等于完整覆盖证明。

2026-11-11停留判定修复：C1的短暂TF/时钟回调错序只等待数据，不立即否定已有停留证据；等待期间不推进计时。距最后有效采样超过`max_data_age_s`、时钟回退或实际距离/朝向/停车/定位条件不合格仍清零。`WAITING_DATA`新增原因、样本年龄和`dwell_retained`，避免静默重置。S1真值核验使用任务结束采样时间前的10秒窗口，边界用相邻真值插值；最多等待3墙钟秒补齐末端审计记录，不把停留前转向纳入核验，不改变导航或任务判据。
