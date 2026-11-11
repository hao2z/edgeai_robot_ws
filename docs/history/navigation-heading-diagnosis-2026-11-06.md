> 历史快照：本文描述当时版本；当前实现见 [项目状态](../project-status.md) 与 [任务流程](../task-workflow.md)。

# 2026-11-06 停滞与终点朝向复查

本文记录11月6日的历史回放。11月10日已改为共享预算内比较多朝向完整路径、成功朝向复用的Jazzy兼容实现；当前参数和验证见[任务流程](../task-workflow.md)。下述“一次只试一个朝向”不再是当前实现。

依据会话`20261106-191330-722-domain86-navigation-test`，对两次错误207使用保存的真值地图复查。
目标来自nav.log，坐标精度0.01m；起点朝向来自对应时刻最近的truth样本。
这是静态地图规划回放，不等同于完整运动测试。

| 任务 | 起点XY | 目标XY | 旧普通点适配器 | 扩大预算的旧适配器 | 修正朝向输入后 |
|---|---|---|---|---|---|
| 3 | (2.31,2.81) | (3.73,1.01) | 0.807s超时 | 2.637s成功 | 0.017s成功，车身朝向−90° |
| 5 | (4.22,2.01) | (3.69,0.97) | 0.809s超时 | 2.366s成功 | 0.020s成功，车身朝向+90° |

两处目标位于W2与C1附近；点本身可达，但矩形车身只有少数终点朝向适合该处净空。
原先只根据参考路径朝向排序，未先看终点车身碰撞，内部多朝向搜索消耗了0.8s预算。
直接给原生Lattice可行车身朝向，也能分别约0.063s、0.011s成功。
因此这两次失败的主要问题是朝向输入与自定义搜索策略，不需要缩小车身或关闭碰撞约束。

当时处理：参考路径建议接近方向；16个原语朝向先按与该方向的接近程度排序，
再把终点车身轮廓检查通过的候选排在前面。倒车允许时也考虑反向行驶对应的车身朝向。
选一个作为原生Lattice输入，每个请求只搜索一次。失败交回原行为树重试，
下一次请求换其他未失败候选；成功朝向复用，目标改变重置。
清除了内部穷举循环、独立总预算及`heading_search_time`参数。
保留薄的输入层，因为Jazzy原生规划器需要明确终点朝向。
原始普通目标只检查XY；指定朝向及救援目标不使用此自由朝向选择。

终点轮廓检查只用于优先级，不排除全部不通过的候选，避免剥夺原生位置容差的作用。
实际全路径和姿态碰撞检查仍由原生Smac完成，膨胀、车身、速度约束未因此放宽。

另一次错误102来自自定义Shim：110.801s查询领先110.800s最新TF仅1ms，
却抛出致命ControllerTFError。现改为使用有年龄上限的最新帧变换做航向对齐；
缺失/过期时抛NoValidControl，停车并由原生controller_server在failure_tolerance内重试。
不会以无限等待或忽略过期TF继续运动。

已通过5个C++回归用例：1ms间隙、缺失后恢复、过期拒绝、静态TF，以及窄槽终点朝向优先级。
同会话`planning-replay.json`、`planning-replay-after.json`保留对照结果，
`planning-replay-after.log`中的XY_GOAL_HEADING记录实际输入朝向。

## 官方方案与当前版本边界

官方Smac Hybrid/Lattice在较新版本支持 `goal_heading_mode`：
DEFAULT固定请求朝向，BIDIRECTIONAL比较请求朝向及其180°反向，
ALL_DIRECTION在所有离散朝向中选择路径。多朝向由一次规划调用内部处理，
比外层反复启动完整规划更合适；当前薄输入层只是Jazzy兼容方案，不能保证选到所有朝向中的最低代价路径。

来源：[Smac Lattice配置](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/smac/smac_lattice/configuring_smac_lattice/)、
[Jazzy到Kilted迁移说明](https://docs.nav2.org/rolling/configuration_and_development/migration_guides/jazzy/Jazzy/#enable-goal-orientation-non-specificity)。
本机安装包为ros-jazzy-nav2-smac-planner 1.3.13，头文件及共享库均未发现goal_heading_mode支持。
因此不能直接添加该参数就认为已经生效；若升级或回移官方功能，可再删除当前XY朝向输入层。

普通目标只要求位置时，应同时设置规划侧的自由终点朝向和执行侧的位置到达检查。
只放宽GoalChecker不会自动改变原生SE2规划器对输入yaw的处理。
对于NavFn等2D规划器，官方use_final_approach_orientation可直接采用路径最后两点的方向，
但这不是矩形车身SE2多朝向规划的替代实现。
来源：[NavFn配置](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/planners_plugins/configuring_navfn/)、
[Smac目标朝向讨论](https://github.com/ros-navigation/navigation2/issues/3789)。
