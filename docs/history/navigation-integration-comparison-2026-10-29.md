# 导航集成对照审查：rosnav、Nav2 与本项目

日期：2026-10-29。检查的是实际源码、运行参数与失败快照；未在本机运行 rosnav，
不能把其 README 的功能表当作所有组合都已验证的证明。参考文件的只读副本保存于
忽略目录 `runtime/rosnav-reference/`。本次没有再调控制权重或启动探索测试。

## 结论

成熟模块之间出现基础问题，并不能由“算法不够先进”解释。本项目缺少可重复的
最小闭环验收，以及每个控制器组合的明确接口约定。之前把 MPPI 配置加载成功就切为
默认过早；在有向前路径的简单起步场景中，MPPI 已复现负速度和反向预测，必须先解决
这个缺陷，再谈扩大预测范围、复杂避障或完整探索。

目前只能把根因定位到 MPPI/其实际运行配置处理闭环附近，尚未确认是参数适配、
优化实现、依赖构建或某项输入的具体缺陷。不能用一次权重对比宣称已找到唯一根因。

## 源码与运行的对照

| 方面 | rosnav 源码 | 本项目与影响 |
| --- | --- | --- |
| 版本/配置选择 | `_common.nav2_params_filename` 按 ROS 发行版、底盘、controller 选择整份文件；Jazzy 会忽略部分 controller 选择并打印警告 | 基于安装环境默认 YAML 生成多份配置，再由 shell 的 profile 覆盖选择；曾有 controller=mppi 却被 rectangle 的 DWB 文件覆盖的路由风险。已添加矩形 MPPI 专用路由，但仍需实际插件名作为证据 |
| MPPI 运动尺度 | Humble 的独立 MPPI 文件是 70×0.04=2.8 s、vx .15、ax .5、az .6；Jazzy 文件却是 56×.05=2.8 s、vx .5、ax 3、az 3.5 | 本项目是 .2/.4/.8 的低速低加速度模型，直接扩展为 4 s 并继承通用 critic 参数，尚无闭环验收，不能照抄 Jazzy 的高速参数 |
| 路径语义 | Humble MPPI 配 Navfn；Jazzy文件配 SmacHybrid REEDS_SHEPP。路径对齐默认不使用姿态、角度模式0 | 本项目用 Lattice、倒车扩展、orientation=true、mode2、强制方向切换。接口复杂度更高；它们理论可配套，不能由“均支持倒车”推出实际通过 |
| TF/里程计 | EKF/UKF 负责 odom→base_link，轮速与 IMU 融合；注释明确禁止原始轮里程计再直接发同一 TF | 本项目 Gazebo DiffDrive 直接发布 odom/TF，模拟 IMU 未融合。这是另一种明确架构，当前不能靠增加第二个 TF 发布者来“补融合” |
| 激光输入 | raw scan→range/shadow filters→scan quality gate→消费端 | 本项目直接桥接 scan 到 SLAM/代价地图。缺少统一输入质量门控；需要测自反射、无效距离和时间戳，不能直接复制其 .35 m 最小距离，避免抹掉近障碍 |
| 目标放置 | builtin explorer 有可达性处理、目标回退、障碍净空和 costmap 目标检查 | 本项目关闭 goal_skip_on_blocked_goal 以免打断恢复，上游同一开关也关闭了派发前检查；调度协调与目标有效性耦合，需要独立化 |
| 失败分类 | builtin explorer 含 timeout、局部停滞与候选黑名单 | 本项目保留 Nav2 到终态再重选正确，但 Start occupied 被连续换目标放大。起点错误应进入状态诊断，不应只抑制目标区域 |
| 速度安全链 | rosnav 有安全输出路由，文档提醒 safety 关闭会使 Gazebo 收不到指令 | 本项目 controller/behavior→smoother→collision monitor→bridge，需按消息时间戳核对。旧DWB快照显示底盘跟随了原地转动指令，非统一“底盘吞指令” |
| 启动 | 分阶段 TimerAction，探索等待 bootstrap spin 完成；脚本 smoke 会检查话题 | 本项目只以 bt_navigator active 为主要 ready 条件，没有独立验证 map/TF/控制入口/扫描新鲜度。固定 sleep 也不是严格 ready 保证 |
| 测试 | 有 fixed-goal planner×controller 矩阵，另有 smoke 脚本 | 本项目主要用探索覆盖率、运行退出码和零散短测；缺少起步方向/路径推进的自动失败判据 |

参考：
- [配置选择和规划器改写](https://github.com/darshmenon/rosnav/blob/main/src/rosnav_bot/launch/_common.py)
- [Humble MPPI 参数](https://github.com/darshmenon/rosnav/blob/main/src/rosnav_bot/config/nav2_params_mppi.yaml)
- [Jazzy 参数](https://github.com/darshmenon/rosnav/blob/main/src/rosnav_bot/config/nav2_params_jazzy.yaml)
- [EKF](https://github.com/darshmenon/rosnav/blob/main/src/rosnav_bot/config/ekf.yaml)
- [激光过滤](https://github.com/darshmenon/rosnav/blob/main/src/rosnav_bot/config/laser_filters.yaml)
- [启动](https://github.com/darshmenon/rosnav/blob/main/src/rosnav_bot/launch/slam_nav.launch.py)
- [builtin 探索器](https://github.com/darshmenon/rosnav/blob/main/src/rosnav_bot/scripts/frontier_explorer.py)
- [固定目标矩阵](https://github.com/darshmenon/rosnav/blob/main/src/rosnav_bot/scripts/benchmark_nav_matrix.sh)

## 参考工程也不能盲信

1. Jazzy 的 controller 参数选择会被发行版分支覆盖；标成 rpp/dwb 的测试可能实际使用 MPPI，源码已对此警告。
2. `apply_planner(..., navfn)` 直接返回原文件，但 Jazzy 文件实际上包含 SmacHybrid。
   因此名为 navfn 的默认标签不保证真是 Navfn；必须核对加载插件。
3. `nav2_params_jazzy.yaml` 的 costmap `depth` 段存在重复键，后面的 LaserScan/data range 覆盖
   前面的 PointCloud2 参数。不能直接复制整份 YAML，须做重复键与实际加载参数检查。
4. `.github/workflows/ci.yml` 在 Humble 容器构建并选择 `test_frontier`，没有运行所有
   Gazebo 导航组合。`smoke_ci.sh` 文件存在不代表已由该 CI 自动执行。
5. 探索行为树和启动源码注释明确记录 TF 卡住、恢复碰撞、扫描门控导致无地图等问题。
   它是有诊断与试验积累的参考工程，不是已消除集成故障的产品级基线。

## 本项目已证实的两个不同问题

### 旧 DWB：局部目标函数允许停滞

`20261029-011304-481-domain86/navigation-evaluation/state-001.json`：440条合法候选，
原地慢转得分31.9435，最佳vx>.05的候选32.4505，路径项增加超过目标项改善。
仅说明该周期存在局部停滞偏好，不能解释随后所有运行，且后来路径权重已经改变。

### 新 MPPI：起步方向违反明显的前进路径

`20261029-164118-948-domain86/navigation-snapshot/state-001.json`：
- 全局路径从(-.674,.011)沿正x走向(.659,.124)，前段yaw=0。
- map→odom=单位变换，机器人yaw接近0。
- cmd_nav vx约-.02146，预测轨迹末端x约-.959，而机器人x约-.674。

`164703` 的快照额外捕获 MPPI 处理后的 `transformed_global_plan`，依然朝正x，
禁止倒车时输出近零，并未朝目标推进。这排除了“只是RViz旧线”“车头朝向造成视觉误解”
和简单的 map/odom 坐标整体反向解释。它没有排除优化器内部目标/代价或依赖构建的问题。

对照试验还发现：关闭 PathAlign/PathAngle 后仍反向；恢复噪声更新/弱前进偏好没有修复；
调整采样与迭代后仍反向。因此不应继续把它说成普通权重敏感，或通过禁止倒车宣称解决。

## 应建立的最小验收顺序

1. 执行器：小幅正线速度、正角速度与 odom/TF/真值方向一致，确认时间和坐标约定。
2. 控制器：固定空闲代价地图，不启动探索/在线SLAM，用直接 FollowPath 测正向直线、曲线、
   原地转向、倒车和方向切换。先测试 MPPI 处理后的路径与首段指令符号，失败立即停止。
3. 规划+控制：保存地图/固定目标，测绕墙必须先远离目标、窄门与死胡同，记录路径弧长推进量，
   不只记录到目标的直线距离。
4. 在线SLAM：记录 map→odom 修正与全局/局部障碍的一致性，不把修正量直接称为定位误差。
5. 探索与恢复：最后接入前沿选择，分别统计导航成功、短期恢复、最终脱困、目标无效和起点无效。

每轮固化：controller/planner实际插件、参数dump、版本/构建来源、模型/场景/配置hash、
所有输入输出时间戳、优化局部路径、分项评分、三个速度话题与静止/进展判据。
配置单测只保证字段和几何配套，不能代替上述闭环运动验收。

官方约束参考：
- [MPPI预测范围、时间步和路径方向参数](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/mppi_controller/configuring_mppic/)
- [Nav2优化器单测](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_mppi_controller/test/optimizer_unit_tests.cpp)

## 本次整理

停止未通过的实验会话；保留MPPI入口与证据；默认恢复DWB，MPPI须显式选择。
生成的MPPI YAML已回到生成器输出，避免第五次试验直接改YAML后留下与生成器不一致的配置。
未改机器人几何、SLAM、膨胀或关闭碰撞检查；未声称DWB已经可靠，也未宣称MPPI故障修复。
