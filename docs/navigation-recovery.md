# 导航恢复

行为树入口：`src/edgeai_navigation/config/behavior_trees/explore_recovery.xml`。
常规BackUp、DriveOnHeading、Spin仍使用原生Nav2行为服务器；当前外层最多恢复5次。
单个恢复动作失败后保留历史并选择下一个候选；每轮最多8次选择，耗尽才结束，不再用Wait兜底。

## 尝试历史

不再因后退几厘米或旋转少许就清空历史。记录动作、幅度和起始位姿；
同一动作在距已尝试位置0.20m、航向差0.35rad内避免重复，最多保留64条。
移动到不同位置可以再次使用相同动作，但旧记录仍保留。
目标显著变化（XY超过0.10m或yaw超过0.35rad）重置本轮记录。

MatchedRPP在实际速度非零且原生控制计算成功的情况下，
连续跟踪至少2仿真秒且净平移至少0.30m，才生成新的progress_epoch。
控制器调用间隔超过0.5仿真秒、输出/实测停止或计算失败均打断连续计数。
选择器下次收到当前目标、对应控制器实例的新epoch后才清空历史。
动作服务器返回成功本身不清空历史。

## RPP衔接预判

MatchedRPP以最多约5Hz发布 `/navigation/rpp_tracking`：状态、前视点、目标、实例和progress_epoch。
状态切换立即报告。恢复选择器只采用2仿真秒内、匹配目标和控制器实例的数据。
使用候选结束位姿到该前视点的纯追踪曲率，检查最长0.12m的短弧车身扫掠；
圆弧通过的候选优先，然后比较净空。缺少匹配数据时只使用净空评分。

该预判没有完整调用未来位姿下的RPP，也不保证恢复后新路径仍相同。
它是候选排序线索；真正的恢复确认来自后续实测跟踪。
无新增图像、激光或点云订阅，只有低频小型状态消息；几何候选检查仅在恢复时运行。

## 占据退出恢复

实际车身与带padding轮廓由同一份运行时局部地图配置派生，避免重复填写尺寸。
如果带padding轮廓覆盖已知障碍格，包括实际车身也被已知障碍格占据时，可以选择专用
`escape_occupied`行为。它复用Nav2 TimedBehavior和DriveOnHeading动作接口，
经现有速度平滑器、碰撞监视器执行，仲裁器也记录其状态和实际位移。

- 只允许前后短移，上限0.10m、速度上限0.03m/s，不允许原地转出。
- 实际车身和带边距轮廓的重叠面积都必须单调不增加；不得进入新的障碍格。无未知格且在地图内。
- 边距与已知障碍的重叠面积必须单调不增加，不得碰到初始范围外的新障碍格，终点须减小重叠或完全释放padding；允许分步退出。
- 执行中每周期重新检查；地图/TF超过0.35仿真秒、出现新障碍或重叠加深即停车。
- 1.5墙钟秒没有实测平移就终止；动作还有仿真和墙钟总时限。
- 下游碰撞监视器始终有效，可能拒绝退出并使动作超时；不通过关闭它强制移动。

实际车身覆盖已知障碍也会尝试退出；未知占据、地图外或缺少物理轮廓时明确报告恢复失败。
只有整段动作可验证且终点减小重叠或完全空闲才执行；物理接触导致无法移动时仍会无进展停车。

关键日志：RECOVERY_SELECTED（含历史数量、短弧结果）、RECOVERY_HISTORY_RESET、
OCCUPIED_ESCAPE_STARTED/COMPLETED/REJECTED/STOPPED。
短时观察：`.\Diagnose-Controller.ps1 -Duration 20`。

## 官方与社区参考

Jazzy标准树采用规划/控制上下文清图重试，以及RoundRobin轮换清图、旋转、等待、后退：
[官方XML](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml)。
适合常规恢复，但动作本身不保证RPP能够重新接入路径。

避免同位置连续后退可参考[BackupWithReset改进PR #6635](https://github.com/ros-navigation/navigation2/pull/6635)。
查阅时PR页面仍显示Open，不能当作本机Jazzy已安装功能。
按错误码配置不同恢复序列也有[社区提案 #6573](https://github.com/ros-navigation/navigation2/issues/6573)，不是已验证的通用恢复插件。

占据起点导致BackUp拒绝的问题也见[官方社区讨论 #3186](https://github.com/ros-navigation/navigation2/issues/3186)。
该讨论针对keepout过滤区；不能直接推广成允许穿过真实障碍。

测试覆盖历史保留/实测重置、RPP短弧、未知占据拒绝和实际车身占据退出，以及隔离ROS行为动作的
边距退出成功、拒绝和无位移停车。整机通过情况以每次实际测试记录为准。

2026-11-06验证：16个C++用例、隔离ROS行为动作测试及配置检查通过。
整机会话`20261106-200804-rpp-narrow_spaces`的进入通道、短步和反向目标均成功。
反向目标仍耗时13.621仿真秒、路程2.089m，说明成功不等同于路线最优；
日志观察到RPP短弧参与选择，以及实测持续跟踪后的历史重置。

2026-11-06追加：支持实际车身占据和接触退出、分步减小重叠；移除Wait兜底。独立ROS域验证，不操控用户当前会话。
本次验证：18个C++用例、独立域ROS动作测试（含车身占据、部分退出、未知拒绝、无位移停止）和导航点策略测试通过。尚未重跑整机碰撞场景。

## 恢复筛选与原生执行器坐标系一致性（2026-11-07）

会话 `20261107-020117-722-domain86` 的局部代价地图 global_frame=map，但行为服务器 local_frame=odom。SelectRecovery 按收到的代价地图 header.frame_id 查询机器人位姿；原生 BackUp/DriveOnHeading/Spin 则按 behavior_server.local_frame 取位姿，再将无 header 的 Pose2D 直接传入局部地图碰撞检查，不做跨帧转换。因此两层实际检查了不同位置，这是候选通过、执行立即报碰撞的一处明确错误。

生成器现将 behavior_server.local_frame 与局部地图 global_frame 联动，同时将 global_frame 与全局地图联动。RPP+use_path_orientations=true 时三者对应 map；局部地图仍用 odom 的控制器保留 odom。robot2 配置按同样规则派生，odom 帧带机器人前缀。碰撞监视器的 odom_frame_id 用于运动补偿，不能因此改成 map。

其他差异：选择器检查整个动作，平移步长≤1 cm、旋转≤0.025 rad；原生执行器按10 Hz、最多2 s预测，并按剩余动作距离/转角截断。选择器对多边形内部与边界格都检查，原生使用轮廓边栅格检查；两者都使用发布的局部 padded footprint，自定义已知占据退出另同时检查实际车身。两层订阅同一地图话题，但各自取得快照，可能相差一个更新周期，不能将所有拒绝都归结为坐标错误。

新增回归构造 map/odom 相差1 m的场景：同一恢复在 map 位姿可行，错误使用 odom 数值会被原生碰撞检查拒绝；统一位姿帧后通过。此测试验证坐标系差异机制，不是历史墙角原始代价地图的逐格回放。修改后需重启生效。
