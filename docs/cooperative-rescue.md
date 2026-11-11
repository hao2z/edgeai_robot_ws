# 双机器人协作救援

更新：2026-11-11。默认robot1使用根命名空间，robot2使用/robot2；共享地图和时钟，各自保留导航与控制权。

## 阶段与职责

第一任务先完成建图及全部必需目标定位，S1成功后才进入C1。C1不承担首次目标搜寻，缺失目标会明确失败，不能继续以未知坐标导航。

C1树位于src/edgeai_mission/config/tasks/c1.xml：

1. 固定SLAM地图及关键帧目标，保存静态地图，停止SLAM，启动两机AMCL。
2. 查询共享map_server及两机AMCL/Nav2真实生命周期，全部active后进入定位；就绪/定位可以有界重试，不重复地图交接。
3. 生成两个停车位并检查完整车身净空、分离距离和各自全局规划。预规划只请求路径，再检查每条路径的车身扫掠是否避开另一台车的预计停车车身。已停稳的车使用实测位姿。
4. 并发派发两机导航；分别保留执行状态，一台先到或失败不重复发送另一台仍健康的动作。
5. 用共同新鲜采样检查距离、朝向、定位与停车；两机同时在0.70m范围内，朝向各自位置到目标坐标连线的误差不超过0.25rad，连续10仿真秒才成功。朝向只根据地图坐标计算，不要求相机再次发现目标。

停车候选从生成时就朝向目标，按该姿态检查完整车身和预规划；标记法线仅用于生成候选扇区。C1树到位后采用与S1相同的“检查→单次修正→重新检查”编排，复用S1的修正几何：圈内转向、圈外先朝接近点转向再最多前进3 cm，一次只修正一台，随后重新检查两机。修正累计上限40次、每轮60仿真秒；失败由原任务重试重选停车位。原生动作保留碰撞预测和取消确认；真值朝向核验同步按world坐标连线计算。参数为`alignment_*`。

短暂TF或时钟回调错序只等待补齐，且不推进停留计时；最后有效采样超过`max_data_age_s`或实际条件不合格才清零。等待反馈记录原因、样本年龄及`dwell_retained`。成功后附加两车共同窗口的真值采样核验与误差，详见[真值诊断](simulation-truth.md)。
6. 失败时优先保持健康机器人的停车位，无法形成组合才取消两机并重新选位；共同停留条件不满足也可有界重选。

底层计算与ROS接口在cooperative_rescue.py，单位适配在task_units.py。默认流程由任务树调用单位操作，旧execute()保留供独立兼容入口，不能据此推断生产编排。

## 参数与使用

配置位于src/edgeai_navigation/config/cooperative_rescue.yaml。主要入口为初始相对位姿、AMCL收敛门槛、radius_m、dwell_s、position_margin_m、min_separation_m、navigation_retries和rescue_repair_rounds。次数配置表示额外重试，运行快照记录包含首次的尝试上限。

`parking_radii_m`控制每个停车点距目标的候选距离（默认0.50/0.55/0.60m），超过`radius_m-position_margin_m`的距离被排除；当前上限0.55m，因此0.60m候选不启用。`min_separation_m`控制两停车点之间的最小距离。`parked_peer_clearance_m`控制预规划路径与对方停车车身之间的净空。`yaw_tolerance_rad`独立控制C1任务层共同停留的朝向检查；Nav2救援到点检查器目前仍共用单救援配置中的角度容差。

~~~bash
bash edgeai.sh mission
# C1专项，使用已有运行配置及真值场景；不等于比赛独立Part2
bash edgeai.sh cooperative --params /path/to/nav2-runtime.yaml
~~~

普通运动、指定朝向和救援到位策略均保留共享Nav2恢复入口。候选预规划不产生运动；未返回路径的停车对不能通过筛选。预规划205（起点占据）不再反复重选终点：C1任务树调用`recover_preplan_start`，通过现有Nav2 Spin、BackUp、DriveOnHeading和受控重叠退出行为尝试恢复，随后重新预规划。默认先正反转向；角度、位移、速度、动作超时和最小实际进展都在`cooperative_rescue.yaml`的`preplan_recovery`配置。每车全任务保留动作尝试历史，无实际进展不算恢复成功，所有尝试失败则有界退出，不清除真实障碍制造净空。该阶段使用行为服务器现有碰撞检查，不等于已确认物理可通行。实际导航期间的恢复仍由导航树触发。`PLAN_BLOCKED_BY_PARKED_PEER`记录路径与预计停车车身的冲突。该筛选不会在假设的停车障碍下另找绕路。

定位就绪后，C1每0.1仿真秒发布两车带时间戳的地图位姿。各自的全局和局部代价地图在膨胀层之前加入对方矩形车身。新位姿到达时重算旧区域，过期后撤下矩形，静态地图和激光障碍层继续工作。开关、边距和数据时限在`peer_costmap`。导航树按2Hz墙钟频率检查剩余路径；发现失效即重规划，有效路径的到期阈值为10秒。这是对实时占据的响应，不能保证两车在窄通道会主动让行。

## 协作定位的机器人回波过滤

两台AMCL分别使用`/scan_localization`、`/robot2/scan_localization`，由`fleet_scan_filter.py --localization`双向过滤。按扫描时间戳查询对方车身到本机激光的TF，删除终点落在对方矩形车身及0.05m余量内的回波（置NaN，不虚构身后空闲空间）。导航代价地图继续使用原始`scan`；建图的`scan_slam`仍使用原有粗略相对位姿过滤，与此模式独立。

参数在`src/edgeai_navigation/config/fleet_scan_filter.yaml`的`localization`段；`enabled: false`可恢复原始AMCL输入。只有两台AMCL位置/角度不确定度均达标、两台`map→odom`足够新鲜且扫描时刻TF存在时才过滤。初始未收敛、TF缺失/过旧等情况原样转发，保证定位能启动；不使用最新TF替代历史扫描TF。过滤依据定位估计，协方差小不等于真值准确，不能保证消除全部相互干扰。

运行时两机目录分别生成`localization-scan-filter.yaml`、`amcl-localization.yaml`，后者记录实际AMCL输入。`cooperative-nav.log`中的`LOCALIZATION_SCAN_FILTER`每5秒记录过滤帧数、删除点数、扫描/TF年龄和降级原因。2026-11-10已验证双向ROS过滤、位置更新、启动/不确定度/陈旧数据降级及开关接线，未以新Gazebo协作测试验证定位改善幅度。

## 证据和边界

当前轮次摘要见 [项目状态](project-status.md)。输出为cooperative-events.jsonl、cooperative-result.json、c1-task.log和task-unit-events.jsonl。

最终车身能容纳不代表两条行驶路线一定互不阻挡；未实现另一车未来停车位置的预留和窄通道预约。估计距离与任务成功需另行对照真值及碰撞证据。SLAM/目标如何一致修正见 [地图绑定](target-map-anchoring.md)。

[11月7日实施与修复历史](history/cooperative-rescue-2026-11-07.md)保留旧轮次描述，当前职责以本文及 [任务流程](task-workflow.md) 为准。
