# 双机器人协作救援

更新：2026-11-10。默认robot1使用根命名空间，robot2使用/robot2；共享地图和时钟，各自保留导航与控制权。

## 阶段与职责

第一任务先完成建图及全部必需目标定位，S1成功后才进入C1。C1不承担首次目标搜寻，缺失目标会明确失败，不能继续以未知坐标导航。

C1树位于src/edgeai_mission/config/tasks/c1.xml：

1. 固定SLAM地图及关键帧目标，保存静态地图，停止SLAM，启动两机AMCL。
2. 查询共享map_server及两机AMCL/Nav2真实生命周期，全部active后进入定位；就绪/定位可以有界重试，不重复地图交接。
3. 生成两个停车位并检查完整车身净空、分离距离和各自全局规划。
4. 并发派发两机导航；分别保留执行状态，一台先到或失败不重复发送另一台仍健康的动作。
5. 用共同新鲜采样检查距离、定位与停车；两机同时在0.70m范围内连续10仿真秒才成功。
6. 失败时优先保持健康机器人的停车位，无法形成组合才取消两机并重新选位；共同停留条件不满足也可有界重选。

底层计算与ROS接口在cooperative_rescue.py，单位适配在task_units.py。默认流程由任务树调用单位操作，旧execute()保留供独立兼容入口，不能据此推断生产编排。

## 参数与使用

配置位于src/edgeai_navigation/config/cooperative_rescue.yaml。主要入口为初始相对位姿、AMCL收敛门槛、radius_m、dwell_s、position_margin_m、min_separation_m、navigation_retries和rescue_repair_rounds。次数配置表示额外重试，运行快照记录包含首次的尝试上限。

~~~bash
bash edgeai.sh mission
# C1专项，使用已有运行配置及真值场景；不等于比赛独立Part2
bash edgeai.sh cooperative --params /path/to/nav2-runtime.yaml
~~~

普通运动、指定朝向和救援到位策略均保留共享Nav2恢复入口。任务层只在Action失败或明确超时后重试，外部取消/控制权丢失不能被重试覆盖。候选预规划不产生运动；起点占据允许实际导航树尝试退出恢复。

## 证据和边界

当前轮次摘要见 [项目状态](project-status.md)。输出为cooperative-events.jsonl、cooperative-result.json、c1-task.log和task-unit-events.jsonl。

最终车身能容纳不代表两条行驶路线一定互不阻挡；未实现另一车未来停车位置的预留和窄通道预约。估计距离与任务成功需另行对照真值及碰撞证据。SLAM/目标如何一致修正见 [地图绑定](target-map-anchoring.md)。

[11月7日实施与修复历史](history/cooperative-rescue-2026-11-07.md)保留旧轮次描述，当前职责以本文及 [任务流程](task-workflow.md) 为准。
