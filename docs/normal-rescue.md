# S1救援与目标补搜

更新：2026-11-10。默认中期配置为src/edgeai_navigation/config/midterm_single_rescue.yaml；
normal_rescue.yaml是旧开发场景对照配置，不能把其半径/停留参数当作中期默认。

第一任务先用tasks/find_targets.xml检查所有必需目标并补搜，S1树仍保留目标丢失后的单目标检查/补搜后备。底层复用normal_rescue.py、target_research.py、frustum_search.py和suspect_search.py。
候选计算不移动机器人；位置导航和转圈通过现有控制权与Nav2 Action执行，疑似候选优先完成当前观察方向，再继续视锥候选。已识别目标附近的疑似物会被排除。

## S1执行

任务树位于src/edgeai_mission/config/tasks/s1.xml：接管 → 选目标 → 选接近位 → 导航 → 独立条件检查 → 必要末端修正 → 连续停留 → 记录结果。
导航和任务两层重试分别有界，保留目标ID与失败历史；目标更新可重算接近位，但不重置任务总预算。

当前默认要求目标水平距离不超过0.50m、停车、朝向符合配置，并连续10仿真秒。比赛圆内判断依据base_footprint到目标地面点的距离；任务额外的朝向和停车要求是实现约束，不是文档要求整个车身入圈。

接近点由目标最新地图位置与表面法线推导；候选角度/距离受地图完整矩形轮廓和实际规划检查。无路径、终点占据/越界可换候选；起点占据交给共享Nav2恢复。导航到点不直接报告救援成功。

## 调整入口

| 需求 | midterm_single_rescue.yaml |
|---|---|
| 视锥、疑似目标、可观测距离 | search_frustum、search_suspects |
| 观察次数和时间 | search_max_positions、search_timeout_s、search_navigation_timeout_s、search_spin_timeout_s |
| 末端小步修正 | alignment_* |
| 距离、朝向、连续停留 | target_radius_m、yaw_tolerance_rad、dwell_s |
| 任务/导航重试和预算 | rescue_max_retries、navigation_max_retries、wall_timeout_s |

第一任务将同一搜寻配置用于指定ID和类型；默认双机依次检查ID1/normal与ID2/cooperative，只补搜缺失项。所有目标共用find_targets任务墙钟预算，每个目标另受现有搜寻仿真时间和候选次数限制。S1和C1本身有独立预算，这尚不是比赛统一13分钟计时。

输出包括targets.json、target-search-result.json、rescue-result.json、target-search.log、rescue.log及task-unit-events.jsonl。地图元数据与目标导出由save_map阶段保存；最终静态交接前会再次固定并保存，详见 [目标地图绑定](target-map-anchoring.md)。

[旧实现与输出字段说明](history/normal-rescue-before-task-trees.md)仅作历史参考。[逐点导航规则](navigation-goal-policy.md)、[视锥搜索](frustum-search.md)和[疑似目标](suspected-target-search.md)提供模块说明。
