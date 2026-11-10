> 历史快照；当前入口与结论见 [项目状态](../project-status.md)。

# 建图、目标清单与一次普通救援

未找到目标时支持 [RGB-D 疑似目标搜寻](../suspected-target-search.md)，与视锥候选共用原有搜寻预算和导航仲裁。

终点规则按每个导航请求绑定，见 [逐点导航要求](../navigation-goal-policy.md)。普通前沿、手动普通导航、再搜寻点使用 `general_goal_checker`（`PositionGoalChecker`），只检查原有 XY 容差；NavFn 使用路径末段朝向。指定朝向的普通点使用 `heading_goal_checker`。最终救援点使用 `rescue_goal_checker`（`StoppedGoalChecker`），检查位置、朝向和停车，随后独立验证连续停留。仲裁器给每个请求绑定固定行为树和规划器，无需切换全局参数或选择器话题。

再搜寻候选点：在机器人所在的已知空闲格连通区域中筛选满足 `search_clearance_m` 整圆净空的观察终点；当前位置和途中格子不要求整圆净空，避免靠近障碍物时直接取消整个搜寻。连通筛选只生成候选，实际车身通过性仍由 Nav2 规划和控制检查。`RESCUE_SEARCH_POSITIONS` 记录起始坐标、净空要求和候选点数量/坐标。

中期配置启用 `approach_selection`：固定目标 ID，以最新目标位置生成法线两侧的救援停车候选，先检查地图中的完整矩形车身净空，再通过实际 `GridBasedHeading` 规划器预检。候选半径与到位容差之和不得超过估计目标的救援半径；这不是相对真实标记的位置保证。默认角度 0/±30/±60°，额外距离 0.30/0.40 m，每轮每轮最多检查 12 个候选，共用原导航与墙钟预算。

错误处理：204 终点越界、206 终点占据和 208 无路径时换候选；208 仅表示此位姿当前无路，不能据此断言终点本身占据。202 TF 和 207 规划超时有界等待并重试原候选；205 起点占据交给导航树执行原有退出恢复。救援专用树不因规划无路反复执行原地恢复，仍保留控制器失败和起点占据恢复，以及一次规划清图重试。未知错误、外部取消与接管不盲目换点。候选耗尽明确失败，并提示检查目标估计和地图一致性。

`RESCUE_APPROACH_SELECTED/REJECTED`、`RESCUE_NAV_FAILURE` 提供简短原因；`rescue-result.json` 的 `approach_attempts` 保存候选、规划错误码和消息。导航阶段更新目标时保留已选候选相对目标的角度与距离；停留阶段也刷新目标估计并重置不再有效的连续停留。关键帧锚定尚未接入，本功能缓解不可达停车位，不修复 SLAM 历史修正不一致。

候选数量不是完成观察的数量。每个点只导航一次，再连续转一圈观察；明确导航失败或单点导航超时后，复用救援的 `navigation_max_retries` 及新鲜 TF/控制槽等待，重试耗尽才跳点。所有尝试仍共用搜寻总预算；外部取消或操作员接管不重试。`SEARCH_VIEW_RESULT` 记录导航结果，`SEARCH_SPIN_RESULT` 记录旋转结果，`SEARCH_POSITION_SKIPPED` 明确记录导航重试耗尽后的跳点。目标优先读取 `/targets/estimates`；文件只作后备，短暂缺失或不完整不直接终止任务，等待仍受总预算约束。

启动方式保持不变：

```bash
bash scripts/simulation/start-autonomous-exploration.sh --controller dwb --arena competition --duration 300
```

Windows 仍使用 `Start-Autonomous-Exploration.ps1`。默认在探索器发出自然完成事件后执行一次普通救援。探索超时、黑名单耗尽或进程异常不触发这一自动交接。`Duration` 是探索阶段墙钟预算；救援有独立预算。设置 `EDGEAI_RESCUE=false` 或配置中的 `enabled: false` 可关闭救援。

## 输出对齐

规则书 V0 §5.4 要求地图及分辨率、原点、坐标系元数据，以及包含 ID、类型、地图位置与朝向的唯一目标清单。比赛最终文件编码规范尚未明确；本实现按这些字段提供本地交换格式：

- `map.yaml` / `map.pgm`：Nav2 地图保存器输出。
- `task1-output.json`：地图路径、分辨率、原点、`frame_id: map`、目标清单路径。
- `target-list.csv`：`id,type,x,y,yaw`，每个 ID 一行，导出已有效定位且平面朝向有效的目标，不要求多帧确认。
- `handover-targets.json`：建图结束时冻结的检测记录，保留交接证据；救援优先读取持续更新的 `targets.json`，仅在该文件不存在时使用交接快照。不要求多帧确认。
- `targets.json`：持续更新的感知结果；保留原 `position: [x,y,z]`，增加 `type`、`orientation: [qx,qy,qz,qw]`、`x/y/z/yaw`、`confirmed`。

四元数表示 OpenCV 标记坐标轴到 map 的旋转，标记 +Z 为面向观察者的外法线。平面 `yaw` 是外法线在地图 XY 平面的方向，不是四元数的普通 ZYX yaw。水平标记的平面外法线无效，`yaw` 输出 null，不能用于本版墙面救援推导。位置和朝向都先在图像时刻变换到 odom，再随当前 map→odom 更新；朝向使用四元数主特征向量融合，避免 q/-q 抵消。融合不等于误差保证。

ID→类型表在 `src/edgeai_perception/config/aruco_targets.yaml` 的 `target_types`，目前为开发映射 `1/2/3: normal, 10: cooperative`，未知 ID 保留 unknown。感知和救援执行器均不读取仿真目标真值坐标。

## 常改配置

`src/edgeai_navigation/config/normal_rescue.yaml`：

| 参数 | 含义 |
| --- | --- |
| `goal_source` | `detected_target` 使用冻结感知结果；`map_pose` 使用显式地图位姿 |
| `require_confirmation` | 默认 false，一次有效定位就允许救援；确认状态仅用于质量记录 |
| `target_id` | -1 选已定位普通目标中观测数最多者，再比较 spread；也可固定 ID |
| `map_pose` | 显式 `[x,y,yaw]`，米/弧度；此模式用于指定目标位姿调试 |
| `normal_offset_m` | 沿标记朝外法线偏移距离，默认 0.70 m；最终朝向标记 |
| `position_tolerance_m` | 默认 0.10 m；同时用于救援 Nav2 checker 和独立停留判定 |
| `yaw_tolerance_rad` | 默认 0.25 rad；按指定朝向到位，规则 V0 本身不要求最终朝向 |
| `dwell_s` | 连续停留，默认 3 秒仿真时间 |
| `stopped_*` | 停车速度门限；移动、越界、姿态不合或数据过期重置计时 |
| `navigation_timeout_s` / `wall_timeout_s` | 仿真时间导航预算 / 墙钟兜底预算 |
| `navigation_max_retries` | 导航明确 ABORTED 后的额外重试次数，默认 2；0 可关闭 |
| `navigation_retry_delay_s` / `navigation_retry_wait_s` | 重试前最短等待 / 等待控制槽与数据就绪的上限，默认 1 / 5 墙钟秒 |
| `retry_tf_max_age_s` | 重试前配对 TF/里程计的仿真时间年龄上限，默认 0.15 秒 |

这些是开发设定，不是已公布的正式比赛尺寸与误差阈值。检测目标定位误差会传递到救援位姿；本版依据 SLAM/里程计估计判定，不能替代独立仿真真值或裁判验收。显式 map_pose 模式不算由感知推导救援目标。

Linux 可用 `EDGEAI_RESCUE_CONFIG=/absolute/path/config.yaml` 指定另一份配置。

## 状态与控制权

`src/edgeai_navigation/scripts/normal_rescue.py` 实现交接和执行；`rescue_policy.py` 实现目标选择、几何推导和连续停留条件。

1. 探索自然完成，停止自动观察调度，保存地图和冻结目标清单。
2. 请求仲裁层 MANUAL，等待旧导航终止、探索暂停确认与 IDLE。
   若缺少符合 ID/类型要求的有效定位且启用 `search_enabled`，先执行已知地图目标再搜寻，再进入救援导航。
3. 通过 `/navigation/rescue_to_pose` 提交唯一任务，仲裁器为该请求绑定 `GridBasedHeading`（遵守目标 yaw）和 `rescue_goal_checker`。原有 Nav2 恢复逻辑继续工作，脚本不直接发布速度。
4. Nav2 成功后，在新鲜 odom 时间戳查询 map→base_footprint，独立检查距离、朝向和停车；连续满足后记录完成。
5. 成功或失败后请求 STOPPED，不自动恢复探索。用户改变控制序号会中止本次任务，不覆盖用户新指令。

运行时配置提供三种检查器和两个 NavFn 实例；行为树保留原恢复结构，在请求派发前绑定对应插件 ID。`authority-events.jsonl` 记录 `arrival_policy` 和实际行为树路径。

日志：`rescue.log`、实时 `events.log`。结果：`rescue-result.json` 包含目标、参与机器人、完成状态/时刻、停留时间、最终误差和 `navigation_attempts`，记录每次导航的接收状态、终态、错误码和仿真时间。一次救援可包含有上限的导航重试，不包含第二机器人或协作救援。

目标再搜寻由 `target_research.py` 执行，`search_policy.py` 从与机器人连通的已知空闲栅格中选取分散观察位置；未知格、障碍和净空不足的位置不参与候选。默认最多 6 个位置、位置间隔至少 1 m。每个位置只通过公开 `/navigate_to_pose` 导航一次，保留当前朝向作为终点朝向；成功后调用原生 `/navigation_backend/spin` 连续旋转 2π，不再分别派发四个方向。整个流程持有 MANUAL 控制权且探索暂停，持续检查控制序号；Spin 保留 Nav2 碰撞检查，旋转拒绝或失败后尝试下一位置。导航失败仍按配置有限重试。

`search_spin_timeout_s: 30.0` 是每次旋转的仿真时间上限，不是转速；旧的 `search_observation_s` 已不再使用。转速由实际导航配置的 `behavior_server.max_rotational_vel`、`min_rotational_vel`、`rotational_acc_lim` 决定，并受速度平滑器与机器人速度限制约束。找到目标或预算耗尽时取消 Spin，等待动作终态后再转入救援或退出。`search_attempts` 每个位置/导航尝试记录一次，内含 `spin` 的接收状态、终态、错误码、起止时间及取消原因。

默认启用再搜寻，`search_timeout_s: 120` 限制整个搜寻的仿真时间，`search_navigation_timeout_s: 20` 限制单次导航，且搜寻计入救援原有墙钟预算。`search_clearance_m` 默认 0.40 m，启动器会确保它不小于当前 Nav2 车身外接半径加 0.05 m。搜寻期间持续检查感知结果，找到合格目标即取消当前搜寻导航，确认终态和仲裁空闲后转入救援。无可用位置、时间耗尽或观察位置用尽时报告失败，不恢复前沿探索、不读取仿真目标真值。设置 `search_enabled: false` 可关闭；显式 `map_pose` 模式不需要搜寻。记录 `RESCUE_SEARCH_STARTED/VIEW/FOUND`，结果中的 `search_attempts` 保存观察位姿及导航终态。该策略改善相机视角覆盖，不保证发现被遮挡或地图中不可达的标志。

导航收到明确 ABORTED 结果后，默认额外重试最多 2 次，保持同一目标 ID，并按最新位置与法向重新计算偏移点。每次先等待至少 1 秒墙钟时间，并确认原动作已经终止、仲裁处于 MANUAL/IDLE、探索已暂停、控制序号未改变且有 0.15 秒以内的配对 TF/里程计，再提交导航请求。等待恢复最多 5 秒墙钟时间。所有尝试共用原导航仿真预算及救援墙钟预算，不会因重试重新计时；原生行为树恢复仍在每次导航内部执行。拒绝接单、外部取消、未知终态或操作员切换控制权不自动重试。持续失败、数据长期不恢复或预算耗尽才结束任务。

导航途中每 `target_update_interval_s`（默认 1 墙钟秒）读取最新定位；偏移点移动达到 `target_update_distance_m`（默认 0.10 m）或朝向变化达到 `target_update_yaw_rad`（默认 0.25 rad）时，主动取消旧导航，确认旧动作终态和仲裁空闲后再派发。更新不消耗失败重试次数，但仍受原总预算限制。每次派发的 `goal` 写入 `navigation_attempts`。不自动切换目标 ID；最新文件存在但目标无有效定位时报告失败，不回退到冻结坐标。此更新采用当前感知融合结果，不要求重新看见标志，也不改变现有终点朝向和停留判定。

2026-10-30：按用户要求，救援和清单导出均不要求三帧确认。图像帧、PnP 重投影、正深度、距离和 TF 检查仍保留；没有有效定位的 ID 不会生成救援目标。检测处理频率提高到 15 Hz，确认标签继续保留。

感知诊断：`target-diagnostics.json` 与 `perception.log` 的 `TARGET_DIAGNOSTICS` 每 5 秒仿真时间记录实际处理频率、处理耗时、无 ID 帧、已解码 ID、定位接受/拒绝和 PnP 候选拒绝原因。`TARGET_DECODED` 表示解码成功，`TARGET_FOUND` 才表示通过定位检查。候选拒绝计数可能包含同一次 PnP 的多个候选，不等同于丢失标记数量。

漏检诊断（2026-10-30）：对同一组真实相机静态帧回放，OpenCV 默认 `minMarkerDistanceRate=0.05` 无解码，改为 0.02 后 frame-04/05/07 恢复 ID 3。扩大阈值窗口、翻转、增强对比度未恢复。配置 `aruco_targets.yaml` 新增此参数，避免把嵌套轮廓误抑制当成算力不足；同帧相同 ID 只接受一次，不能重复计入确认。另修复救援停留时最新 odom 先于 TF 到达造成的反复重置，改用 0.5 秒内最新可配对样本，过期仍重置。


## 最后位姿修正原型

救援通过 `/navigation/rescue_to_pose` 执行完整位姿导航，使用原生Shim+RPP。Shim启用终点原地转向，起步使用路径中间点的车身姿态；RPP内部转向关闭。中期配置 `alignment_enabled: true` 仅控制导航成功后、独立检查不达标时的末端修复。
中期 detected_target 模式按 base_footprint 到最新估计目标中心的距离不超过 target_radius_m（0.50 m）判定位置合格，停车偏移点只负责引导航接近。map_pose 模式或未配置救援半径时保留精确位置容差。任务位置不合格时才先用Spin对准接近点（单次最多0.6rad），再用DriveOnHeading前进最多3cm、速度0.03m/s。
位置已满足则只修正最终朝向；位置与朝向均满足后才计连续停留。每个动作结束后等待新测量，不能复用动作前的TF/里程计。
停留阶段继续刷新同一目标估计；圈内不追逐偏移停车点。沿用配置的朝向和停车条件，全部满足才累计停留，出圈或数据过期即重置。保持原任务身份和MANUAL控制权检查。
最多20个旋转/前进行为、60仿真秒，共享任务墙钟和整体时限；每动作6仿真秒、客户端12墙钟秒上限。
确认导航/修正终态失败后，等待控制槽空闲及新鲜测量，重检任务条件；已合格则仅检查停车和连续停留，仍不合格才重新导航、重新选择停车位。rescue_max_retries 默认允许额外2轮任务重试；navigation_max_retries 仍限制每次导航内部重试。重试不重置总时间和修正动作预算。预算耗尽、取消或控制权变化才退出；继续使用原行为服务器、速度平滑器与碰撞监视器。
日志 `RESCUE_ALIGN_ACTION/RESULT` 简述误差和动作；`rescue-result.json` 的 `alignment_attempts` 保存结构化动作与测量。
正常到位/转向由Shim完成。导航成功后的修复动作期间仍持续检查位姿，满足条件或位置漂移时取消本动作并确认终止，再重新判断下一阶段。位置仍合格时只旋转，不前进。
验证：10个隔离ROS测试及8个救援策略测试通过，新增覆盖先转向再短进、最终朝向、完成停留、动作拒绝、次数耗尽，以及位置到达后只旋转并持续检查/取消动作；尚未整机验证。

任务流程入口和行为树边界见 [任务流程](../task-workflow.md)。
