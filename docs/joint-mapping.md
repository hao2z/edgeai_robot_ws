# 双机器人联合建图

入口为 `Start-Autonomous-Mission.ps1 -JointMapping` 或 `bash edgeai.sh mission --joint-mapping`。保留不带选项的单机建图集成任务，中期检查的独立 P1/P2 不启用联合模式。

配置集中在 `src/edgeai_mapping/config/joint_mapping.yaml`。两车初始相对位姿是粗估值，不读取世界坐标；机器人在初始激光配准完成前应保持静止。配准要求重叠、残差、几何可观性及多个不同时间样本一致，不确定或超时直接报告启动失败。

## 坐标、数据和优化

两车传感器、里程计和实体 TF 使用 `robot1`、`robot2` 前缀。公共 `map` 下由单个 `joint_slam_toolbox` 发布 `map→robot1/odom` 和 `map→robot2/odom`。两车本身分别发布 `odom→base_footprint`，不会通过 TF 把另一台机器人当成主车的刚性附件。

初始相对位置由双边激光配准确认，并换算成 `odom1_from_odom2`。这个初值只负责将两条里程计链放到公共参考中；后续 Karto 为不同传感器保留独立扫描链，并由跨传感器扫描关联及回环约束联合优化。两车的最新关键帧均在每次图更新后重新计算各自 `map→odom`。不能把当前两车从同一张地图计算出的相对位姿再次当作独立测量加入优化器。

复用 Jazzy 安装的 SLAM Toolbox/Karto/Ceres。参考上游 `ros2` 分支（评估时 SHA `7376b23689d47d0be2a1dd0c58f5eb7085ac8a55`）的多来源扫描、公共里程计参考和外来扫描图优化思路；本实现是单个集中后端，不是直接运行上游去中心化节点。后者未包含在本机安装的 Jazzy 2.8.5 中。参阅 [上游说明](https://github.com/SteveMacenski/slam_toolbox/blob/7376b23689d47d0be2a1dd0c58f5eb7085ac8a55/docs/decentralized_multi_robot_slam.md)。

两车导航、仲裁、传感器与探索接口原生使用 `/robot1`、`/robot2`，不再保留根命名空间传感器转发或旧TF别名。单机同样使用 `/robot1`。`/map`、`/clock`、`/tf`、`/tf_static` 及目标注册信息为公共数据；TF帧以机器人名称区分。

## 同时探索与避障

两个前沿探索器读取同一张 `/map`，分别使用自己的真实代价地图，独立发布 `/robotN/explore/candidates`，不派发导航。顶层任务协调器调用探索任务；任务层 `fleet_exploration_coordinator` 按距离和稳定规则给双方分配不同目标，管理预留、超时、失败历史和空闲车让路。活动目标保持稳定；空闲车可领取任意未预留候选。配置在 `src/edgeai_mission/config/exploration.yaml`，职责与失败分支见[任务层协调](fleet-task-coordination.md)。运行时传感器节点只处理扫描过滤及状态，不承担目标分配。

公共地图扫描在扫描时刻按对方估计车身过滤，TF 暂不可用时有界等待并跳过该扫描；不会把对方车体永久写入墙体。Nav2 保留原始激光，且通过既有 peer footprint layer 加入对方实时位置。初始配准本身也按粗估位姿剔除对方车身回波，避免让两台车互相配准到车壳。

建图完成须两车候选快照均新鲜有效且连续为空、全部导航终态已确认，再由顶层协调行为树检查面积门槛与目标完整性。缺数据、冷却、候选失败或让路中均不代表完成。缺失目标由双机搜寻任务并行分配观察点，S1 由主车执行。

## 目标和救援交接

两台机器人均启用 RGB-D，ArUco 观测保留来源机器人，疑似检测分别输出候选。目标先按图像时刻定位，再后台绑定来源机器人关键帧。图快照携带 `robot_id`、`sensor_id` 和公共图实例；局部重复 ID 不会串绑，位置与法线随绑定帧共同修正。重新观测继续更新目标。

双机搜寻完成后，`save_map` 先冻结公共图和目标再保存地图。顶层 `EnableLocalization` 随即读取双方最终位姿、停用 SLAM，并由 AMCL 接管两条 `map→odom` 边；定位收敛前不下发救援。定位运行节点持续负责静止更新及对方车身发布，S1/C1 复用同一实例。启动失败由顶层有限重试，保留先验并补偿重试期间自身里程计位移；不会在 S1 期间只靠冻结变换传播里程计。

## 记录和核验

- `joint-initialization.json`：初值、激光配准结果、残差、重叠率、协方差和源时间戳。
- `joint-mapping-status.jsonl`：每秒简要记录两条扫描链、跨机器人边数量、地图/关键帧更新时间和双方位姿。
- `joint-mapping.log`、`joint-startup.log`：公共后端、第二台导航/探索和启动失败原因。
- `joint-mapping-ready.json`：确认初始配准通过、双方扫描链已入图、存在跨机器人边、TF 新鲜、第二台导航激活后才放行主任务。Karto 初始连边本身不保证配准质量，仍须独立激光配准通过。
- 既有 `target-observations.jsonl`、`target-anchor-history.jsonl` 与救援真值核验继续保留。

任务层分配的 Gazebo 联合探索已完成23.41 m²建图，双方并行运动并能从部分导航失败中继续。空闲车让路已通过真实ROS接口与模拟导航服务验证；尚未完成自然动态会车及完整救援验证。详见任务层协调文档的验证范围。

重新拉取代码后运行 `bash edgeai.sh build`。若只更新已安装环境，运行 `bash scripts/setup/build-frontier-runtime.sh` 和 `bash scripts/setup/build-workspace.sh`；前沿监控带版本标识，旧 overlay 无法用于联合模式，会在停止旧会话之前提示重建。
