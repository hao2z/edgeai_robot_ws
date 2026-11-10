# 目标坐标与地图修正

更新：2026-11-10。

建图期间，ArUco历史观测绑定附近SLAM关键帧，不再用当前机器人处的map→odom重新投影全部历史观测。关键帧被回环优化修正后，其绑定的目标位置和法线一起更新，再在map坐标系融合。该方法复用SLAM Toolbox，不向SLAM优化器添加标记约束，也不使用仿真真值。

`edgeai_navigation/anchored_slam_toolbox`继承异步SLAM节点，每秒发布一次`/slam_toolbox/anchor_graph`，包含关键帧ID、采集时间、原始里程计位姿和优化后的地图位姿。目标相对于关键帧的关系来自观测时刻的odom坐标，发布时使用该帧的最新map位姿。它是局部刚性近似，不能消除标记测距误差、观测与关键帧之间的里程计误差。

入口：

- SLAM导出：`src/edgeai_navigation/src/anchored_slam_toolbox.cpp`。
- 坐标计算：`src/edgeai_perception/scripts/landmark_anchors.py`。
- 观测融合：`src/edgeai_perception/scripts/aruco_targets.py`。
- 配置：`src/edgeai_perception/config/aruco_targets.yaml`，`map_anchoring: slam_graph`；直接使用静态地图时设为`map`。

关键帧选择要求odom平面距离不超过`anchor_max_distance_m`（默认1米），通常还要求时间差不超过5秒；距离不超过0.25米时允许静止期间使用较早帧。没有匹配帧时，观测最多暂存5秒，不退回旧的当前TF投影。地图图实例更换时清除旧图观测。

双机器人地图交接前，任务调用`/targets/finalize_map`。它请求`/slam_toolbox/finalize_anchors`暂停新扫描，生成最终地图及关键帧快照，然后固定目标坐标。服务成功后才保存地图、停止SLAM并启动AMCL。此后新观测直接存入静态map坐标，旧观测不随AMCL的map→odom变化。最终化失败时交接报错，不假称坐标已经固定。

测试关注：

- `TARGET_FOUND`、`TARGET_CONFIRMED`表示图像观测及多帧确认。
- `TARGET_MAP_FINALIZED`和`COOPERATIVE_TARGET_MAP_FINALIZED`表示地图交接完成坐标定稿。
- `TARGET_MAP_RESET`、`TARGET_ANCHOR_GRAPH_INVALID`、`TARGET_ANCHOR_UNAVAILABLE`用于定位图实例变化或绑定异常。
- 会话目录中的`target-observations.jsonl`记录观测绑定的帧；`target-anchor-history.jsonl`只在使用的关键帧发生变化时记录位姿；`targets.json`含`map_binding`、`anchor_nodes`和`coordinate_source`。

已完成包构建、几何单测、无Gazebo的ROS合成观测/地图交接测试，以及实际SLAM节点的合成激光生命周期测试。启动后尚无第一帧激光的空图情况已有回归检查。完整仿真由用户运行；仍需对比真值确认C1实际距离与任务估计距离，而不能仅凭任务成功判断定位质量。
