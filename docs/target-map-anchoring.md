# 目标坐标与地图修正

更新：2026-11-11。

建图期间，ArUco历史观测绑定附近SLAM关键帧，不再用当前机器人处的map→odom重新投影全部历史观测。关键帧被回环优化修正后，其绑定的目标位置和法线一起更新，再在map坐标系融合。该方法复用SLAM Toolbox，不向SLAM优化器添加标记约束，也不使用仿真真值。

`edgeai_navigation/anchored_slam_toolbox`继承异步SLAM节点，每秒发布一次`/slam_toolbox/anchor_graph`，包含关键帧ID、采集时间、原始里程计位姿和优化后的地图位姿。目标相对于关键帧的关系来自观测时刻的odom坐标，发布时使用该帧的最新map位姿。它是局部刚性近似，不能消除标记测距误差、观测与关键帧之间的里程计误差。

入口：

- SLAM导出：`src/edgeai_navigation/src/anchored_slam_toolbox.cpp`。
- 坐标计算：`src/edgeai_perception/scripts/landmark_anchors.py`。
- 观测融合：`src/edgeai_perception/scripts/aruco_targets.py`。
- 配置：`src/edgeai_perception/config/aruco_targets.yaml`，`map_anchoring: slam_graph`；直接使用静态地图时设为`map`。

新观测先使用图像时刻的map→odom变换定位，通过PnP、TF和融合一致性检查后直接发布map坐标。相机回调只记录图像时刻的位置、方向及机器人位置，关键帧关联由独立定时回调处理。

后台按map平面距离选最近节点，将位置及方向记录为相对于该节点当前map位姿的量。完成关联后，目标跟随该节点的图优化修正，位置与方向一同更新。重访旧区域时，不再将旧节点的里程计修正重复套到新观测上。关联升级替换同一条观测，不增加多帧确认次数。

重新入镜时，新观测继续更新位置、方向和最后观测时间，并重新后台关联。如果新位置偏离旧融合中心超过`outlier_distance_m`，会在`reobservation_window_s`内用`confirmation_observations`条相互一致的观测替换旧位置，记录`TARGET_RELOCALIZED`。零散异常观测仍被过滤，旧位置不会永久挡住经重新确认的新位置。

范围由`anchor_max_distance_m`控制（默认3米）。关键帧年龄不设门槛，时间差只用于距离相同时排序。找不到节点时，保留图像时刻的map坐标并继续发布，后台持续重试仍保留的观测。已删除待绑定超时和独立队列容量参数，存储统一受`max_observations_per_id`及融合窗口约束。输出中的`pending_anchor_observations`说明尚未关联的观测数量。尚未关联的旧观测无法跟随局部图优化修正，新的相机观测会补充当前位置。

后台回调`anchor_processing_hz`和目标发布`target_publish_hz`均默认5Hz，配置在`src/edgeai_perception/config/aruco_targets.yaml`。SLAM图快照周期`anchor_graph_publish_period_s`默认0.5墙钟秒，配置在`src/edgeai_simulation/config/slam_toolbox.yaml`。图快照到达后立即更新关联，目标在下一次发布时反映图优化结果。图像时刻TF不可用时记录`image_time_tf_unavailable`，地图图实例更换时清除旧图目标。

双机器人地图交接前，任务调用`/targets/finalize_map`。它请求`/slam_toolbox/finalize_anchors`暂停新扫描，生成最终地图及关键帧快照，然后固定目标坐标。服务成功后才保存地图、停止SLAM并启动AMCL。此后新观测直接存入静态map坐标，旧观测不随AMCL的map→odom变化。最终化失败时交接报错，不假称坐标已经固定。

测试关注：

- `TARGET_FOUND`、`TARGET_CONFIRMED`表示图像观测及多帧确认。
- `TARGET_MAP_FINALIZED`和`COOPERATIVE_TARGET_MAP_FINALIZED`表示地图交接完成坐标定稿。
- `TARGET_MAP_RESET`、`TARGET_ANCHOR_GRAPH_INVALID`、`TARGET_ANCHOR_UNAVAILABLE`用于定位图实例变化或绑定异常。
- 会话目录中的`target-observations.jsonl`记录观测绑定的帧；`target-anchor-history.jsonl`只在使用的关键帧发生变化时记录位姿；`targets.json`含`map_binding`、`anchor_nodes`和`coordinate_source`。

已完成包构建、几何单测、无Gazebo的ROS合成观测/地图交接测试，以及实际SLAM节点的合成激光生命周期测试。启动后尚无第一帧激光的空图情况已有回归检查。完整仿真由用户运行；仍需对比真值确认C1实际距离与任务估计距离，而不能仅凭任务成功判断定位质量。
