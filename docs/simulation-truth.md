# 仿真真值诊断

仿真启动默认启用 `truth:=true`。Gazebo PosePublisher 仅以 2 Hz 发布机器人模型的世界位姿，诊断节点以每墙钟秒一次配对同一时间戳的导航 TF，不等待、不回退到最新 TF。最大真值年龄为 0.75 仿真秒。无有效配对时发布原因并清除旧图层。

为避免高频 `/clock` 回调的额外开销，该诊断节点使用墙钟定时器，日志 `sim_time` 取最近导航 TF 的仿真时间戳（`time_source=latest_navigation_tf`）。真值超过3墙钟秒未接收时`truth_valid=false`；导航TF缺失或超过2墙钟秒未接收时，只使叠图配对无效，仍记录新鲜Gazebo位姿和目标真值供独立救援核验。

节点计算 `world_from_reference = world_from_robot * inverse(reference_from_robot)`；探索的 reference 为 `map`，未启动 SLAM 的纯仿真为 `odom`。仅在节点内部转换，不发布导航 TF，不向规划器输入真值。

RViz 的 **Simulation truth (robot aligned)** 图层显示蓝色半透明真实障碍、青色真实标记及朝向、橙色实际下发导航点。障碍取自 SDF 静态模型的 box collision（当前场地墙体、箱体、标记支架）；标记取自目标配置。不是栅格地图或完整 Gazebo 渲染，不包含动态障碍和非 box 几何。

这是按当前机器人对齐的局部对比：真值与估计机器人必然重合，不是绝对定位误差测量，也不能消除地图的非刚性形变。

`/navigation_authority/dispatched_goal` 为仲裁器实际向后端提交的原始 PoseStamped，不采用路径末端猜测。保留最后目标；是否仍活动须结合记录的 `phase`。该话题不发导航命令。

`/truth/summary` 发布简要 JSON；启动脚本在本次运行目录保存 `truth.jsonl`，每个墙钟秒一条，包含：

- `sim_time`、`sample_time`、`pose_age_s`、`status/reason`：时间配对是否有效。
- `reference_frame`、`robot_world`、`robot_estimated`、`world_from_reference`：XY 米、yaw 弧度。
- `goal_estimated`、`goal_world`、`goal_stamp_s`、`goal_to_targets_m`：最后下发点及其到每个真实标记的水平距离；目标 frame 不匹配时记录 `goal_error`。
- `target_estimates`：检测标记转换到 world 后的位置、水平误差和观测年龄（如存在 targets.json）。
- `truth_valid`、`targets_world`：独立于导航TF的真值状态和目标world位姿。启动时另保存`truth-targets.json`，避免目标元数据缺失影响回放。

S1/C1任务判定成功后，`edgeai_mission/scripts/rescue_success_log.py`读取对应时间的历史记录，输出真值距离和朝向误差、距离/朝向估计误差、目标定位误差，以及停留窗口的距离、朝向和差分速度采样核验。C1要求两车覆盖共同停留时段。通过/未通过/无法确认在主日志分别显示OK/ERROR/WARN；完整结果位于救援结果文件的`success_evidence`。核验是独立仿真诊断，不向任务决策输入真值，不把有限频率采样表述为连续碰撞证明。

真值来源是静态场景文件和 Gazebo 物理位姿，不是相机/SLAM 推断。多机器人独立命名空间发布，日志命名为 `<robot>-truth.jsonl`。直接 ros2 launch 时可通过 `EDGEAI_TRUTH_DIRECTORY` 指定日志目录；不设置则只发布 ROS 话题。

核验窗口严格使用任务采样时间`[结束时间-dwell_s, 结束时间]`，不以滞后的最后审计时间向前平移。边界使用相邻有效真值位姿插值，角度走最短弧；审计末端尚未写到时最多等待3墙钟秒。该等待只读取文件，不发送运动命令，超时不伪造核验通过。
