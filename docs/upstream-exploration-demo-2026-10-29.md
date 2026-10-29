# 原版探索 demo 实测（2026-10-29）

使用探索器作者的 autonomous-exploration-demo-benchmark，场景 corridor。
第三方源代码及构建缓存在 ~/.cache/edgeai-native/exploration-demo-upstream，
原生 /opt/ros/jazzy；没有加载 edgeai 的仲裁器、机器人、导航参数或探索器补丁。
确切版本见运行目录中的 benchmark-revision.txt 和 explorer-revision.txt。

运行目录：tests/results/runs/20261029-upstream-corridor-domain89-retry。
启动入口：tests/exploration/run-upstream-exploration-demo.sh，域 89。
该脚本直接调用作者 launch.sh 中的 launch 入口及相同 spawn 参数，避开其
无域隔离的全局 pkill 清理。原有项目测试和终端在启动前关闭。

第一次启动因 topic_tools 缺失失败。补齐 topic_tools、interactive_marker_twist_server、
rviz_imu_plugin 官方 apt 包后，第二次仿真、SLAM、Nav2、RViz 和 frontier 均启动。
上游 package.xml 未完全列出这些运行依赖。未改动上游导航配置。

第二次运行约 40 秒后停止运动，累计 /odom 行驶 18.735 m。
最终 /map 为 359 × 259，分辨率 0.05 m，已知 50851 格，完全空闲 47962 格。
探索器最后记录局部目标 cost=99，取消该小前沿；随后记录 No more frontiers found，
并发布探索完成事件。这是算法停止，不能直接等同场地真值覆盖 100%。

8 个 NavigateToPose 均观察到 CANCELED 终态；不能将取消解释为导航成功或失败。
探索器允许按地图增益抢占，因此运动流畅并不要求每个 NavigateToPose SUCCEEDED。
没有观察到 spin/backup/wait 恢复执行；本轮不能验证真正脱困。
存在 Failed to get result for follow_path in node halt!，说明该日志也能发生在
无自编仲裁器的原版 demo 中，不能单独证明仲裁器故障。

关键配置：MPPI vx_max=0.75、vx_min=0、wz_max=3.2、ax_max=3.5、az_max=4.2，
vx_std=0.2、wz_std=0.6、time_steps=30、batch_size=700，consider_footprint=true。
因此“完整 footprint 检查必然导致慢速”不成立；需要结合机器人/采样/限幅作控制对照。
原始日志、最终地图和实际检查过的 Gazebo/RViz 截图保存在运行目录。
