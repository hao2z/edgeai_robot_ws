# 探索实现对照（2026-10-29）

只审查源码和配置，未在同一场地重跑三个外部项目，因此不是性能排名。

## 审查快照

- RachitIshwe/autonomous_frontier_exploration_ROS2: ba9c99cc1d69b73c2aeaa35a6803346490c7e316
- Szymon-Jureczko/ros2-autonomous-explorer: 8f719cdf68f84fcfb5b13b4665e123e75e145b90
- darshmenon/rosnav: a8c080dccf1e5e8af65d53b3831092a0e08a0565

## 实现差异

Rachit：Explore Lite 在全局代价地图上搜前沿，0.33 Hz 选点、30秒进展超时，
距离权重0.0001、规模权重2.0、最小前沿0.15m。正在执行的同一目标不重复发送；
ABORTED和无进展会进入黑名单，匹配范围5个栅格（5cm地图对应各轴0.25m）。
导航配置为NavFn Dijkstra + MPPI，预测2.8秒（56×0.05），vx范围[-0.35,0.5]，
wz上限1.9rad/s。局部圆半径0.18m、全局0.22m，不能直接复制到本机器人。

Szymon：自编Python BFS前沿聚类，按簇大小/(距离+0.1)评分。
launch实际覆盖默认值为5秒处理一次、最小5格、黑名单半径1.0m。
导航期间保留目标，60秒现实时间超时取消并加入黑名单；记录最多50项，
全部候选被屏蔽时直接清空黑名单。没有同等完整的跨任务低收益记忆。
NavFn A* + MPPI，预测2.5秒（25×0.1）、400批、3轮迭代，vx范围[0,1.0]。
圆半径0.22m，膨胀0.55m，进展门槛0.3m/15秒，位置容差0.4m。
取消后立即释放本地状态，未等待终态，不能当作严谨异步交接范本。

rosnav：有builtin、Explore Lite和frontier等多后端；不能说整个项目固定一种算法。
builtin有WFD可达区域筛选、自由栅格BFS路径距离、代价地图目标校验、目标回退，
成功后二次核验（耗时、距目标、实际位移）、visited记录、失败冷却、
位移或地图自由格增长的局部停滞判断。失败冷却默认120秒（time.monotonic现实时钟）。
RPP配置为NavFn A* + RPP，目标速度0.15m/s、启用朝向旋转和碰撞检测；
Jazzy配置则为SmacHybrid + MPPI，说明不同入口不可混为一谈。
其concepts.md第39、40节也记载未完成地图、TF冻结及DWB/MPPI停滞，演示不等于全面可靠。

## 当前项目的差距和优先级

1. 探索器时间估计vmax=0.2m/s，DWB实际上限0.6m/s，排序模型未同步。
2. Nav2进展门槛0.1m/7秒，探索器另有10秒外层无进展计时；
   不能因为两个数字不同就认定不会冲突，两者使用的进展量不同。
3. 新repeat_guard只统计局部已知面积净增量，不校验最终距目标和实际位移；
   SLAM修正、不同格子的已知/未知交换、未等到新地图消息均可能误判。
4. MRTSP有规模与几何距离近似，不能等同于对每个候选调用Nav2规划验证；
   跨墙几何近、实际绕行远的候选仍需检验。
5. min_frontier_size_cells=5和候选最小距离1.0m可能同时追逐细小残留，排除近处有价值前沿；
   不宜仅用全局固定门槛，需要按真实可见收益、可达观测位姿筛选。
6. 全局仅static+inflation，实时障碍仅在局部考虑；适合静态仿真，但动态场景可能反复堵路。
7. DWB矩形和MPPI半径0.37m圆形不是公平控制器对照（圆直径0.74m）。
8. 手调YAML与生成模板未完全一致；重新生成会覆盖手调参数，尚缺单一配置来源。
9. 新repeat_guard两项测试通过，但尚无完整GUI对照；完整上游测试有3个旧抢占策略冲突。
10. 最近完整测试224535-858达到时限，观测覆盖97.35%、下方右侧房间89.43%；
    不是完成事件，不证明全覆盖或物理访问覆盖。

优先收敛进展/取消职责和速度模型，再补成功二次核验、地图新鲜度检查与候选规划验证。
RPP值得作为同图、同机器人尺寸、同速度限制的对照，不应直接替换后宣布解决。

## 可核查源文件

- https://github.com/RachitIshwe/autonomous_frontier_exploration_ROS2/blob/ba9c99cc1d69b73c2aeaa35a6803346490c7e316/autonomous_description/config/nav2_params.yaml
- https://github.com/RachitIshwe/autonomous_frontier_exploration_ROS2/blob/ba9c99cc1d69b73c2aeaa35a6803346490c7e316/autonomous_description/config/explore_params.yaml
- https://github.com/RachitIshwe/autonomous_frontier_exploration_ROS2/blob/ba9c99cc1d69b73c2aeaa35a6803346490c7e316/m-explore-ros2/explore/src/explore.cpp
- https://github.com/Szymon-Jureczko/ros2-autonomous-explorer/blob/8f719cdf68f84fcfb5b13b4665e123e75e145b90/src/frontier_explorer.py
- https://github.com/Szymon-Jureczko/ros2-autonomous-explorer/blob/8f719cdf68f84fcfb5b13b4665e123e75e145b90/config/nav2_explore_params.yaml
- https://github.com/Szymon-Jureczko/ros2-autonomous-explorer/blob/8f719cdf68f84fcfb5b13b4665e123e75e145b90/launch/auto_explore.launch.py
- https://github.com/darshmenon/rosnav/blob/a8c080dccf1e5e8af65d53b3831092a0e08a0565/src/rosnav_bot/scripts/frontier_explorer.py
- https://github.com/darshmenon/rosnav/blob/a8c080dccf1e5e8af65d53b3831092a0e08a0565/src/rosnav_bot/config/nav2_params_rpp.yaml
- https://github.com/darshmenon/rosnav/blob/a8c080dccf1e5e8af65d53b3831092a0e08a0565/concepts.md
