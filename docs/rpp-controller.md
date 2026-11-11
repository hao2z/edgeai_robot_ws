# RPP运动控制

实时手测时可另开PowerShell运行 `.\Diagnose-Controller.ps1 -Duration 20`，
查看FollowPath状态、前视点和速度链；说明见 [控制器短时诊断](controller-diagnostics.md)。

手动真值地图测试：

```powershell
.\Start-Navigation-Test.ps1 -Controller rpp
```

自动固定点测试（普通位置、指定朝向、救援停车）：

```powershell
.\Start-Autonomous-Mission.ps1 -NavigationTest -Controller rpp -MotionTest points -Duration 300
```

长U形绕W2往返：增加 `-PointScenario u_wall`。窄U形绕O2/W1/W1b往返：增加 `-PointScenario narrow_u`。每个绕墙段只下发一个目标，不插入中间导航点。窄U形世界坐标为准备点(1.50,2.78)、墙另一侧(0.48,3.87)，再返回准备点。普通探索启动也可指定 `-Controller rpp`，仍使用SLAM，不向导航注入仿真真值。默认控制器仍为DWB。

参数集中在 `src/edgeai_navigation/config/rpp_controller.yaml`。启动时由 `navigation_config.py` 组合公共 `nav2_exploration_rectangle.yaml`、`navigation_profiles.yaml` 的场景差异和独立控制器；实际配置保存到会话 `nav2-runtime.yaml`。DWB和MPPI也通过同一入口加载各自的控制器文件。

初始调参采用0.3 m/s、固定前瞻0.35 m、最近路径点沿路径搜索上限0.6 m、曲率减速半径0.5 m、转弯最低调节速度0.05 m/s、接近最低速度0.04 m/s；旋转速度0.5 rad/s、角加速度0.8 rad/s²。保留RPP预测碰撞检查；不额外启用膨胀代价减速，避免窄通道中重复减速，但实际车身碰撞检查和下游碰撞监视仍生效。地图窗口与前瞻距离分开配置，本轮使用实际1.5×1.5 m窗口、5 cm分辨率。

逐点终点策略由固定行为树绑定，统一使用原生 RotationShimController。普通位置点不要求终点朝向，Lattice允许时保留RPP倒车；指定朝向和救援点启用 `rotate_to_goal_heading: true`、`use_path_orientations: true`，进入位置容差后由Shim旋转到路径终点朝向。RPP的 `use_rotate_to_heading` 统一关闭，避免重复转向。旧LatticeRotationShim不再生成、注册或构建。起步使用路径车身朝向，支持倒车段保留车头方向。Jazzy原生Shim直接比较yaw，因此RPP局部代价地图与全局路径统一使用map帧；SLAM校正时滚动局部地图也会随之调整；行为服务器local_frame同步采用局部地图帧，避免恢复时混用odom坐标。NavFn对照不启用普通点倒车。

普通点倒车开关：`rpp_controller.yaml` 的 `allow_reversing` 与 `smac_lattice.yaml` 的 `allow_reverse_expansion` 须同时为true。`reverse_penalty` 越大越偏好前进；后方目标只建议保持接近时的车身朝向，最终路线仍由SE2规划检查。速度仍受公共 `velocity_smoother.min_velocity` 限制，目前倒车下限为−0.3 m/s。[官方RPP说明](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/controller_plugins/configuring_regulated_pp/)禁止同时启用倒车和RPP主动朝向旋转，因此运行时分别生成实例。

三个到点实例统一以 `MatchedRPP` 替代原生RPP作为primary：复用原生路径/前视计算，在调用原生控制与碰撞检查前，根据当前曲率收紧速度预算，使 `|v|` 和 `|v×曲率|` 不超过公共限幅。`reverse_velocity_limit` 默认为0.25 m/s，与公共倒车边界取较小值。Lattice Shim只负责起步姿态对齐，不再输出后缩放。朝向和救援点同样匹配速度边界，但保留各自的朝向要求和禁止倒车策略。

Lattice Shim按最新公共帧变换进行航向对齐，变换年龄受 `transform_tolerance` 限制，静态TF不按此过期。短时缺失/过期抛出可重试的NoValidControl，由controller_server停车并在 `failure_tolerance` 内重试；不再把1 ms的未来外推缺口直接升级成致命TF_ERROR，也不使用任意过期变换继续运动。

速度联动流程、自动派生项及限制见 [导航速度约束](navigation-speed-limits.md)。所有动态速度限制仍受公共边界约束；修改YAML后需重启，避免只在线改某一个节点造成不一致。

RPP启动时RViz显示黄色前瞻点 `/lookahead_point` 和粉色预测碰撞圆弧 `/lookahead_arc`。自动点测试的 `point-trace.json` 记录位姿、速度命令、实际里程计速度及RPP前瞻点；`turn-result.json` 记录各点误差、仿真用时、路程、恢复次数、最长静止和原地转向时间。

RPP不替代全局规划。矩形车身在狭窄转弯处需要可通过的路径；路径中心线无障碍不保证车身扫掠无碰撞。真值地图测试成功也不代表SLAM定位、动态障碍和完整救援流程已验证。

2026-11-06实测（仿真秒）：

| 测试 | 结果 | 恢复 |
|---|---|---|
| 六个基础点，含指定朝向和救援停车 | 6/6通过；普通点误差约5.9–6.2 cm，指定朝向误差0.239 rad，救援误差8.5 cm/0.034 rad | 0 |
| 窄U形去/回 | 20.019 / 23.633 s，4.853 / 4.745 m，误差5.88 / 5.98 cm | 0 / 0 |
| 长U形去/回，复查航向配置 | 20.680 / 23.986 s，4.913 / 4.856 m，误差5.87 / 5.63 cm | 0 / 0 |

会话分别为 `20261106-170413-rpp-simple_points`、`20261106-170623-rpp-narrow_u`、`20261106-171140-rpp-u_wall`，详见各自 `tests/results/runs/<会话>/turn-result.json`。前两轮使用一次性Shim对齐；最初长U形虽成功，但返程遇到TF缺口和一次后退恢复（反馈恢复计数3），改为重规划复查航向后重测长U形，两段均无恢复。基础点和窄U形尚未在这项最终调整后重复测试，随后按用户要求交给手动测试。

本轮仍观察到真值定位TF短暂时间缺口和控制循环频率告警；不能将功能测试通过表述为所有告警已消失。

2026-11-06晚间切换为原生Shim后的专项测试：会话 `20261106-215553-003-domain86-turn-navigation-test` 中，指定朝向点通过（位置误差5.1 cm、朝向误差0.217 rad）；紧随其后的短距救援点绕行，20秒仿真时间到后被测试客户端取消。救援轨迹采样中的最小位置误差为11.0 cm，未记录到进入10 cm终点窗口；因此本次未验证救援终点旋转成功，也不能仅凭本次结果认定终点旋转本身失效。该测试直接调用救援导航入口，不运行救援执行器的独立末端修复。上表较早的通过记录不代表本次配置已全部通过。
