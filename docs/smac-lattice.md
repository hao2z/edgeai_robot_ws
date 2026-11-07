# 矩形差速车的全局规划

默认启动入口现在选择Smac State Lattice。手动测试：

```powershell
.\Start-Navigation-Test.ps1 -Controller rpp -Planner lattice
```

旧规划器对照用 `-Planner navfn`。自主探索启动入口也接受该选项。WSL入口使用 `--planner lattice|navfn`。

参数入口为 `src/edgeai_navigation/config/smac_lattice.yaml`；`planner_profile.py` 在启动时替换全局规划器，实际生效配置保存在会话 `nav2-runtime.yaml`。原底盘模板中的NavFn块用于回退对照，不代表本轮仍在使用NavFn。

使用Nav2安装包自带的5 cm分辨率、0.5 m转弯半径、diff运动模型、16方向控制集，包含原地旋转基元。规划器采用车轴中心坐标下的真实矩形轮廓；全局边距至少2 cm，局部沿用1 cm。未知区域允许搜索；普通点允许倒车扩展，指定朝向/救援点强制关闭倒车。禁用重型调试可视化和XY路径后处理，保留姿态/旋转基元。

Jazzy的原生Lattice需要终点朝向。普通位置点使用薄的 `edgeai_navigation::XYLatticePlanner` 输入层：NavFn仅建议路径接近方向，在原语支持的朝向中优先选终点矩形车身净空合适、接近该方向的朝向，再交给原生Smac搜索一次。已删除原来的内部穷举循环和 `heading_search_time` 总超时参数。失败后由现有行为树重试，下一次尝试其他朝向；成功朝向优先复用。终点净空检查只排序，不删除候选，保留原生终点位置容差的作用。普通点依然只检查XY；指定朝向和救援点直接使用原生Smac，不放宽目标朝向。

`XY_GOAL_HEADING` 日志记录规划输入的终点XY、车身yaw及净空合适的候选数量，便于区分坐标不可达和朝向选择不佳。

RPP调参以当前 `rpp_controller.yaml` 为准。普通点启用倒车时使用 `LatticeRotationShim`，按路径车身朝向对齐并修正Jazzy原生姿态分支的map/odom坐标混用；其他策略保留原生Shim的位置采样对齐。普通点接近方向在车后时，XY适配器优先尝试反向行驶对应的车身朝向，原生搜索仍应用倒车惩罚和车身碰撞检查。MPPI对照启用完整车身碰撞检查与路径朝向评分，尚未进行本轮整机验证。

验证包含配置与分辨率检查、真实ROS规划服务，以及窄空间整机测试。独立规划在合成窄通道和保存的真实地图上成功，普通点不适合的输入yaw也可自动选择可行接近方向。首次整机接入曾因上述Shim坐标配置转错方向，已修正。

修正后的会话 `20261106-174948-rpp-narrow_spaces` 进入窄通道目标成功，但用时56.895仿真秒、反馈恢复计数6；随后25 cm短步目标报告205（起点占用），尚未完成窄空间全部验证。按用户要求停止自动序列，切换到手动测试；不能据此宣称新组合已解决所有停滞问题。恢复行为未在本次全局规划器修改中改动。

后续应重点检查窄空间中的起步、短步、反向目标和退出，而不仅是U形路线。可复用自动场景：

```powershell
.\Start-Autonomous-Exploration.ps1 -NavigationTest -Controller rpp -Planner lattice -MotionTest points -PointScenario narrow_spaces -Duration 300
```

官方参考：[Smac Lattice配置](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/planners_plugins/smac/smac_lattice/configuring_smac_lattice/)、[运动基元说明](https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_smac_planner/lattice_primitives)。
