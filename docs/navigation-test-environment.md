# 真值导航测试环境

启动：

```powershell
.\Start-Navigation-Test.ps1
```

RPP对照模式用 `.\Start-Navigation-Test.ps1 -Controller rpp`，参数和自动U形测试见 [RPP运动控制](rpp-controller.md)。

全局规划默认使用Smac State Lattice，普通点自动选择可行接近朝向；旧NavFn对照增加 `-Planner navfn`。配置及验证限制见 [矩形差速车全局规划](smac-lattice.md)。

默认中期场地、单机器人、DWB、手动控制，最长运行一小时；可传 `-Duration`、`-DomainId`、`-Controller`、`-CheckOnly`。`Ctrl+C` 或现有 `Stop-Simulation.ps1` 关闭。正常自主探索启动方式保持原样。

此模式使用仿真真实碰撞几何生成完整静态地图，并以Gazebo世界位姿提供机器人定位。`map` 的XY轴与仿真world一致，坐标单位m；不启动SLAM或AMCL，不需要2D Pose Estimate初始化。机器人起点仍由 `robots_midterm.yaml` 指定，不硬编码导航坐标偏移。地图包含墙、障碍、支架和抬高的标记板，按整个二维碰撞投影保守占格；分辨率5 cm。它不是待评估的SLAM地图。

地图由 `generate_navigation_truth_map.py` 根据与仿真相同的场景/目标配置生成，每次启动写入会话 `map.yaml`、`map.pgm`、`map.geometry.json`，不加载旧运行的建图结果。`navigation_truth_localization.py` 将同一时间的世界机器人位姿与odom→base配对，发布map→odom；没有对应时间TF或真值过期时不使用最新变换代替。仅测试模式提高真值发布频率至20 Hz，普通探索仍保持原诊断频率且不向导航提供真值。

手动测试：

1. 等待日志出现“真值地图已就绪”，确认RViz中机器人与真实场景对齐。
2. 使用RViz工具栏 **2D Goal Pose** 在空闲位置点击并拖动箭头。默认只要求位置，箭头不作为终点朝向要求。
3. 勾选面板 **手动目标要求朝向** 后，下一次点击会使用箭头方向作为终点要求；当前正在执行的点不受勾选变化影响。
4. 使用 **停止任务** 取消。停止后点击 **手动接管** 再选点。此模式不启动自动探索，也不执行自动救援。

底盘速度平滑、车身碰撞检查、雷达障碍检测、碰撞监视与原有恢复仍开启。手动选点会执行导航，不是只画路径；全局路径、局部轨迹和速度状态可在RViz查看。要求朝向的动作入口仍可直接调用 `/navigation/heading_to_pose`，仅位置入口为 `/navigate_to_pose`。

终端方式：

```bash
EDGEAI_NAVIGATION_TEST=true bash scripts/simulation/start-autonomous-exploration.sh --duration 3600 --controller dwb --arena midterm --domain 86
```

自动固定点/U形测试可在此模式同时设置 `EDGEAI_TURN_TEST=true EDGEAI_MOTION_TEST=points`；U形再设置 `EDGEAI_POINT_TEST_SCENARIO=u_wall`。真值模式使用世界坐标测试点：西侧准备点(2.85,1.00)，东侧点(4.50,1.40)，正式往返跨过W2墙，仅各下发一个目标，不插入中间点。详见 `docs/local-controller-point-tests.md`。

会话保存 `nav2-runtime.yaml`、实际逐点行为树、`authority-events.jsonl`、导航/定位日志和真值诊断。测试地图与定位只用于仿真控制器调试，不能用于评估自主建图或目标发现能力。
