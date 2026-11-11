# 矩形差速车的全局规划

默认使用Smac State Lattice与原生Rotation Shim + RPP。Lattice按矩形车身检查碰撞；原语为5 cm栅格、0.5 m转弯半径、diff模型，含16个方向及原地旋转。16是运动原语方向数，不代表每个目标都试16次。

Windows手动测试：`Start-Navigation-Test.ps1 -Controller rpp -Planner lattice`；Linux：`bash edgeai.sh navigation --controller rpp --planner lattice`。NavFn对照使用`-Planner navfn`或`--planner navfn`。

参数在`src/edgeai_navigation/config/smac_lattice.yaml`；`planner_profile.py`生成会话`nav2-runtime.yaml`。

| 参数 | 默认 | 用途 |
|---|---|---|
| `goal_heading_mode` | `ALL_DIRECTION` | 多方向选路；可选`BIDIRECTIONAL`、`DEFAULT` |
| `goal_heading_count` | `4` | 全周分散选择原语方向；0选全部，当前最多16 |
| `heading_search_budget_s` | `2.0` | 所有候选共享墙钟预算 |
| `heading_reuse` | `true` | 同一目标优先复用成功朝向 |
| `cost_penalty` | `2.0` | 对高代价区域的偏好，配合膨胀层调节 |
| `rotation_penalty` | `5.0` | 原地转向惩罚 |
| `reverse_penalty` | `2.0` | 普通点倒车惩罚；指定朝向/救援关闭倒车扩展 |

Jazzy由`XYLatticePlanner`兼容多目标朝向：每条候选路径仍由原生Smac生成并检查完整车身，适配器在预算内比较距离、地图代价、旋转和倒车评分，不保证全局最优。`XY_GOAL_HEADING`记录尝试数、可行数、评分及耗时。普通点的输入yaw只影响排序/等代价偏好，到位只检查位置。

指定朝向和救援使用`GridBasedHeading`固定目标yaw；局部控制由原生Shim完成终点转向。旧`LatticeRotationShim`已移除。接口见[逐点策略](navigation-goal-policy.md)，控制及速度联动见[RPP](rpp-controller.md)。

历史窄空间问题见[11月6日回放](history/navigation-heading-diagnosis-2026-11-06.md)，不代表当前完整运动验收。
