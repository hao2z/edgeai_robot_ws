# 逐点导航要求

RPP派生三个固定控制器，统一使用原生 `RotationShimController` + RPP，替代自定义 `LatticeRotationShim`。指定朝向及救援点启用 `rotate_to_goal_heading: true`、`use_path_orientations: true`，由Shim在位置容差内对齐终点朝向；RPP的 `use_rotate_to_heading` 关闭。普通位置点保留不要求最终朝向的规则，运行时关闭Shim的终点旋转；仅普通点+Lattice允许倒车。RPP参数和运动测试入口见 [RPP运动控制](rpp-controller.md)。

运行时也按每个请求绑定局部控制器：普通点使用 `FollowPath`，DWB 移除 `RotateToGoal`，避免在位置未达标时因终点转向锁住平移；指定朝向点使用 `FollowPathHeading`，救援使用 `FollowPathRescue`，两者保留 `RotateToGoal`。各 DWB 实例的终点窗口自动匹配对应检查器的 XY 容差。所有实例继承同一份速度、避障和路径跟踪调参，只运行当前任务所选控制器。MPPI配置则对普通点去掉 `GoalAngleCritic`，要求朝向的点保留它。

使用相同的 `nav2_msgs/action/NavigateToPose` 消息，通过动作入口选择这个点的到达要求：

| 入口 | 该点到达要求 | 规划器 |
|---|---|---|
| `/exploration/navigate_to_pose` | 自动探索，仅 XY | `GridBased`，多朝向选路；输入yaw仅作偏好 |
| `/navigate_to_pose` | 手动普通点，仅 XY | `GridBased`，多朝向选路 |
| `/navigation/heading_to_pose` | 手动指定朝向点，XY 和 yaw | `GridBasedHeading`，目标 yaw |
| `/navigation/rescue_to_pose` | 救援，XY、yaw 和停车 | `GridBasedHeading`，目标 yaw |

手动入口均需先取得 MANUAL 控制权并等待 IDLE；RViz 的 `/goal_pose` 默认仅检查位置。指定朝向使用合法四元数；不给朝向约束也应填写合法四元数，例如 `w: 1.0`，而不是全零。

客户端可按每个点的 `require_heading` 选择入口，无需修改 Nav2 消息定义：

```python
position_client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
heading_client = ActionClient(node, NavigateToPose, '/navigation/heading_to_pose')
client = heading_client if require_heading else position_client
# 等待 client.server_is_ready()；pose 包含 map 坐标和合法四元数。
future = client.send_goal_async(NavigateToPose.Goal(pose=pose))
```

两个入口发送的都是普通导航点，不会触发救援任务或连续停留。最终救援执行器单独选择救援入口，并在导航成功后验证停留。普通指定朝向点的位置容差沿用普通导航配置，角度容差沿用救援配置的 `yaw_tolerance_rad`；救援使用其自身位置及停车容差。

`navigation_requests.py`接入请求，路由及策略在`goal_policy.py`中定义；`navigation_authority.py`保存请求策略并负责排队、取消和交接，恢复期间不会改变策略。实际派发时使用生成的固定行为树（会话目录`goal-policy-trees/`），绑定规划器、控制器和终点检查器ID，保留原恢复逻辑。默认Jazzy Lattice配置中，`GridBased`在共享预算内比较多个原语朝向的原生Smac路径，`GridBasedHeading`保留原生固定朝向；两者共用已有全局代价地图。NavFn对照配置中，普通点仍使用路径末段朝向。多朝向预算及模式见`smac_lattice.yaml`，实现边界及验证见[任务流程](task-workflow.md)。

原 `/goal_checker_selector`、`/planner_selector`、`/controller_selector` 话题不能覆盖这些受管理任务的逐点规则。客户端不能用任意 `behavior_tree` 绕开统一管理。`GOAL_RESERVED` 和 `BACKEND_ACCEPTED` 事件记录 `arrival_policy`，后者还记录实际控制器、检查器和行为树路径。
