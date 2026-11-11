# 分用途导航配置

各入口使用独立规划器、控制器和到点检查器实例，配置位于`src/edgeai_navigation/config/navigation_tasks/`。公共控制器/规划器文件提供默认值，各任务文件只写需要覆盖的参数；公共速度平滑器仍是底盘速度边界。

| 文件 | NavigateToPose Action入口 | 用途 |
|---|---|---|
| `position.yaml` | `/navigate_to_pose` | 普通点、换位 |
| `frontier.yaml` | `/exploration/navigate_to_pose` | 前沿 |
| `heading.yaml` | `/navigation/heading_to_pose` | 指定朝向 |
| `rescue.yaml` | `/navigation/rescue_to_pose` | S1救援导航 |
| `cooperative.yaml` | `/navigation/cooperative_to_pose` | C1救援导航 |

robot2的入口加`/robot2`前缀。入口负责选择策略，权限节点仍负责唯一执行槽、取消和交接。S1/C1执行器分别调用各自入口；C1候选预规划也使用其独立规划器。

例如仅调整C1：

```yaml
require_heading: true
allow_reverse: true
controller:
  desired_linear_vel: 0.25
  reverse_velocity_limit: 0.10
planner:
  rotation_penalty: 3.0
goal_checker:
  xy_goal_tolerance: 0.08
  yaw_goal_tolerance: 0.20
```

`allow_reverse`同时控制Lattice倒车扩展和RPP倒车跟踪，不再按救援身份强制关闭；NavFn无倒车原语，不能据此生成有方向的倒车路径。不要在覆盖块重复写`allow_reversing`或`allow_reverse_expansion`，生成器会拒绝，避免两层不一致。RPP仍由Shim负责转向，底盘限速仍由公共配置联动。

修改后重启。实际实例可在会话`nav2-runtime.yaml`中查找`GridBasedCooperative`、`FollowPathCooperative`、`cooperative_goal_checker`，固定导航树位于`goal-policy-trees/`。

这些配置只负责导航，不替代救援条件。C1的停车间距、救援半径、共同停留、末端`alignment_*`仍在`cooperative_rescue.yaml`；S1在`midterm_single_rescue.yaml`。修改导航容差不会修改比赛任务判据。当前默认仍保留S1/C1前进，启用C1倒车只需改其`allow_reverse`。
