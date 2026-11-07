# 导航调参入口

| 要调整的内容 | 文件 |
|---|---|
| 地图、车身、进展/到点检查、恢复、速度平滑 | `nav2_exploration_rectangle.yaml` |
| 深度相机局部障碍开关、限频、采样、高度与距离过滤 | `depth_obstacles.yaml` |
| DWB速度、轨迹采样、评分权重、Rotation Shim | `dwb_controller.yaml` |
| RPP速度、前视、转弯、Rotation Shim | `rpp_controller.yaml` |
| MPPI速度、预测、采样、评分 | `mppi_controller.yaml` |
| Smac Lattice全局规划 | `smac_lattice.yaml` |
| 恢复顺序和重新规划频率 | `behavior_trees/explore_recovery.xml` |
| 救援及补充搜寻任务 | `normal_rescue.yaml`、`midterm_single_rescue.yaml` |
| 双机初始相对位姿、AMCL门槛、C1停车位与共同停留 | `cooperative_rescue.yaml` |
| 建图时过滤停放的第二台机器人回波 | `fleet_scan_filter.yaml` |
| 历史baseline/safe配置与MPPI圆形对照差异 | `navigation_profiles.yaml` |

加载顺序：公共配置 → 场景差异 → 独立控制器 → 全局规划器 → 普通/朝向/救援到点策略。
`navigation_config.py` 组合配置；Lattice再统一矩形车身和5cm栅格。
控制器文件的 `profiles` 仅覆盖历史baseline/safe参数，rectangle使用顶层 `FollowPath`。
`conservative`复用safe几何配置；控制器选择始终遵守 `--controller`。
公共速度平滑器仍会限制控制器输出，调速度时需同时检查。
RPP启动时从公共速度/加速度边界自动派生三个实例的预测前限制，并收紧旋转恢复边界。
改速度后重启，查看会话 `motion-limits.json` 和 `MOTION_LIMITS_WARNING`；流程见 `docs/navigation-speed-limits.md`。

修改后重启生效，检查对应会话的 `nav2-runtime.yaml`。公共配置是组合输入，不能单独作为完整Nav2参数文件启动。
已移除重复整套配置和旧生成脚本；依赖安装不再覆盖手调导航参数。

深度障碍当前默认关闭（`depth_obstacles.yaml: enabled: false`）。PowerShell `-DepthObstacles false`、Linux `--depth-obstacles false`
可恢复单雷达局部代价图；`true`强制开启，`config`遵守YAML的`enabled`。
关闭时既不加载深度层，也不启动点云降采样进程；相机仍可用于目标识别。
协作方式、限制和验证见 `docs/depth-obstacles.md`。
