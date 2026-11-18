# 原生命名空间迁移与验证（2026-11-16）

工作目录：`D:\KTH\Competition\edgeai_robot_ws`。后续构建与安装链接均指向此目录；WSL中的项目副本不再作为本轮工作目录。

## 已实现

- 机器人接口统一为 `/robot1`、`/robot2`，移除机器人1到根话题的图像、深度、雷达、里程计、速度转发和TF别名。
- 导航、感知、搜寻、S1/C1、恢复退让和可视化使用机器人自己的接口与TF帧；公共地图、目标登记和任务接口保持共享。
- 公共地图管理器仅管理 `map_server`；各车的AMCL管理器只管理本命名空间的 `amcl`。
- 默认BT退让action使用相对路径；启动检查查询实际 `/robot1/bt_navigator`。
- 单机按活动机器人列表仅分配机器人1；探索、搜寻、S1沿用相同任务类、单位服务和任务树，删除旧AUTO建图任务分支。
- 单机、双机、ROS命令行和诊断工具通过 `scripts/ros/workspace-env.sh` 使用相同DDS participant index配置。疑似检测配置改为通配节点匹配，避免命名空间变化后仿真时间等参数失效。
- 退让action结果早于最终TF到达时，保留执行槽并在现有数据时效预算内确认移动，不撤销已提交的对方退让。

## 已通过

- 双机完整仿真两次通过：`20261116-221811-964-domain86` 与最终复测 `20261116-225004-297-domain86` 的 `mission-result.json` 均为 success=true。建图、两目标搜寻、SLAM到双AMCL交接、S1、C1均完成；两项救援真值核验通过。该记录已加 `.keep` 保留。
- 53项导航策略、分配、换点、任务配置和换位交接单元测试：`runtime/native-namespace-final-unit-tests.log`。
- 2项真实单/双AMCL生命周期测试：STARTUP、RESET、再次STARTUP，并保持超过bond超时：`runtime/native-namespace-lifecycle-tests.log`。
- 6项真实退让action测试：`runtime/native-namespace-yield-tests.log`。
- 11项真实ROS搜寻构造、S1分配/回退、双机冲突及换点测试：`runtime/native-namespace-fleet-tests.log`。
- 9项单机任务适配和定位先验/重试测试：`runtime/native-namespace-adapter-tests.log`；其中另有1项按域限制跳过的初始化测试，已在指定域单独执行通过：`runtime/native-namespace-initializer-tests.log`。
- 19项日志回归：`runtime/native-namespace-logging-tests.log`。
- 共101项选定回归用例通过；工作区布局检查通过。8包构建、RViz面板重建、任务协调器重建通过。
- 实际Gazebo/RViz截图已检查，窗口最大化及图像显示正常；没有机器人1兼容转发进程。

## 尚未通过与限制

- 单机全程尚未通过。`20261116-223337-272-domain86` 已走通统一分配、建图、搜寻、地图保存；后续地图保存进程报DDS participant index耗尽。入口DDS配置已统一，真实单机完整交接尚未复验成功。
- 后续 `20261116-223900-369-domain86` 在单机探索中，两个候选连续返回导航错误208（No feasible path for candidate goal headings），用尽任务重试预算后明确失败，没有误报完成；尚未进入定位/救援。未改规划器、碰撞或速度限制来绕过此失败。
- 原有 `validate_rectangle_footprint.py` 的DWB限速一致性断言失败：DWB最小速度 -0.3 m/s，速度平滑器下限 -0.2 m/s。本轮没有修改这两项配置，也没有放宽该断言。
- 最终双机复测使用统一单机分支后的代码，S1/C1真值核验再次通过；此前 `20261116-224751-025-domain86` 曾在Gazebo启动时出现 gz-transport/getenv 段错误，清理后原配置重试成功。
- 独立P2/专项启动入口已适配相关命名空间，未逐一完整实测。

当前默认双机入口保持不变。单机可用 `Start-Autonomous-Mission.ps1 -SingleRobot` 启动，但不得将其称为全程已验证通过。未提交或推送。
控制器短时诊断支持 Diagnose-Controller.ps1 -Robot robot1 / -Robot robot2，Linux入口使用 --robot robot1|robot2。机器人1点位/转向/恢复专项脚本已适配原生命名空间，未逐项完整仿真。

最终共享DDS配置后的17项真实ROS复验全部通过（41.1秒）：runtime/native-namespace-final-ros-tests.log。诊断面板按机器人身份区分足迹颜色，联合建图时保留机器人2相机状态。测试结束后未发现Gazebo、任务协调器或回归测试进程残留。

2026-11-16 后续清理：搜寻/S1任务树各收敛为唯一文件；移除旧AUTO派发、探索器控制同步、Explore Lite启动/构建分支及旧周期观察器。保留任务层搜寻转圈、启动扫描和手动/专项运动测试。本段后续清理后的验证结果另行记录，不将前文完整仿真视为清理后的实测。
