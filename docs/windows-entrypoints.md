# Windows入口与测试职责

所有命令从仓库根目录运行。

| 入口 | 职责 |
|---|---|
| `Start-Autonomous-Mission.ps1` | 主任务及其参数；默认双机器人流程 |
| `Start-Simulation.ps1` | 只启动场景，可选机器人数量、场地、SLAM |
| `Start-Navigation-Test.ps1` | 人工选点、真值地图运动测试快捷入口 |
| `Start-Cooperative-Test.ps1` | 协作救援专项测试 |
| `Stop-Simulation.ps1` | 统一停止任务与仿真，默认关闭项目工作终端；加`-KeepTerminals`保留终端 |
| `Open-ROS-Terminal.ps1` / `Open-ROS-Monitor.ps1` | 分别打开命令终端、实时日志监视 |
| `Diagnose-Controller.ps1` / `Repair-WSLg.ps1` | 控制器诊断、显示环境修复 |

```powershell
.\Start-Simulation.ps1 -Robots 2 -SLAM
.\Start-Navigation-Test.ps1
.\Stop-Simulation.ps1
.\Stop-Simulation.ps1 -KeepTerminals
```

`midterm`场地最多两台机器人；四台机器人开发测试使用`-Arena simple`。

常规算法、配置和ROS Action回归保留在`tests/exploration/`。人工仿真检查在`tests/simulation/`，执行前需停止已有仿真；`verify-simulation-extension.sh`明确使用开发场地和四机器人配置，检查传感器及可选运动，不代表中期任务验收。

已删除无独立检查能力的`tests/simulation/launch-test.sh`，使用上述纯仿真入口替代。已删除路径失效的`run-authority-tests.sh`包装，策略与ROS回归仍分别运行：

```bash
bash scripts/ros/ros-command.sh python3 tests/exploration/test_navigation_authority.py
bash scripts/ros/ros-command.sh python3 tests/exploration/test_authority_ros.py
```

旧MPPI专项工具集中到`tests/manual/mppi/`；当前默认控制器不是MPPI，无需在日常回归中执行这些工具。
