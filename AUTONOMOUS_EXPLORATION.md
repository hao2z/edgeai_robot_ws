# 自主探索与救援入口

当前基线（2026-11-10）为native ROS2 Jazzy、RPP/Rotation Shim、Smac Lattice、SLAM Toolbox和前沿探索。
默认任务：robot1建图并找到S1/C1 → S1 → 最终地图交接 → 两机AMCL → C1。

Windows运行 Start-Autonomous-Exploration.ps1；Linux运行：

~~~bash
bash edgeai.sh build
bash edgeai.sh check
bash edgeai.sh mission
~~~

- [当前成果、证据与待办](docs/project-status.md)
- [Linux安装与日常命令](docs/linux-quickstart.md)
- [任务树和职责](docs/task-workflow.md)
- [中期要求核对](docs/competition/midterm-checklist.md)
- [控制器参数](docs/rpp-controller.md)、[速度联动](docs/navigation-speed-limits.md)
- [手动选点测试](docs/navigation-test-environment.md)、[短时诊断](docs/controller-diagnostics.md)
- [前沿/观察历史基线](docs/history/autonomous-exploration-2026-10-29.md)

最新会话路径记录在runtime/latest-session.txt；实际参数见会话nav2-runtime.yaml和mission-runtime.json。
探索超时不等于探索成功，导航成功不等于救援成功，估计成功不等于真值验收。
