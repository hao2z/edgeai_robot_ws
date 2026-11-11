# 当前文档入口

日常使用从以下指南开始；带日期的诊断和[历史目录](history/README.md)用于追溯，不作为当前默认配置说明。

| 内容 | 文档 |
|---|---|
| 成果、实测证据、遗留问题 | [项目状态](project-status.md) |
| Linux安装、构建、启动 | [队员上手](linux-quickstart.md) |
| 比赛要求和场地 | [中期待办](competition/midterm-checklist.md)、[仿真环境](competition/midterm-environment.md) |
| 行为树、协调与调用接口 | [任务流程](task-workflow.md) |
| S1与C1 | [单机救援](normal-rescue.md)、[协同救援](cooperative-rescue.md) |
| 导航朝向、规划、控制、恢复 | [逐点接口](navigation-goal-policy.md)、[Smac Lattice](smac-lattice.md)、[RPP](rpp-controller.md)、[恢复](navigation-recovery.md) |
| 速度联动、手动测试 | [速度约束](navigation-speed-limits.md)、[运动测试环境](navigation-test-environment.md) |
| 目标与地图、疑似点、视锥 | [地图绑定](target-map-anchoring.md)、[疑似目标](suspected-target-search.md)、[视锥搜寻](frustum-search.md) |
| 双机状态、诊断、真值、性能 | [RViz状态](rviz-status.md)、[控制器诊断](controller-diagnostics.md)、[真值](simulation-truth.md)、[性能采样](simulation-performance.md) |

修改源码后先构建；配置修改在下一次启动生效。会话实际参数及运行证据保存在`tests/results/runs/`，不会随代码上传。
