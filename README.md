# EdgeAI Robot Workspace

ROS 2 Jazzy / Gazebo 多机器人探索与救援。工作目录为本仓库；单机作为一台活动机器人的特例，使用 `/robot1`，双机使用 `/robot1`、`/robot2`，共享 `/map` 和 TF。

默认流程：联合建图与寻找全部目标 → 保存地图、切换AMCL定位 → S1 → C1。

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\Start-Autonomous-Mission.ps1
```

```bash
bash edgeai.sh build
bash edgeai.sh mission --joint-mapping
```

Windows启动参数在 `Start-Autonomous-Mission.ps1` 顶部；Linux入口运行 `bash edgeai.sh --help`。

手动导航使用 `Start-Navigation-Test.ps1`（如需确认实际专项入口，见[Windows入口](docs/windows-entrypoints.md)）或：

```bash
bash edgeai.sh navigation
```

## 日常维护入口

| 事项 | 唯一主要入口 |
|---|---|
| 安装与启动 | [Linux上手](docs/linux-quickstart.md)、[Windows入口](docs/windows-entrypoints.md) |
| 任务、分配、机器人职责 | [任务层协调](docs/fleet-task-coordination.md) |
| 导航策略与调参 | [分用途导航配置](docs/navigation-task-profiles.md) |
| 验证方式 | [测试说明](tests/README.md) |

比赛原文保存在 `docs/competition/`；专题资料与历史记录见[文档索引](docs/README.md)。
单机全流程的最新验证限制见[命名空间验证记录](docs/native-namespace-validation.md)。

源码按职责放在 `src/edgeai_mapping`、`edgeai_exploration`、`edgeai_navigation`、`edgeai_mission`、`edgeai_perception`、`edgeai_simulation`、`edgeai_robot_description` 和 `edgeai_navigation_rviz`、`edgeai_mission_control`。
`scripts/` 负责环境与启动；`runtime/`、`tests/results/` 保存本地状态和运行证据，不提交。

不再维护独立AUTO探索模式或Explore Lite分支。探索器只产生候选，由任务层派发导航；手动导航和专项运动测试保留。
