# 测试

只运行本次修改涉及的检查，不要求每次全量回归或完整仿真。

```bash
bash edgeai.sh test                         # 快速检查，无ROS节点
bash edgeai.sh test --profile ros-smoke     # 导航接口、退让和任务接管
bash edgeai.sh test --profile ros-tasks     # 任务流程有变化时使用
bash edgeai.sh test --list                  # 查看选择
bash edgeai.sh test test_navigation_authority # 单独指定模块
```

ROS测试使用独立域，默认197，不启动Gazebo或停止用户仿真。启动完整仿真验证时使用任务或运动测试入口；不要将模拟后端测试当作真实运动成功。

维护边界：

- 快速组检查导航策略、执行槽、分配规则和任务配置。
- ROS组检查实际action/service的接单、取消、终态、超时、退让及任务交接。
- 感知、建图、控制器专项仅在对应模块变化时运行。
- 全程仿真用于阶段性验收，结果留在 `results/runs/`，不复制到多份文档。

已删除旧独立救援执行入口、AUTO恢复、根命名空间阶段适配的4个过时ROS测试模块及旧观察器测试；保留当前任务树、分配、修正和导航恢复覆盖。旧版测试在本地 `runtime/retired-tests-20261116/` 留有备份。
顶层任务回归收敛为8个阶段/交接/取消关键用例；底层重试和修正由对应单位行为测试负责。

人工选点、转向、恢复、协作及中期检查入口继续保留，见[Windows入口](../docs/windows-entrypoints.md)。
