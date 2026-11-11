# 导航、任务与候选算法的边界

本轮将13个模块/工具从navigation移出，正式安装三个Python包，保持原算法与Action协议。

| 位置 | 职责 |
|---|---|
| `edgeai_navigation/scripts` | Nav2配置生成、请求入口、唯一执行槽、运动证据、定位/障碍过滤 |
| `edgeai_navigation/edgeai_nav` | 导航与任务共用的平面坐标变换、命名空间和代价地图配置 |
| `edgeai_mission/edgeai_tasks` | 单机/协作救援ROS适配、救援几何和条件、末端修正、目标再搜寻编排 |
| `edgeai_exploration/edgeai_search` | 视锥、疑似目标视角和分散观察位候选计算 |
| `edgeai_mission/scripts` | 单位服务适配、任务准备、地图交接和导出、成功证据 |

任务树仍控制默认任务顺序、检查、修正和重试；`task_units.py`调用可复用模块。S1/C1末端修正代码集中在`edgeai_tasks.rescue_alignment`，不再把C1修正动作实现留在服务分发器内。候选几何不发送运动命令，实际运动仍由Nav2 Action执行。

Python导入使用`edgeai_nav.*`、`edgeai_tasks.*`、`edgeai_search.*`，通过`ament_cmake_python`安装，新增依赖已登记。导航激光过滤器直接依赖公共几何，不反向依赖任务包。任务适配器已移除向navigation脚本目录插入`sys.path`的代码。旧任务类的完整`execute()`仍用于专项回归，未改成默认任务入口；后续可独立迁移这些专项测试，再淘汰其兼容流程。原`fleet_config.py`保留薄兼容导入，不复制实现。

修改布局后必须重新构建，再启动：

```bash
bash scripts/setup/build-workspace.sh --packages-select edgeai_exploration edgeai_mission edgeai_navigation edgeai_perception
```

S1导航现在使用独立`GridBasedRescue`，指定朝向导航继续用`GridBasedHeading`，C1用`GridBasedCooperative`。候选预规划和实际任务使用一致ID，避免配置覆盖。任务配置入口仍为`edgeai_navigation/config/navigation_tasks/`，救援参数路径暂保持兼容，以免同时改变使用习惯。
