# 导航、任务与候选算法的边界

## 联合建图后的模块边界

`edgeai_mapping` 已从导航与仿真包拆出；多机器人目标分配仍由任务层负责。

| 包 | 归属 |
|---|---|
| `edgeai_mapping` | 单/多来源SLAM后端、初始激光相对配准、公共地图与关键帧图发布、冻结和地图导出接口 |
| `edgeai_exploration` | 每台机器人独立的前沿检测、可达端点/观察候选、候选可视化；不决定多车目标分配 |
| `edgeai_mission` | 顶层任务协调器、各任务多机器人协调节点、任务行为树、目标预留、超时/重试、空闲车让路和救援点重分配 |
| `edgeai_navigation` | 单车Nav2规划/控制/恢复、请求执行槽、代价层，包括对方车身障碍 |
| `edgeai_perception` | 目标检测、图像时刻定位、按来源关键帧绑定和地图更新后的目标修正 |

探索任务的多机器人协调当前是 `edgeai_tasks/exploration.py` 中的 `fleet_exploration_coordinator`，树在 `config/tasks/explore.xml`；不随SLAM一起迁入mapping，也不塞进前沿生成器。共享让路选点与诊断在 `edgeai_tasks/fleet_yield.py`，由任务树调用；后续各任务复用时仍由各自任务协调器判断哪些机器人可以移动。顶层 `mission_coordinator` 只选择任务和检查阶段门槛。

建图后端、初始相对配准与扫描过滤位于 `edgeai_mapping`，参数为 `config/slam_toolbox.yaml` 和 `config/joint_mapping.yaml`。其 `launch/joint_mapping.launch.py` 只启动公共SLAM及初始化，不启动导航或任务。

`edgeai_mission/launch/joint_mapping_mission.launch.py` 组合建图、第二台Nav2、探索器与兼容relay。前沿计数汇总位于mission的 `scripts/joint_frontier_monitor.py`；对方导航障碍位姿发布位于navigation的 `scripts/joint_peer_poses.py`。navigation的激光定位过滤复用mapping的纯几何 `edgeai_mapping.scan_mask`，不再依赖任务脚本。

前沿优先保留现有上游算法与适配，不另建重复包。联合模式采用被动候选接口；旧单机自动派发入口暂时兼容。完成调用迁移后，再统一为候选计算与任务派发分离，避免在包迁移同时重写算法。

依赖原则：mission组合mapping、exploration、navigation；mapping不导入mission或navigation业务模块；perception消费关键帧图接口，不访问SLAM内部对象。通用几何若仍被多包依赖，单独整理共享模块，避免mapping反向依赖navigation只为导入一个函数。

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
bash scripts/setup/build-workspace.sh --packages-select edgeai_mapping edgeai_exploration edgeai_mission edgeai_navigation edgeai_perception edgeai_simulation
```

S1导航现在使用独立`GridBasedRescue`，指定朝向导航继续用`GridBasedHeading`，C1用`GridBasedCooperative`。候选预规划和实际任务使用一致ID，避免配置覆盖。任务配置入口仍为`edgeai_navigation/config/navigation_tasks/`，救援参数路径暂保持兼容，以免同时改变使用习惯。
