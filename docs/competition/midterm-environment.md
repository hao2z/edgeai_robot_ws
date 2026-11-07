# 中期检查仿真场景

> 2026-11-07 更新：默认任务入口现为双机器人流程：robot1 探索/S1 → 共享地图与双 AMCL → C1。
> 单机对照使用 `Start-Autonomous-Exploration.ps1 -SingleRobot`。
> 当前结果、预算限制与入口见 [项目状态](../project-status.md)。

场景依据 `KTH_EdgeAI_Midterm_Check_Task_Description.docx` 第 3 节和 Figure 1，使用文档原始的 0–5 米坐标。中期场景与原 `competition_rooms.sdf` 开发场景分开。

## 启动

Windows 从仓库根目录运行，默认在中期场景启动双机器人任务，先由 robot1 自主探索并完成 S1，再执行 C1：

```powershell
powershell -ExecutionPolicy Bypass -File .\Start-Simulation.ps1
# 同一自主任务入口，可额外设置探索预算等参数。
powershell -ExecutionPolicy Bypass -File .\Start-Autonomous-Exploration.ps1
```

robot1 世界起点为 (0.45, 0.55)、朝北，保持根命名空间；robot2 等待并使用独立前缀。检测使用 0.16 m 尺寸和 `1:normal,2:cooperative` 映射。探索自然完成后选择 S1（ID 1）救援；未发现时执行有预算的补充搜寻。超时或进程异常不算探索完成。S1 完成后保存最终地图和目标、停止 SLAM，并启动共享地图与两机 AMCL，随后执行 C1。单机对照加 `-SingleRobot`，仅完成 S1。

S1 参数在 `src/edgeai_navigation/config/midterm_single_rescue.yaml`，C1 参数在 `cooperative_rescue.yaml`；距离、停车与连续停留由任务协调器独立判定，不能仅以导航成功代替救援成功。S1要求0.50m内连续10秒，C1要求两机同时在0.70m内连续10秒。Windows 默认探索预算3600秒，各阶段另有预算，尚未统一成中期共享13分钟计时；详见 [任务流程](../task-workflow.md)。

救援导航明确失败（ABORTED，包括 TF_ERROR）后，默认保留 S1 目标额外重试 2 次；等待至少 1 秒墙钟时间，确认控制槽空闲、控制权未改变及配对 TF/里程计不超过 0.15 秒后重发。单次等待恢复上限为 5 秒墙钟时间，各次共用原导航和救援总预算。取消或控制权切换不会被重试覆盖。参数为 `navigation_max_retries`、`navigation_retry_delay_s`、`navigation_retry_wait_s` 和 `retry_tf_max_age_s`，详见 [救援执行说明](../normal-rescue.md)。

仅查看双机环境、不开启任务时显式使用：

```powershell
.\Start-Simulation.ps1 -SimulationOnly -Robots 2
```

仅仿真模式需要独立 SLAM 时再添加 `-SLAM`。

WSL 可使用现有带会话日志的入口：

```bash
EDGEAI_SIM_ARENA=midterm bash scripts/simulation/run-simulation.sh wslg \
  robot_count:=2 multi_robot:=true slam:=false
```

已激活 ROS 和本仓库安装环境时，也可直接启动独立 launch：

```bash
ros2 launch edgeai_simulation midterm.launch.py robot_count:=2
# 单机同样使用中期 robot1 起点，而不是原开发场地的默认起点。
ros2 launch edgeai_simulation midterm.launch.py robot_count:=1 slam:=true
```

双机使用 `/robot1/…`、`/robot2/…` 话题和前缀 TF；单机保留原无命名空间话题。两机独立 SLAM 不会自动产生共享地图；world 坐标也不能直接当作 SLAM 的 map 坐标。Part 2 仍需建立预建地图定位与文档坐标的对应。

## 布局与目标

| 元素 | 世界坐标或尺寸 |
| --- | --- |
| 边界基准线 | x/y = 0 和 5 m；墙体中心线位于基准线上，厚 0.10 m，内侧自由边界约为 0.05–4.95 m |
| W1 | (0, 3.1)–(2.2, 3.1)，厚 0.10 m |
| W1b | (2.2, 3.1)–(2.2, 3.6)，厚 0.10 m，与 W1 相连 |
| W2 | (3.4, 0)–(3.4, 2.8)，厚 0.10 m，与下边界相连 |
| O2 | 中心 (2.2, 2.0)，0.6 × 1.0 m |
| O3 | 中心 (4.4, 3.6)，0.8 × 0.6 m |
| S1 | 地面参考点 (1.0, 4.1)，ID 1，normal，面朝南方 −Y |
| C1 | 地面参考点 (4.1, 1.0)，ID 2，cooperative，面朝北方 +Y |
| robot1 | base_footprint 初始世界位姿 (0.45, 0.55, 0.01)，yaw = π/2，面朝北 |
| robot2 | base_footprint 初始世界位姿 (1.05, 0.55, 0.01)，yaw = π/2，面朝北 |

两车完整导航轮廓均在起始区域 x∈[0,1.4]、y∈[0,1.0] 内，互不重叠。起点不是文档强制坐标，而是满足图示位置关系和数值约束的选定位置。浅蓝起始区为无碰撞视觉标识。

两目标使用 `DICT_4X4_50`，黑色标记边长 0.16 m，白色面板边长 0.20 m，中心高 0.15 m；每侧白边为 0.02 m。独立支架底座为 0.11 × 0.11 × 0.025 m，立杆宽 0.02 m，接到面板下沿。面板、立杆和底座均有碰撞几何。墙和障碍高度取 0.50 m；这些高度和支架结构是团队实现选择，文档主要约束平面尺寸和标记参数。

地面圆盘以目标支架地面点为圆心，S1 半径 0.5 m、C1 半径 0.7 m；只用于视觉核对，无碰撞、不发送导航目标、不判定救援完成。两个圆盘不是旧开发场景的偏移停车点。

## 资源与验证

- `src/edgeai_simulation/worlds/midterm.sdf`：静态场地、支架和起始区。
- `src/edgeai_simulation/config/targets_midterm.yaml`：目标 ID、类型、位姿、救援半径和 10 秒要求。
- `src/edgeai_simulation/config/robots_midterm.yaml`：一台或两台机器人的起点。
- `src/edgeai_simulation/models/midterm/`：独立 0.16 m 标记资产，不覆盖原 0.20 m 开发资产。
- `src/edgeai_simulation/launch/midterm.launch.py`：中期专用启动入口。

可复现生成与数值验证：

```bash
/usr/bin/python3 src/edgeai_simulation/tools/generate_midterm_arena.py
/usr/bin/python3 src/edgeai_simulation/scripts/generate_targets.py \
  --package-dir src/edgeai_simulation \
  --config src/edgeai_simulation/config/targets_midterm.yaml
/usr/bin/python3 tests/simulation/validate_midterm_arena.py
bash scripts/setup/build-workspace.sh --packages-select edgeai_simulation
```

测试检查墙和障碍数值、标记解码、朝向、支架碰撞几何、目标圆盘半径和两车起始轮廓。几何检查不替代实际导航和相机识别验收。

2026-11-04 实机仿真检查记录在 `tests/results/runs/20261104-midterm-environment-check/`：场地和双机已在 Gazebo 显示，SDF 校验通过，两车相机/雷达/里程计话题均建立。使用真实 `/robot1/camera/image` 静态采图，S1 从世界位置 (1.7, 3.5)、朝向目标时解码 ID 1，C1 从 (4.1, 2.0)、朝南时解码 ID 2；叠加图为 `S1-camera-decoded.png` 和 `C1-camera-decoded.png`。该检查通过仿真 set_pose 放置相机采图，未验证自主导航或探索覆盖率，完成后已将机器人恢复到起点。

当前相机较低，在 S1 正南约 0.5 m 处近距离平视时，标记上半部超出图像，无法完整解码。保留文档规定的标记高度，观察任务应选择留有距离和完整视野的位置；不要把这一视场裁切现象直接当成转速或任务冲突。

自主任务入口和普通仿真入口的默认场景均为 midterm。显式 `-Arena competition`（自主入口）仍可使用旧开发场景，纯仿真 `-Arena simple` 可使用旧简单场景。中期自主入口在运行时覆盖标记尺寸和类型映射，原开发感知 YAML 保留供旧场景使用。中期场景不会调用假设旧起点和旧场地坐标的覆盖率评估器；地图照常保存，尚不报告中期覆盖率百分比。
