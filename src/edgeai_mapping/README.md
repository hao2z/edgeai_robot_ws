# 建图与公共定位

本包包含单机/多来源SLAM、初始激光相对配准、对方车身扫描过滤、公共地图和来源关键帧图发布。它不分配任务或导航目标。

- `config/slam_toolbox.yaml`：单机SLAM基线、图发布周期。
- `config/joint_mapping.yaml`：双机粗略初值、配准门槛、公共图更新和扫描过滤。
- `launch/joint_mapping.launch.py`：公共SLAM后端、初始化及传感器状态；需要已运行的双车传感器与TF。
- `edgeai_mapping/relative_registration.py`：相对配准；`scan_mask.py`：可复用的扫描过滤几何。
- `anchored_slam_toolbox` / `joint_slam_toolbox`：单机/公共图可执行程序。

公共输出为 `/map`、双方 `map→robotN/odom`、`/slam_toolbox/anchor_graph` 和 `/joint_mapping/status`；冻结接口沿用 `/slam_toolbox/finalize_anchors`。目标定位通过公开图快照按来源绑定，不依赖后端内部对象。

完整联合任务使用 `Start-Autonomous-Mission.ps1 -JointMapping` 或 `bash edgeai.sh mission --joint-mapping`，由 `edgeai_mission/launch/joint_mapping_mission.launch.py` 组合导航和独立探索器。任务分配/让路位于mission，实时对方导航障碍适配位于navigation。

```bash
bash scripts/setup/build-workspace.sh --packages-select edgeai_mapping edgeai_navigation edgeai_mission edgeai_simulation
```
