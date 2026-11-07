# 任务流程入口

当前默认流程：robot1 探索/建图 → 未发现 S1 时补充搜寻 → S1 导航/独立救援判定/重试 → S1 完成 → 保存地图并停用 SLAM → 两机 AMCL → C1 双停车位导航 → 两机共同连续停留。

| 层级 | 定义位置 | 职责 |
|---|---|---|
| 启动与阶段交接 | `scripts/simulation/run-open-source-exploration.sh` | 启动进程、自然探索完成、S1结果交给协作协调器 |
| S1任务 | `src/edgeai_navigation/scripts/normal_rescue.py` | `execute`取得控制权/搜寻，`complete_rescue`重检重试，`rescue_attempt`导航后独立验收 |
| S1判定与修正 | `rescue_policy.py`、`rescue_alignment.py` | 救援半径或精确点规则、朝向、停车、连续时间；小幅修正 |
| C1协调 | `src/edgeai_navigation/scripts/cooperative_rescue.py` | 地图/定位交接、两机目标分配、共同验收 |
| 每次导航 | `src/edgeai_navigation/config/behavior_trees/explore_recovery.xml` | 规划、路径复用、局部控制和脱困；`goal_policy.py`生成逐任务派生树 |

配置：S1使用 `midterm_single_rescue.yaml`，C1使用 `cooperative_rescue.yaml`。修改参数后重启。`navigation_max_retries` 控制每次导航内部重试，`rescue_max_retries` 控制确认失败后的任务级重检和重做，均共用原有时间预算。最终结果保存每次导航、候选位、修正和任务重试记录。

## 是否统一到行为树

可以。建议增加上层任务行为树，保留现有 Nav2 导航行为树。上层表达探索、目标搜寻、S1救援、地图交接、两机定位、C1救援的顺序与有界重试；几何计算、目标感知、定位质量和停留判定继续由各自模块负责。

接入前应把现有阶段包装成可取消、可反馈进度、返回结构化失败原因的 ROS action。上层树只调用阶段 action 和条件节点，不另建速度控制权，不绕过导航仲裁。现有 Nav2 树继续负责一次导航的内部恢复。不要在两个层级重复叠加未知次数的重试，也不要让行为树把等待超时误当成动作已停止。

当前尚未迁移上层任务树；总流程仍由脚本与两个任务协调器执行。先验证本轮半径判定与失败重试，再迁移上层编排，可减少同时改变任务语义和调度机制带来的排查困难。
