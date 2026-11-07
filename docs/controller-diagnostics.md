# 控制器短时诊断

仿真运行时，在工作区PowerShell执行：

```powershell
.\Diagnose-Controller.ps1
.\Diagnose-Controller.ps1 -Duration 30 -Hz 10
```

默认20墙钟秒、10 Hz；自动读取最新会话和ROS域。采样期间可继续手动选点。
Ctrl+C可提前结束并保存已有摘要。不启动/停止仿真、不发布速度、不发送导航动作，
不订阅图像、激光或整张代价地图；采样上限120秒、20 Hz。

实时显示局部控制器实例、最新FollowPath状态、反馈距离和反馈年龄、
RPP主动转向标志及前视点，以及导航指令→平滑后→最终→里程计的v/w。
原地对齐可能由外层Shim执行，RPP转向标志false不代表机器人没有旋转。
控制器实例按仲裁器到点策略对应，插件类型取会话启动配置；不冒充实时参数快照。

结果保存在当前会话的 `controller-diagnostics/<时间-进程号>/`：

- `summary.txt`：人读摘要、各阶段速度范围/墙钟接收Hz、FollowPath状态计数、代表性告警。
- `summary.json`：结构化摘要、最近局部控制器状态、按节点区分的告警及原始片段。
- `samples.jsonl`：逐次速度、输入年龄、FollowPath目标UUID/反馈、RPP前视及跟踪路径端点。
- 启动配置副本：`nav2-runtime.yaml`、`motion-limits.json`（存在时）。

“平滑器前后差异”可能来自正常加减速或异步采样；“最终指令非零但实测静止”也可能有短时延迟。
这些仅作为线索，不能单凭计数认定碰撞、控制器失效或底盘故障。
FollowPath最新状态可能属于刚结束的动作；反馈按目标UUID匹配，并保留数据年龄。
碰撞监视状态可能仅在变化时发布，未收到不等于碰撞监视器未启用。

WSL直接调用或指定历史会话所属域：

```bash
bash scripts/ros/with-ros-env.sh python3 scripts/ros/controller_probe.py --duration 20 --hz 10
# 可添加 --session /mnt/d/.../tests/results/runs/<会话>
```

指定会话只用于选择域和记录位置；脚本采集的是执行时的实时话题，不会重放历史数据。
