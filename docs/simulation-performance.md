# 仿真性能检查

## 2026-11-18 双机基准

Competition 整合任务在 1 ms 物理步长、performance 模式下完成。
建图、AMCL 交接、S1 和 C1 分别约为 0.60、0.58、0.58 和 0.57 倍。
S1 和 C1 均通过真值核验。Gazebo 在任务成功后的退出阶段出现段错误。
历史同场地成功样本的对应倍率约为 0.41、0.41、0.40 和 0.37。
两次任务路线和重试次数不同，倍率变化无法分摊到单项改动。

当前相机默认 15 Hz，ArUco 和同车疑似检测共享 RGB 图像和内参。
退让 Action 常驻并按请求启停里程计和 TF 订阅，开关见导航包的
`config/peer_yield.yaml`。AMCL 车身过滤对雷达束批量计算，只在实际命中时复制扫描。
任务派发器负责取消和换点，每车导航仲裁只管理执行槽与终态确认。

## 运行模式（2026-11-12）

Windows 主入口 `Start-Autonomous-Mission.ps1` 顶部的 `RuntimeProfile` 可设置
`performance`（Windows 默认）或 `debug`；命令行示例：

```powershell
.\Start-Autonomous-Mission.ps1 -RuntimeProfile performance
```

Linux/WSL 对应 `bash edgeai.sh mission --runtime-profile performance`。
设置下次启动生效，不会停止或重配正在运行的仿真。

| 项目 | debug | performance |
| --- | --- | --- |
| Gazebo GUI | 开 | 默认关；传感器渲染仍运行 |
| RViz | 30 FPS | 15 FPS |
| 详细导航诊断节点 | 开，两车状态面板与 alignment 记录 | 关，相应面板数据及 alignment 记录不可用 |
| 联合建图 mapping_diagnostics | 开，图优化跳变诊断及 joint-mapping-status 记录 | 关；SLAM、初始配准、实时建图与对方过滤保留 |
| Windows 自动截图 | 开 | 关 |
| 相同单元反馈 | 每单元限频 1 秒 | 每单元限频 5 秒 |
| 单元 JSONL 审计 | 全部调用 | 相同结果每操作 5 秒；变化及失败保留 |
| 相机、感知、控制、定位、避障、真值核验 | 保留 | 保留，频率不变 |

`EDGEAI_GUI`、`EDGEAI_RVIZ`、`EDGEAI_RVIZ_FPS` 可在 Linux 环境单独覆盖显示默认值。
以上入口负责选取配置，不修改任务树、检查周期、超时或重试规则。
单双车终端及 RViz 均隐藏无动作的例行检查文字，实际分配、让路、失败、恢复和
救援结果仍显示。`TASK_UNIT_STARTED` 改为 ROS DEBUG 日志，单元服务客户端复用，
避免每次树循环重复创建 DDS 客户端。反馈限频和客户端复用需要重新编译 mission 包。

此前双机采样中详细导航诊断约占单核73%、RViz51%、Gazebo GUI49%。
这证明它们消耗资源，尚不能证明此模式将提升多少整轮仿真倍率；应使用相同场景、
相同阶段做对照。performance 无 alignment 日志，原 performance_probe 的日志倍率
不可用，仍可采样 CPU；倍率需独立采样仿真时钟。

2026-11-11，启动异常来自 `simulation_real.launch.py` 内部重复导入 `yaml`，导致使用前被视为未赋值的局部变量。已移除局部导入，启动配置测试直接构造双机 SLAM 的 launch actions，不运行仿真。

会话 `20261111-022352-944-domain86` 的 10 秒只读采样保存于会话目录 `performance.json`：日志窗口倍率约 0.50x。CPU 100% 表示一个核心，Gazebo 服务约150%，诊断100%，导航权限68%，RViz65%，Gazebo界面59%，ArUco57%，疑似目标56%。检测耗时记录中，ArUco中位数2.49ms、p95为13.63ms；疑似目标平均8.57ms，不能仅凭进程CPU认定图像算法是主要瓶颈。实际应用使用D3D12 NVIDIA渲染，当前没有内存不足证据。

该轮 `/clock` 实测约393–449墙钟Hz，多个Python节点有显著主线程开销。新增标准C++ `topic_tools/throttle`，默认以100墙钟Hz转发原始仿真时间戳，减少全体节点接收时钟的次数。没有修改物理步长、传感器时间戳或控制周期；隔离ROS测试已检查限频、暂停不推进和仿真时间重置。是否改善整轮倍率仍需下一轮用户仿真对照确认。

主要调节入口（启动前设置，下次启动生效）：

| 入口 | 默认 | 用途 |
| --- | --- | --- |
| `EDGEAI_CLOCK_RATE` | `100` | 时钟转发墙钟Hz；`0`恢复不限频，或设置20–1000用于对照 |
| `EDGEAI_CAMERA_RATE` | `15` | RGB-D 仿真 Hz，保留 640×480 |
| `suspect_targets.yaml` → `processing_hz` | `5.0` | 疑似目标仿真Hz，由2提高；仍只处理最新可配对图像 |
| `aruco_targets.yaml` → `processing_hz` | `30.0` | ArUco 检查新帧的频率，处理量受相机帧率限制 |
| `EDGEAI_RVIZ_FPS` | `15` / `30` | performance / debug 的 RViz 显示帧率 |
| `EDGEAI_EAGER_POINT_CLOUD` | `false` | 完整点云桥默认按需启用；深度避障继续使用深度图投影 |

原生安装脚本包含 `ros-jazzy-topic-tools`，Pixi准备脚本会补齐该依赖。改变launch配置不会热更新正在运行的仿真，请由操作者自行决定重启时机。

运行期间可从WSL项目根目录执行低开销采样：

```bash
bash scripts/ros/ros-command.sh python3 scripts/ros/performance_probe.py --duration 10
```

该脚本只读取所选ROS域进程的 `/proc` 计数和现有日志，不订阅点云、不发指令、不停止进程。默认覆盖当前会话 `performance.json`，要保留多次采样可指定 `--output`。倍率由约1Hz日志端点估算；无新记录时不报告倍率。对照应选择相同场景和任务阶段。

本机自编译C++缓存的 `CMAKE_BUILD_TYPE` 仍为空，可能增加自定义模块开销；后续空闲时可用 `bash scripts/setup/build-workspace.sh --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo` 重建。当前运行期间未触发重编译。已安装的ROS规划器不受此自定义缓存影响。

## 2026-11-11 双机性能记录

双机会话 `20261111-234335-804-domain86` 的10秒只读采样见本地
`runtime/joint-performance-current.json`：倍率约0.28x，Gazebo服务106%单核、
探索任务79%、双车导航诊断73%、RViz51%、Gazebo界面49%、两路桥接合计72%。
历史单机会话 `232015-311` 倍率中位数约0.515，双机 `233225-295` 约0.259；
阶段与后台负载不同，不能当作严格对照，也不能把共享SLAM直接认作主因。
当时双车建图开启两路 640×480、30 Hz RGB-D，单车建图期间等待车不启用相机。

已实施，下一次启动生效：

- `edgeai_mission/scripts/task_units.py`：数据节点持续注册在同一个执行器中；
  使用公开的 `wait_for_ready_callbacks`，队列为空即结束本轮处理，
  每轮最多32×数据节点数个回调、5ms处理预算，随后处理任务服务。
  旧算法嵌套等待继续使用同一个全局执行器，保持单线程，不引入后台竞争。
  隔离双节点、各20个订阅的空闲实验中，100轮轮询CPU时间由2.61s降至0.093s；
  这衡量空闲轮询开销，并非整轮任务或仿真倍率收益。
- `scripts/ros/navigation_diagnostics.py`：两车面板共享一个TF缓存与订阅，
  仍各自记录状态、坐标、速度和恢复信息。
- `edgeai_robot_description/urdf/edgeai_robot_real.urdf.xacro`：
  `JointStatePublisher/update_rate=20`，避免按1000Hz物理步发布轮关节状态。
  当前Gazebo8插件已确认支持此字段；里程计50Hz、雷达20Hz、相机30Hz和控制频率保持不变。

后续优先对照：相同双机阶段、相同界面数量，采样优化前后进程CPU及倍率；
再分别隔离Gazebo GUI/RViz渲染与传感器渲染开销。机器人一的图像兼容转发仍有开销，
后续迁移消费者到命名空间原始话题后可以移除。以上改动没有声称已提高完整仿真倍率，
当前任务适配器仍支持两台机器人，仿真实体入口支持1–4台。

验证：相关基础回归60项通过，新增双节点回调测试通过。双车S1的6个场景中5项首轮通过，
“就近机器人2执行”场景首次因TF/里程计配对不及时失败，单独复测通过；保留
`runtime/fleet-performance-rescue-tests.log` 与 `runtime/fleet-performance-rescue-recheck.log`，
不以复测通过代替整轮稳定性验证。没有启动或停止操作者的Gazebo仿真。
