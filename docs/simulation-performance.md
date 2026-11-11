# 仿真性能检查

2026-11-11，启动异常来自 `simulation_real.launch.py` 内部重复导入 `yaml`，导致使用前被视为未赋值的局部变量。已移除局部导入，启动配置测试直接构造双机 SLAM 的 launch actions，不运行仿真。

会话 `20261111-022352-944-domain86` 的 10 秒只读采样保存于会话目录 `performance.json`：日志窗口倍率约 0.50x。CPU 100% 表示一个核心，Gazebo 服务约150%，诊断100%，导航权限68%，RViz65%，Gazebo界面59%，ArUco57%，疑似目标56%。检测耗时记录中，ArUco中位数2.49ms、p95为13.63ms；疑似目标平均8.57ms，不能仅凭进程CPU认定图像算法是主要瓶颈。实际应用使用D3D12 NVIDIA渲染，当前没有内存不足证据。

该轮 `/clock` 实测约393–449墙钟Hz，多个Python节点有显著主线程开销。新增标准C++ `topic_tools/throttle`，默认以100墙钟Hz转发原始仿真时间戳，减少全体节点接收时钟的次数。没有修改物理步长、传感器时间戳或控制周期；隔离ROS测试已检查限频、暂停不推进和仿真时间重置。是否改善整轮倍率仍需下一轮用户仿真对照确认。

主要调节入口（启动前设置，下次启动生效）：

| 入口 | 默认 | 用途 |
| --- | --- | --- |
| `EDGEAI_CLOCK_RATE` | `100` | 时钟转发墙钟Hz；`0`恢复不限频，或设置20–1000用于对照 |
| `EDGEAI_CAMERA_RATE` | `30` | RGB-D仿真Hz，已恢复原频率，保留640×480 |
| `suspect_targets.yaml` → `processing_hz` | `5.0` | 疑似目标仿真Hz，由2提高；仍只处理最新可配对图像 |
| `aruco_targets.yaml` → `processing_hz` | `30.0` | ArUco检测仿真Hz；与默认相机频率对齐 |
| `EDGEAI_RVIZ_FPS` | `30` | RViz显示帧率 |
| `EDGEAI_EAGER_POINT_CLOUD` | `false` | 完整点云桥默认按需启用；深度避障继续使用深度图投影 |

原生安装脚本包含 `ros-jazzy-topic-tools`，Pixi准备脚本会补齐该依赖。改变launch配置不会热更新正在运行的仿真，请由操作者自行决定重启时机。

运行期间可从WSL项目根目录执行低开销采样：

```bash
bash scripts/ros/ros-command.sh python3 scripts/ros/performance_probe.py --duration 10
```

该脚本只读取所选ROS域进程的 `/proc` 计数和现有日志，不订阅点云、不发指令、不停止进程。默认覆盖当前会话 `performance.json`，要保留多次采样可指定 `--output`。倍率由约1Hz日志端点估算；无新记录时不报告倍率。对照应选择相同场景和任务阶段。

本机自编译C++缓存的 `CMAKE_BUILD_TYPE` 仍为空，可能增加自定义模块开销；后续空闲时可用 `bash scripts/setup/build-workspace.sh --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo` 重建。当前运行期间未触发重编译。已安装的ROS规划器不受此自定义缓存影响。
