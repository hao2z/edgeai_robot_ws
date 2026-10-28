# 测试运行与观察

使用根目录 `Start-Autonomous-Exploration.ps1` 或 `Start-Simulation.ps1`。
重启前先等待本项目已登记的 Linux 进程退出，再关闭旧 PowerShell 和
Windows Terminal 标签页。只包含项目标签页的窗口会关闭；共用窗口只关
匹配的项目标签页，不终止共享 WindowsTerminal.exe。

每次测试保留在 `tests/results/runs/<时间>-domain<域号>/`，普通仿真为
`<时间>-simulation/`。`runtime/latest-session.txt` 记录最新目录的 WSL 路径。
同一测试不复用旧日志目录。

| 文件 | 内容 |
|---|---|
| sim.log | Gazebo、传感器、SLAM 及仿真 launch 原始输出 |
| nav.log | Nav2 规划、控制、行为树、恢复与碰撞监测原始输出 |
| explore.log | 开源探索器原始输出 |
| rviz.log | RViz 输出 |
| rosout.log | 自主探索测试的 /rosout 和节点发现记录 |
| console.log | 各组件输出汇总，增加接收时间和来源标记 |
| events.log | 中文关键事件、阶段变化和截图结果，监测窗口实时显示 |
| events.jsonl | 从原始日志识别的事件，包含对应原文证据 |
| ros/ | ROS 原生日志目录 |
| session.json | 自主探索测试参数、PID 标识、ROS 域、Gazebo partition |
| map.*, map-save.log, evaluation.log | 地图及保存、离线覆盖率评估结果 |
| exit-code.txt | 退出码，0 正常，143 为外部停止 |
| screenshots/ | 启动、每 60 秒、退出前的 Gazebo/RViz 截图及抓图记录 |

普通仿真的 ROS 节点输出集中在 sim.log，其余不适用的日志文件为空。
原始文件由组件直接写入；汇总和事件约每 0.15 秒更新，不会在退出时删除。
中文事件是对节点日志的解释，不能把“Start occupied”或安全停车当成物理碰撞证据。
停止阶段的任务取消与运行中的导航失败分别记录。

实时查阅，例如：

```powershell
$run = Get-ChildItem D:\KTH\Competition\edgeai_robot_ws\tests\results\runs -Directory |
    Sort-Object Name -Descending | Select-Object -First 1
Get-Content (Join-Path $run.FullName 'events.log') -Encoding UTF8 -Wait
# 将 events.log 换为 nav.log、sim.log 或 console.log 可查看原始输出。
```

Gazebo 出现后自动最大化。每次运行生成 observation.sdf，复用已安装的
Gazebo 原生 GUI 插件；相机位置为 (3.8, -4.35, 6) 米，RPY 为
(0, 0.80, 2.288) 弧度，倾斜观察 5×5 米场地。
调整位置可编辑 `simulation/tools/prepare_observation_world.py`。

截图流程：

1. 按窗口标题定位；多个同名窗口会明确报错，避免误拍旧测试。
2. 优先 PrintWindow；失败或近黑图时切换到前台屏幕抓取，并核实前台句柄。
3. 每张图片写入 JSON，包含时间、窗口尺寸、是否最大化、采样颜色数和黑色比例。
4. 启动就绪、运行中每 60 秒、退出前分别抓取；退出前等待抓图完成，有超时上限。
5. 检查 capture.log 和 JSON 后，仍须实际打开图片，确认场地完整、机器人可见、
   RViz 地图/轮廓位置合理，并检查相机画面等是否存在渲染异常。颜色检查不能证明内容正确。

Windows 截图事件经独立小文件交给 Linux 日志写入器，避免跨系统共享文件写入冲突。
抓图失败会记录原因，不会被当成成功；失败不会阻断机器人进程的最终清理。
