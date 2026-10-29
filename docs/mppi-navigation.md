# MPPI 探索导航

MPPI 是尚未通过起步方向验收的实验配置，须显式使用 `-Controller mppi`。
2026-10-29 原生 Jazzy 迁移进展与最新实测见 `native-ros-migration.md`。
原生 apt MPPI 的官方 demo 和比赛机器人前方目标已通过，侧向转向仍待修复。
以下反向记录描述旧环境，不能当作原生环境当前结果。

默认保留矩形 DWB 基线，MPPI 使用显式参数选择。
直接 WSL 入口用 `EDGEAI_CONTROLLER=mppi` 选择实验配置，默认 `dwb`。

```powershell
.\Start-Autonomous-Exploration.ps1 -Controller mppi -Duration 300
```

专用配置为 `src/edgeai_navigation/config/nav2_exploration_rectangle_mppi.yaml`，
由 `src/edgeai_navigation/tools/prepare_mppi_config.py` 组合现有矩形 DWB 几何配置和 MPPI 基础参数生成。
完整配置生成入口 `prepare_nav2_config.py` 同样调用该函数。
runner 必须同时选择 mppi + rectangle，避免 rectangle 配置覆盖 controller 选择而实际仍运行 DWB。

控制器直接使用 MPPI DiffDrive，不再经过 Rotation Shim。预测为 80 步 × 0.05 秒 = 4 秒，
每周期 1200 个样本，控制频率 20 Hz；最大前进 0.2 m/s、倒车 0.08 m/s、角速度 0.65 rad/s，
加速度与速度平滑器一致。4 秒最大直行 0.8 m，连同车体外接半径仍在 3 m 局部窗口内。

使用完整矩形足迹评估，每一步检查障碍代价；保留碰撞监控、代价地图和原生恢复树。
PathAlign 使用可行路径朝向，PathAngle mode=2，启用路径方向切换约束，使用权重 2 的较弱 PreferForward 偏好，
以配合 Smac Lattice 的倒车段。路径跟踪与到点接管距离设为 0.8 m。
这是一份需实测的初始配置，不是全局最优或必然脱困保证。

MPPI 预测轨迹通过 `optimal_trajectory → local_plan` 映射复用 RViz 粉色轨迹和诊断输入。
启用轨迹发布但不默认显示所有采样轨迹；可视化抽样间隔 100/5 控制额外显示开销。
如果改成部署环境，应评估关闭 visualize 的性能收益；关闭后当前 local_plan 显示也会停止更新。

验证：`tests/exploration/test_mppi_profile.py` 检查与基线几何、规划器、安全配置一致，
并验证执行器限制、预测窗口和路径方向性约束；图形运行结果见后续记录。

2026-10-29 起步验证未通过：164118、164408、164546、164841 运行均观察到目标与处理后的路径向前、
控制输出向后的情况。164703 临时禁止普通倒车后停在起点，不能算通过。
关闭对齐 critic 也未消除反向现象；试验不支持简单归因于路径对齐权重。
实验输出已重新由生成器生成，清除试验期间直接编辑 YAML 留下的参数差异。
故障日志保留，不将 exit-code=0 等同于导航成功。

参考：https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/mppi_controller/configuring_mppic/
