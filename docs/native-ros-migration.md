# 原生 ROS Jazzy 迁移与验证

2026-10-29，WSL Ubuntu 24.04。源码继续使用 `edgeai_robot_ws`，
系统包来自 apt；原 Pixi 环境保留回退。

## 环境与构建

```bash
export EDGEAI_ROS_BACKEND=native
bash scripts/setup/prepare-open-source.sh
```

首次系统安装以 root 运行 `scripts/setup/install-native-ros.sh`，普通用户运行
`rosdep update`。项目 5 个包和固定版本 frontier 探索器已在原生环境编译通过；
`rosdep check --from-paths src --ignore-src` 在探索器 overlay 下通过。
Python 为 `/usr/bin/python3`，rclpy/Nav2/ros_gz 来源 `/opt/ros/jazzy`。
原生构建目录 `~/.cache/edgeai-native-build`，探索器及可选补丁位于
`~/.cache/edgeai-native`，不加载旧环境的二进制。

已安装 Nav2 1.3.13、SLAM Toolbox 2.8.5、ros_gz 1.0.24。
系统 xtensor 0.25.0 / xsimd 12.1.1 的独立 cos(0) SIMD fast-math 检查通过。
这与旧环境匹配头文件组合的反号故障不同。

## 已完成的测试与边界

- 官方 apt MPPI：`20261029-182451-domain88-native-official`，
  前方 0.6 m 目标成功；移动约 0.35 m，符合官方约 0.25 m 容差。
  已查看 Gazebo/RViz 截图，RViz 显示 reached、0 次恢复。
- 比赛机器人：`20261029-182753-180-domain86-turn` 实际 scenario=forward，
  前方 0.6 m 目标成功，最终 x=0.451 m，误差 0.149 m，当前容差 0.15 m。
- 矩形轮廓/门宽/2.5 cm 栅格一致性通过；6 个配置及日志测试通过，目录检查通过。
- 比赛场地：`20261029-183056-731-domain86`，首个目标成功；第二个侧向目标
  小幅前后摆动并触发无进展，尚未通过完整建图。
  软件渲染实测仿真倍率约 0.3--0.4，放大了墙钟时间上的缓慢。
  本轮因运行期间启动脚本被编辑而中断（127），不能作为正常超时完成结果。
- 原生环境仍见 RViz indexed-image 着色器错误、一次 Gazebo GUI OpenGL
  初始化失败，以及停测时 collision_monitor 段错误；不宣称 GUI/退出流程全部稳定。

## 运行与回退

默认 backend 为 native；`runtime/ros-backend` 和显式环境变量可覆盖。
PowerShell 支持 `-Backend native` / `-Backend pixi`，明确转交给 WSL 和子窗口，
启动前检查新模块是否已在所选环境构建。已写入本机 native 标记。
Linux 显式运行：

```bash
EDGEAI_ROS_BACKEND=native EDGEAI_CONTROLLER=mppi \
  bash scripts/simulation/start-autonomous-exploration.sh 300
```

可选低加速度补丁已使用系统编译器/apt 头文件构建，并通过确定性回归；
图形运动对照尚未完成：

```bash
EDGEAI_ROS_BACKEND=native bash scripts/setup/build-mppi-runtime.sh
EDGEAI_ROS_BACKEND=native EDGEAI_MPPI_USE_SOURCE=true EDGEAI_CONTROLLER=mppi \
  bash scripts/simulation/start-autonomous-exploration.sh 300
```

设置 `EDGEAI_ROS_BACKEND=pixi` 可回退。`EDGEAI_GALLIUM_DRIVER=llvmpipe`
用于软件渲染对照，默认仍为 d3d12。曾尝试强制 Qt Quick software backend，
随后 Gazebo MinimalScene 在 QOpenGLContext::doneCurrent 崩溃，已撤回此设置。
撤回后的 GUI 运行尚待验证。Linux `--check` 与 Windows `-CheckOnly` 均通过，
两端使用同一环境检查入口；这只证明依赖和包路径可解析。
