# Linux 队员上手

支持目标为 Ubuntu 24.04、ROS 2 Jazzy、有图形桌面的本机环境；WSL也可使用同一入口。以下命令在仓库根目录执行，不需要PowerShell，不要求仓库位于固定路径。

## 首次准备

```bash
git clone --branch codex/Junnan-dev https://github.com/hao2z/edgeai_robot_ws.git
cd edgeai_robot_ws
# 尚未安装依赖时执行；该安装器要求Ubuntu 24.04
sudo bash scripts/setup/install-native-ros.sh
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
# 普通用户构建固定版本探索器和本仓库
bash edgeai.sh build
bash edgeai.sh check
```

默认native环境；不要在已激活的Conda/Pixi环境中运行。`check`只检查依赖，不启动或停止测试。构建缓存位于用户目录，源码只维护本仓库。已安装依赖的队员可跳过安装步骤。

## 日常操作

| 命令 | 用途 |
|---|---|
| `bash edgeai.sh mission` | 默认完整任务：RPP、Smac Lattice、中期场地、双机器人、3600秒墙钟上限 |
| `bash edgeai.sh mission --single-robot` | 单机建图和S1救援对照 |
| `bash edgeai.sh navigation` | 加载仿真真值地图与定位，单机手动选点测试 |
| `bash edgeai.sh cooperative` | 使用上一轮运行参数，跳过探索/S1，直接做静态地图C1专项 |
| `bash edgeai.sh diagnose --duration 20 --hz 10` | 只读采样最新会话的控制器和速度链，保存诊断摘要 |
| `bash edgeai.sh monitor` | 另开终端查看最新会话关键事件；Ctrl+C只关闭该查看器 |
| `bash edgeai.sh stop` | 停止已登记的本项目任务和仿真进程 |
| `bash edgeai.sh shell` | 当前终端进入已加载工作区的交互ROS环境 |
| `bash edgeai.sh simulation multi_robot:=true robot_count:=2 primary_unscoped:=true` | 仅启动双机仿真，不执行救援流程 |

```bash
# 常用可选参数；--help查看说明
bash edgeai.sh mission --controller rpp --planner lattice --domain 86 --duration 600
bash edgeai.sh mission --depth-obstacles true
bash edgeai.sh navigation --controller dwb
# 协作专项也可指定已有运行配置，路径含空格时加引号
bash edgeai.sh cooperative --params /path/to/nav2-runtime.yaml --duration 600
bash edgeai.sh cooperative --filter-check
```

任务与导航测试会打开Gazebo、RViz，启动前清理已登记的旧项目会话；Ctrl+C结束当前任务。启动新任务前先保存需要保留的证据。`navigation` 在RViz导航面板/选点工具中指定目标；真值导航与协作专项都是诊断环境，不能作为自主探索验收。

`cooperative` 暂复用先前的生成配置，因此新克隆应先运行一次 `mission` 或 `navigation`，也可用 `--params` 指定队友提供的生成配置；默认尚不支持从零生成独立专项配置。输出目录自动创建，也可指定一个尚不存在的 `--directory`。过滤专项使用 `--filter-check`，不执行C1救援。

## 参数与输出

- 通常修改 `src/edgeai_navigation/config/` 的对应YAML，实际生效值检查本轮 `nav2-runtime.yaml`。入口支持 `--controller`、`--planner`、`--domain`、`--duration`、`--single-robot` 和深度开关。
- 最新会话路径在 `runtime/latest-session.txt`；日志、地图、诊断和结果在 `tests/results/runs/`。这些生成文件不提交Git。完整任务自动管理历史保留，协作专项暂不自动裁剪历史。
- `simulation` 是低层仿真入口，ROS启动参数按 `name:=value` 传递。它没有完整任务的会话日志和历史管理；停止可用当前终端Ctrl+C。
- 原生Linux保留已有 `DISPLAY`、`WAYLAND_DISPLAY`、`XAUTHORITY` 和GPU驱动设置；WSL自动应用WSLg默认值。排查自动检测时可显式设置 `EDGEAI_DISPLAY_PLATFORM=native` 或 `wslg`。
- Windows自动截图、窗口最大化和终端窗口关闭没有Linux等效；Linux日志照常保存，截图手动完成。当前仅做入口和环境检查，尚未在队员的原生Ubuntu桌面完成图形验收。

任务流程、验证与遗留问题见 [项目状态](project-status.md) 和 [开发记录](development-record-2026-11-07.md)。
