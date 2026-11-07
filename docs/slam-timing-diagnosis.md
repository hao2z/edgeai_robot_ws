# 周期性 TF 延迟诊断（2026-11-04）

## 结论

发现并复现 WSL 系统墙钟周期性回拨引起的 ROS 定时停顿。不是 SLAM 每隔 20 仿真秒进行某项计算的证据。没有修改 NTP、WSL 或导航配置。

主要基线是成功运行 `20261104-212305-880-domain86`；调查期间用户另一次运行 `20261104-213009-965-domain86` 正在进行，对其只增加短时只读采样，未启动/停止用户仿真。

## 证据

- 多次运行的 TF 失败相隔约 31 秒系统墙钟，不严格相隔 20 仿真秒。成功基线的四次数据时间为 34.08、54.14、74.30、94.22 仿真秒；对应 TF 落后 0.33、0.34、0.35、0.32 秒。
- `timedatectl timesync-status`：systemd-timesyncd 活跃，服务器 ntp.ubuntu.com，轮询间隔 32 秒。两次查询偏移分别为 -1.135549、-1.115947 秒。
- 独立采样 `CLOCK_REALTIME - CLOCK_MONOTONIC`，直接检测到 -1.13554883 秒跳变。探针文件为 `runtime/slam-timing-probe.jsonl`。它自己的系统时钟定时器也停顿，因此没有连续覆盖该间隙内的 TF；不能将其有限的瞬时 TF 年龄当作没有中断的证明。
- 用已安装的 rclcpp 运行两个 50 Hz 线程，持续 45 单调秒：系统时钟 `rclcpp::Rate` 最大循环间隔 1.136212 秒；`rclcpp::WallRate` 最大间隔 0.028648 秒。同期检测到墙钟回拨 -1.11595 秒。结果在 `runtime/clock-rate-check.txt`，复现源码在 `runtime/clock-rate-check.cpp`。
- 系统时钟回拨幅度与 NTP 状态偏移吻合，轮询周期也吻合。WSL 主机校时与 Linux NTP 是否相互纠正，尚未单独验证，不作为已证实结论。

## 为什么影响 SLAM 和导航

安装版本：slam_toolbox 2.8.5、rclcpp 28.1.22。官方源码中，SLAM 的 `publishTransformLoop` 和 `publishVisualizations` 均使用默认 `rclcpp::Rate`；其默认是 RCL_SYSTEM_TIME。已经进入睡眠的系统时间定时器在回拨后可能等待墙钟重新到达截止时刻，独立实验已复现这一点。仿真 `/clock` 不会因为 use_sim_time=true 而自动替换这里另行构造的默认 Rate 时钟。

TF 发布线程停顿而仿真继续推进，就可能超出 DWB 的 0.3 秒变换容差。本次日志在原始 scan/odom 仍新鲜时报告 map/odom 变换过旧；符合该机制。独立实验确认定时故障机制，但不表示排除了所有其他调度和算力问题。

源码：

- https://github.com/SteveMacenski/slam_toolbox/blob/2.8.5/src/slam_toolbox_common.cpp （publishTransformLoop、publishVisualizations、updateMap）
- https://github.com/ros2/rclcpp/blob/jazzy/rclcpp/include/rclcpp/rate.hpp （Rate 默认时钟）
- https://github.com/ros2/rclcpp/blob/jazzy/rclcpp/src/rclcpp/rate.cpp （Rate/WallRate）

## 地图年龄与配置

成功基线中，排除最初 5 秒和退出阶段的 176 条采样：地图年龄最小 0.05、中位 0.25、95 分位 0.35、最大 1.15 仿真秒；55 条达到 0.3 秒。scan 年龄中位 0.05 秒、odom 中位 0.02 秒。仿真倍率中位 0.626。

RViz 的 SLAM 年龄是 `/map` 消息年龄，不是 map→odom TF 年龄。`map_update_interval: 0.5` 控制栅格重绘；源码使用扫描时间戳标记地图，故采样年龄还包含扫描与传输延迟。0.3 秒本身不足以判定异常。

相关配置在 `src/edgeai_simulation/config/slam_toolbox.yaml`：

| 参数 | 当前值 | 作用 |
|---|---:|---|
| map_update_interval | 0.5 | 栅格地图更新间隔 |
| transform_publish_period | 0.02 | map→odom 发布线程的循环间隔 |
| transform_timeout | 0.2 | 扫描 TF 等待时间，且默认给发布 TF 时间戳增加此时长 |
| minimum_time_interval | 0.2 | 接受扫描进入建图的最小时间间隔；不是所有 TF 发布的间隔 |
| tf_buffer_duration | 30 | TF 历史缓存长度，不是周期性停顿的周期 |

实际 Nav2 配置 `nav2-runtime.yaml` 中 DWB `FollowPath.transform_tolerance` 为 0.3。它是错误触发阈值，增加阈值不能修复墙钟回拨。

40 秒采样中 SLAM 进程平均约使用单核 10.4% CPU；不足以证明不存在瞬时负载峰值，但配合原始传感器年龄与独立定时实验，当前证据优先指向系统校时而非算力饱和。未采集全系统 CPU/GPU 历史，不能仅凭仿真倍率约 0.63 推断 SLAM 算力不足。

## 后续处理方向

先验证并解决 WSL 系统校时反复回拨；可选择受控的短时 NTP 停止/恢复对照实验，但本次未执行。应用层的稳健修复是将相关周期工作改为单调时钟（如 rclcpp::WallRate），这需要 SLAM 源码补丁或上游修复，不是现有 YAML 选项。暂不通过扩大导航容差或关闭回环来掩盖问题。

## 官方资料与本机补充核查

2026-11-04 查询到 Ubuntu 官方 WSL 时间同步文档（页面更新于 2026-10-20）：现代 WSL 默认通过 Hyper-V 同步 Windows 时间；Ubuntu 24.04 及更早版本默认启动的 systemd-timesyncd 可能与其冲突。官方建议使用主机校时时禁用 Linux 内的 timesyncd。

- 官方：https://ubuntu.com/wsl/docs/stable/explanation/time-sync/
- Microsoft WSL 讨论：https://github.com/microsoft/WSL/issues/11790 （2026-09-28 报告每 32.5 秒回拨 1.4～1.8 秒；2026-10-22 报告停用各发行版 timesyncd 后，601 秒对照采样未再出现周期跳变。这些是社区个案，不是对所有 WSL 环境的修复保证。）

本机只读检查确认 `/sys/module/hv_utils/parameters/timesync_implicit` 为 `Y`，内核命令行含 `hv_utils.timesync_implicit=1`，同时 systemd-timesyncd 活跃。因此双重校时风险具备实际条件。

另一次成功的 `w32tm /query /status` 查询报告 Windows 为未同步（Leap Indicator 3）、Stratum 0、Source Local CMOS Clock；注册表的 Type=NTP，NtpServer=time.windows.com,0x9，说明配置了网络时间源不等于已成功同步。更完整的 Windows 配置查询被当前非管理员权限拒绝；本次没有尝试提权或修改。

推荐顺序：仿真空闲时先恢复 Windows 正常外部校时并确认成功，再停用 WSL 内 timesyncd，保留 Hyper-V 隐式校时；复测至少数分钟及一次 ROS 运行。这样既移除重复校时，也避免长期继承未校准的主机时间。不优先更换 clocksource、调整 Linux NTP 轮询周期或尝试禁用 Hyper-V 校时；Ubuntu 文档提醒后者在部分 WSL 版本中不能可靠关闭。

## 已执行修复及短时验证

随后用户授权执行推荐修复。仿真停止时完成了 Windows W32Time 重新同步（仍使用 time.windows.com,0x9，Leap Indicator 0、Stratum 5），并将启动类型从 Manual 改为 Automatic；Ubuntu-24.04 内的 systemd-timesyncd 已停止且禁用，Hyper-V 隐式校时仍为 Y。

修复后以 50 Hz 对照运行 90 单调秒：系统时钟线程和单调时钟线程均完成 4500 次循环，最大间隔分别为 0.0217003 秒和 0.0216221 秒；没有检测到 >=0.01 秒的墙钟跳变。相比修复前系统时钟线程 1.136212 秒的最大间隔，周期性长停顿在该验证窗口内消失。验证不是完整导航测试，也未覆盖休眠/重启后的长期状态。

备份、实际修改和回退命令在 `runtime/time-sync-repair/`；未修改 SLAM/Nav2 参数，未重启 WSL。
