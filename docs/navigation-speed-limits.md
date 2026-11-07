# 导航速度约束检查

## 当前联动修改流程

以下审计记录保留了修复前的问题。当前已改为三个到点策略统一使用MatchedRPP，
不再由Lattice Shim在碰撞检查后缩放速度。它复用原生路径、前视、曲率减速及碰撞检测，
先由相同前视点计算速度预算，再交给原生RPP计算和检查。

1. **调整底盘边界**：修改 `nav2_exploration_rectangle.yaml` 的
   `velocity_smoother.max_velocity/min_velocity/max_accel/max_decel`，它们是公共边界来源。
2. **调整导航偏好**：修改 `rpp_controller.yaml` 的 `desired_linear_vel`、
   `reverse_velocity_limit`、原地对齐速度、转弯及接近参数。
   不需要手填运行时 `motion_forward_limit/motion_reverse_limit/motion_angular_limit`。
3. **重启并检查会话**：生成器为普通、朝向和救援实例派生速度预算，
   自动收紧旋转速度/角加速度及恢复旋转边界，RPP模式启用平滑器scale_velocities。
   `motion-limits.json` 保存有效限制、倒车是否启用及警告；`nav2-runtime.yaml` 是实际配置。

不合法的公共边界阻止启动。偏好超过公共速度/加速度、末段减速距离小于理想制动估算、
公共平移速度低于BT固定恢复速度0.06 m/s时，会输出 `MOTION_LIMITS_WARNING`。
自动派生只修改运行时配置，不反写手调YAML。DWB/MPPI控制器不由此替换。

速度匹配先限制RPP预算：前进取巡航和公共正向上限，倒车再取倒车偏好与公共反向上限，
曲率非零时再取 `公共角速度上限/|曲率|`，然后由原生RPP做调节和碰撞预测。
原地旋转速度和角加速度也不超过公共边界。外部SpeedLimit仍可进一步收紧速度。
碰撞监视器仍可停车；加减速过渡、定位延迟、真实底盘动力学仍可能让实测速度与指令不同，
不能将这项匹配表述为每一时刻都精确跟踪同一曲率。

## 联动修复后的验证

编译通过；配置联动、逐点策略、Lattice及全场景生成检查通过。
会话`20261106-185742-rpp-simple_points`的100个采样中，
`cmd_vel_nav`的vx范围为[−0.25,0.60] m/s、最大角速度幅值0.90 rad/s；
平滑后与最终速度的最大角速度幅值均为0.875 rad/s，未再观察到原先的超边界请求。
明细见该会话`speed-chain-summary.json`。

前三个普通点通过；第四点Nav2返回成功，但独立位置测量误差0.115 m，
超过测试0.08 m阈值，整轮未通过，朝向/救援停车未在这一轮运行。
保留此边界，不把速度匹配成功等同于全部导航回归成功。
随后按用户要求转为手动测试，会话`20261106-185929-362-domain86-navigation-test`。

## 修复前的审计依据

2026-11-06，针对rectangle + RPP + Lattice。以会话
`20261106-184521-rpp-narrow_spaces/nav2-runtime.yaml` 为配置依据。
随后核对手动会话`20261106-184804-009-domain86-navigation-test`，上述关键速度配置相同。
本检查不再调整参数；当前新增Lattice Shim仍处于验证阶段。

实际指令链：RPP/起步Shim或Nav2恢复动作 → `/cmd_vel_nav` → 速度平滑器
→ `/cmd_vel_smoothed` → 碰撞监视器 → `/cmd_vel` → ROS–Gazebo桥 → DiffDrive。
仲裁器选择动作，不对速度做额外比例缩放。全局规划器本身不限制执行速度。

| 层级 | 当前约束 | 作用范围/注意事项 |
|---|---|---|
| RPP期望线速度 | 0.60 m/s | 三个到点策略共用；启用倒车时原生也可能请求−0.60 |
| 曲率减速 | 半径阈值0.40 m，调节最低速度0.05 m/s | 普通转弯中角速度可接近0.60/0.40=1.50 rad/s；极小半径受最低速度影响，可能更高 |
| 末段减速 | 0.35 m，最低接近速度0.08 m/s | 不是底盘最低速度；后续缩放/停车可更低 |
| 膨胀代价减速 | 关闭 | 不代表关闭车身碰撞检查 |
| Shim/RPP主动旋转 | 0.90 rad/s，角加速度0.80 rad/s² | 角加速度参数用于原地对齐，不约束原生RPP所有曲线跟踪输出 |
| 新Lattice Shim | 倒车0.25 m/s、跟踪角速度0.90 rad/s | 仅普通点；同比例缩放v和w，但发生在RPP碰撞预测之后 |
| 原生RPP碰撞预测 | 最长1.0 s，范围还受前视点约束 | 遇碰撞抛出NoValidControl；不是自动尝试更慢的可行速度 |
| 平移恢复 | 0.06 m/s | BT选5/10 cm短移；正退均经过后续速度链 |
| Spin恢复 | 0.30–0.90 rad/s，角加速度0.80 rad/s² | 独立于RPP起步对齐；小转角还会受终止减速影响 |
| 速度平滑器硬边界 | vx∈[−0.30,0.60] m/s，vy=0，wz∈[−0.90,0.90] rad/s | 先逐轴截断绝对速度；可能改变曲率 |
| 速度平滑器加减速 | 线速度±0.40 m/s²，角速度±0.80 rad/s² | 20 Hz，OPEN_LOOP，scale_velocities=false；各轴独立限制速度增量 |
| 平滑器死区/超时 | 死区0，1.0 s无新命令开始停车 | 不是导航任务超时；未配置jerk限制 |
| 碰撞监视器 | approach预测1.2 s，1个激光点即可参与触发 | 可进一步等比例减速/停车；不受上游最低接近速度约束 |
| 监视器数据有效性 | 激光源超时1.0 s，TF容忍0.20 s | 无有效数据可能停车；需与碰撞触发区分 |
| Gazebo DiffDrive | 未显式配置额外速度、加速度、jerk上限 | 不代表真实底盘无限制；接触/动力学仍可能影响运动 |

参数入口：`config/rpp_controller.yaml`、`config/nav2_exploration_rectangle.yaml`、
`config/behavior_trees/explore_recovery.xml`，机器人模型在
`src/edgeai_robot_description/urdf/edgeai_robot_real.urdf.xacro`。

## 已确认的问题

1. **上游与下游速度边界不一致。** 未加出口缩放的
   `20261106-184130-rpp-simple_points/point-trace.json` 中，采样记录
   `/cmd_vel_nav` 最低−0.60 m/s、最大角速度幅值2.898 rad/s；
   `/cmd_vel` 分别受限为−0.30、0.90。最大值不是同一时刻的配对测量，
   但结合原生逐轴clamp实现，足以确认存在会改变曲率的限制链。
2. **新出口缩放还不完整。** 普通点输出v/w同比例缩放后，稳态命令可保持曲率，
   但RPP已经用缩放前速度做过碰撞预测。若它先抛出异常，出口限速没有机会执行。
   不能据此认为预测与执行一致，也不能保证它会尝试低速通过。
3. **指定朝向/救援实例仍可能超出角速度边界。** 新缩放仅在普通点实例。
   `20261106-184346-rpp-simple_points` 的普通点已限速，
   但整轮`cmd_vel_nav`角速度仍达到1.50 rad/s，最终输出不超过0.90。
4. **动态加减速仍可改变曲率。** OPEN_LOOP使用上次平滑输出而非实际速度。
   scale_velocities=true可协调速度增量，不能修复前面已发生的逐轴绝对速度截断，
   也不保证从任意当前速度精确保持目标曲率。
5. **0.35 m末段减速距离偏紧，值得验证。** 按配置的0.40 m/s²计算，
   从0.60降至0.08 m/s理论距离约0.442 m；这是理想估算，
   不是本次仿真实测制动距离，不能单独据此认定过冲原因。
6. **旋转参数职责混合。** 新Shim还将rotate_to_heading_angular_vel用作曲线跟踪角速度上限。
   这便于临时验证，但不是清晰的长期调参接口，应改为统一运动边界下的独立职责。

## 验证边界与下一步

加入出口缩放后，`20261106-184346-rpp-simple_points`四个普通点均通过且无恢复。
同轮指定朝向Nav2返回成功，独立测量yaw误差0.271 rad略超测试阈值0.270 rad，
整轮未通过，救援点未运行。
`20261106-184521-rpp-narrow_spaces`前三个点通过，但反向点耗时10.405仿真秒并用过一次恢复，
不能把较早版本的1.65秒结果当作当前性能。标记旁复杂路线尚未在最终限速版本复测。

现有trace只记录`cmd_vel_nav`、最终`cmd_vel`和里程计，没有中间`cmd_vel_smoothed`及完整限速原因。
因此不能精确分摊平滑器、碰撞监视和底盘各自造成的瞬时速度变化。

继续前先统一三个导航实例的可执行v/w边界，尽量在RPP碰撞预测前确定执行速度，
再协调加减速。之后依次检查短距离倒车、曲线倒车、窄通道退出和指定朝向/救援停车。
不通过关闭碰撞检查或增加恢复次数掩盖这些约束不一致。

官方实现依据：[RPP输出与碰撞检查顺序](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp)、
[速度平滑器先clamp再限制加减速](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_velocity_smoother/src/velocity_smoother.cpp)。
