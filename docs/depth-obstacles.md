# 深度相机与雷达协作避障

参数入口：`src/edgeai_navigation/config/depth_obstacles.yaml`。重启生效。

```powershell
# 雷达+深度，三个控制器均可使用
.\Start-Autonomous-Exploration.ps1 -Controller rpp -DepthObstacles true
# 恢复原来的单雷达局部代价；保留视觉识别功能
.\Start-Autonomous-Exploration.ps1 -Controller rpp -DepthObstacles false
```

```bash
bash scripts/simulation/start-autonomous-exploration.sh --controller rpp --depth-obstacles true
bash scripts/simulation/start-autonomous-exploration.sh --controller rpp --depth-obstacles false
```

默认`config`读取YAML；当前`enabled: false`，即单雷达局部代价。设置`true`可启用深度融合。关闭时不添加任何深度插件/处理节点；原有激光、膨胀、控制器、车身和全局地图参数保持原值。

## 融合原则

- 激光ObstacleLayer提供周向扫描障碍；深度VoxelLayer补充相机可见的矮障碍和扫描面外障碍。
- 两层取最大值，随后统一一次InflationLayer；不叠加两次膨胀，不用深度空值擦掉激光障碍。
- 深度标记只保留高度4~45cm、距离0.15~2m的点；清障另用包含地面回波的三维射线，避免标记过滤同时丢掉清障依据。
- 各层维护自己的障碍：激光空射线不会错误删除扫描面下面的深度障碍。
- Nav2按原始点云时间戳查询TF。过滤器保持字段、坐标、时间戳和大小端，不伪造最新姿态。

## 调整顺序

| 参数 | 当前 | 作用 |
|---|---|---|
| `enabled` | true | 总开关 |
| `processing_hz` | 8.0 | 仿真时间频率；降低减轻代价图输入负担 |
| `pixel_stride` | 4 | 每4行/列采样，点数减为1/16；更细的障碍可改2，但开销增加 |
| `max_age_s` | 0.4 | 丢弃积压点云，不能修复TF延迟 |
| `min_obstacle_height` | 0.04m | 防止地面噪声成为障碍；过高会漏矮障碍 |
| `max_obstacle_height` | 0.45m | 碰撞高度范围，换机型时复核；层和标记源同时调整 |
| `obstacle_max_range` | 2.0m | 标记范围 |
| `raytrace_max_range` | 2.5m | 可见回波射线清障范围，应大于标记范围 |

不额外加大膨胀或车身，先观察新增障碍是否真实。当前保留激光原参数。

## 观察与限制

查看每次会话`nav2-runtime.yaml`：开启时局部`plugins`包含`depth_layer`，有`depth_obstacle_filter`参数；关闭时二者不存在。机器人2使用自己的命名空间输入/输出。节点启动信息`DEPTH_OBSTACLES enabled`进入`nav.log`，机器人2进入其导航日志。

RViz局部代价图直接反映融合结果。需要观察点云时订阅`/camera/obstacle_points`，Robot2为`/robot2/camera/obstacle_points`，QoS使用Best Effort。

探索RViz默认显示`Depth costmap input (cyan)`：青色为降采样的深度层输入，第二台机器人为蓝色。输入仍包含用于清障的地面/高处回波，不等于最终标记的障碍；实际碰撞代价看`Local live obstacle costs`。关闭深度层时启动器自动关闭对应点云显示。`Depth image (optional)`可手动勾选查看原始深度图，独立于深度避障开关。

原生VoxelLayer没有基于时间的障碍衰减。`observation_persistence: 0`仅指最新帧缓存；不表示标过的障碍自动消失。障碍离开视野或深度相机断流时，已有标记要等待有效射线、车身清障、滚动窗口移出或Nav2清图恢复。无效/无穷深度不能证明空间为空，不用于盲目清障。若动态障碍残影显著，再评估STVL；不要用雷达清整条三维柱体掩盖残影。

深度只能覆盖相机视野；细线、透明/反光物和低于高度阈值的物体仍可能漏检。RPP仍以跟踪和碰撞停车为主；加入感知不会把它变为主动绕障控制器。全局地图仍不加入实时深度层，局部停车而全局路径不变时由现有重规划/恢复处理。

输入等待排查：查看`/camera/depth_obstacle_status`和nav.log的`DEPTH_PIPELINE`状态变化。计数区分收到原始点云、限频跳过、过期/未来时间戳、格式无效、成功发布。RViz显示收/发及过期丢弃数，避免把“配置开启”当作融合成功。输出使用Reliable，兼容Nav2可靠订阅和Best Effort可视化；输入仍接受Best Effort传感器。不要仅靠放大`max_age_s`处理数据积压，应先查收发计数和实际年龄。

默认`filter.input_mode: depth_image`：订阅深度图和同一光学坐标系的camera_info，先采样再按相机内参投影为XYZ，保持深度图时间戳。支持32FC1米和16UC1毫米，拒绝畸变/裁剪/尺寸或坐标系不匹配，零/无效深度变为NaN。640×480图每4像素采样后输出约230KB；不再依赖每帧约7.4MB的Gazebo原始点云订阅。`pointcloud`模式保留，使用`input_topic`。两种模式都发布同一个局部代价图输入话题，开关和融合规则不变。

## 无仿真验证

`tests/exploration/test_depth_obstacles.py`检查三控制器/三配置组合的开关隔离，以及含行填充、大小端、NaN点的点云采样。
`tests/exploration/test_depth_obstacles_ros.py`在独立ROS域运行原生Nav2和合成点云/雷达，不发导航目标或速度指令，检查双方障碍均保留、地面不标记、深度自己的射线能清障。
这些检查验证接入和融合机制；实际仿真中的帧率、噪声和通道通过效果需要下一次测试观察。
