# 文档入口与维护规则

当前开发只维护与改动直接相关的一份主说明，不要求每次同步所有专题、报告和测试清单。

| 职责 | 主说明 |
|---|---|
| 使用、安装与启动 | [仓库说明](../README.md)、[Linux上手](linux-quickstart.md)、[Windows入口](windows-entrypoints.md) |
| 任务与机器人协调 | [任务层协调](fleet-task-coordination.md) |
| 导航对外接口与配置 | [分用途导航配置](navigation-task-profiles.md) |
| 测试入口与验证范围 | [测试说明](../tests/README.md) |

实现细节调整通常只需修改代码；公共接口、启动方式或配置含义改变时，更新对应主说明即可。验证结果保存在运行目录，不为每次修改增加新报告或复制测试列表。

专题资料按需查阅，不承担逐次改动同步：

- 建图与目标：[联合建图](joint-mapping.md)、[目标地图绑定](target-map-anchoring.md)。
- 调试：[运动测试](navigation-test-environment.md)、[控制器诊断](controller-diagnostics.md)、[性能](simulation-performance.md)。
- 比赛：[原始文档与要求](competition/)、[场地](competition/midterm-environment.md)。
- 已验证版本的结果与边界：[命名空间验证记录](native-namespace-validation.md)。

日期报告已收拢到[历史目录](history/README.md)。历史文字描述当时实现，不作为当前接口或参数依据，也不随代码更新。
