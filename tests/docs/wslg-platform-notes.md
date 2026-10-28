# WSLg 调试归档清理后保留的信息

2026-10-28 按用户要求删除已结束的平台故障调试产物，包括旧 MSI 安装包、
Weston/Mesa 探针、临时二进制、重复截图和历史日志。逐文件清单见
`../results/cleanup-20261028.json`。这些旧平台验证结论属于历史记录。

之前的问题表现为共享内存重定向失败和图形异常。项目仍保留当前可用入口
`Repair-WSLg.ps1` 与 `scripts/display/check-gfxredir.sh`：就绪后检查
Weston 的 `use_gfxredir`，健康时继续，已知故障时仅恢复 Weston。
不应为了清理调试产物删除 WSL 虚拟磁盘、已安装环境或运行依赖。

本次真实图形验证采用 Windows 单窗口截图，Gazebo 为最大化倾斜视角，
能同时看到完整场地和机器人。新流程与证据目录见 `test-observation.md`。
