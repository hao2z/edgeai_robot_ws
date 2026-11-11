# 可选人工专项

`mppi/`保存MPPI数学检查、回移植编译探针和评分探针生成工具，供切回MPPI时使用。它们依赖对应MPPI源码或缓存，不属于当前RPP默认回归。

从仓库根目录执行`bash tests/manual/mppi/check-mppi-math.sh`。`scripts/setup/build-mppi-runtime.sh`仍会使用这里的回移植检查与评分工具。

导航与协作的人工测试入口保留在仓库根目录，见[入口说明](../../docs/windows-entrypoints.md)。
