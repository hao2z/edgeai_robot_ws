# Unified workspace

All editable sources, Windows entry points and competition reference documents
live in this repository.

| Directory | Contents |
| --- | --- |
| `src/edgeai_robot_description/` | Robot URDF and basic RViz configuration |
| `src/edgeai_simulation/` | Gazebo launch files, worlds, target models, simulation configuration and arena tools |
| `src/edgeai_navigation/` | Nav2 parameters, behavior trees, navigation authority and configuration generators |
| `src/edgeai_exploration/` | Frontier configuration, upstream adapters, completion watcher and exploration RViz configuration |
| `src/edgeai_navigation_rviz/` | RViz navigation panel plugin |
| `scripts/` | Environment, installation, build, launch, monitoring and capture entry points |
| `tests/` | Simulation checks, navigation diagnostics and local run evidence |
| `docs/competition/` | Original competition reference documents |

Build this source tree with `scripts/setup/build-workspace.sh`. Generated files
live in `EDGEAI_BUILD_ROOT`: native ROS defaults to `~/.cache/edgeai-native-build`,
and the retained Pixi backend uses `~/.cache/edgeai-robot-build`.
Environment selection is centralized in `scripts/ros/workspace-env.sh`.

The previous root `algorithms/` and `simulation/` directories have been retired.
Use the package directories above for source edits. Live logs and screenshot
handling are described in [test observation](../tests/docs/test-observation.md).

Machine-local state under `runtime/` and generated results under `tests/results/`
are excluded from Git. Retired one-time scripts and duplicate text exports are
stored under `runtime/archive/cleanup-20261029/`, with original relative paths
and a SHA-256 manifest. Competition originals remain in `docs/competition/`.
