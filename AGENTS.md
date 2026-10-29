# Development conventions

- This repository is the canonical working directory. Keep simulation,
  algorithms, tests and operational scripts here; do not edit a second source
  tree under a WSL home directory.
- Continue development on `codex/Junnan-dev` unless the user requests another
  branch. `origin` is `https://github.com/hao2z/edgeai_robot_ws.git`; the original
  repository is retained as `upstream`.
- Build with `bash scripts/setup/build-workspace.sh`. Generated files live in
  `EDGEAI_BUILD_ROOT` (native: `~/.cache/edgeai-native-build`, Pixi:
  `~/.cache/edgeai-robot-build`); environment
  activation is centralized in `scripts/ros/workspace-env.sh`.
- Keep shell scripts in LF. Resolve repository resources relative to the
  script, and convert Windows paths with `wslpath`; avoid machine-specific paths.
- Keep `runtime/` credentials, PID files, logs, maps, screenshots, source backups,
  Pixi environments and build outputs out of commits.
- For simulation tests, enable Gazebo/RViz, stop previous project sessions and
  terminal tabs before restarting, retain per-run logs and inspect actual GUI
  screenshots. Do not infer physical contact solely from a costmap warning.
- Relevant checks: `tests/validate_layout.py`,
  `tests/exploration/test_session_logging.py`,
  `tests/simulation/validate_rectangle_footprint.py`, PowerShell syntax,
  and a graphical smoke test after runtime/path changes.
- Before pushing, run `git diff --cached --check`; afterward verify local HEAD
  matches the remote branch SHA. Do not overwrite unrelated remote branches.
