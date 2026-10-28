# Tests and evidence

- `simulation/`: bounded sensor, motion and simulation-extension checks.
- `exploration/`: map coverage evaluation, feature audits and navigation diagnostics.
- `results/exploration/`: saved maps and exploration comparison evidence.
- `results/simulation-extension/`: multi-robot/IMU/target validation results.
- `results/environment/`: original environment smoke-test logs and screenshots.
- `results/backups/`: recoverable workspace/configuration snapshots.
- `results/screenshots/`: new screen/window captures.
- `docs/`: verification notes and reference-text extracts.

Generated results are local and excluded from Git. Retired system diagnostics
and one-time scripts have been cleaned up. Current operational display scripts
are under `../scripts/display/`; graphical runs retain logs and captures in
`results/runs/<timestamp>/`.
Saved-map evaluation runs without starting the robot, for example:

```bash
bash scripts/ros/ros-command.sh python \
  tests/exploration/evaluate_arena_map.py \
  /path/to/saved/map.yaml \
  src/edgeai_robot_description/worlds/competition_rooms.sdf
```

Run from the repository root; the ROS wrapper returns to that same directory.
