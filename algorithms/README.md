# Algorithms

`exploration/` contains Nav2/frontier configuration generators, the upstream
completion-event adapter and `nav2-env/` with the tested Pixi manifest/lockfile.
Runtime parameter YAML and RViz files stay in the ROS description package.
Pinned upstream implementations are built in WSL at
`~/.cache/edgeai-open-source`; they are not copied into this directory.

Install/build with `../scripts/setup/prepare-open-source.sh`.
Run with the root `Start-Autonomous-Exploration.ps1`.
Evaluation and diagnostics belong to `../tests/exploration/`.
