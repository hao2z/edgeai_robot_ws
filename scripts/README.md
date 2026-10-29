# Operational scripts

- `simulation/`: launch and stop simulator/exploration sessions.
- `ros/`: environment activation, interactive shell and live monitoring.
- `setup/`: host/workspace installation and pinned open-source overlay setup.
- `display/`: WSLg readiness and VcXsrv authentication/display helpers.
- `capture/`: Windows/WSLg window capture; default output is `../tests/results/screenshots/` from the workspace root.

Use the root PowerShell entry points for normal Windows operation. Scripts
derive workspace-relative resource paths from their own location. PID files
and display authentication live in `../runtime/`.

Native ROS activation goes through `ros/with-ros-env.sh`; select it with
`EDGEAI_ROS_BACKEND=native`. `setup/install-native-ros.sh` installs the official
Jazzy apt packages. The Pixi installation and MPPI backport scripts remain for
regression comparisons while native compilation and validation are completed.
Retired one-time probes are stored in `../runtime/archive/cleanup-20261029/`.

Windows entry points load `ros/windows-host-tools.ps1` to resolve WSL,
Windows PowerShell and Windows Terminal without relying on the terminal's PATH.
Test-history retention runs with Ubuntu's `python3`; Windows Python is not
required for the launch workflow. No system-wide PATH changes are made.
`Start-Autonomous-Exploration.ps1` delegates test execution and retention to
the same managed Linux entry used directly in WSL. It supplies a session path
and Windows monitoring/capture helpers; no history scan blocks startup.
