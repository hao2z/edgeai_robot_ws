# Operational scripts

- `simulation/`: launch and stop simulator/exploration sessions.
- `ros/`: environment activation, interactive shell and live monitoring.
- `setup/`: host/workspace installation and pinned open-source overlay setup.
- `display/`: WSLg readiness and VcXsrv authentication/display helpers.
- `capture/`: Windows/WSLg window capture; default output is `../tests/results/screenshots/` from the workspace root.

Use the root PowerShell entry points for normal Windows operation. Scripts
derive workspace-relative resource paths from their own location. PID files
and display authentication live in `../runtime/`.
