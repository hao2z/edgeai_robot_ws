# Runtime state

PID files, display authentication and live display logs are mutable process
state. They are not test results or algorithm configuration. Do not publish the
VcXsrv authentication cookie. Historical logs are under `../tests/results/`.

`archive/cleanup-20261029/` contains retired one-time scripts, superseded display
and feature probes, and duplicate competition-text exports. Files retain their
original relative paths; `manifest.json` records sizes and SHA-256 hashes.
This archive is local, excluded from Git, and is not a runtime dependency.
