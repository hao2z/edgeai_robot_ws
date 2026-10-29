#!/usr/bin/env bash
# Invoked inside the project Pixi environment. Uses stock installed demo assets.
set -euo pipefail
ulimit -c 0
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
directory=${1:?session directory required}
mode=${2:-stock}
[[ $mode == stock || $mode == fixed-low-acceleration || $mode == fixed-stock ]] || exit 2
mkdir -p "$directory/ros" "$directory/screenshots"
export ROS_DISTRO=jazzy ROS_DOMAIN_ID=88 GZ_PARTITION="edgeai_official_demo_$$"
export DISPLAY=:0 QT_QPA_PLATFORM=xcb GALLIUM_DRIVER=d3d12
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA ROS_LOG_DIR="$directory/ros"
unset XAUTHORITY
# Do not source the project or patched MPPI overlay for this baseline.
if [[ $mode != stock ]]; then
    set +u
    source "${EDGEAI_RUNTIME_CACHE:-$HOME/.cache/edgeai-open-source}/mppi-install/local_setup.bash"
    set -u
    export EDGEAI_MPPI_SCORE_PROBE=1
fi
ros2 pkg prefix nav2_mppi_controller > "$directory/mppi-prefix.txt"
cp "$(ros2 pkg prefix --share nav2_bringup)/params/nav2_params.yaml" "$directory/nav2-params.yaml"
printf '%s\n' "$mode" > "$directory/mppi-mode.txt"
if [[ $mode == fixed-low-acceleration ]]; then
    python3 - "$directory/nav2-params.yaml" <<'PY'
import sys, yaml
from pathlib import Path
p = Path(sys.argv[1])
c = yaml.safe_load(p.read_text())
c['controller_server']['ros__parameters']['FollowPath'].update(
    ax_max=0.4, ax_min=-0.4, az_max=0.8, vx_std=0.05, wz_std=0.1)
c['velocity_smoother']['ros__parameters'].update(
    max_accel=[0.4, 0.0, 0.8], max_decel=[-0.4, 0.0, -0.8])
p.write_text(yaml.safe_dump(c, sort_keys=False))
PY
fi
printf '%s [session] Official Nav2 TB3 demo, MPPI mode=%s, domain 88\n' "$(date --iso-8601=seconds)" "$mode" | tee "$directory/events.log"
setsid ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False use_composition:=False \
    params_file:="$directory/nav2-params.yaml" \
    > "$directory/demo.log" 2>&1 &
launch_pid=$!
cleanup() {
    result=$?
    trap - EXIT
    touch "$directory/final-capture"
    # Allow the Windows observer to capture both live windows before shutdown.
    for i in {1..60}; do
        [[ ! -f "$directory/capture-required" || -f "$directory/capture-finished" ]] && break
        sleep .25
    done
    kill -INT -- "-$launch_pid" 2>/dev/null || true
    for i in {1..40}; do
        kill -0 "$launch_pid" 2>/dev/null || break
        sleep .25
    done
    kill -TERM -- "-$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
    printf '%s\n' "$result" > "$directory/exit-code.txt"
    exit "$result"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
python3 -u "$root/tests/exploration/official_demo_check.py" "$directory" | tee -a "$directory/events.log"
