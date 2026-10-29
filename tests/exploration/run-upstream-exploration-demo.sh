#!/usr/bin/env bash
# Run the benchmark author's unmodified robot/navigation/explorer without project overlays.
set -eo pipefail
ulimit -c 0
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
demo="$HOME/.cache/edgeai-native/exploration-demo-upstream"
directory=${1:?session directory required}
mkdir -p "$directory/ros" "$directory/screenshots"
source /opt/ros/jazzy/setup.bash
source "$demo/install/setup.bash"
export ROS_DOMAIN_ID=89 GZ_PARTITION="edgeai_exploration_demo_$$"
export ROS_LOG_DIR="$directory/ros" DISPLAY=:0 QT_QPA_PLATFORM=xcb
export GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
unset XAUTHORITY
unset QT_QUICK_BACKEND QSG_RENDER_LOOP
cd "$demo"
git rev-parse HEAD > "$directory/benchmark-revision.txt"
git -C exploration_packages/frontier_exploration_ros2 rev-parse HEAD > "$directory/explorer-revision.txt"
cp simulation/Week-7-8-ROS2-Navigation/bme_ros2_navigation/config/navigation.yaml "$directory/nav2-params.yaml"
cp config/frontier_exploration_ros2/config.yaml "$directory/frontier-params.yaml"
pids=()
cleanup() {
    trap - EXIT
    for pid in "${pids[@]}"; do kill -INT -- "-$pid" 2>/dev/null || true; done
    sleep 3
    for pid in "${pids[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
setsid ros2 launch bme_ros2_navigation spawn_robot.launch.py world:=corridor x:=-8.11 y:=0.31 z:=0.10 yaw:=0.00 > "$directory/sim.log" 2>&1 &
pids+=("$!")
sleep 5
kill -0 "${pids[0]}" || { echo "Simulator launch exited; see sim.log"; exit 1; }
setsid ros2 launch bme_ros2_navigation navigation_with_slam.launch.py > "$directory/nav.log" 2>&1 &
pids+=("$!")
ready=false
for attempt in {1..60}; do
    kill -0 "${pids[0]}" && kill -0 "${pids[1]}" || exit 1
    if timeout 3 ros2 lifecycle get /bt_navigator 2>/dev/null | rg -q '^active'; then ready=true; break; fi
    sleep 1
done
$ready || { echo "Nav2 activation timed out"; exit 1; }
setsid ros2 launch frontier_exploration_ros2 frontier_explorer.launch.py params_file:="$demo/config/frontier_exploration_ros2/config.yaml" use_sim_time:=true > "$directory/explore.log" 2>&1 &
pids+=("$!")
printf '%s\n' "Exploration demo started: corridor, domain 89, unmodified upstream configuration." | tee "$directory/events.log"
python3 -u "$root/tests/exploration/observe-upstream-demo.py" "$directory"
printf '%s\n' 'Demo observation finished.' | tee -a "$directory/events.log"
