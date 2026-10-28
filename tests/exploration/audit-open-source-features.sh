#!/usr/bin/env bash
set -eo pipefail
source "$HOME/.cache/edgeai-open-source/frontier-install/local_setup.bash"
export ROS_DOMAIN_ID=86
control() {
    timeout 10s ros2 service call /control_exploration \
        frontier_exploration_ros2/srv/ControlExploration \
        "{action: $1, delay_seconds: 0.0, quit_after_stop: false}"
}
trap 'control 1 >/dev/null 2>&1 || true' EXIT
echo 'Stop using the upstream control service:'
control 2
echo 'Resume using the upstream control service:'
control 1
ros2 topic info /explore/frontiers
ros2 topic info /exploration_complete --verbose
ros2 topic info /collision_monitor_state
