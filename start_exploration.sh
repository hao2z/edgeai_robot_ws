#!/usr/bin/env bash
set -eo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS_DIR"

export EDGEAI_ROS_BACKEND=${EDGEAI_ROS_BACKEND:-pixi}
source "$WS_DIR/scripts/ros/workspace-env.sh"

# Pixi 后端：在准备好的 ROS 环境里重新运行本脚本。
if [[ $EDGEAI_ROS_BACKEND == pixi && ${EDGEAI_SIMPLE_PIXI_ACTIVE:-false} != true ]]; then
    export EDGEAI_SIMPLE_PIXI_ACTIVE=true
    exec pixi run --manifest-path "$EDGEAI_ENV_MANIFEST" bash "$0" "$@"
fi

# Native 后端：从干净的 ROS Jazzy 环境启动。
if [[ $EDGEAI_ROS_BACKEND == native ]]; then
    if [[ -n ${CONDA_PREFIX:-} || -n ${PIXI_PROJECT_ROOT:-} ]]; then
        echo "Native ROS 请在未激活 Pixi/Conda 的终端中运行。" >&2
        exit 1
    fi
    set +u
    source /opt/ros/jazzy/setup.bash
    set -u
fi

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-86}

if [[ ! -f "$EDGEAI_BUILD_ROOT/install/local_setup.bash" ]]; then
    echo "工作区尚未构建，请先运行：bash scripts/setup/build-workspace.sh" >&2
    exit 1
fi

if [[ ! -f "$EDGEAI_RUNTIME_CACHE/frontier-install/local_setup.bash" ]]; then
    echo "Frontier runtime 不存在，请先运行：EDGEAI_ROS_BACKEND=$EDGEAI_ROS_BACKEND bash scripts/setup/prepare-open-source.sh" >&2
    exit 1
fi

set +u
source "$EDGEAI_BUILD_ROOT/install/local_setup.bash"
source "$EDGEAI_RUNTIME_CACHE/frontier-install/local_setup.bash"
set -u

NAV_CONFIG="$WS_DIR/src/edgeai_navigation/config/nav2_exploration_rectangle.yaml"
FRONTIER_CONFIG="$WS_DIR/src/edgeai_exploration/config/frontier_exploration.yaml"
RVIZ_CONFIG="$WS_DIR/src/edgeai_exploration/rviz/exploration.rviz"
RECOVERY_TREE="$WS_DIR/src/edgeai_navigation/config/behavior_trees/explore_recovery.xml"
SESSION_DIR="$HOME/.cache/edgeai-simple"
mkdir -p "$SESSION_DIR"

PIDS=()

cleanup() {
    result=$?
    trap - EXIT INT TERM
    echo "正在停止启动的模块..."
    for pid in "${PIDS[@]}"; do
        kill -INT "$pid" 2>/dev/null || true
    done
    for pid in "${PIDS[@]}"; do
        wait "$pid" 2>/dev/null || true
    done
    exit "$result"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo "启动仿真和 SLAM..."
ros2 launch edgeai_simulation simulation_real.launch.py &
PIDS+=("$!")
sleep 5

echo "启动 Nav2..."
ros2 launch edgeai_navigation managed_navigation.launch.py \
    "params_file:=$NAV_CONFIG" use_sim_time:=true &
PIDS+=("$!")
sleep 8

echo "启动导航仲裁..."
python3 "$WS_DIR/src/edgeai_navigation/scripts/navigation_authority.py" \
    --directory "$SESSION_DIR" \
    --mode AUTO \
    --tree "$RECOVERY_TREE" \
    --ros-args -p use_sim_time:=true &
PIDS+=("$!")
sleep 2

echo "启动 ArUco 感知..."
python3 "$WS_DIR/src/edgeai_perception/scripts/aruco_targets.py" \
    --ros-args \
    --params-file "$WS_DIR/src/edgeai_perception/config/aruco_targets.yaml" &
PIDS+=("$!")

echo "启动 Frontier 探索..."
ros2 run frontier_exploration_ros2 frontier_explorer --ros-args \
    --params-file "$FRONTIER_CONFIG" \
    -r control_exploration:=/exploration/control_exploration &
PIDS+=("$!")

echo "启动 RViz..."
rviz2 -d "$RVIZ_CONFIG" --ros-args -p use_sim_time:=true &
PIDS+=("$!")

echo "模块已启动，按 Ctrl+C 停止。"
wait "${PIDS[0]}"