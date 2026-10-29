#!/usr/bin/env bash
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
set -e
export PATH="$HOME/.pixi/bin:$PATH"
export COLUMNS=120 LINES=40
export GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
CHECK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export QT_QPA_PLATFORM=xcb
if [[ ${1:-wslg} == vcxsrv ]]; then
    export DISPLAY="$(ip -4 route show default | awk 'NR==1 {print $3}'):1"
    export XAUTHORITY="$PROJECT_ROOT/runtime/vcxsrv.xauth" QT_X11_NO_MITSHM=1
    unset WAYLAND_DISPLAY
else
    export DISPLAY=:0
    unset XAUTHORITY
fi
source "$PROJECT_ROOT/scripts/ros/workspace-env.sh"
cd "$PROJECT_ROOT"
if [[ -n ${2:-} ]]; then
    exec bash "$PROJECT_ROOT/scripts/ros/with-ros-env.sh" env "ROS_DOMAIN_ID=$2" bash --rcfile "$PROJECT_ROOT/scripts/ros/ros-shell.rc" -i
fi
exec bash "$PROJECT_ROOT/scripts/ros/with-ros-env.sh" bash --rcfile "$PROJECT_ROOT/scripts/ros/ros-shell.rc" -i
