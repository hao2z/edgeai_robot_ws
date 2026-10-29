#!/usr/bin/env bash
# Official ROS Jazzy binary packages for Ubuntu Noble; run as root in WSL.
set -euo pipefail
[[ $(id -u) == 0 ]] || { echo 'Run this installer as root.' >&2; exit 1; }
source /etc/os-release
[[ $ID == ubuntu && $VERSION_CODENAME == noble ]] || {
    echo 'This installer targets Ubuntu 24.04 (Noble).' >&2; exit 1;
}
export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y --no-install-recommends ca-certificates curl locales software-properties-common
locale-gen en_US.UTF-8
add-apt-repository -y universe
if ! dpkg-query -W ros2-apt-source >/dev/null 2>&1; then
    temp_deb=$(mktemp --suffix=.deb)
    trap 'rm -f "$temp_deb"' EXIT
    python3 - "$temp_deb" <<'PY'
import json, sys, urllib.request
request = urllib.request.Request(
    'https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest',
    headers={'User-Agent': 'edgeai-native-ros-setup'})
release = json.load(urllib.request.urlopen(request))
assets = [a for a in release['assets'] if a['name'].startswith('ros2-apt-source_')
          and a['name'].endswith('.noble_all.deb')]
if len(assets) != 1:
    raise RuntimeError('Cannot uniquely resolve official Noble ros2-apt-source package')
print('Official ROS apt source:', release['tag_name'], assets[0]['name'], flush=True)
urllib.request.urlretrieve(assets[0]['browser_download_url'], sys.argv[1])
PY
    dpkg -i "$temp_deb"
fi
apt-get update
apt-get install -y --no-install-recommends \
    ros-jazzy-desktop ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-minimal-tb3-sim ros-jazzy-ros-gz ros-jazzy-slam-toolbox \
    ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-xacro ros-jazzy-joint-state-publisher \
    python3-colcon-common-extensions python3-rosdep python3-yaml python3-opencv \
    python3-numpy python3-pil python3-pytest python-is-python3 build-essential cmake git \
    libxtensor-dev libxsimd-dev xtl-dev
rosdep init || [[ -f /etc/ros/rosdep/sources.list.d/20-default.list ]]
echo 'Native ROS packages installed. Run rosdep update as the regular user.'
