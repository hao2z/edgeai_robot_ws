#!/usr/bin/env bash
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y curl git lsb-release gnupg build-essential mesa-utils x11-utils
test "$(lsb_release -cs)" = noble
curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
printf 'deb [arch=%s signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable noble main\n' "$(dpkg --print-architecture)" > /etc/apt/sources.list.d/gazebo-stable.list
apt-get update
apt-get install -y gz-harmonic
gz sim --version
apt-get check
