#!/usr/bin/env bash
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
if [[ ! -S /mnt/wslg/.X11-unix/X0 || ! -r /mnt/wslg/weston.log ]]; then
    echo starting
    exit 0
fi
awk '/RDP backend: use_gfxredir =/ { state=$NF } END { if (state ~ /^[01]$/) print state; else print "starting" }' /mnt/wslg/weston.log
