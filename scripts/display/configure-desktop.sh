#!/usr/bin/env bash
# Source this file. Native Linux keeps its display, authentication and GPU setup.
edgeai_display_platform=${EDGEAI_DISPLAY_PLATFORM:-auto}
if [[ $edgeai_display_platform == auto ]]; then
    if [[ -n ${WSL_DISTRO_NAME:-}${WSL_INTEROP:-} ]] || grep -qi microsoft /proc/sys/kernel/osrelease; then
        edgeai_display_platform=wslg
    else
        edgeai_display_platform=native
    fi
fi
case "$edgeai_display_platform" in
    native) ;;
    wslg)
        export DISPLAY=${DISPLAY:-:0} QT_QPA_PLATFORM=${QT_QPA_PLATFORM:-xcb}
        export GALLIUM_DRIVER=${EDGEAI_GALLIUM_DRIVER:-${GALLIUM_DRIVER:-d3d12}}
        export MESA_D3D12_DEFAULT_ADAPTER_NAME=${MESA_D3D12_DEFAULT_ADAPTER_NAME:-NVIDIA}
        ;;
    *) echo 'EDGEAI_DISPLAY_PLATFORM must be auto, native or wslg.' >&2; return 2 ;;
esac
