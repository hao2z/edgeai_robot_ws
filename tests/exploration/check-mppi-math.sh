#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cache=${EDGEAI_NAV_CACHE:-$HOME/.cache/edgeai-open-source}
out="$cache/mppi-math-check"
mkdir -p "$out"
for compiler in /usr/bin/g++ "$cache/env/.pixi/envs/default/bin/c++"; do
    for mode in simd-fast simd-strict scalar-fast; do
        flags=(-O3 -mavx2 -mfma -I"$cache/mppi-deps/install/include")
        [[ $mode == simd-strict ]] || flags+=(-ffast-math)
        [[ $mode == scalar-fast ]] || flags+=(-DXTENSOR_USE_XSIMD -DXTENSOR_ENABLE_XSIMD)
        label=system
        [[ $compiler == /usr/bin/g++ ]] || label=conda
        "$compiler" "${flags[@]}" "$root/tests/exploration/mppi_math_check.cpp" -o "$out/$label-$mode"
        printf '%s %s: ' "$label" "$mode"
        "$out/$label-$mode" || true
    done
done
