#!/usr/bin/env bash
# Use the upstream generator unchanged; never rescale only JSON metadata.
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../ros/workspace-env.sh"
cache=${EDGEAI_NAV_CACHE:-$HOME/.cache/edgeai-open-source}
revision=d5d768b7b38a05e7cfd2b6c693b786b402161bed
source_dir="$cache/nav2-primitives"
mkdir -p "$source_dir" "$cache/lattice-env" "$PROJECT_ROOT/tests/results/lattice"
if [[ ! -d "$source_dir/.git" ]]; then
    git -C "$source_dir" init -q
    git -C "$source_dir" remote add origin https://github.com/ros-navigation/navigation2.git
fi
if ! git -C "$source_dir" cat-file -e "$revision^{commit}" 2>/dev/null; then
    git -C "$source_dir" fetch --depth 1 --filter=blob:none origin "$revision"
fi
git -C "$source_dir" sparse-checkout set nav2_smac_planner/lattice_primitives
git -C "$source_dir" checkout --detach "$revision"
cp "$PROJECT_ROOT/algorithms/exploration/lattice-env/pixi.toml" "$cache/lattice-env/"
if [[ -f "$PROJECT_ROOT/algorithms/exploration/lattice-env/pixi.lock" ]]; then
    cp "$PROJECT_ROOT/algorithms/exploration/lattice-env/pixi.lock" "$cache/lattice-env/"
fi
pixi run --manifest-path "$cache/lattice-env/pixi.toml" env MPLBACKEND=Agg \
    python "$source_dir/nav2_smac_planner/lattice_primitives/generate_motion_primitives.py" \
    --config "$PROJECT_ROOT/algorithms/exploration/lattice-r020.json" \
    --output "$PROJECT_ROOT/src/edgeai_robot_description/config/lattice/diff_5cm_r020.json" \
    --visualizations "$PROJECT_ROOT/tests/results/lattice"
cp "$cache/lattice-env/pixi.lock" "$PROJECT_ROOT/algorithms/exploration/lattice-env/pixi.lock"
