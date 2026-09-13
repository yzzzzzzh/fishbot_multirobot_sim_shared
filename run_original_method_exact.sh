#!/usr/bin/env bash
# Exact launcher used by the first successful run: no replacement compose file
# and no replacement runtime mount.  This wrapper only keeps the entry point
# in the shared directory.
set -euo pipefail

shared_dir=$(cd "$(dirname "$0")" && pwd)
repo_dir=${FISHBOT_REPO_DIR:-"$shared_dir/../fishbot_multirobot_sim"}
if [ ! -x "$repo_dir/scripts/run_legged_single_exploration.sh" ]; then
  printf 'FISHBOT_REPO_DIR must point to fishbot_multirobot_sim; got %s\n' "$repo_dir" >&2
  exit 2
fi

duration=${1:-1000}
run_dir=${2:-/tmp/original_method_exact_$(date +%Y%m%d_%H%M%S)}
export RACER_CONSISTENCY_SIGN=1.0
export RACER_FIRST_GRID_BONUS=6.0
export RACER_MAP_WARMUP_SECONDS=8
exec bash "$repo_dir/scripts/run_legged_single_exploration.sh" "$duration" "$run_dir"
