#!/usr/bin/env bash
## The Task 7 sweep through tools/offline_replay.py: same run names as run_sweep.sh.
##
##   tools/offline_sweep.sh ~/bags/maze_drive "1 5 10 20 50" "1 2 3" 4
##
## The last argument is how many runs to execute in parallel (default 4).
## Run from the package directory. Writes runs/fs{1,2}_n<N>_s<seed>.npz.
set -euo pipefail
BAG=$(realpath "${1:?usage: offline_sweep.sh <bag_dir> [particle_counts] [seeds] [jobs]}")
NS=${2:-"1 5 10 20 50"}
SEEDS=${3:-"1 2 3"}
JOBS=${4:-4}
cd "$(dirname "$0")/.."
mkdir -p runs

for n in $NS; do
  for s in $SEEDS; do
    for improved in true false; do
      tag="$([ "$improved" = true ] && echo fs2 || echo fs1)_n${n}_s${s}"
      echo "$tag -p num_particles:=$n -p seed:=$s -p use_improved_proposal:=$improved"
    done
  done
done | xargs -P "$JOBS" -L 1 sh -c \
  'echo "=== $0"; python3 tools/offline_replay.py "'"$BAG"'" "$0" "$@" > "runs/$0.log" 2>&1'
echo "done -- run logs are in runs/"
