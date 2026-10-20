#!/usr/bin/env bash
# Improve a saved map WITHOUT replaying the bag: re-runs RTAB-Map's graph SLAM over the stored RGB-D
# keyframes with settings that suit a drone's PX4 odometry (measured on a real bag: keyframe depth
# mismatch 52.8 -> 35.8 mm): realistic odometry uncertainty, visual refinement of every link,
# more loop closures, local bundle adjustment, and optionally a corrected camera mounting.
#   scripts/refine_map.sh <in_map> <out_map> [--cam-tf "x y z roll pitch yaw"] [--odom-var LIN ANG]
# The mounting is base_link -> camera_link as in replay_px4.sh (m, rad). Get it from
# scripts/calibrate_rig.sh. Input is never modified. Check with scripts/map_quality.sh.
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_lib.sh"
ensure_container "$0" "$@"
in="${1:?usage: refine_map.sh <in_map> <out_map> [--cam-tf \"x y z r p y\"] [--odom-var LIN ANG]}"; shift
out="${1:?out map name}"; shift
cam=(); lin=0.05; ang=0.03
while [[ $# -gt 0 ]]; do
  case "$1" in
    --cam-tf) cam=(-cam_tf "$2"); shift 2 ;;
    --odom-var) lin="$2"; ang="$3"; shift 3 ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
  esac
done
[[ -f "/maps/${in}.db" ]] || { echo "/maps/${in}.db not found" >&2; exit 1; }
[[ "${in}" != "${out}" ]] || { echo "out_map must differ from in_map" >&2; exit 1; }
[[ ! -e "/maps/${out}.db" ]] || { echo "/maps/${out}.db exists" >&2; exit 1; }
rtabmap-reprocess "${cam[@]}" --RGBD/NeighborLinkRefining true --Reg/Strategy 0 \
  -odom_lin_var "${lin}" -odom_ang_var "${ang}" --Vis/MinInliers 12 --RGBD/OptimizeMaxError 6 \
  --RGBD/ProximityPathMaxNeighbors 10 --RGBD/LocalBundleOnLoopClosure true \
  "/maps/${in}.db" "/maps/${out}.db" 2>&1 | sed 's/\x1b\[[0-9;]*m//g' | grep -E "Processed ([0-9]*0|[0-9]*5)/|Error|FATAL" | tail -3
echo "wrote /maps/${out}.db. Compare: scripts/map_quality.sh --no-cloud ${in} ${out}"
