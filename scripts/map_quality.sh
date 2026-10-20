#!/usr/bin/env bash
# Comparaison between maps of the same scene:
#  * keyframe consistency (tools/map_consistency.py): depth of one keyframe projected into an
#    overlapping one with the map's poses. Sensitive to odometry errors. The main number.
#  * spread around local planes of the exported cloud (tools/map_thickness.py): coarse.
#   scripts/map_quality.sh <name> [<name2> ...]     (--no-cloud skips the second one)
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_lib.sh"
ensure_container "$0" "$@"
[[ $# -ge 1 ]] || { sed -n '2,4p' "$0"; exit 2; }
cloud=1; names=()
for a in "$@"; do [[ "$a" == "--no-cloud" ]] && cloud=0 || names+=("$a"); done
tools=/workspaces/isaac_ros-dev/src/d455_mapping/tools
for n in "${names[@]}"; do
  q="/maps/${n}/_quality"; rm -rf "${q}"; mkdir -p "${q}"
  rtabmap-export --output "${n}" --output_dir "${q}" --poses_camera --images_id "/maps/${n}.db" >/dev/null 2>&1
  python3 "${tools}/map_consistency.py" "${q}" "${n}"
  rm -rf "${q}"
  if [[ ${cloud} -eq 1 ]]; then
    "$(dirname "${BASH_SOURCE[0]}")/export_map.sh" "${n}" --prior > /dev/null 2>&1
    python3 "${tools}/map_thickness.py" "/maps/${n}/${n}_prior_cloud.pcd" | sed "s#/maps/${n}/##"
  fi
done
