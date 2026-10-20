#!/usr/bin/env bash
# Find where the camera sits on the drone from the data itself.
# USE:
#   scripts/calibrate_rig.sh <map> <bag>        e.g. scripts/calibrate_rig.sh indoor1 stup_indoor_1
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_lib.sh"
ensure_container "$0" "$@"
map="${1:?usage: calibrate_rig.sh <map> <bag>}"; bag="${2:?bag name}"
db3="$(ls /bags/"${bag}"/*.db3 2>/dev/null | head -1 || true)"
[[ -n "${db3}" ]] || { echo "no .db3 in /bags/${bag}" >&2; exit 1; }
[[ -f "/maps/${map}.db" ]] || { echo "/maps/${map}.db not found" >&2; exit 1; }
q="/maps/${map}/_calib_rig"; rm -rf "${q}"; mkdir -p "${q}"
trap 'rm -rf "${q}"' EXIT
rtabmap-export --output "${map}" --output_dir "${q}" --poses_camera --images_id "/maps/${map}.db" >/dev/null 2>&1
python3 /workspaces/isaac_ros-dev/src/d455_mapping/tools/calibrate_rig.py "${q}" "${map}" "/maps/${map}.db" "${db3}"
