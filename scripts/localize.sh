#!/usr/bin/env bash
# Localize against an existing map (RTAB-Map localization mode, database read-only use).
# USE:
#   scripts/localize.sh <name> [platform=auto] [--no-prior-map | --prior-map] [extra launch args...]
#
#  platform=auto: auto-detect laptop vs. Jetson (default), or force with laptop / orin_nano.
#  --prior-map: show the saved map as a point cloud in RViz 
#  --no-prior-map: do not show the saved map in RViz 
#
# By default (laptop) RViz also shows the WHOLE saved map as a point cloud (/prior_map, made from
# the database on first use and cached in maps/<name>/<name>_prior_cloud.pcd
# On the Jetson the cloud is off by default (heavy over Wi-Fi); force with --prior-map.
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_lib.sh"
ensure_container "$0" "$@"

name="${1:?usage: localize.sh <name> [platform] [--no-prior-map|--prior-map] [launch args]}"; shift
platform="${1:-$(default_platform)}"; shift || true
check_platform "${platform}"
db="/maps/${name}.db"
[[ -f "${db}" ]] || { echo "${db} not found" >&2; exit 1; }

want_prior=0; [[ "${platform}" == "laptop" ]] && want_prior=1
extra=()
for a in "$@"; do
  case "$a" in
    --prior-map) want_prior=1 ;;
    --no-prior-map) want_prior=0 ;;
    *) extra+=("$a") ;;
  esac
done

prior_arg=()
if [[ ${want_prior} -eq 1 ]]; then
  cfg="/workspaces/isaac_ros-dev/src/d455_mapping/config/${platform}.yaml"
  rng="$(awk '/^depth_max_m:/{print $2; exit}' "${cfg}")"; rng="$(printf '%g' "${rng:-0}")"
  pcd="/maps/${name}/${name}_prior_cloud.pcd"
  info="/maps/${name}/${name}_prior_cloud.info"
  if [[ ! -f "${pcd}" || "${db}" -nt "${pcd}" || "$(cat "${info}" 2>/dev/null)" != "max_range=${rng} voxel=0.02" ]]; then
    echo "Building the saved-map cloud for RViz (once; max range ${rng} m)..."
    "$(dirname "${BASH_SOURCE[0]}")/export_map.sh" "${name}" --prior --max-range "${rng}" >/dev/null
  fi
  prior_arg=(prior_map:="${pcd}")
fi
echo "Localizing in ${db} (platform ${platform}). Watch: ros2 topic echo /rtabmap/info --field loop_closure_id"
exec ros2 launch d455_mapping localization.launch.py platform:="${platform}" \
  database_path:="${db}" "${prior_arg[@]}" "${extra[@]}"
