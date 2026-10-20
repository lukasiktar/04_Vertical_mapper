#!/usr/bin/env bash
# Build a map from a bag with an RGB-D camera + PX4 odometry (no IR images / IMU needed).
#   scripts/replay_px4.sh <bag_name> <map_name> [--rate R] [--start-offset S] [--force] [launch args]
# Launch args (see src/d455_mapping/launch/bag_px4.launch.py), e.g.
#   cam_xyz:='0.10 0 -0.05'   camera position in the drone body frame FLU [m]
#   cam_rpy:='0 0.35 0'       camera roll pitch yaw in that frame [rad] (pitch + = looking down)
#   camera_ns:=/camera/camera topic namespace in the bag;  rviz:=false;  time_offset_s:=0.02
# Result: maps/<map_name>.db (same format as maps from map.sh; localize with localize.sh).
# The bag must be sqlite3 (.db3) because PX4's odometry is read from the file (px4_msgs not needed).
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_lib.sh"
ensure_container "$0" "$@"

bag="/bags/${1:?usage: replay_px4.sh <bag_name> <map_name> [--rate R] [--start-offset S] [launch args]}"; shift
name="${1:?map name needed}"; shift
rate=1.0; offset=0.0; force=0; extra=(); ns=/camera/camera
while [[ $# -gt 0 ]]; do
  case "$1" in
    --rate) rate="$2"; shift 2 ;;
    --start-offset) offset="$2"; shift 2 ;;
    --force) force=1; shift ;;
    camera_ns:=*) ns="${1#camera_ns:=}"; extra+=("$1"); shift ;;
    *) extra+=("$1"); shift ;;
  esac
done
[[ -d "${bag}" ]] || { echo "${bag} not found" >&2; exit 1; }
db3="$(ls "${bag}"/*.db3 2>/dev/null | head -1 || true)"
[[ -n "${db3}" ]] || { echo "no .db3 file in ${bag}: this script reads PX4 odometry from sqlite3 bags" >&2; exit 1; }
db="/maps/${name}.db"
if [[ -e "${db}" && ${force} -ne 1 ]]; then echo "${db} exists; use another name or --force" >&2; exit 1; fi
rm -f "${db}"

log=/tmp/replay_px4_launch.log
trap '"$(dirname "${BASH_SOURCE[0]}")/stop.sh" 120 >/dev/null || true' EXIT
ros2 launch d455_mapping bag_px4.launch.py bag_db:="${db3}" database_path:="${db}" \
  "${extra[@]}" > "${log}" 2>&1 &
echo "waiting for the PX4 odometry to load from ${db3} (a large bag can take a minute) ..."
for _ in $(seq 300); do
  grep -q "px4 odometry ready" "${log}" 2>/dev/null && break
  if grep -qE "process has died|RuntimeError|Traceback" "${log}" 2>/dev/null; then
    sed 's/\x1b\[[0-9;]*m//g' "${log}" | grep -E "ERROR|RuntimeError|Traceback|rror" | head -5 >&2; exit 1; fi
  sleep 1
done
sed 's/\x1b\[[0-9;]*m//g' "${log}" | grep -E "px4 odometry ready|reset #" | sed -E 's/^.*\] //'
sleep 8   # let RTAB-Map and the depth registration come up before data arrives
qos="$(ros2 pkg prefix d455_mapping)/share/d455_mapping/config/bag_qos_overrides.yaml"
echo "playing ${bag} at ${rate}x (camera topics only; PX4 comes from the file) ..."
ros2 bag play "${bag}" --clock --rate "${rate}" --start-offset "${offset}" \
  --qos-profile-overrides-path "${qos}" \
  --topics "${ns}/color/image_raw" "${ns}/color/camera_info" "${ns}/depth/image_rect_raw" \
           "${ns}/depth/camera_info" /tf_static
sleep 4
echo "bag finished; stopping (RTAB-Map writes ${db} on shutdown) ..."
