#!/usr/bin/env bash
# Extend an existing map in ITS coordinates: camera + cuVSLAM + RTAB-Map mapping as a new session
# in a copy of the old database (RTAB-Map multi-session).
# USE:
#   scripts/extend_map.sh <old> <new> [platform=auto] [--force] [extra launch args...]
#   e.g. scripts/extend_map.sh flight_map1 flight_map2 orin_nano with_cross_detector:=true px4_vio:=true
#
#  maps/<old>.db (+ <old>_crosses.json, <old>_pins.json) are copied to maps/<new>.*; <old> is never
#  modified. The old map is loaded and the new session joins it at its first loop closure on an
#  old keyframe: from then on everything new is in the old map's coordinates. So START (or pass)
#  where the old map is, looking at what it saw. Until it has joined, the new part is drawn at the
#  origin and the cross detector does not fuse (it logs "joined the old map" when it does).
#  --force: overwrite an existing maps/<new>.db
#  Use the same base_link setting as the old map (px4_vio:=true or not).
#
# Stop with Ctrl-C once and wait (RTAB-Map writes the database). Then scripts/map_info.sh <new>
# ("sessions joined") and scripts/localize.sh <new> ...
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_lib.sh"
ensure_container "$0" "$@"

usage="usage: extend_map.sh <old> <new> [platform] [--force] [launch args]"
old="${1:?${usage}}"; shift
new="${1:?${usage}}"; shift
platform="${1:-$(default_platform)}"; shift || true
force=0
extra=()
for a in "$@"; do [[ "$a" == "--force" ]] && force=1 || extra+=("$a"); done
check_platform "${platform}"
src="/maps/${old}.db"
db="/maps/${new}.db"
[[ -f "${src}" ]] || { echo "${src} not found" >&2; exit 1; }
[[ "${old}" != "${new}" ]] || { echo "<new> must differ from <old> (the old map is kept as it is)" >&2; exit 1; }
if [[ -e "${db}" && ${force} -ne 1 ]]; then
  echo "${db} exists; extending would REPLACE it. Use another name or --force." >&2
  exit 1
fi
cp -f "${src}" "${db}"
for kind in crosses pins; do
  if [[ -f "/maps/${old}_${kind}.json" ]]; then
    cp -f "/maps/${old}_${kind}.json" "/maps/${new}_${kind}.json"
  else
    rm -f "/maps/${new}_${kind}.json"
  fi
done
last_id="$(python3 -c 'import sqlite3, sys; print(sqlite3.connect(sys.argv[1]).execute("select max(id) from Node").fetchone()[0] or 0)' "${db}")"
echo "Extending ${old} (keyframes up to id ${last_id}) into ${db} (platform ${platform})."
echo "Start where ${old} was mapped. Ctrl-C to stop and save."
exec ros2 launch d455_mapping mapping.launch.py platform:="${platform}" \
  database_path:="${db}" keep_database:=true link_node_max:="${last_id}" "${extra[@]}"
