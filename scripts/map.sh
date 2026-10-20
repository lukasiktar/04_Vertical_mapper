#!/usr/bin/env bash
# Create a new map: camera + cuVSLAM + RTAB-Map (+ nvblox, + RViz per platform config).
# USE:
#   scripts/map.sh [name=map] [platform=auto] [--force] [extra launch args...]
#
#  platform=auto: auto-detect laptop vs. Jetson (default), or force with laptop / orin_nano.
#  --force: delete an existing maps/<name>.db (otherwise it refuses to overwrite)
#
# Database: maps/<name>.db (container: /maps/<name>.db). 
#Stop with Ctrl-C; RTAB-Map writes
# the database on shutdown.
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_lib.sh"
ensure_container "$0" "$@"

name="${1:-map}"; shift || true
platform="${1:-$(default_platform)}"; shift || true
force=0
extra=()
for a in "$@"; do [[ "$a" == "--force" ]] && force=1 || extra+=("$a"); done
check_platform "${platform}"
db="/maps/${name}.db"
if [[ -e "${db}" && ${force} -ne 1 ]]; then
  echo "${db} exists; mapping would DELETE it. Use another name or --force." >&2
  exit 1
fi
echo "Mapping into ${db} (platform ${platform}). Ctrl-C to stop and save."
exec ros2 launch d455_mapping mapping.launch.py platform:="${platform}" \
  database_path:="${db}" "${extra[@]}"
