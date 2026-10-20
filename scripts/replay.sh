#!/usr/bin/env bash
# Re-run mapping offline from a recorded bag (sim time, no camera).
#   scripts/replay.sh <bag_name> [map_name=<bag_name>_replay] [platform=auto] [--rate R]
# Works for bags recorded on either platform (e.g. Jetson bag on the laptop).
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_lib.sh"
ensure_container "$0" "$@"

bag="/bags/${1:?usage: replay.sh <bag_name> [map_name] [platform] [--rate R]}"; shift
name="${1:-$(basename "${bag}")_replay}"; shift || true
platform="${1:-$(default_platform)}"; shift || true
rate=1.0
[[ "${1:-}" == "--rate" ]] && rate="$2"
check_platform "${platform}"
[[ -d "${bag}" ]] || { echo "${bag} not found" >&2; exit 1; }
db="/maps/${name}.db"
[[ -e "${db}" ]] && { echo "${db} exists; pick another map name" >&2; exit 1; }

qos="$(ros2 pkg prefix d455_mapping)/share/d455_mapping/config/bag_qos_overrides.yaml"
ros2 launch d455_mapping mapping.launch.py platform:="${platform}" camera:=false \
  use_sim_time:=true database_path:="${db}" &
launch_pid=$!
# On exit make sure the WHOLE mapping stack is gone (RTAB-Map writes the database on a graceful
# shutdown, and a container that outlives this script would keep mapping whatever topics it
# sees next, e.g. the live camera). stop.sh sends SIGINT, waits, then kills.
trap '"$(dirname "${BASH_SOURCE[0]}")/stop.sh" 90 >/dev/null || true; wait ${launch_pid} 2>/dev/null || true' EXIT
sleep 8   # let containers/nodes come up before data arrives
ros2 bag play "${bag}" --clock --rate "${rate}" --qos-profile-overrides-path "${qos}"
sleep 3   # let RTAB-Map process the tail
echo "bag finished; stopping mapping (database ${db} is written on shutdown)"
