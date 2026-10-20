#!/usr/bin/env bash
# Replay a recorded bag in LOCALIZATION mode with the cross detector, to tune its parameters
# without the camera. The map (maps/<map>.db) is only read; the result goes to its own file
# /maps/<map>_crosses_replay.json (deleted at start), never to <map>_crosses.json.
# USE:
#   scripts/crosses_from_bag.sh <bag_name> <map_name> [platform=auto] [--rate R] [--no-rviz]
#                               [--params FILE.yaml] [extra launch args...]

set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_lib.sh"
ensure_container "$0" "$@"

bag="/bags/${1:?usage: crosses_from_bag.sh <bag_name> <map_name> [platform] [--rate R] [--no-rviz] [--params FILE] [launch args]}"; shift
name="${1:?map name missing}"; shift
platform="$(default_platform)"
[[ "${1:-}" == laptop || "${1:-}" == orin_nano ]] && { platform="$1"; shift; }
check_platform "${platform}"
rate=1.0; extra=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --rate) rate="$2"; shift 2 ;;
    --no-rviz) extra+=(rviz:=false); shift ;;
    --params) extra+=(cross_params:="$2"); shift 2 ;;
    *) extra+=("$1"); shift ;;
  esac
done
[[ -d "${bag}" ]] || { echo "${bag} not found" >&2; exit 1; }
db="/maps/${name}.db"
[[ -f "${db}" ]] || { echo "${db} not found" >&2; exit 1; }
out="/maps/${name}_crosses_replay.json"
rm -f "${out}"

qos="$(ros2 pkg prefix d455_mapping)/share/d455_mapping/config/bag_qos_overrides.yaml"
ros2 launch d455_mapping localization.launch.py platform:="${platform}" camera:=false \
  use_sim_time:=true database_path:="${db}" with_cross_detector:=true with_pins:=true \
  landmarks_file:="${out}" "${extra[@]}" &
launch_pid=$!
trap '"$(dirname "${BASH_SOURCE[0]}")/stop.sh" 90 >/dev/null || true; wait ${launch_pid} 2>/dev/null || true' EXIT
sleep 8   # let containers/nodes come up before data arrives
ros2 bag play "${bag}" --clock --rate "${rate}" --qos-profile-overrides-path "${qos}"
sleep 3   # let the detector process the tail
"$(dirname "${BASH_SOURCE[0]}")/stop.sh" 90 >/dev/null || true   # the detector saves on SIGINT
wait ${launch_pid} 2>/dev/null || true
echo
if [[ -f "${out}" ]]; then
  echo "Confirmed crosses in ${out}:"
  python3 - "${out}" <<'PY'
import json, sys
d = json.load(open(sys.argv[1]))
for c in d['crosses']:
    print('  cross %d: %3d observations, center [%.3f %.3f %.3f] in %s' % (c['id'], c['count'], *c['center'], d['frame']))
print('  %d cross(es)' % len(d['crosses']))
PY
else
  echo "No landmark file written (the detector did not start?)."
fi
