#!/usr/bin/env bash
# Record raw camera + IMU topics (MCAP) into bags/<name>.
#   scripts/record.sh [name=d455_<date>] [platform=auto] [extra launch args]
# Camera + IMU only:                scripts/record.sh lab_raw
# Camera + IMU + every PX4 topic:   scripts/record.sh lab_raw laptop px4:=true
# Other extra data (MAVROS, ...):   scripts/record.sh lab_raw laptop extra_regex:='/mavros/.*'
#                                   scripts/record.sh lab_raw laptop extra_topics:=/a,/b
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_lib.sh"
ensure_container "$0" "$@"

name="${1:-d455_$(date +%Y%m%d_%H%M%S)}"; shift || true
platform="${1:-$(default_platform)}"; shift || true
check_platform "${platform}"
[[ -e "/bags/${name}" ]] && { echo "/bags/${name} exists" >&2; exit 1; }
exec ros2 launch d455_mapping record.launch.py platform:="${platform}" name:="${name}" "$@"
