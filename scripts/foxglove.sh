#!/usr/bin/env bash
# Foxglove WebSocket bridge: view topics (images, TF, RTAB-Map cloud, paths) from another computer
# in Foxglove Studio, with no ROS/DDS/X11 needed there. Run next to map.sh / localize.sh.
# USE:
#   scripts/foxglove.sh [--open] [port=8765]
#
# Default binds to 127.0.0.1 inside the container (host network) = reachable only through an SSH tunnel:
#   viewer computer:  ssh -L 8765:localhost:8765 <jetson>     then Foxglove "Open connection"
#                     -> Foxglove WebSocket -> ws://localhost:8765
# --open binds 0.0.0.0 (anyone on the network can read all topics, there is no authentication):
#                     ws://<jetson-ip>:8765
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_lib.sh"
ensure_container "$0" "$@"

addr=127.0.0.1; port=8765
for a in "$@"; do
  case "$a" in
    --open) addr=0.0.0.0 ;;
    [0-9]*) port="$a" ;;
    *) echo "unknown option $a" >&2; exit 2 ;;
  esac
done
echo "foxglove_bridge on ${addr}:${port}. Ctrl-C to stop."
exec ros2 launch foxglove_bridge foxglove_bridge_launch.xml address:="${addr}" port:="${port}"
