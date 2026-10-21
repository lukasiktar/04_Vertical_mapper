#!/usr/bin/env bash
# RViz on a virtual desktop inside the container, viewed from another computer over VNC (SSH tunnel).
# Use when the Jetson has no monitor and X11 forwarding is not available.
# USE (on the Jetson):
#   scripts/rviz_vnc.sh start [config=mapping_remote] [WxH=1600x900]
#       config: rviz/<config>.rviz in d455_mapping (mapping_remote, localization_remote, show_map, ...)
#   scripts/rviz_vnc.sh restart [config] [WxH]   (full reset: kills everything, removes stale X locks, starts again)
#   scripts/rviz_vnc.sh stop
#   scripts/rviz_vnc.sh status
# On the viewer computer:
#   ssh -L 5901:localhost:5901 <jetson>      (keep open)
#   vncviewer localhost:5901                 (no password; the server only listens on localhost)
# Rendering is software OpenGL (CPU): fine for camera, TF, paths; a huge point cloud will be slow.
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_lib.sh"
ensure_container "$0" "$@"

cmd="${1:-status}"; cfg="${2:-mapping_remote}"; geom="${3:-1600x900}"
rviz_cfg="/workspaces/isaac_ros-dev/src/d455_mapping/rviz/${cfg}.rviz"

up() { pgrep -x "$1" >/dev/null; }

stop_all() {
  pkill -x rviz2 || true; pkill -x openbox || true; pkill -x Xtigervnc || true
  sleep 1; pkill -9 -x rviz2 || true; pkill -9 -x openbox || true; pkill -9 -x Xtigervnc || true
  # a crashed X server leaves these behind and the next Xtigervnc :1 then refuses to start
  rm -f /tmp/.X1-lock /tmp/.X11-unix/X1
}

case "${cmd}" in
  restart) stop_all; exec "$0" start "${cfg}" "${geom}" ;;
  start)
    command -v Xtigervnc >/dev/null && command -v openbox >/dev/null || {
      echo "tigervnc/openbox missing in this image. Rebuild it (scripts/dev.sh --build-image) or, as a stopgap:" >&2
      echo "  docker exec -u root isaac_ros_dev-$(uname -m)-d455-container apt-get install -y --no-install-recommends tigervnc-standalone-server openbox" >&2
      exit 1; }
    [[ -f "${rviz_cfg}" ]] || { echo "${rviz_cfg} not found" >&2; exit 1; }
    pkill -x rviz2 || true
    up Xtigervnc || { setsid nohup Xtigervnc :1 -geometry "${geom}" -depth 24 -localhost yes -SecurityTypes None \
        -rfbport 5901 -AlwaysShared -noreset > /tmp/xvnc.log 2>&1 & sleep 2; }
    up Xtigervnc || { echo "Xtigervnc failed to start:" >&2; tail -20 /tmp/xvnc.log >&2
      echo "Try: scripts/rviz_vnc.sh restart" >&2; exit 1; }
    up openbox || { DISPLAY=:1 setsid nohup openbox > /tmp/openbox.log 2>&1 & sleep 1; }
    # Software OpenGL is CPU-hungry (~300% of 6 cores unrestricted): keep it on cores 4-5 with 2 render
    # threads so cuVSLAM/RTAB-Map (cores 0-3) are not starved (RTAB-Map drops frames when odom is late).
    DISPLAY=:1 LIBGL_ALWAYS_SOFTWARE=1 LP_NUM_THREADS=2 QT_X11_NO_MITSHM=1 \
      setsid nohup taskset -c 4,5 nice -n 10 rviz2 -d "${rviz_cfg}" > /tmp/rviz.log 2>&1 &
    sleep 3
    echo "RViz (${cfg}) running on the virtual desktop. On your computer:"
    echo "  ssh -L 5901:localhost:5901 <jetson>    then    vncviewer localhost:5901" ;;
  stop)   stop_all; echo "stopped" ;;
  status) for p in Xtigervnc openbox rviz2; do up "$p" && echo "$p: running" || echo "$p: not running"; done ;;
  *) echo "usage: rviz_vnc.sh start|stop|restart|status [config] [WxH]" >&2; exit 2 ;;
esac
