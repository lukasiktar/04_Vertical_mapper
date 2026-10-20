# Shared helpers for scripts/*.sh.
# Scripts marked "container" re-exec themselves inside the dev container when started on
# the host, so both `./scripts/map.sh lab` (host) and `scripts/map.sh lab` (container) work.

WS_HOST="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

in_container() { [[ -f /.dockerenv && -d /workspaces/isaac_ros-dev ]]; }

# ROS setup scripts reference unset variables
ros_source() {
  local had_u=0; [[ $- == *u* ]] && had_u=1
  set +u
  # shellcheck disable=SC1090
  source "$1"
  [[ ${had_u} -eq 1 ]] && set -u
  return 0
}

# Usage: ensure_container "$0" "$@"
ensure_container() {
  if ! in_container; then
    local script; script="scripts/$(basename "$1")"; shift
    exec "${WS_HOST}/scripts/dev.sh" -- "${script}" "$(printf '%q ' "$@")"
  fi
  cd /workspaces/isaac_ros-dev
  ros_source /opt/ros/humble/setup.bash
  [[ -f install/setup.bash ]] && ros_source install/setup.bash
  if ! ros2 pkg prefix d455_mapping &>/dev/null; then
    echo "d455_mapping is not built yet; running scripts/build.sh" >&2
    scripts/build.sh
    ros_source install/setup.bash
  fi
}

check_platform() {
  case "$1" in
    laptop|orin_nano) ;;
    *) echo "platform must be laptop or orin_nano (got '$1')" >&2; exit 2 ;;
  esac
}

# Default platform from the CPU architecture.
default_platform() { [[ "$(uname -m)" == "aarch64" ]] && echo orin_nano || echo laptop; }
