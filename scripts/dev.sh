#!/usr/bin/env bash
# Start / enter the Isaac ROS 3.2 dev container for this project.
# USE:
#   ./scripts/dev.sh                 start container, open a shell (build image if needed)
#   ./scripts/dev.sh -- <cmd...>     run <cmd> inside the (running) container, e.g.
#                                    ./scripts/dev.sh -- scripts/map.sh lab laptop
#   ./scripts/dev.sh --build-image   build/refresh the Docker image
#   ./scripts/dev.sh --detach        start the container in the background
#   ./scripts/dev.sh --stop          stop the container
#
# Environment (optional):
#   ROS_DOMAIN_ID   passed into the container (default 0); must match on Jetson + laptop
#   D455_RMW        RMW_IMPLEMENTATION inside the container (default: image default,
#                   rmw_fastrtps_cpp); see docs/JETSON.md for remote RViz
#   CYCLONEDDS_URI, D455_PEER, FASTRTPS_DEFAULT_PROFILES_FILE   passed through if set

set -euo pipefail


tput setaf 1 &>/dev/null || export TERM=xterm

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMMON="${WS}/src/isaac_ros_common"
PLATFORM="$(uname -m)"
CONTAINER="isaac_ros_dev-${PLATFORM}-d455-container"
IMAGE="isaac_ros_dev-${PLATFORM}-d455"
IMAGE_KEY="${PLATFORM}.ros2_humble.realsense.d455_mapping"

if [[ ! -f "${COMMON}/scripts/.isaac_ros_common-config" ]]; then
  "${WS}/scripts/setup_workspace.sh"
fi

running() { [[ -n "$(docker ps -q --filter "name=^${CONTAINER}$" --filter status=running)" ]]; }

# Interactive shell / command in an already running container.
exec_in() {
  local env='source /opt/ros/humble/setup.bash; [ -f install/setup.bash ] && source install/setup.bash;'
  if [[ -t 0 && -t 1 ]]; then
    docker exec -it -u admin -w /workspaces/isaac_ros-dev "${CONTAINER}" bash -ic "$*"
  else
    docker exec -i -u admin -w /workspaces/isaac_ros-dev "${CONTAINER}" bash -c "${env} $*"
  fi
}

# X11. No 'xhost +'.
x11_args() {
  [[ -z "${DISPLAY:-}" ]] && return 0
  command -v xauth >/dev/null || { echo "WARNING: xauth not found, X11 disabled" >&2; return 0; }
  local xa="${XDG_RUNTIME_DIR:-/tmp}/d455_mapping.xauth"
  rm -f "${xa}"; touch "${xa}"; chmod 600 "${xa}"
  xauth nlist "${DISPLAY}" | sed -e 's/^..../ffff/' | xauth -f "${xa}" nmerge - 2>/dev/null || true
  # run_dev.sh bind-mounts ~/.Xauthority; make sure Docker does not create it as a directory.
  [[ -e "${HOME}/.Xauthority" ]] || touch "${HOME}/.Xauthority"
  RUN_ARGS+=(-a "-v ${xa}:/tmp/.d455.xauth:ro" -a "-e XAUTHORITY=/tmp/.d455.xauth" -a "-e QT_X11_NO_MITSHM=1")
}

build_image() {
  ( cd "${COMMON}/scripts" && ./build_image_layers.sh --image_key "${IMAGE_KEY}" --image_name "${IMAGE}" )
}

start_container() {
  # run_dev.sh forwards ROS_DOMAIN_ID from this environment.
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
  RUN_ARGS=(-d "${WS}"
            -a "-v ${WS}/maps:/maps" -a "-v ${WS}/bags:/bags")
  [[ -n "${D455_RMW:-}" ]] && RUN_ARGS+=(-a "-e RMW_IMPLEMENTATION=${D455_RMW}")
  [[ -n "${CYCLONEDDS_URI:-}" ]] && RUN_ARGS+=(-a "-e CYCLONEDDS_URI=${CYCLONEDDS_URI}")
  [[ -n "${D455_PEER:-}" ]] && RUN_ARGS+=(-a "-e D455_PEER=${D455_PEER}")
  # Fast DDS (default RMW): large shared-memory segments for in-host image topics.
  if [[ -z "${D455_RMW:-}" || "${D455_RMW}" == "rmw_fastrtps_cpp" ]]; then
    RUN_ARGS+=(-a "-e FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-/workspaces/isaac_ros-dev/config/dds/fastdds_shm.xml}")
  fi
  x11_args
  "${COMMON}/scripts/run_dev.sh" "${RUN_ARGS[@]}" "$@"
}

case "${1:-}" in
  --build-image)
    build_image ;;
  --stop)
    docker stop "${CONTAINER}" >/dev/null && echo "stopped ${CONTAINER}" ;;
  --detach)
    if running; then echo "${CONTAINER} already running"; exit 0; fi
    start_container -a "-d"
    # wait until the entrypoint created the 'admin' user
    for _ in $(seq 60); do docker exec "${CONTAINER}" id admin &>/dev/null && break; sleep 1; done
    echo "started ${CONTAINER}" ;;
  --)
    shift
    if ! running; then "$0" --detach; fi
    exec_in "$@" ;;
  "")
    if running; then exec_in bash; else start_container; fi ;;
  *)
    sed -n '2,20p' "$0"; exit 1 ;;
esac
