#!/usr/bin/env bash
# Fetch pinned third-party sources and write the isaac_ros_common config.
# Run on the HOST (laptop or Jetson) after cloning this repo. No sudo, idempotent.
#
#   src/isaac_ros_common   isaac_ros_common @ ISAAC_ROS_TAG (only its docker/ + scripts/
#                          are used; COLCON_IGNOREd so colcon uses the apt packages)
#   src/realsense_splitter realsense_splitter from isaac_ros_nvblox @ ISAAC_ROS_TAG
#                          (not shipped as a .deb; the Isaac ROS docs build it from source)
set -euo pipefail

ISAAC_ROS_TAG="v3.2-15"   # keep in sync with VERSIONS.md
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="${WS}/src"

command -v git-lfs >/dev/null || { echo "git-lfs is required (sudo apt-get install git-lfs && git lfs install --skip-repo)" >&2; exit 1; }

# --- isaac_ros_common ---------------------------------------------------------
if [[ ! -d "${SRC}/isaac_ros_common/.git" ]]; then
  git clone -q https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git "${SRC}/isaac_ros_common"
fi
git -C "${SRC}/isaac_ros_common" fetch -q --tags origin
git -C "${SRC}/isaac_ros_common" -c advice.detachedHead=false checkout -q "${ISAAC_ROS_TAG}"
touch "${SRC}/isaac_ros_common/COLCON_IGNORE"
echo "isaac_ros_common @ ${ISAAC_ROS_TAG} ($(git -C "${SRC}/isaac_ros_common" rev-parse --short HEAD))"

# --- realsense_splitter ------------------------------------------------------
tmp="$(mktemp -d)"
trap 'rm -rf "${tmp}"' EXIT
GIT_LFS_SKIP_SMUDGE=1 git -c advice.detachedHead=false clone -q --depth 1 -b "${ISAAC_ROS_TAG}" \
  https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git "${tmp}/isaac_ros_nvblox"
rm -rf "${SRC}/realsense_splitter"
cp -r "${tmp}/isaac_ros_nvblox/nvblox_examples/realsense_splitter" "${SRC}/realsense_splitter"
rm -f "${SRC}/realsense_splitter/COLCON_IGNORE"
cp "${tmp}/isaac_ros_nvblox/LICENSE" "${SRC}/realsense_splitter/LICENSE"
# Local patch: invert_emitter_mode parameter (default off), see VERSIONS.md / README.
patch -s -d "${SRC}/realsense_splitter" -p1 < "${WS}/patches/realsense_splitter_invert_emitter_mode.patch"
echo "realsense_splitter @ isaac_ros_nvblox ${ISAAC_ROS_TAG} ($(git -C "${tmp}/isaac_ros_nvblox" rev-parse --short HEAD))"

# --- run_dev.sh config -------------------------------------------------------
# Image key: <arch>.ros2_humble.realsense.d455_mapping
#   ros2_humble  -> isaac_ros_common/docker/Dockerfile.ros2_humble
#   realsense    -> isaac_ros_common/docker/Dockerfile.realsense (librealsense + realsense-ros pins)
#   d455_mapping -> ${WS}/docker/Dockerfile.d455_mapping (this project)
cat > "${SRC}/isaac_ros_common/scripts/.isaac_ros_common-config" <<EOF
CONFIG_IMAGE_KEY=ros2_humble.realsense.d455_mapping
CONFIG_DOCKER_SEARCH_DIRS=(${WS}/docker)
CONFIG_CONTAINER_NAME_SUFFIX=d455
EOF
echo "wrote ${SRC}/isaac_ros_common/scripts/.isaac_ros_common-config"

if [[ -f "${HOME}/.isaac_ros_common-config" ]]; then
  echo "WARNING: ${HOME}/.isaac_ros_common-config exists and OVERRIDES the project config." >&2
fi
mkdir -p "${WS}/maps" "${WS}/bags"
