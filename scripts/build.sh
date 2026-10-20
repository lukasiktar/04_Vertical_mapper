#!/usr/bin/env bash
# Build the ROS workspace (container; re-execs into it from the host).
# USE:
#   scripts/build.sh [extra colcon args]
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_lib.sh"
if ! in_container; then exec "${WS_HOST}/scripts/dev.sh" -- scripts/build.sh "$@"; fi
cd /workspaces/isaac_ros-dev
ros_source /opt/ros/humble/setup.bash
colcon build --symlink-install --base-paths src \
  --packages-up-to d455_mapping realsense_splitter \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF "$@"
echo "built. source install/setup.bash (new shells do it automatically)"
