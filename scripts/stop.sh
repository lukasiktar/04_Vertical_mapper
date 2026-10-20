#!/usr/bin/env bash
# Stop all d455_mapping launches in the container (Ctrl-C equivalent, then force).
# RTAB-Map writes its database on the graceful (SIGINT) shutdown, so give it time.
#   scripts/stop.sh [grace_seconds=60]
set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_lib.sh"
if ! in_container; then exec "${WS_HOST}/scripts/dev.sh" -- scripts/stop.sh "$@"; fi
grace="${1:-60}"
# Match the real executables only (not shells whose command line mentions them).
pat='^/usr/bin/python3 /opt/ros/humble/bin/ros2 (launch|bag) |/lib/rclcpp_components/component_container|/lib/rviz2/rviz2|/lib/imu_filter_madgwick/|/lib/realsense2_camera/realsense2_camera_node|/lib/tf2_ros/static_transform_publisher|/lib/d455_mapping/|/lib/pcl_ros/pcd_to_pointcloud'
pkill -INT -f "${pat}" || { echo "nothing running"; exit 0; }
for ((i = 0; i < grace; i++)); do pgrep -f "${pat}" >/dev/null || { echo "stopped"; exit 0; }; sleep 1; done
echo "still running after ${grace}s; killing"; pkill -9 -f "${pat}"
