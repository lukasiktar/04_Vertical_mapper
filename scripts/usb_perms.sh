#!/usr/bin/env bash
# Let the container's non-root user open the RealSense USB device.
# The host has no RealSense udev rules (scripts/host_setup.sh not run), so /dev/bus/usb nodes are
# root-only. This opens them inside the container only (the container has its own /dev copy).
# Re-run after a container (re)start or after unplugging/replugging the camera. Host, no sudo.
#   scripts/usb_perms.sh
set -euo pipefail
CONTAINER="isaac_ros_dev-$(uname -m)-d455-container"
docker ps -q --filter "name=^${CONTAINER}$" | grep -q . || { echo "${CONTAINER} is not running (scripts/dev.sh --detach)" >&2; exit 1; }
docker exec -u root "${CONTAINER}" bash -c 'chmod a+rw /dev/bus/usb/*/*'
echo "USB device nodes opened in ${CONTAINER}"
