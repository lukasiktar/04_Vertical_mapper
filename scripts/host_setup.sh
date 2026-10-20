#!/usr/bin/env bash
# One-time HOST setup (laptop or Jetson). Run on the host, NOT in the container:
#   ./scripts/host_setup.sh
#
# What it does::
#   1. Removes a stale root-owned ~/.Xauthority *directory* (created by an earlier
#      docker bind mount of a missing file); X11 itself is handled by scripts/dev.sh.
#   2. Installs the librealsense udev rules (pinned version, see VERSIONS.md) as
#      root:root 0644 and reloads udev, so the non-root container user can open the camera.
#   3. Raises kernel UDP receive buffers / IP fragment limits for DDS
#      (https://docs.ros.org/en/rolling/How-To-Guides/DDS-tuning.html), persisted in
#      /etc/sysctl.d/60-ros2-dds.conf.
#   4. Keeps the kernel uvcvideo driver off RealSense cameras (config/udev/), because it fights
#      with librealsense's libusb backend in the container (USB -71 errors, failed starts).
#      The udev rule can only unbind AFTER uvcvideo has probed the camera, so with
#        ./scripts/host_setup.sh --no-uvcvideo
#      uvcvideo is also blacklisted and unloaded (recommended when no other USB/UVC webcam is
#      needed; the XPS 14 built-in camera uses IPU6, not uvcvideo). Undo:
#        sudo rm /etc/modprobe.d/blacklist-uvcvideo-d455.conf && sudo modprobe uvcvideo
# It does NOT install drivers, Docker or the NVIDIA Container Toolkit (see README.md).
set -euo pipefail

NO_UVCVIDEO=0
[[ "${1:-}" == "--no-uvcvideo" ]] && NO_UVCVIDEO=1

LIBREALSENSE_VERSION="v2.55.1"   # keep in sync with VERSIONS.md
RULES_URL="https://raw.githubusercontent.com/IntelRealSense/librealsense/${LIBREALSENSE_VERSION}/config/99-realsense-libusb.rules"
RULES_DST="/etc/udev/rules.d/99-realsense-libusb.rules"
SYSCTL_DST="/etc/sysctl.d/60-ros2-dds.conf"

run() { echo "+ $*"; "$@"; }

if [[ $(id -u) -eq 0 ]]; then
  echo "Run as your normal user; the script calls sudo itself." >&2
  exit 1
fi

echo "== 1. ~/.Xauthority"
if [[ -d "${HOME}/.Xauthority" ]]; then
  if [[ -z "$(ls -A "${HOME}/.Xauthority")" ]]; then
    run sudo rmdir "${HOME}/.Xauthority"
  else
    echo "~/.Xauthority is a NON-empty directory; inspect it manually." >&2
  fi
else
  echo "ok (not a directory)"
fi

echo "== 2. RealSense udev rules (${LIBREALSENSE_VERSION})"
tmp_rules="$(mktemp)"
trap 'rm -f "${tmp_rules}"' EXIT
curl -fsSL "${RULES_URL}" -o "${tmp_rules}"
if [[ -f "${RULES_DST}" ]] && cmp -s <(grep -v '^#' "${RULES_DST}" | grep .) <(grep -v '^#' "${tmp_rules}" | grep .); then
  echo "rules content already matches ${LIBREALSENSE_VERSION}"
elif [[ -f "${RULES_DST}" ]] && \
     diff <(grep -v '^#' "${RULES_DST}" | grep .) <(grep -v '^#' "${tmp_rules}" | grep .) | grep -q '^>'; then
  run sudo install -m 0644 -o root -g root "${tmp_rules}" "${RULES_DST}"
elif [[ -f "${RULES_DST}" ]]; then
  echo "existing rules are a superset of ${LIBREALSENSE_VERSION}; keeping content"
else
  run sudo install -m 0644 -o root -g root "${tmp_rules}" "${RULES_DST}"
fi
# Rules with RUN+= execute as root, so the file must not be user-writable.
run sudo chown root:root "${RULES_DST}"
run sudo chmod 0644 "${RULES_DST}"
run sudo udevadm control --reload-rules
run sudo udevadm trigger

echo "== 3. DDS network buffers"
printf '%s\n' \
  'net.core.rmem_max=2147483647' \
  'net.core.rmem_default=8388608' \
  'net.ipv4.ipfrag_time=3' \
  'net.ipv4.ipfrag_high_thresh=134217728' > /tmp/60-ros2-dds.conf.$$
if ! cmp -s /tmp/60-ros2-dds.conf.$$ "${SYSCTL_DST}" 2>/dev/null; then
  run sudo install -m 0644 -o root -g root /tmp/60-ros2-dds.conf.$$ "${SYSCTL_DST}"
fi
rm -f /tmp/60-ros2-dds.conf.$$
run sudo sysctl -p "${SYSCTL_DST}"

echo "== 4. uvcvideo off RealSense cameras"
NOUVC_SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/config/udev/99-realsense-no-uvcvideo.rules"
NOUVC_DST="/etc/udev/rules.d/99-realsense-no-uvcvideo.rules"
if ! cmp -s "${NOUVC_SRC}" "${NOUVC_DST}" 2>/dev/null; then
  run sudo install -m 0644 -o root -g root "${NOUVC_SRC}" "${NOUVC_DST}"
  run sudo udevadm control --reload-rules
fi
# Unbind now from an already plugged-in camera (the rule fires on future binds).
for itf in /sys/bus/usb/drivers/uvcvideo/*:*; do
  [[ -e "${itf}" ]] || continue
  dev="$(dirname "$(readlink -f "${itf}")")"
  if [[ "$(cat "${dev}/idVendor" 2>/dev/null)" == "8086" ]] && \
     grep -qE '^(0b07|0b3a|0b5c)$' "${dev}/idProduct" 2>/dev/null; then
    echo -n "$(basename "${itf}")" | run sudo tee /sys/bus/usb/drivers/uvcvideo/unbind >/dev/null
  fi
done

if [[ ${NO_UVCVIDEO} -eq 1 ]]; then
  echo "== 4b. blacklist + unload uvcvideo"
  BL=/etc/modprobe.d/blacklist-uvcvideo-d455.conf
  if [[ ! -f "${BL}" ]]; then
    echo '# d455_mapping: librealsense (libusb backend, in Docker) owns RealSense cameras' | run sudo tee "${BL}" >/dev/null
    echo 'blacklist uvcvideo' | run sudo tee -a "${BL}" >/dev/null
  fi
  if lsmod | grep -q '^uvcvideo '; then
    run sudo modprobe -r uvcvideo
  fi
fi

echo "Host setup done. No reboot needed."
