#!/usr/bin/env bash
# Read-only pre-flight checks on the Jetson Orin Nano (no sudo, changes nothing).
#   ./scripts/jetson_check.sh
set -uo pipefail
ok()   { printf '  [ OK ] %s\n' "$*"; }
warn() { printf '  [WARN] %s\n' "$*"; }
bad()  { printf '  [FAIL] %s\n' "$*"; }

echo "== Platform"
[[ "$(uname -m)" == "aarch64" ]] && ok "aarch64" || bad "not aarch64 ($(uname -m))"
if [[ -f /etc/nv_tegra_release ]]; then
  rel="$(head -1 /etc/nv_tegra_release)"; echo "  ${rel}"
  # Isaac ROS 3.2: JetPack 6.1 (L4T R36.4.0) or 6.2 (R36.4.3+)
  if [[ "${rel}" =~ R36\ \(release\),\ REVISION:\ 4\.([0-9]+) ]]; then
    ok "L4T R36.4.${BASH_REMATCH[1]} (JetPack 6.1/6.2)"
  else
    bad "need L4T R36.4.x (JetPack 6.1/6.2) for Isaac ROS 3.2"
  fi
else
  bad "/etc/nv_tegra_release missing (not a Jetson?)"
fi
dpkg-query -W -f='  nvidia-jetpack ${Version}\n' nvidia-jetpack 2>/dev/null || warn "nvidia-jetpack meta package not installed"
model="$(tr -d '\0' < /proc/device-tree/model 2>/dev/null)"; echo "  model: ${model}"
mem_kb="$(awk '/MemTotal/{print $2}' /proc/meminfo)"; mem_gb=$(( (mem_kb + 524288) / 1048576 ))
if (( mem_gb >= 7 )); then ok "RAM ${mem_gb} GB (8 GB module)"; else bad "RAM ${mem_gb} GB: 4 GB Orin Nano is not supported"; fi

echo "== Power"
command -v nvpmodel >/dev/null && nvpmodel -q 2>/dev/null | sed 's/^/  /' || warn "nvpmodel not found"

echo "== Storage"
if lsblk -d -o NAME,SIZE,MODEL | grep -q '^nvme'; then
  ok "NVMe present: $(lsblk -d -o NAME,SIZE | awk '/^nvme/{print $1, $2}' | xargs)"
else
  bad "no NVMe SSD (required for Isaac ROS dev images + bags)"
fi
root_src="$(findmnt -n -o SOURCE /)"; echo "  / is on ${root_src}"
droot="$(docker info -f '{{.DockerRootDir}}' 2>/dev/null)"
if [[ -n "${droot}" ]]; then
  dsrc="$(findmnt -n -o SOURCE --target "${droot}")"
  [[ "${dsrc}" == /dev/nvme* ]] && ok "Docker root ${droot} on ${dsrc}" || warn "Docker root ${droot} on ${dsrc} (not NVMe)"
fi
df -h / "${droot:-/var/lib/docker}" 2>/dev/null | sed 's/^/  /'
swapon --show 2>/dev/null | sed 's/^/  /'

echo "== Docker"
docker --version 2>/dev/null | sed 's/^/  /' || bad "docker missing"
docker buildx version >/dev/null 2>&1 && ok "buildx" || bad "docker-buildx-plugin missing"
id -nG | grep -qw docker && ok "user in docker group" || bad "user not in docker group"
docker info 2>/dev/null | grep -q 'Runtimes:.*nvidia' && ok "nvidia runtime configured" || bad "nvidia runtime missing"
command -v git-lfs >/dev/null && ok "git-lfs" || bad "git-lfs missing"

echo "== RealSense"
lsusb | grep -iE '8086:0b5c|8086:0b3a' | sed 's/^/  /' || warn "no D455 (0b5c) / D435i (0b3a) on USB"
[[ -f /etc/udev/rules.d/99-realsense-libusb.rules ]] && ok "udev rules installed" || bad "udev rules missing (scripts/host_setup.sh)"

echo "== Tools"
command -v jtop >/dev/null && ok "jtop" || warn "jtop missing (scripts/jetson_setup.sh jtop)"
sysctl -n net.core.rmem_max | awk '{ if ($1 >= 10485760) print "  [ OK ] rmem_max " $1; else print "  [WARN] rmem_max " $1 " (scripts/host_setup.sh)" }'
