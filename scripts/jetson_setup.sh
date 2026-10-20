#!/usr/bin/env bash
# Jetson Orin Nano host setup. Each step prints its sudo commands and asks before running.
#   ./scripts/jetson_setup.sh power            MAXN / MAXN SUPER power mode + jetson_clocks
#   ./scripts/jetson_setup.sh swap [SIZE=8G]   swap file on the NVMe (for builds)
#   ./scripts/jetson_setup.sh docker-root DIR  move Docker data root to DIR (on the NVMe)
#   ./scripts/jetson_setup.sh docker-tools     docker group, buildx plugin, git-lfs
#   ./scripts/jetson_setup.sh jtop             install jetson-stats (jtop)
# Then run ./scripts/host_setup.sh (udev rules, DDS buffers) as on the laptop.
set -euo pipefail

confirm() {
  echo; echo "About to run:"; printf '    %s\n' "$@"
  read -r -p "Proceed? [y/N] " a; [[ "${a}" == [yY] ]]
}
run_all() { for c in "$@"; do echo "+ ${c}"; bash -c "${c}"; done; }

step="${1:-}"; shift || true
case "${step}" in
  power)
    echo "Available modes:"; grep -E '^< POWER_MODEL' /etc/nvpmodel.conf | sed 's/^/  /' || true
    # On JetPack 6.2 the Orin Nano 'MAXN SUPER' mode is the highest; on 6.1 it is mode 0 (15W).
    id="$(grep -E '^< POWER_MODEL' /etc/nvpmodel.conf | grep -i 'MAXN' | head -1 | sed -E 's/.*ID=([0-9]+).*/\1/')"
    id="${id:-0}"
    cmds=("sudo nvpmodel -m ${id}" "sudo jetson_clocks")
    confirm "${cmds[@]}" && run_all "${cmds[@]}"
    echo "Note: nvpmodel may ask to reboot when switching modes. jetson_clocks is not persistent."
    ;;
  swap)
    size="${1:-8G}"
    file="${SWAP_FILE:-/mnt/nova_ssd/swapfile}"
    cmds=("sudo fallocate -l ${size} ${file}" "sudo chmod 600 ${file}" "sudo mkswap ${file}"
          "sudo swapon ${file}"
          "grep -q '^${file} ' /etc/fstab || echo '${file} none swap sw 0 0' | sudo tee -a /etc/fstab")
    confirm "${cmds[@]}" && run_all "${cmds[@]}"
    ;;
  docker-root)
    dir="${1:?usage: jetson_setup.sh docker-root /mnt/nova_ssd/docker}"
    cat <<EOT
This follows the Isaac ROS 3.2 'Jetson Setup' guide: stop Docker, rsync /var/lib/docker to
${dir}, set "data-root" (and "default-runtime": "nvidia") in /etc/docker/daemon.json,
keep the old dir as /var/lib/docker.old.
EOT
    cmds=("sudo systemctl stop docker docker.socket"
          "sudo mkdir -p ${dir}"
          "sudo rsync -axPS /var/lib/docker/ ${dir}/"
          "sudo python3 -c \"import json,os;p='/etc/docker/daemon.json';d=json.load(open(p)) if os.path.exists(p) else {};d.setdefault('runtimes',{})['nvidia']={'path':'nvidia-container-runtime','runtimeArgs':[]};d['default-runtime']='nvidia';d['data-root']='${dir}';json.dump(d,open(p,'w'),indent=4)\""
          "sudo mv /var/lib/docker /var/lib/docker.old"
          "sudo systemctl daemon-reload"
          "sudo systemctl restart docker"
          "docker info -f '{{.DockerRootDir}}'")
    confirm "${cmds[@]}" && run_all "${cmds[@]}"
    ;;
  docker-tools)
    cmds=("sudo usermod -aG docker ${USER}"
          "sudo apt-get update"
          "sudo apt-get install -y docker-buildx-plugin git-lfs"
          "git lfs install --skip-repo")
    echo "docker-buildx-plugin comes from Docker's apt repo; if apt cannot find it, add the repo"
    echo "first exactly as in docs/JETSON.md (Isaac ROS 3.2 Compute Setup)."
    confirm "${cmds[@]}" && run_all "${cmds[@]}"
    echo "Log out and back in for the docker group to apply."
    ;;
  jtop)
    cmds=("sudo apt-get install -y python3-pip" "sudo pip3 install -U jetson-stats"
          "sudo systemctl restart jtop.service")
    confirm "${cmds[@]}" && run_all "${cmds[@]}"
    echo "Log out and back in (jtop group), then run: jtop"
    ;;
  *)
    sed -n '2,9p' "$0"; exit 1 ;;
esac
