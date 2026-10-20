#!/usr/bin/env bash
# View an exported map offline (no camera): PCD -> /prior_map (frame map) + mesh marker.
# USE:
#   scripts/show_map.sh <name> [--no-mesh] [--no-rviz]
#
# --no-mesh: do not show the mesh marker (RViz MESH_RESOURCE) even if it exists
# --no-rviz: do not launch RViz (just the map frame and prior_map topic)
#
# Run scripts/export_map.sh <name> first.
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_lib.sh"
ensure_container "$0" "$@"

name="${1:?usage: show_map.sh <name> [--no-mesh] [--no-rviz]}"; shift
mesh_arg=1; rviz=true
for a in "$@"; do
  case "$a" in --no-mesh) mesh_arg=0 ;; --no-rviz) rviz=false ;; esac
done
pcd="/maps/${name}/${name}_cloud.pcd"
# fall back to the lighter cloud made by localize.sh / map_quality.sh (export_map.sh <name> --prior)
[[ -f "${pcd}" ]] || pcd="/maps/${name}/${name}_prior_cloud.pcd"
[[ -f "${pcd}" ]] || { echo "no exported cloud for ${name}; run scripts/export_map.sh ${name} [--prior]" >&2; exit 1; }
mesh=""
if [[ ${mesh_arg} -eq 1 ]]; then
  for ext in dae stl; do
    [[ -f "/maps/${name}/${name}_mesh.${ext}" ]] && { mesh="/maps/${name}/${name}_mesh.${ext}"; break; }
  done
fi
mesh_arg=(); [[ -n "${mesh}" ]] && mesh_arg=(mesh:="${mesh}")   # an empty "mesh:=" is rejected
exec ros2 launch d455_mapping show_map.launch.py pcd:="${pcd}" "${mesh_arg[@]}" rviz:="${rviz}"
