#!/usr/bin/env bash
# Export a map database to files usable outside RTAB-Map.
# USE:
#   scripts/export_map.sh <name> [--voxel 0.01] [--max-range R] [--with-images] [--prior]
#       --max-range defaults to depth_max_m of the platform config (src/d455_mapping/config/)
#       --with-images: <name>_rgb/, <name>_depth/, <name>_calib/ keyframe images (by node id) +
#         calibration, for projecting detections (e.g. intersections) into the map later.
#       --prior: only a lighter cloud for showing the saved map while localizing
#         (<name>_prior_cloud.pcd, default voxel 0.02, plus a .info file with the settings). This is what
#         scripts/localize.sh uses; it never touches the full export above.

# Output in maps/<name>/:
#   <name>_cloud.ply / <name>_cloud.pcd         colored point cloud (map frame)
#   <name>_mesh.ply                             colored mesh (rtabmap-export --mesh)
#   <name>_mesh.dae / <name>_mesh.stl           mesh for an RViz Marker MESH_RESOURCE
#   <name>_poses.txt, <name>_camera_poses.txt   keyframe poses (base_link / optical frame)


set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_lib.sh"
ensure_container "$0" "$@"

name="${1:?usage: export_map.sh <name> [--voxel V] [--max-range R] [--with-images] [--prior]}"; shift
voxel=""; max_range=""; images=0; prior=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --voxel) voxel="$2"; shift 2 ;;
    --max-range) max_range="$2"; shift 2 ;;
    --with-images) images=1; shift ;;
    --prior) prior=1; shift ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
  esac
done
if [[ -z "${max_range}" ]]; then
  cfg="/workspaces/isaac_ros-dev/src/d455_mapping/config/$(default_platform).yaml"
  max_range="$(awk '/^depth_max_m:/{print $2; exit}' "${cfg}" 2>/dev/null || true)"
  max_range="${max_range:-0}"
fi
base="${name}"
if [[ ${prior} -eq 1 ]]; then base="${name}_prior"; voxel="${voxel:-0.02}"; else voxel="${voxel:-0.01}"; fi
db="/maps/${name}.db"
out="/maps/${name}"
[[ -f "${db}" ]] || { echo "${db} not found" >&2; exit 1; }
mkdir -p "${out}"
echo "export: max range ${max_range} m (0 = unlimited), voxel ${voxel} m"

# rtabmap >= 0.23 requires the export targets to be named explicitly (--cloud, --mesh, ...).
if [[ ${prior} -eq 1 ]]; then
  echo "== point cloud (prior map)"
  rtabmap-export --output "${base}" --output_dir "${out}" --voxel "${voxel}" \
    --max_range "${max_range}" --cloud "${db}"
else
  echo "== point cloud + mesh"
  rtabmap-export --output "${base}" --output_dir "${out}" --voxel "${voxel}" \
    --max_range "${max_range}" --cloud --mesh "${db}"
  echo "== poses (format 11: stamp x y z qx qy qz qw id, map frame)"
  # <name>_poses.txt = base_link, <name>_camera_poses.txt = camera optical frame
  rtabmap-export --output "${base}" --output_dir "${out}" --poses --poses_camera "${db}"
  if [[ ${images} -eq 1 ]]; then
    echo "== keyframe images (named by node id) + calibration"
    rtabmap-export --output "${base}" --output_dir "${out}" --images_id "${db}"
  fi
fi

cloud_ply="${out}/${base}_cloud.ply"
mesh_ply="${out}/${base}_mesh.ply"
[[ -f "${cloud_ply}" ]] || { echo "expected ${cloud_ply} missing; files:"; ls -la "${out}"; exit 1; }
echo "== PLY -> PCD"
pcl_ply2pcd -format 1 "${cloud_ply}" "${out}/${base}_cloud.pcd" >/dev/null
if [[ ${prior} -eq 1 ]]; then
  rm -f "${cloud_ply}"
  echo "max_range=$(printf '%g' "${max_range}") voxel=$(printf '%g' "${voxel}")" > "${out}/${base}_cloud.info"
  ls -la "${out}/${base}_cloud.pcd"
  exit 0
fi
if [[ -f "${mesh_ply}" ]]; then
  echo "== mesh -> DAE + STL (RViz MESH_RESOURCE)"
  assimp export "${mesh_ply}" "${out}/${base}_mesh.dae" >/dev/null
  assimp export "${mesh_ply}" "${out}/${base}_mesh.stl" >/dev/null
fi
ls -la "${out}"
echo "View: scripts/show_map.sh ${name}"
