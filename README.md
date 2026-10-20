# D455 Vertical Mapper

3D mapping and relocalization with an Intel RealSense **D455** (also D435i). It runs on
ROS 2 **Humble** with **Isaac ROS 3.2**, entirely in Docker. 
Developed for **Jetson Orin Nano 8 GB**.

| Stage | Package | Role |
|---|---|---|
| Camera | `realsense2_camera` 4.51.1-isaac | IR stereo, depth, aligned depth, color, IMU (gyro+accel united), emitter on/off interleave |
| Splitter | `realsense_splitter` (isaac_ros_nvblox, + `invert_emitter_mode` patch) | emitter-OFF IR → cuVSLAM, emitter-ON depth → RTAB-Map/nvblox |
| Odometry | Isaac ROS Visual SLAM (cuVSLAM) | GPU stereo-inertial odometry, `odom → base_link` |
| Mapping | RTAB-Map | graph SLAM, loop closure, database, localization mode, `map → odom` |
| Dense mesh | Isaac ROS nvblox | TSDF + mesh on the GPU (on by default on the laptop, off on the Jetson) |
| Viz | RViz2 | camera, TF, trajectories, RTAB-Map MapCloud, nvblox mesh |

Versions are pinned in [VERSIONS.md](VERSIONS.md).

Designed for the later **intersection detection** step:
- The RTAB-Map database keeps RGB, aligned depth and calibration for every keyframe, together
  with its optimized pose (`Mem/BinDataKept=true`).
- Raw bags keep the full sensor stream for offline re-runs.
- The frames are consistent (`map → odom → base_link → camera_*_optical_frame`), so a pixel
  `(u,v)` in keyframe *k* can be projected into the map with depth plus the intrinsics, as
  `T_map_optical(k) · K⁻¹ · [u,v,1]·d`. `scripts/export_map.sh --with-images` writes the
  keyframe images, calibration and poses.

---

## 1. Prerequisites

Isaac ROS 3.2 x86_64 requirements:

| Requirement | Isaac ROS 3.2 | Our device |
|---|---|---|
| OS | Ubuntu 22.04+ (the containers are 22.04) | Ubuntu 24.04 |
| NVIDIA driver | ≥ 560 | 580 |
| GPU | Ampere+ with ≥ 8 GB VRAM | RTX 4050 Laptop, **6 GB**. Works for this stack |
| Docker Engine | ≥ 27.2 + buildx | 29.1 |
| NVIDIA Container Toolkit | installed, `nvidia` runtime configured | 1.18.1 |
| git-lfs | required by `run_dev.sh` | yes |
| Disk | ≥ 30 GB | the dev image is about 39 GB, plus build cache |

One-time host setup. It is idempotent and prints every sudo command:

```bash
./scripts/host_setup.sh --no-uvcvideo
```

It does 4 things:
1. Removes a stale root-owned `~/.Xauthority` **directory**, if present.
2. Installs the librealsense v2.55.1 udev rules as `root:root 0644`.
3. Raises the DDS socket buffers (`/etc/sysctl.d/60-ros2-dds.conf`).
4. Keeps the kernel `uvcvideo` driver off the RealSense (udev rule, plus with `--no-uvcvideo` it
   blacklists and unloads the module). **Do this on the laptop.** The container drives the
   camera through librealsense's libusb backend; every time it releases an interface,
   `uvcvideo` re-binds and probes the camera at the same time. Result without it: USB `-71`
   errors, `Error starting device: get_data_usb failed`, and on this laptop even a crash of the
   Thunderbolt USB controller (`xHCI host controller not responding, assume dead`, no USB-C
   port works until reboot). The laptop's built-in webcam uses IPU6, not `uvcvideo`. To undo:
   `sudo rm /etc/modprobe.d/blacklist-uvcvideo-d455.conf && sudo modprobe uvcvideo`.

Plug the camera **directly** into a laptop USB-C port (not through a hub) before starting the
container.

GPU in Docker check: `docker run --rm --gpus all ubuntu:22.04 nvidia-smi`.

## 2. Build

```bash
./scripts/setup_workspace.sh     # fetch pinned isaac_ros_common + realsense_splitter, write config
./scripts/dev.sh --build-image   # build <arch>.ros2_humble.realsense.d455_mapping (first time: long)
./scripts/build.sh               # colcon build (re-execs inside the container)
./scripts/dev.sh                 # interactive shell in the container (again = 2nd shell)
```

How the image is built: Isaac ROS `run_dev.sh` / `build_image_layers.sh` resolves the image
key `ros2_humble.realsense.d455_mapping` into a chain of Dockerfiles:
1. `isaac_ros_common/docker/Dockerfile.{x86_64|aarch64}`
2. `isaac_ros_common/docker/Dockerfile.ros2_humble`
3. `isaac_ros_common/docker/Dockerfile.realsense` (librealsense 2.55.1 + realsense-ros 4.51.1-isaac)
4. **[docker/Dockerfile.d455_mapping](docker/Dockerfile.d455_mapping)**, our layer: cuVSLAM,
   nvblox + RViz plugin, rtabmap_ros + CLI tools, pcl_ros, imu_filter_madgwick, MCAP, assimp,
   pcl-tools.

`scripts/setup_workspace.sh` writes the key and search path into
`src/isaac_ros_common/scripts/.isaac_ros_common-config`. A `~/.isaac_ros_common-config` in
your home directory would override it, and the script warns if one exists.

Container layout:
- The repo is mounted at `/workspaces/isaac_ros-dev` (= `$ISAAC_ROS_WS`).
- `./maps` is mounted at `/maps` and `./bags` at `/bags`.
- The container uses host networking, `--privileged` (USB access) and the GPU through the
  `nvidia` runtime.
- X11 uses a per-session Xauthority cookie file
  (`$XDG_RUNTIME_DIR/d455_mapping.xauth`, wildcard host entry). There is **no `xhost +`**.

Checks inside the container (Phase 2):
```bash
nvidia-smi                       # GPU visible
ros2 doctor                      # (network warnings about multicast on VPN/Wi-Fi are harmless)
rs-enumerate-devices -s          # D455 listed, "Usb Type Descriptor" 3.2
rviz2                            # opens on the laptop screen
```

## 3. Scripts

All of them work from the host, where they re-exec inside the container, and from inside the
container. `platform` defaults to `laptop` on x86_64 and `orin_nano` on aarch64.

| Script | What it does |
|---|---|
| `scripts/host_setup.sh [--no-uvcvideo]` | host: udev rules, DDS buffers, `~/.Xauthority` fix, `uvcvideo` off the RealSense (sudo, prompts) |
| `scripts/setup_workspace.sh` | host: fetch pinned third-party sources, write the Isaac ROS config |
| `scripts/dev.sh` | host: start or enter the container; `-- <cmd>` runs a command in it; `--build-image`, `--detach`, `--stop` |
| `scripts/build.sh` | colcon build (`--symlink-install`, Release) |
| `scripts/stop.sh [grace_s]` | stop all launches (SIGINT first so RTAB-Map saves its database), then force. Use it instead of `pkill` |
| `scripts/map.sh [name] [platform] [--force] [args]` | new map → `maps/<name>.db` (+ RViz on the laptop). Refuses to overwrite without `--force`. Ctrl-C saves |
| `scripts/localize.sh <name> [platform] [--no-prior-map] [args]` | localization against `maps/<name>.db`; RViz also shows the whole saved cloud (`/prior_map`, built once and cached) |
| `scripts/record.sh [name] [platform] [args]` | raw MCAP bag → `bags/<name>` |
| `scripts/replay.sh <bag> [map_name] [platform] [--rate R]` | mapping from a bag with `use_sim_time` → `maps/<map_name>.db` |
| `scripts/export_map.sh <name> [--voxel V] [--max-range R] [--with-images]` | `maps/<name>/`: cloud PLY+PCD, mesh PLY → DAE+STL, base_link and camera-optical poses (+ keyframe images named by node id, and calibration) |
| `scripts/show_map.sh <name> [--no-mesh] [--no-rviz]` | offline viewer: PCD → `/prior_map` (PointCloud2, frame `map`), mesh → `/prior_map_mesh` (Marker MESH_RESOURCE), RViz |
| `scripts/replay_px4.sh <bag> <map> [args]` | map from a bag with RGB-D camera + PX4 odometry (sqlite3, no IR/IMU needed); camera mounting via `cam_xyz` / `cam_rpy`, see USAGE.md §11 |
| `scripts/calibrate_rig.sh <map> <bag>` | find the camera mounting on the drone and the PX4 time delay from the data |
| `scripts/refine_map.sh <in> <out> [--cam-tf ...]` | re-optimize a saved map offline (neighbor refinement, more loop closures) |
| `scripts/map_quality.sh [--no-cloud] <map>...` | cleanliness numbers of maps (keyframe depth mismatch, local spread); lower is cleaner |
| `scripts/map_info.sh <map> \| --all` | keyframes, RGB-D coverage and loop closures of saved maps |
| `scripts/measure.sh [seconds] [label]` | peak CPU / RAM / GPU memory sampler → `bags/measure_*.csv` |
| `scripts/jetson_check.sh`, `scripts/jetson_setup.sh` | Jetson only, see [docs/JETSON.md](docs/JETSON.md) |

Extra launch arguments pass through, for example `scripts/map.sh lab laptop nvblox:=false rviz:=false`.

Helper tools (`ros2 run d455_mapping <tool>`, inside the container):

| Tool | What it does |
|---|---|
| `emitter_ensure` | started by every launch with a camera: checks that the emitter on/off labels alternate and toggles `depth_module.emitter_on_off` if the camera kept a stuck state from a previous session |
| `emitter_check [--save DIR] [--roi y0,y1,x0,x1]` | end-to-end test of the splitter: cuVSLAM must get the frames with fewer projector dots (PASS / FAIL / INCONCLUSIVE) |
| `odom_drift_check [--seconds N]` | records cuVSLAM odometry while you move: path length, end-point error, tracking losses |
| `prior_map_publisher` | publishes a saved PCD once, latched, as `/prior_map` (used by `localize.sh` and `show_map.sh`) |
| `mesh_marker_publisher` | used by `show_map.launch.py` |

## 4. Launch files (`ros2 launch d455_mapping <file>`)

Every launch file takes `platform:=laptop|orin_nano`, `use_sim_time:=false|true`,
`camera:=true|false` (false = bag replay) and `depth_fps:=<n>` (override).

| Launch | Contents |
|---|---|
| `camera.launch.py` | D455 + splitters |
| `vslam.launch.py` | camera + cuVSLAM (`rviz:=true` optional) |
| `mapping.launch.py` | camera + cuVSLAM + RTAB-Map (new DB) [+ nvblox] [+ RViz]; `database_path:=/maps/map.db`, `nvblox:=`, `rviz:=` |
| `localization.launch.py` | same, RTAB-Map in localization mode on an existing DB |
| `nvblox.launch.py` | camera + cuVSLAM + nvblox |
| `record.launch.py` | raw topics → MCAP (`name:=`, `out_dir:=/bags`) |
| `show_map.launch.py` | `pcd:=`, `mesh:=`, `rviz:=` |

RViz configs (`rviz/`): `mapping.rviz`, `localization.rviz`, `show_map.rviz` and the
lightweight `*_remote.rviz` for the Jetson. They are generated by
`src/d455_mapping/tools/gen_rviz.py`, so edit that script rather than the `.rviz` files.

## 5. Platform configs

Platform-specific values live in
[src/d455_mapping/config/laptop.yaml](src/d455_mapping/config/laptop.yaml) and
[orin_nano.yaml](src/d455_mapping/config/orin_nano.yaml). Both files have the same keys:

- `depth_max_m`: depth limit for geometry (RTAB-Map grid/cloud, nvblox, RViz cloud, export default);
  the camera, bags and database keep full depth (USAGE.md §10).
- `camera`: profiles (depth/IR 848x480 @ 30, color 640x480 @ 30/15), IMU rates,
  `emitter_on_off`, `invert_emitter_mode` (see §8), `initial_reset` (keep `false`), serial number.
- `base_to_camera`: mounting transform `base_link → camera_link`.
- `vslam`: IMU fusion, jitter threshold (0 = auto), debug clouds.
- `rtabmap`: detection rate, feature limits, STM size, update thresholds, grid/cloud
  settings, IMU use, `odom_source` (`tf` = cuVSLAM pose via `odom → base_link` with fixed
  variances, the default; `topic` = `/visual_slam/tracking/odometry` with its covariance, which
  can be zero and then aborts RTAB-Map), `features_max_depth` (loop-closure feature range, 0 = off).
- `nvblox`: enable, voxel size, range, clearing radius, rates.
- `rviz` (`show_prior_map`: localization also shows the saved cloud), `record`.

Nothing platform-specific is hard-coded in the launch files.

## 6. TF tree

```
map ──(RTAB-Map, map→odom; identity until the first loop closure / relocalization)──▶ odom
odom ──(cuVSLAM, 15–30 Hz, smooth VIO)──▶ base_link
base_link ──(static, config base_to_camera)──▶ camera_link
camera_link ──(realsense2_camera static extrinsics)──▶ camera_{depth,color,infra1,infra2,gyro,accel,imu}_optical_frame …
```

- cuVSLAM: `publish_odom_to_base_tf=true`, `publish_map_to_odom_tf=false`,
  `enable_localization_n_mapping=false`. RTAB-Map is the only `map → odom` publisher.
- cuVSLAM uses `camera_infra1/2_optical_frame` and IMU frame `camera_gyro_optical_frame`.
- nvblox integrates in `odom` (smooth). RViz shows everything in `map`.
- `vslam.launch.py` and `nvblox.launch.py` have no RTAB-Map, so they publish an identity
  `map → odom` themselves (so RViz's fixed frame exists). Never mix them with the mapping
  launch.
- RTAB-Map takes the cuVSLAM pose from the `odom → base_link` TF at each image stamp
  (`odom_source: tf`).

## 7. Topic graph

```
realsense2_camera (/camera)
 ├─ infra1/image_rect_raw + metadata ─┐                         ┌─ output/infra_1 ─┐
 ├─ infra2/image_rect_raw + metadata ─┼─▶ realsense_splitter_node┼─ output/infra_2 ─┼─▶ cuVSLAM ─▶ /visual_slam/tracking/odometry
 ├─ depth/image_rect_raw + metadata ──┘   (emitter OFF IR,       └─ output/depth ───┼─▶ nvblox ─▶ /nvblox_node/mesh
 │                                         emitter ON depth)                         │       (+ color/image_raw, TF odom→cam)
 ├─ aligned_depth_to_color/image_raw ─▶ aligned_depth_splitter ─ output/depth ─┐     │
 ├─ color/image_raw + camera_info ────────────────────────────────────────────┼─▶ rtabmap/rgbd_sync ─▶ rtabmap/rtabmap
 ├─ imu (gyro+accel, linear interpolation, 200 Hz) ─┬─▶ cuVSLAM               │        ▲ odometry: TF odom→base_link
 │                                                  └─▶ imu_filter_madgwick ─▶ /imu/data ┘ (gravity)
 └─ /tf_static (extrinsics)
rtabmap outputs: /rtabmap/mapData, mapGraph, mapPath, cloud_map, info, localization_pose, TF map→odom
```

With `emitter_on_off: false` the splitters are skipped and the raw topics are used directly.
The camera, both splitters, cuVSLAM, nvblox and RTAB-Map (`rgbd_sync` + `rtabmap`) are components
of one container (`/d455_container`), so large images move in-process.

## 8. Camera firmware and the emitter check

Isaac ROS 3.2 asks for D455 firmware **5.13.0.50**. On our D455 (newer production unit),
that firmware broke the color sensor and crashed the laptop's USB-C controller. 5.16.0.1
streamed nothing with Isaac's librealsense 2.55.1. We therefore run the **factory firmware
5.15.1.55**. With it, the camera reports the emitter on/off flag **inverted**, so
`camera.invert_emitter_mode: true` in both platform configs flips it in our patched
`realsense_splitter` (`patches/`). Details: [VERSIONS.md](VERSIONS.md).

Check the firmware and the splitter end to end whenever the camera or firmware changes. Have a
textured object about 0.5–1.5 m away in the lower-left of the IR image:

```bash
rs-enumerate-devices -s                                    # "Firmware Version"
ros2 launch d455_mapping camera.launch.py &                # in the container
ros2 run d455_mapping emitter_check --save /bags           # PASS = cuVSLAM gets dot-free frames
#   FAIL "cuVSLAM gets the frames WITH projector dots" -> toggle camera.invert_emitter_mode
#   INCONCLUSIVE -> nothing close enough; move the camera or pass --roi y0,y1,x0,x1
```

Flashing (host, camera plugged in directly, nothing else using it; download the `.bin` from
<https://dev.realsenseai.com/docs/firmware-releases-d400/>):

```bash
/opt/ros/jazzy/bin/rs-fw-update -l                         # or rs-fw-update in the container
/opt/ros/jazzy/bin/rs-fw-update -s <serial> -f Signed_Image_UVC_5_15_1_55.bin
```

Flash from the **host**. The container's `/dev` is fixed when it starts, so it cannot see the
camera re-enumerate in update mode. If a flash stops with "Failed to locate a device in FW
update mode", run `rs-fw-update -r -f <bin>` on the host. After an unexpected failure,
unplug the camera for 10 s. Keep `camera.initial_reset: false`: a hardware reset at start
left this unit streaming nothing.

## 9. Tests (Phase 4)

All passed on the laptop (2026-10-16); numbers are in [docs/TEST_RESULTS.md](docs/TEST_RESULTS.md).
**C** = you need to hold or move the camera. Everything else runs unattended.

| # | | Test | How | Result |
|---|---|---|---|---|
| 1 | | Camera rates + splitter | `ros2 launch d455_mapping camera.launch.py`, `ros2 run d455_mapping emitter_check` (PASS), `ros2 topic hz` on `/camera/infra1/image_rect_raw` (30), `/camera/realsense_splitter_node/output/infra_1` (15), `.../output/depth` (15), `/camera/aligned_depth_splitter/output/depth` (15), `/camera/imu` (200) | all as expected |
| 2 | **C** | cuVSLAM drift | `vslam.launch.py rviz:=true` and `ros2 run d455_mapping odom_drift_check --seconds 60`; move out and back to the start pose | 1.6 cm over 3.3 m, no loss |
| 3 | **C** | Mapping | `scripts/map.sh room1`; slow loop back to the start; Ctrl-C (saves) | 193 keyframes, 8 global loop closures |
| 4 | **C** | Localization | `scripts/localize.sh room1` from another spot, pan slowly over mapped areas | hit after 28.5 s from 5 m away, pose snapped 4.4 m |
| 5 | | Export + viewer | `scripts/export_map.sh room1 --with-images`, `scripts/show_map.sh room1` | 1.56 M points, mesh, 193 keyframes with calibration |
| 6 | **C** | Bag replay | `scripts/record.sh room1_raw2` (walk about 75 s; about 85 MB/s, so start walking right after the recorder is up), `scripts/replay.sh room1_raw2 room1_replay2` | clean map, loop closed, no pose jumps |
| 7 | **C** | Resources | `scripts/measure.sh 100 live_mapping` during a mapping walk, `scripts/measure.sh 90 live_localization` during localization | GPU about 1 GB, CPU 12–15 % peak |

## 10. Troubleshooting

**USB 3 vs USB 2.** `rs-enumerate-devices -s` shows "Usb Type Descriptor 2.1". On the host,
`lsusb -t` shows the camera at 480M instead of 5000M. On USB 2 the 848x480@30 profiles fail
or fall back. Use a USB-C port directly with a short USB 3 cable (not a hub), plug it in
firmly, and try the cable in the other orientation (USB-C). Restart the container after
re-plugging.

**RealSense permission errors** ("failed to set power state", "Permission denied
/dev/bus/usb/...", or no device found while `lsusb` lists it):
- Run `./scripts/host_setup.sh`, re-plug the camera, then restart the container.
- `ls -l /dev/bus/usb/*/*` should show `crw-rw-rw-` for the camera.
- Plug the camera in **before** starting the container (Isaac ROS guide).
- Only one process can own the camera, so close `realsense-viewer` and any other
  realsense node.

**X11 errors** ("could not connect to display", "Authorization required"):
- Start the container from a desktop terminal so `DISPLAY` is set, then restart it with
  `dev.sh --stop && dev.sh`, which regenerates the cookie.
- Inside the container: `echo $DISPLAY $XAUTHORITY` should show `:1 /tmp/.d455.xauth`.
- If you see "MIT-SHM" errors, `QT_X11_NO_MITSHM=1` is already set.
- If `~/.Xauthority` is a root-owned directory, run `scripts/host_setup.sh` to remove it.

**GPU out of memory** (cudaErrorMemoryAllocation, or nvblox stops integrating; 6 GB on the
laptop):
- Run `nvidia-smi` in a second shell.
- Coarsen `nvblox.voxel_size` (0.05 → 0.08), reduce `max_integration_distance_m`, set
  `map_clearing_radius_m` to 8 m, or run with `nvblox:=false`.
- Close other GPU users (browser hardware acceleration).
- The RTAB-Map MapCloud display in RViz also uses GPU memory. Raise its "Cloud decimation".

**DDS discovery between Jetson and laptop:**
- Same `ROS_DOMAIN_ID` and same `RMW_IMPLEMENTATION` on both sides.
- Use unicast peers over Wi-Fi (`config/dds/cyclonedds_unicast.xml`, docs/JETSON.md §4).
- Firewalls: `sudo ufw allow in proto udp from <peer ip>`.
- The DDS buffers from `host_setup.sh` are needed on both machines.
- Keep the clocks synced (NTP/chrony).
- Large topics: use the `*_remote.rviz` configs (compressed image, no raw clouds).
- The host's own ROS Jazzy (sourced in `~/.bashrc`) is a different distro. Don't run its
  nodes on the same `ROS_DOMAIN_ID`.

**cuVSLAM tracking loss on low-texture surfaces** (`/visual_slam/status` → `vo_state`
not 1, or jumps in the odometry):
- cuVSLAM needs texture visible in the **emitter-off** IR, so blank walls, floors and glass
  fail.
- Point the camera at textured scenes and move slower, especially when rotating.
- Keep `depth_fps` at 30 (15 Hz to cuVSLAM). At 15 fps cuVSLAM gets only 7.5 Hz.
- IMU fusion (on) bridges short gaps.
- As a last resort, set `camera.emitter_on_off: false` with the emitter on. cuVSLAM then
  sees the projector dots, which add texture but can cause false features when moving.
- After tracking loss, RTAB-Map starts a new sub-map and re-links it on the next loop closure.

**Splitter outputs nothing / splitter IR at 30 Hz instead of 15 Hz:**
- The splitter needs `frame_emitter_mode` in the metadata. Check
  `ros2 topic echo /camera/infra1/metadata --once | grep emitter`.
- If every frame has the same label, the camera kept a stuck emitter state from a previous
  session. `emitter_ensure` (started with the camera) fixes this automatically; by hand:
  `ros2 param set /camera/camera depth_module.emitter_on_off false` then `... true`.
- Old firmware or a USB 2 link can drop metadata.

**`Error starting device: get_data_usb failed`, USB `-71`/`-110` in `journalctl -k`, or no
frames at all:** the host's `uvcvideo` driver is probing the camera. Run
`./scripts/host_setup.sh --no-uvcvideo` (§1). If `xHCI host controller not responding` appears,
no USB-C port works until you reboot. After a reboot, plug the camera in directly. A hardware
reset at start (`camera.initial_reset`) made this camera worse, so keep it `false`.

**RTAB-Map container dies at start** (`process has died ... exit code -6`, for example
`UNIQUE constraint failed: Statistics.id`, or `Linear information X should not be null`):
- Stale processes from an earlier run are still in the ROS graph (an old RViz or container).
  Run `scripts/stop.sh`, and if needed `dev.sh --stop` and start again. Do not use
  `pkill -f <pattern>` inside `dev.sh -- '...'`: the pattern matches the command's own shell.
- `rtabmap.odom_source` must stay `tf` (see §5).

**`dev.sh` exits silently without starting the container:** `run_dev.sh` calls `tput`, which fails
with `TERM` unset or `TERM=dumb`. `dev.sh` sets `TERM=xterm` in that case; if you still see it,
run `TERM=xterm ./scripts/dev.sh`.

**Depth/color at 7 Hz in `ros2 topic hz`, `ros2 bag record` or RViz:** Fast DDS's default
shared-memory segment (0.5 MB) is smaller than an 848x480 or 640x480 image. `dev.sh` sets
`FASTRTPS_DEFAULT_PROFILES_FILE=config/dds/fastdds_shm.xml` (32 MB segments); make sure the
container was started through `dev.sh`.

**`Hardware Notification: Right MIPI error` right at camera start:** seen a few times at startup on
this unit, harmless as long as the streams run at full rate afterwards.

**Isaac apt pin fails** ("Version '3.2.x' for 'ros-humble-isaac-ros-…' was not found"):
NVIDIA published a newer 3.2 patch. Update the ARGs in `docker/Dockerfile.d455_mapping` and
in VERSIONS.md.

## 11. Where this setup deviates from the original plan (and why)

| Plan | Docs say / reality | What we did |
|---|---|---|
| Laptop on Ubuntu 22.04 | Host is 24.04. Isaac ROS 3.2 supports 22.04+ hosts; the containers are 22.04 | Nothing changes inside the containers |
| RealSense splitter via apt | No `.deb` exists. The nvblox RealSense tutorial builds `realsense_splitter` from `isaac_ros_nvblox` source (remove `COLCON_IGNORE`) | `setup_workspace.sh` copies it from tag `v3.2-15` into `src/`, and colcon builds it |
| isaac_ros_common in `src/` builds its packages | We only need its Docker scripts; the ROS packages come from apt | `COLCON_IGNORE` in `src/isaac_ros_common` |
| D455 FW 5.13.0.50 | Fails on our newer D455 (color sensor, USB controller crashes); 5.16.0.1 streams nothing with librealsense 2.55.1 | Factory FW 5.15.1.55 + `invert_emitter_mode` patch, verified by `emitter_check` (§8) |
| Camera as a separate process | 0.8–0.9 MB images exceed Fast DDS's 0.5 MB shared-memory segment (depth/color at 7 Hz) | Camera, splitters, cuVSLAM, nvblox and RTAB-Map run in one component container; `config/dds/fastdds_shm.xml` gives other processes 32 MB segments |
| NumPy from the base image | Unpinned `pip -U` steps pull NumPy 2.x, which breaks apt `cv2`/`cv_bridge` | `numpy<2`, `cupy-cuda12x<14` in our layer |
| Kernel `uvcvideo` active | Fights librealsense's libusb backend in the container; crashed the laptop's USB controller | `host_setup.sh --no-uvcvideo` (udev rule + blacklist) |
| RTAB-Map takes `/visual_slam/tracking/odometry` | cuVSLAM can publish a zero covariance, which aborts RTAB-Map (`setInfMatrix`) | `odom_source: tf`: same pose via the `odom → base_link` TF with fixed variances |
| Emitter interleave set once by the driver | The camera keeps a stuck emitter state between sessions | `emitter_ensure` toggles it until the labels alternate |
| ≥ 8 GB VRAM | RTX 4050 has 6 GB | Bounded nvblox (0.05 m, 4 m range); about 1 GB GPU measured in test 7 |
| `rtabmap` with `visual_odometry:=false` | That argument belongs to `rtabmap_launch` | We compose `rgbd_sync` + `rtabmap` ourselves with no odometry node, which is equivalent |
| emitter-on depth for RTAB-Map | The splitter only splits raw depth | A second splitter instance on `aligned_depth_to_color` + `depth/metadata` (same timestamps in realsense-ros 4.51) |
