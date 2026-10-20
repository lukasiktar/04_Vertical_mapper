# Deploying to the Jetson Orin Nano 8 GB

The Jetson runs the **same repo, image layers and launch files** as the laptop. Only
`platform:=orin_nano` changes, which selects `src/d455_mapping/config/orin_nano.yaml`.
The camera, cuVSLAM, RTAB-Map (and optionally nvblox) run on the Jetson. RViz runs on the
laptop over the network.

Every `sudo` command below is something **you** run. The helper scripts print each command
and ask before running it.

---

## 0. Prerequisites (verified by `scripts/jetson_check.sh`)

| Check | Command | Must be |
|---|---|---|
| L4T release | `cat /etc/nv_tegra_release` | `R36 (release), REVISION: 4.x` (JetPack 6.1 = R36.4.0, 6.2 = R36.4.3+) |
| JetPack | `dpkg-query -W nvidia-jetpack` | 6.1 or 6.2 (Isaac ROS 3.2 supports only these) |
| Module RAM | `free -h` | ~7.4 GiB visible (8 GB module). The 4 GB Orin Nano is not supported |
| NVMe SSD | `lsblk -d` | an `nvme0n1` disk. **Required:** the dev image is about 20 GB, and bags are large |
| Docker | `docker --version`, `docker buildx version` | Docker + buildx plugin, user in `docker` group, `nvidia` runtime |

```bash
git clone <this repo> ~/d455_mapping     # later moved to the SSD, see 1.
cd ~/d455_mapping
./scripts/jetson_check.sh
```

## 1. Host setup (once)

The steps follow the Isaac ROS 3.2 *Compute Setup → Jetson Platforms* and *Jetson Setup*
(SSD + Docker migration) pages.

1. **NVMe SSD.** If the Jetson already boots from the NVMe, skip this step. Otherwise
   format and mount it as `/mnt/nova_ssd`:
   ```bash
   sudo mkfs.ext4 /dev/nvme0n1          # ERASES the SSD
   sudo mkdir -p /mnt/nova_ssd && sudo mount /dev/nvme0n1 /mnt/nova_ssd
   lsblk -f                              # copy the UUID, then add to /etc/fstab:
   #   UUID=<uuid> /mnt/nova_ssd/ ext4 defaults 0 2
   sudo chown ${USER}:${USER} /mnt/nova_ssd
   ```
2. **Workspace on the SSD:**
   ```bash
   mkdir -p /mnt/nova_ssd/workspaces
   mv ~/d455_mapping /mnt/nova_ssd/workspaces/ && cd /mnt/nova_ssd/workspaces/d455_mapping
   ```
   (If the root filesystem is already on NVMe, keep it anywhere under `$HOME`.)
3. **Docker group, buildx, git-lfs.** buildx comes from Docker's apt repo, exactly as the
   Isaac ROS docs describe:
   ```bash
   sudo apt-get update && sudo apt-get install -y ca-certificates curl gnupg
   sudo install -m 0755 -d /etc/apt/keyrings
   curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
   sudo chmod a+r /etc/apt/keyrings/docker.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo $VERSION_CODENAME) stable" \
     | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
   ./scripts/jetson_setup.sh docker-tools      # usermod -aG docker, buildx, git-lfs
   ```
   Log out and back in.
4. **Docker data root on the SSD** (skip if `/` is already on NVMe). This also sets
   `"default-runtime": "nvidia"`:
   ```bash
   ./scripts/jetson_setup.sh docker-root /mnt/nova_ssd/docker
   ```
5. **Highest power mode.** On JetPack 6.2 this is *MAXN SUPER*; on 6.1 it is mode 0 (15 W):
   ```bash
   ./scripts/jetson_setup.sh power             # nvpmodel -m <MAXN id> && jetson_clocks
   ```
   `jetson_clocks` does not survive a reboot. Re-run it before measurements.
6. **Swap for builds.** Building the image layers compiles librealsense and realsense-ros:
   ```bash
   SWAP_FILE=/mnt/nova_ssd/swapfile ./scripts/jetson_setup.sh swap 8G
   ```
7. **RealSense udev rules, DDS buffers, `uvcvideo` off the camera** (same script as on the laptop;
   `--no-uvcvideo` unloads the kernel UVC driver, which otherwise probes the camera while
   librealsense in the container uses it. Skip the flag only if you need another UVC webcam on
   the Jetson):
   ```bash
   ./scripts/host_setup.sh --no-uvcvideo
   ```
8. **jtop:**
   ```bash
   ./scripts/jetson_setup.sh jtop              # then log out/in; run `jtop`
   ```
9. Re-run `./scripts/jetson_check.sh`. Everything should be `[ OK ]`.

## 2. Build the image and workspace (aarch64)

The same layer, `docker/Dockerfile.d455_mapping`, is used on both platforms.
`isaac_ros_common` picks `Dockerfile.aarch64` automatically, based on L4T R36.4, CUDA 12.6
and the same Isaac apt packages and versions.

```bash
./scripts/setup_workspace.sh          # pinned isaac_ros_common + realsense_splitter
./scripts/dev.sh --build-image        # first build: about 1 h+ (on the NVMe, with swap)
./scripts/build.sh                    # colcon build inside the container
./scripts/dev.sh                      # shell in the container
```

Firmware: the Isaac ROS 3.2 guide requires D455 firmware **5.13.0.50**, but on our (newer) D455 only
the factory firmware **5.15.1.55** works, together with `camera.invert_emitter_mode: true` (already set
in `orin_nano.yaml`). The laptop's D455 findings are in VERSIONS.md and README §8. On the Jetson,
check the camera's firmware with `rs-enumerate-devices -s` inside the container and always run

```bash
ros2 launch d455_mapping camera.launch.py platform:=orin_nano &
ros2 run d455_mapping emitter_check          # PASS = cuVSLAM gets dot-free frames
```

before the first mapping run. If the Jetson gets a different camera (for example the D435i),
`invert_emitter_mode` may need to be `false`.

## 3. Run with `platform:=orin_nano`

```bash
./scripts/map.sh lab orin_nano                 # mapping, maps/lab.db, no RViz on the Jetson
./scripts/localize.sh lab orin_nano            # localization against maps/lab.db
./scripts/record.sh lab_raw orin_nano          # raw bag -> bags/lab_raw (replay on the laptop)
./scripts/map.sh lab orin_nano depth_fps:=15   # low-load option (see the warning in orin_nano.yaml)
./scripts/map.sh lab orin_nano nvblox:=true    # nvblox on (0.10 m voxels, 3 m range, 6 m radius)
```

Platform differences (`orin_nano.yaml` vs `laptop.yaml`):

| Item | laptop | orin_nano |
|---|---|---|
| Color | 640x480 @ 30 | 640x480 @ 15 |
| RTAB-Map detection rate | 2 Hz | 1 Hz |
| Kp/MaxFeatures, Vis/MaxFeatures | 500 / 1000 | 400 / 600 |
| nvblox | on, 0.05 m, 4 m range, no clearing | **off**, 0.10 m, 3 m range, clear > 6 m |
| cuVSLAM debug clouds | on | off |
| RViz | local | remote, `*_remote.rviz` |

## 4. Remote RViz from the laptop

Both machines run the container. Both must use the **same `ROS_DOMAIN_ID` and the same RMW**.
Multicast discovery is usually unreliable on Wi-Fi, so use CycloneDDS with unicast peers
(`config/dds/cyclonedds_unicast.xml`).

On the **Jetson** (IP e.g. 192.168.1.50):
```bash
export ROS_DOMAIN_ID=42 D455_RMW=rmw_cyclonedds_cpp D455_PEER=192.168.1.20   # laptop IP
export CYCLONEDDS_URI=file:///workspaces/isaac_ros-dev/config/dds/cyclonedds_unicast.xml
./scripts/dev.sh --stop 2>/dev/null; ./scripts/map.sh lab orin_nano
```
On the **laptop** (IP e.g. 192.168.1.20):
```bash
export ROS_DOMAIN_ID=42 D455_RMW=rmw_cyclonedds_cpp D455_PEER=192.168.1.50   # Jetson IP
export CYCLONEDDS_URI=file:///workspaces/isaac_ros-dev/config/dds/cyclonedds_unicast.xml
./scripts/dev.sh --stop 2>/dev/null
./scripts/dev.sh -- 'rviz2 -d $(ros2 pkg prefix d455_mapping)/share/d455_mapping/rviz/mapping_remote.rviz'
# localization: .../rviz/localization_remote.rviz
```

Notes:
- The container environment is fixed when it starts. After changing these variables, run
  `dev.sh --stop` and start it again.
- `*_remote.rviz` uses `/camera/color/image_raw/compressed` (best effort). It has no RTAB-Map
  MapCloud and no raw point clouds; it shows the graph, both paths, TF and the nvblox mesh
  (incremental and small). Turn MapCloud on only on a wired link.
- Check discovery first: `ros2 node list` on the laptop must show the Jetson nodes.
- If FastDDS is required instead (Isaac ROS default), leave `D455_RMW` unset on both sides
  and give FastDDS an initial-peers profile through `FASTRTPS_DEFAULT_PROFILES_FILE`.
- Both hosts need the `rmem_max` tuning from `scripts/host_setup.sh`.
- **Not tested yet:** the Jetson and the remote-RViz setup were never run. The Fast DDS profile
  `config/dds/fastdds_shm.xml` (32 MB shared-memory segments, the reason full-rate image topics
  work between processes on the laptop) is only applied when the RMW is Fast DDS. With
  CycloneDDS, check `ros2 topic hz /camera/depth/image_rect_raw` (30 Hz) and `ros2 bag record`
  rates on the Jetson first; the camera, splitters, cuVSLAM and RTAB-Map are components of one
  container, so most traffic stays in-process either way.
- The Jetson clock must be NTP-synced with the laptop, or RViz will drop TF ("extrapolation
  into the future").

## 5. Test checklist (mirrors README → Phase 4 tests)

Run `jtop` on the Jetson host in a second SSH session throughout, and `sudo jetson_clocks`
before measuring.

| # | Test | Command | Pass criteria |
|---|---|---|---|
| 1 | Camera rates | `ros2 launch d455_mapping camera.launch.py platform:=orin_nano`, then `ros2 topic hz` on `/camera/realsense_splitter_node/output/infra_1`, `.../output/depth`, `/camera/aligned_depth_splitter/output/depth`, `/camera/imu` | IR/depth ≈ 15 Hz each (30 fps interleaved), IMU ≈ 200 Hz. `lsusb -t` shows the D455 at 5000M |
| 2 | cuVSLAM | `ros2 launch d455_mapping vslam.launch.py platform:=orin_nano` + `ros2 topic echo /visual_slam/status`; move 2 m forward and back | returns within ~2–5 cm, no tracking loss. IR images (remote compressed view) have no projector dots |
| 3 | Mapping | `./scripts/map.sh lab orin_nano`, walk a loop | `/rtabmap/info` shows `loop_closure_id > 0`; `maps/lab.db` written on Ctrl-C |
| 4 | Localization | `./scripts/localize.sh lab orin_nano` from another spot | localization hit; pose snaps into the map in remote RViz |
| 5 | Export | copy `maps/lab.db` to the laptop → `./scripts/export_map.sh lab` → `./scripts/show_map.sh lab` | cloud + mesh shown in frame `map` |
| 6 | Bag | `./scripts/record.sh lab_raw orin_nano`; copy `bags/lab_raw` to the laptop; `./scripts/replay.sh lab_raw lab_replay laptop` | loop closures similar to the live run |
| 7 | Resources | `./scripts/dev.sh -- scripts/measure.sh 180 orin_mapping` during 3 and 4, plus `jtop` | record peak RAM (shared CPU+GPU), GPU %, CPU %. Keep < ~6.5 GB to leave headroom |

Memory budget (8 GB shared): L4T + desktop about 1.5 GB (disable the desktop:
`sudo systemctl set-default multi-user.target`), realsense driver about 0.3 GB,
cuVSLAM about 0.5–1 GB, RTAB-Map 0.3–1.5 GB (grows with map size), nvblox 0.5–2 GB at
0.10 m. That is why nvblox is off by default here.
