# Pinned versions

Everything below is pinned so the laptop (x86_64) and the Jetson Orin Nano (aarch64) build
the same software. 

| Component | Version | Pinned in | Source |
|---|---|---|---|
| ROS 2 | Humble | Isaac ROS base image | Isaac ROS 3.2 supports Humble only |
| Isaac ROS | 3.2 | — | <https://nvidia-isaac-ros.github.io/v/release-3.2/> |
| isaac_ros_common | tag `v3.2-15` (commit `fcf4d9e`, = `release-3.2` head) | `scripts/setup_workspace.sh` (`ISAAC_ROS_TAG`) | GitHub NVIDIA-ISAAC-ROS/isaac_ros_common |
| realsense_splitter | isaac_ros_nvblox tag `v3.2-15` (commit `7908a18`) | `scripts/setup_workspace.sh` (`ISAAC_ROS_TAG`) | built from source, as the nvblox RealSense tutorial says (no .deb) |
| Image key | `<arch>.ros2_humble.realsense.d455_mapping` | `scripts/setup_workspace.sh`, `scripts/dev.sh` | `ros2_humble` + `realsense` from isaac_ros_common, `d455_mapping` = `docker/Dockerfile.d455_mapping` |
| x86_64 base image | `nvcr.io/nvidia/tritonserver:24.08-py3` (CUDA 12.6) | `isaac_ros_common/docker/Dockerfile.x86_64` | via isaac_ros_common tag |
| Jetson base | JetPack 6.1/6.2 (L4T R36.4.x), CUDA 12.6 | `isaac_ros_common/docker/Dockerfile.aarch64` | via isaac_ros_common tag |
| librealsense | `v2.55.1` (RSUSB/libusb backend, CUDA on) | `isaac_ros_common/docker/Dockerfile.realsense`; host udev rules in `scripts/host_setup.sh` | required by the Isaac ROS 3.2 RealSense guide |
| realsense-ros | `release/4.51.1-isaac` (NVIDIA fork) | `isaac_ros_common/docker/Dockerfile.realsense` | required by the Isaac ROS 3.2 RealSense guide |
| RealSense firmware | **5.15.1.55** on our D455 (s/n 260722303845) — *deviates* from the docs' 5.13.0.50, see below | flash with `rs-fw-update` (README §8) | docs require 5.13.0.50; that firmware does not work on this hardware revision |
| realsense_splitter patch | `patches/realsense_splitter_invert_emitter_mode.patch` | applied by `scripts/setup_workspace.sh` | adds `invert_emitter_mode` (default false) |
| ros-humble-isaac-ros-visual-slam | `3.2.6-0jammy` | `docker/Dockerfile.d455_mapping` (`ISAAC_ROS_VSLAM_VERSION`) | Isaac apt repo (same version amd64/arm64) |
| ros-humble-isaac-ros-nvblox, ros-humble-nvblox-rviz-plugin | `3.2.5-0jammy` | `docker/Dockerfile.d455_mapping` (`ISAAC_ROS_NVBLOX_VERSION`) | Isaac apt repo (same version amd64/arm64) |
| ros-humble-rtabmap-ros, rtabmap | latest Humble from packages.ros.org at build time | not pinnable (ROS apt keeps only the latest); the installed version is printed during the build and stored in `/opt/d455_mapping_versions.txt` in the image | see "Recorded at build" below |

The Isaac apt repo keeps **only the latest** build of each package. If NVIDIA publishes a
newer 3.2.x patch, the build fails on the old pin. Then update the ARG in
`docker/Dockerfile.d455_mapping` and this table together.

## RealSense firmware on our D455 (tested 2026-10-16)

The Isaac ROS 3.2 RealSense guide requires firmware 5.13.0.50. Our D455 is a newer production
unit (BMI085 IMU, factory firmware 5.15.1.55). Tested with the container's librealsense 2.55.1:

| Firmware | Result |
|---|---|
| 5.15.1.55 (factory) | **Used.** All streams at full rate (IR/depth/color/aligned 30 Hz, IMU 200 Hz). `frame_emitter_mode` metadata is **inverted** (verified with `emitter_check`), so `camera.invert_emitter_mode: true`. Setting `emitter_on_off` logs `hwmon 0x7b failed`, but interleaving works. |
| 5.13.0.50 (docs) | Unusable: the color sensor fails to initialize (USB `-71`) in both kernel and librealsense, and the laptop's TB4 xHCI controller crashed twice. |
| 5.16.0.1 | Unusable with librealsense 2.55.1: "IR stream start failure" / "Motion Module failure", no frames. |

A different camera (for example the D435i, or an older D455) may need `invert_emitter_mode: false`.
Always run `ros2 run d455_mapping emitter_check` after changing camera or firmware.

## Host requirements (Isaac ROS 3.2 docs → found on the laptop, 2026-10-15)

| Requirement | Docs | Laptop |
|---|---|---|
| OS | Ubuntu 22.04+ | Ubuntu 24.04.3, kernel 6.14 |
| NVIDIA driver | ≥ 560 | 580.178.04 (CUDA 13.0) |
| GPU | Ampere+ with ≥ 8 GB VRAM | RTX 4050 Laptop, Ada, **6 GB** (below spec, see README) |
| Docker Engine | ≥ 27.2.0 | 29.1.3 (buildx 0.30.1) |
| NVIDIA Container Toolkit | installed + Docker runtime configured | 1.18.1, `nvidia` runtime |
| Disk | ≥ 30 GB | 329 GB free |
| RAM | ≥ 16 GB | 62 GB |

## Recorded at build

Image `isaac_ros_dev-x86_64-d455` (39 GB, built 2026-10-16), Ubuntu 22.04.4, Python 3.10.12,
CUDA toolkit 12.6.20. From `/opt/d455_mapping_versions.txt` and `dpkg-query` inside the image:

| Package | Version |
|---|---|
| ros-humble-isaac-ros-visual-slam | 3.2.6-0jammy |
| ros-humble-isaac-ros-nvblox, -nvblox-ros, -nvblox-rviz-plugin, -isaac-ros-common | 3.2.5-0jammy |
| ros-humble-realsense2-camera, -camera-msgs | 4.51.1-0jammy (librealsense2 2.55.1 built from source) |
| ros-humble-rtabmap | 0.23.13-1jammy.20261002.103658 |
| ros-humble-rtabmap-ros | 0.23.13-1jammy.20261011.115255 |
| ros-humble-rtabmap-slam / -rtabmap-rviz-plugins | 0.23.13 (…20261011.111917 / …20261002.123908) |
| ros-humble-imu-filter-madgwick | 2.1.6-1jammy.20260921.031003 |
| ros-humble-pcl-ros | 2.4.5-2jammy.20260915.212316 |
| ros-humble-rosbag2-storage-mcap | 0.15.17-1jammy.20260908.085225 |
| ros-humble-rviz2 | 11.2.29-1jammy.20260916.003100 |
| ros-humble-rmw-fastrtps-cpp (default RMW) | 6.2.10-1jammy.20260907.210327 |
| assimp-utils / pcl-tools | 5.2.2~ds0-1 / 1.12.1+dfsg-3build1 |
| Python: numpy / cupy-cuda12x / torch | **1.26.4** (pinned `<2`) / **13.6.0** (pinned `<14`) / 2.14.1+cu130 (unused by our stack; unpinned upstream) |

The `ros-humble-*` packages from packages.ros.org change over time; to rebuild exactly this
image later, keep the Docker image (`docker save`) or compare against this table.

## Test environment (laptop)

Dell XPS 14 9440, Ubuntu 24.04.3 (kernel 6.14), RTX 4050 Laptop 6 GB (driver 580.178.04),
Docker 29.1.3, D455 s/n 260722303845 with firmware 5.15.1.55 (see above), `uvcvideo` blacklisted.
