# Jetson Orin Nano: quick commands (mapping, RViz on your laptop)

Everything runs in the mapper's own Docker container (`isaac_ros_dev-aarch64-d455-container`).
It does not touch the VANGUARD / PX4 container (`vanguard_ros2`); both share the host network, so
their topics see each other. This repo is the `mapper/` submodule of VANGUARD. All Jetson commands:
from `~/VANGUARD/mapper`.

The mapper owns the D455; nothing in `vanguard_ros2` should open the camera.

## 0. Before you start (Jetson, every session)

```bash
ssh drone_EDU
cd ~/VANGUARD/mapper
./scripts/dev.sh --detach        # start the container (takes a while after a reboot / image change)
./scripts/usb_perms.sh           # camera access inside the container. Redo after a container
                                 # restart or after unplugging / replugging the camera
```

Optional check that the camera is seen (serial 260722303845, firmware 5.15.1.55):

```bash
./scripts/dev.sh -- 'rs-enumerate-devices -s'
```

## 1. Look at RViz on your laptop

On the Jetson:

```bash
./scripts/rviz_vnc.sh start mapping_remote        # RViz on a virtual desktop (1600x900)
```

On your laptop (not on the Jetson), two terminals (the first stays open):

```bash
ssh -L 5901:localhost:5901 drone_EDU
vncviewer localhost:5901
```

Nothing shows / blank screen -> the virtual desktop is not running:

```bash
./scripts/rviz_vnc.sh status
./scripts/rviz_vnc.sh start mapping_remote (light_remote)
```

In RViz tick **RTAB-Map MapCloud** to see the map cloud (heavier, CPU rendering).
`./scripts/rviz_vnc.sh stop` closes the desktop. Other configs: `localization_remote`, `show_map`.

## 2. Make a map

Second Jetson terminal (or a new SSH session):

```bash
./scripts/map.sh myroom4 orin_nano               # new name each time; add --force to overwrite
./scripts/map.sh myroom4 orin_nano with_cross_detector:=true     # same + cross detector
```

- Hold the camera still for about 30 s at the start (warm-up), then move slowly and keep moving.
- Finish where you started (same view) so a loop closure happens.
- Warnings `Did not receive data since 5 seconds` while the camera is still are normal.
- **Stop and save: Ctrl-C and wait until the prompt returns** (up to a minute). Do not kill it.

Check progress from another terminal:

```bash
./scripts/dev.sh -- 'ros2 topic echo /rtabmap/info --field loop_closure_id'    # > 0 = loop closure
./scripts/dev.sh -- 'ros2 topic echo /visual_slam/status --once | grep vo_state'   # 1 = tracking OK
./scripts/map_info.sh myroom4                    # after stopping: keyframes, loop closures
```

## 2b. Map with PX4 odometry fused (drone)

(For flying without OptiTrack use 2c, `px4_vio:=true`, instead: PX4 then already contains cuVSLAM,
and `fuse_px4:=true` would count it twice; the launch refuses both together.)

Odometry for RTAB-Map then comes from an EKF (robot_localization) that fuses PX4's odometry
(OptiTrack / optical flow / IMU, already fused by EKF2) with cuVSLAM. When cuVSLAM loses tracking
(camera too close to something, blank wall) it is dropped and the EKF runs on PX4 alone.

### Once (after pulling this change)

```bash
cd ~/VANGUARD/mapper
./scripts/dev.sh --build-image                         # mapper image + robot_localization (takes a while)
./scripts/build.sh                                     # mapper workspace (vslam_twist, fuse_px4 launch arg)

cd ~/VANGUARD
./ros2-docker.sh colcon --packages-select vanguard     # px4_odom_bridge
```

### Every flight: one terminal per command (each stays busy, stop with Ctrl-C)

Terminal 1, PX4 link (vanguard_ros2):

```bash
cd ~/VANGUARD
./ros2-docker.sh up
./ros2-docker.sh run MicroXRCEAgent udp4 -p 8888
```

Terminal 2, OptiTrack driver (skip terminals 2 and 3 when flying without OptiTrack):

```bash
cd ~/VANGUARD
./ros2-docker.sh run ros2 launch mocap4r2_optitrack_driver optitrack2.launch.py
```

Terminal 3, activate the driver, then OptiTrack -> PX4:

```bash
cd ~/VANGUARD
./ros2-docker.sh run ros2 lifecycle set /mocap4r2_optitrack_driver_node activate
./ros2-docker.sh run ros2 run vanguard mocap_to_px4_node
```

Terminal 4, PX4 odometry -> mapper:

```bash
cd ~/VANGUARD
./ros2-docker.sh run ros2 run vanguard px4_odom_bridge      # /fmu/out/vehicle_odometry -> /px4/odom
```

Terminal 5, mapper (camera setup as in step 0):

```bash
cd ~/VANGUARD/mapper
./scripts/dev.sh --detach
./scripts/usb_perms.sh
./scripts/map.sh flight1 orin_nano fuse_px4:=true           # new name each time; Ctrl-C saves
```

Same rules as step 2: drone still on the ground for about 30 s, fly slowly, finish where you started,
Ctrl-C once after landing and wait for the prompt. Then:

```bash
./scripts/map_info.sh flight1                               # want "OK: loops closed"
./scripts/localize.sh flight1 orin_nano fuse_px4:=true      # relocalize (instead of map.sh)
```

### Check (another terminal, from ~/VANGUARD/mapper)

```bash
./scripts/dev.sh -- 'ros2 topic hz /px4/odom'                                     # PX4 odometry arrives (steady rate)
./scripts/dev.sh -- 'ros2 topic hz /vslam/twist'                                  # cuVSLAM is fused (stops while not tracking)
./scripts/dev.sh -- 'ros2 topic echo /visual_slam/status --once | grep vo_state'  # 1 = cuVSLAM tracking
./scripts/dev.sh -- 'ros2 topic echo /odometry/filtered --once'                   # EKF output
./scripts/dev.sh -- 'ros2 run tf2_ros tf2_echo odom base_link'                    # fused pose RTAB-Map uses
```

Nothing on `/px4/odom` -> terminal 1 (agent) or 4 (bridge) is not running. The mapping terminal
logs `cuVSLAM not tracking -> EKF on PX4 only` and `cuVSLAM tracking -> fused with PX4` when it drops
and resumes cuVSLAM.

The camera mounting used while fusing is `px4_fusion.base_to_camera` in `config/orin_nano.yaml`
(from `calibrate_rig.sh`; check it against the drone). Maps made with and without `fuse_px4` use a
different `base_link`; localize with the same setting the map was made with.

## 2c. cuVSLAM as PX4's external vision (fly without OptiTrack)

cuVSLAM replaces OptiTrack as the input to PX4's EKF2, which fuses it with the H-Flow (optical flow
+ rangefinder) and the IMU. While cuVSLAM is not tracking nothing is sent and PX4 flies on the
optical flow; when it comes back it continues from its last pose plus the motion PX4 measured
meanwhile, so a cuVSLAM jump never reaches PX4. Not together with `fuse_px4:=true` (the launch
refuses it) or `mocap_to_px4_node`.

Bench test 2026-11-04 (props off, carried by hand): EKF2 fused cuVSLAM position + yaw + height
with flow and range throughout, PX4 within 10 cm of cuVSLAM while moving (height within 2 cm),
no resets. Hold the drone from the top in such tests: a hand under the H-Flow makes PX4's height
follow the rangefinder down to the floor. Covering the camera does not make cuVSLAM report a loss
(it holds its pose), so don't test tracking loss that way.

PX4 (QGC): same parameters as with OptiTrack except `EKF2_EV_DELAY = 90` (ms, measured; the bridge
logs the actual value every 10 s). Keep the OptiTrack set in its own parameter file.

### Once (after pulling this change)

```bash
cd ~/VANGUARD/mapper && ./scripts/build.sh                         # vslam_px4, px4_vio launch arg
cd ~/VANGUARD && ./ros2-docker.sh colcon --packages-select vanguard  # vslam_to_px4_node
```

### Every flight: one terminal per command

Drone on the ground at the take-off spot and still when cuVSLAM starts (its frame is levelled
from PX4's attitude in the first second).

```bash
# 1: PX4 link
cd ~/VANGUARD && ./ros2-docker.sh up && ./ros2-docker.sh run MicroXRCEAgent udp4 -p 8888
# 2: camera + cuVSLAM, base_link = PX4 body (map.sh / localize.sh ... px4_vio:=true to map at the same time)
cd ~/VANGUARD/mapper && ./scripts/dev.sh --detach && ./scripts/usb_perms.sh
./scripts/dev.sh -- 'ros2 launch d455_mapping vslam.launch.py platform:=orin_nano px4_vio:=true'
# 3: cuVSLAM -> PX4
cd ~/VANGUARD && ./ros2-docker.sh run ros2 run vanguard vslam_to_px4_node
```

Terminal 3 should log `cuVSLAM odom frame levelled ...`, `cuVSLAM anchored ...` and then every
10 s `15.0 Hz to PX4, camera -> PX4 delay ~90 ms`. Watch what PX4 makes of it (one line per second,
listens only; add a file name to save it, e.g. `/ros2_ws/bags/flight1.npz`):

```bash
cd ~/VANGUARD && ./ros2-docker.sh run python3 check_vio.py
#  EV pos/yaw/hgt YYY flow Y rng Y | PX4 x -0.23 y -0.13 z -0.19 yaw -2.6 | sent x -0.23 ... | range 0.19 m resets xy 3 z 2
```

Before arming: `EV pos/yaw/hgt YYY`, PX4 and "sent" within a few cm. The reset counters step a
few times when EKF2 first takes cuVSLAM; they should not change in flight.

Don't build or run other heavy jobs on the Jetson while flying: a starved cuVSLAM loses frames and
can come back metres off (the gate and the anchoring keep that away from PX4, but it is then
flying on optical flow only).

## 2d. Extend a saved map (same coordinates)

A new walk or flight added to an existing map, in that map's coordinates (RTAB-Map multi-session).
`<old>` is copied to `<new>` and never changed:

```bash
./scripts/extend_map.sh myroom4 myroom5 orin_nano with_cross_detector:=true [px4_vio:=true]
```

- Start where `<old>` was mapped, looking at what it saw. The new session joins the old map at its
  first loop closure on an old keyframe; before that the new part is drawn at the origin and the
  cross detector does not fuse (it logs `joined the old map (loop closure to node N)` when it does).
- Use the same `px4_vio` setting as the old map (same `base_link`).
- Ctrl-C once and wait, as for `map.sh`. Then `./scripts/map_info.sh myroom5` should say
  `sessions joined`; `NOT joined` means the new part is not aligned (start again closer to the old map).
- Localize in the result as usual: `./scripts/localize.sh myroom5 ...`.

## 3. Localize in a saved map

```bash
./scripts/localize.sh myroom4 orin_nano          # loads maps/myroom4.db
./scripts/rviz_vnc.sh start localization_remote (light_remote) # RViz for localization
```

Localized when `loop_closure_id` is above 0 (it took about 40 s of gentle movement last time):

```bash
./scripts/dev.sh -- 'ros2 topic echo /rtabmap/info --field loop_closure_id'
```

Localize with the same `px4_vio` / `fuse_px4` setting the map was made with (`base_link` differs).
RViz keeps drawing the camera / drone at its last localized place (RTAB-Map also restores it from
the database at start), so "it is on the map" does not mean localized. RTAB-Map publishes
`/rtabmap/localization_pose` also while lost, with covariance 9999; the cross detector and pins only
count it with a position variance <= 1 m^2. Check:
`./scripts/dev.sh -- 'ros2 topic echo /rtabmap/localization_pose --field pose.covariance --once' | head -1`.

Note: localization changes the `.db` file's bytes (metadata) although nodes and links stay the same.
Keep a copy of maps you care about: `cp maps/myroom4.db ~/backup/`.

### 3b. Pick crosses and see the distance from the camera to them

Localize with the cross detector AND the pin tool (`with_pins:=true`):

```bash
./scripts/localize.sh myroom4 orin_nano --prior-map with_cross_detector:=true with_pins:=true
./scripts/rviz_vnc.sh start localization_remote
```

In RViz (over VNC):
1. Wait until you are localized (point the camera at the mapped area, move gently; `loop_closure_id` > 0)
   and the orange crosses appear.
2. Click the **Publish Point** tool in the RViz toolbar (or press `P`).
3. Click **near an orange cross** (within 0.8 m; the click snaps to the nearest cross).
   The cross turns into a magenta pin and a cyan line with the distance (m) is drawn from the camera
   to it. Click the same cross again to remove the pin. The label near your click says what happened.

Distances only show while localized. Pins are saved in `maps/myroom4_pins.json` on every change
(they stay after a restart). The pin tool can only pin crosses the detector has confirmed.

```bash
# pin / unpin by cross id without the mouse (ids are in maps/myroom4_crosses.json)
./scripts/dev.sh -- 'ros2 topic pub --once /pins/pin_cross std_msgs/msg/Int32 "{data: 1}"'
# distances as JSON (camera frame: x right, y down, z forward)
./scripts/dev.sh -- 'ros2 topic echo /pins/distances --once'
# remove all pins
./scripts/dev.sh -- 'ros2 service call /pins/clear std_srvs/srv/Trigger'
```

## 4. Stop everything

```bash
# Ctrl-C in the map.sh / localize.sh terminal (saves the map), then:
./scripts/rviz_vnc.sh stop
./scripts/stop.sh                  # stops all launches; RTAB-Map saves its database first
./scripts/dev.sh --stop            # stops the container (after changing env vars, or to free RAM)
```

Do not use `pkill -f` inside `dev.sh -- '...'`: the pattern matches its own shell.

## 5. Export a map and look at it

```bash
./scripts/export_map.sh myroom4                  # maps/myroom4/: cloud (.pcd/.ply), mesh (.ply/.dae/.stl), poses
./scripts/export_map.sh myroom4 --with-images    # + keyframe RGB, depth, calibration
```

Copy to your laptop (run on the laptop) and open in MeshLab / CloudCompare:

```bash
scp -r drone_EDU:VANGUARD/mapper/maps/myroom4 .
scp drone_EDU:VANGUARD/mapper/maps/myroom4.db .
```

(Viewing the export through the VNC desktop with `show_map.sh` is not tested yet.)

## 6. Handy checks

```bash
./scripts/dev.sh -- 'ros2 topic hz /camera/imu'          # should be about 200 Hz
./scripts/measure.sh 100 mapping                         # RAM / CPU / GPU for 100 s (in the container)
./scripts/dev.sh                                         # interactive shell in the container
free -h; uptime                                          # Jetson RAM and load (about 4 GB RAM while mapping)
```

Reference numbers measured on this Jetson (orin_nano config, nvblox off):
15 Hz IR / depth / color, 200 Hz IMU, RAM about 4 GB, cuVSLAM return-to-start about 4 cm over 2 m.

## 7. If something goes wrong

| Symptom | What to do |
|---|---|
| `rs-enumerate-devices` says `RS2_USB_STATUS_ACCESS` / camera not found | `./scripts/usb_perms.sh` |
| VNC viewer connects but shows nothing | `./scripts/rviz_vnc.sh status`, then `start` again; check the `ssh -L 5901:...` tunnel is still open |
| `rviz_vnc.sh start` fails, `/tmp/xvnc.log`: `Address already in use` | the `ssh -L 5901:...` tunnel was started on the Jetson itself and holds the port: close it there (it belongs on the laptop) |
| Map stays empty / few nodes | Camera was probably still. Move slowly and keep moving. Make sure nothing else uses the camera |
| Stuck / odd behaviour after a crash | `./scripts/rviz_vnc.sh stop; ./scripts/stop.sh; ./scripts/dev.sh --stop`, then start from step 0 |
| Leftover camera / TF processes after killing a launch | stop with Ctrl-C or `./scripts/stop.sh`; they are orphaned if you kill only the wrapper |
| `ros2 run` / launch can't find a package | `./scripts/build.sh` |

## 8. What is customised on this Jetson (all inside ~/VANGUARD/mapper)

- `src/isaac_ros_common/scripts/run_dev.sh`: `NVIDIA_VISIBLE_DEVICES=all` (no CDI GPU spec on this host);
  `patches/isaac_ros_common_no_cdi.patch`, applied by `scripts/setup_workspace.sh`.
- `docker/Dockerfile.d455_mapping`: adds `ros-humble-diagnostic-updater` (RTAB-Map needs
  `libdiagnostic_updater.so` from 4.0.7), `tigervnc-standalone-server` + `openbox`, and
  `ros-humble-robot-localization` (EKF for `fuse_px4:=true`, step 2b).
  `dev.sh` does not rebuild an existing image on start; after editing the Dockerfile run
  `./scripts/dev.sh --build-image`.
- Scripts added: `scripts/usb_perms.sh`, `scripts/rviz_vnc.sh`, `scripts/foxglove.sh` (optional, not used).
- Not done yet: `scripts/host_setup.sh` (it unloads `uvcvideo`). It was skipped because the VANGUARD
  container's ArUco test used the camera through `uvcvideo`; that test is retired, so running it is
  now worth trying against the camera USB drops. The host is L4T R36.5.2; the guide asks for
  R36.4.x, but it works.

## 9. Power

Shut the Jetson down before unplugging it: `sudo shutdown now`.
