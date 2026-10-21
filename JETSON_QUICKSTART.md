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

On your laptop, two terminals (the first stays open):

```bash
ssh -L 5901:localhost:5901 edu_drone_1
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

## 3. Localize in a saved map

```bash
./scripts/localize.sh myroom4 orin_nano          # loads maps/myroom4.db
./scripts/rviz_vnc.sh start localization_remote (light_remote) # RViz for localization
```

Localized when `loop_closure_id` is above 0 (it took about 40 s of gentle movement last time):

```bash
./scripts/dev.sh -- 'ros2 topic echo /rtabmap/info --field loop_closure_id'
```

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
| Map stays empty / few nodes | Camera was probably still. Move slowly and keep moving. Make sure nothing else uses the camera |
| Stuck / odd behaviour after a crash | `./scripts/rviz_vnc.sh stop; ./scripts/stop.sh; ./scripts/dev.sh --stop`, then start from step 0 |
| Leftover camera / TF processes after killing a launch | stop with Ctrl-C or `./scripts/stop.sh`; they are orphaned if you kill only the wrapper |
| `ros2 run` / launch can't find a package | `./scripts/build.sh` |

## 8. What is customised on this Jetson (all inside ~/VANGUARD/mapper)

- `src/isaac_ros_common/scripts/run_dev.sh`: `NVIDIA_VISIBLE_DEVICES=all` (no CDI GPU spec on this host);
  `patches/isaac_ros_common_no_cdi.patch`, applied by `scripts/setup_workspace.sh`.
- `docker/Dockerfile.d455_mapping`: adds `ros-humble-diagnostic-updater` (RTAB-Map needs
  `libdiagnostic_updater.so` from 4.0.7) and `tigervnc-standalone-server` + `openbox`.
  `dev.sh` does not rebuild an existing image on start; after editing the Dockerfile run
  `./scripts/dev.sh --build-image`.
- Scripts added: `scripts/usb_perms.sh`, `scripts/rviz_vnc.sh`, `scripts/foxglove.sh` (optional, not used).
- Not done yet: `scripts/host_setup.sh` (it unloads `uvcvideo`). It was skipped because the VANGUARD
  container's ArUco test used the camera through `uvcvideo`; that test is retired, so running it is
  now worth trying against the camera USB drops. The host is L4T R36.5.2; the guide asks for
  R36.4.x, but it works.

## 9. Power

Shut the Jetson down before unplugging it: `sudo shutdown now`.
