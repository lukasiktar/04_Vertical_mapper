# USAGE: run the system, save maps, relocalize

All commands run on the **host**, in a terminal, from the project folder. The scripts start
the Docker container and run inside it, so you never need to enter it yourself.

```bash
cd ~/<PATH>/4_Vertical_mapper
```

Cheat sheet:

| Function: | Command |
|---|---|
| start the container | `./scripts/dev.sh --detach` |
| make a new map `lab` | `./scripts/map.sh lab` (stop with **Ctrl-C**, see §3) |
| check that map | `./scripts/map_info.sh lab` |
| localize in the saved map (RViz shows the whole saved cloud) | `./scripts/localize.sh lab` |
| stop everything | `./scripts/stop.sh` |
| export / view the map | `./scripts/export_map.sh lab` then `./scripts/show_map.sh lab` |
| record raw data / replay it | `./scripts/record.sh <bag>` / `./scripts/replay.sh <bag> <new_map>` |
| shut the container down | `./scripts/dev.sh --stop` |

---

## 1. Every time you start working

1. **Plug the D455 straight into a laptop USB-C port**.
2. Start the container (the first start after a reboot takes longer):
   ```bash
   ./scripts/dev.sh --detach
   ```
3. *Optional*: Check that the camera is seen (USB 3.2, serial 260722303845, firmware 5.15.1.55):
   ```bash
   ./scripts/dev.sh -- 'rs-enumerate-devices -s'
   ```
4. ROS2 rebuild workspace:
   ```bash
   ./scripts/build.sh
   ```

One-time per computer (already done on this laptop): `./scripts/host_setup.sh --no-uvcvideo`
(sudo; see README §1). If the camera gives USB errors or "Error starting device", that step is
the first thing to check.

## 2. Open a second terminal when you need one

`map.sh` and `localize.sh` keep their terminal busy. For checks, open another terminal in the
project folder and either run single commands:

```bash
./scripts/dev.sh -- 'ros2 topic hz /camera/imu'
```

or get an interactive shell inside the running container:

```bash
./scripts/dev.sh
```

## 3. Make a map

```bash
./scripts/map.sh lab          # saves to maps/lab.db; RViz opens
```

- **To overwrite** an existing map - add `--force`.
- *Optional* switches: `nvblox:=false` (no dense mesh), `rviz:=false`, e.g.
  `./scripts/map.sh lab laptop nvblox:=false rviz:=false`.
- **Depth limit:** geometry is built only from depth up to `depth_max_m` (3 m by default, see §10).
- Wait for RViz to show the camera and the cuVSLAM path before you walk. In another
  terminal you can confirm tracking is OK:
  ```bash
  ./scripts/dev.sh -- 'ros2 topic echo /visual_slam/status --once | grep vo_state'   # vo_state: 1 = tracking
  ```

**Stop and save**: press **Ctrl-C** in the mapping terminal and wait until the prompt returns** (up
to about a minute; RTAB-Map writes the database during shutdown). From another terminal you can
use `./scripts/stop.sh` instead.

**Check the result**:

```bash
./scripts/map_info.sh lab       # keyframes, RGB-D coverage, loop closures
./scripts/map_info.sh --all     # every map in maps/
```

A good map shows `global loop closures` of 1 or more and "OK: loops closed". If it says there
was no loop closure, map again and finish where you started.

*To watch loop closures live*: `./scripts/dev.sh -- 'ros2 topic echo /rtabmap/info --field loop_closure_id'`
(a number above 0 means a closure just happened).

## 4. Relocalize in a saved map

```bash
./scripts/localize.sh lab       # loads maps/lab.db; the database is not changed
```

- **RViz shows the whole saved map** as a point cloud (topic `/prior_map`). On first use it is built from the database
  (about 30 s, cached in `maps/lab/lab_prior_cloud.pcd`; rebuilt automatically when the database or
  `depth_max_m` changes) and then published **once**. Switch it off with
  `./scripts/localize.sh lab laptop --no-prior-map` (only the trajectory/graph). On the
  Jetson it is off by default; `--prior-map` turns it on.


- Check localization status from another terminal:
  ```bash
  ./scripts/dev.sh -- 'ros2 topic echo /rtabmap/info --field loop_closure_id'      # > 0 = localized
  ./scripts/dev.sh -- 'ros2 run tf2_ros tf2_echo map base_link'                    # camera pose in the map
  ./scripts/dev.sh -- 'ros2 topic echo /rtabmap/localization_pose --once'
  ```
- Stop with **Ctrl-C** or `./scripts/stop.sh`.



## 5. What to store (so you can relocalize later)

**The only file you need for relocalization is `maps/<name>.db`.** It holds the keyframes, the
loop-closed graph, and every keyframe's RGB image, depth and camera calibration (about 100 MB for
a 2.5 minute walk). It is written when mapping stops. Copy it only when nothing is running.

| What | Where | Needed for | Keep? |
|---|---|---|---|
| `maps/<name>.db` | `maps/` | relocalizing, exporting, intersection detection later | **always** |
| `maps/<name>/` (cloud, mesh, poses, keyframe images) | made by `export_map.sh` | viewing, other software | can be regenerated from the `.db` |
| `bags/<bag>/` raw recording | made by `record.sh` | re-running mapping, offline detection | keep what you may re-process; about 85 MB/s |
| `src/d455_mapping/config/*.yaml`, the repo | git | reproducing the setup | already versioned |

`maps/` and `bags/` are **not in git** (too large), so back them up yourself:

```bash
# copy one map to an external drive or another computer
cp maps/lab.db /media/$USER/backup/
scp maps/lab.db user@other-machine:~/D455/maps/        # e.g. to the Jetson

# back up all maps (they compress poorly, so plain rsync is fine)
rsync -a --info=progress2 maps/ /media/$USER/backup/maps/

# raw bags
rsync -a --info=progress2 bags/lab_raw /media/$USER/backup/bags/
```

To use a map on another machine, put the `.db` in that machine's `maps/` folder and run
`./scripts/localize.sh lab` there (on the Jetson: `./scripts/localize.sh lab orin_nano`).

## 6. Export and view a map

```bash
./scripts/export_map.sh lab                  # point cloud + mesh + poses → maps/lab/
./scripts/export_map.sh lab --with-images    # also keyframe RGB, depth, calibration (for detection later)
./scripts/show_map.sh lab                    # open it in RViz, no camera needed
```

Points farther than the depth limit (`depth_max_m`, 3 m by default) are left out of the export;
override with `--max-range 5` (or `--max-range 0` for no limit). The database always keeps the
full depth.

What is in `maps/lab/`: `lab_cloud.ply` / `.pcd` (colored points), `lab_mesh.ply` / `.dae` / `.stl`,
`lab_poses.txt` and `lab_camera_poses.txt` (one pose per keyframe id), and with `--with-images`
`lab_rgb/`, `lab_depth/`, `lab_calib/`. Add `--voxel 0.02` for smaller files.

The live dense mesh from nvblox (only while mapping or running `nvblox.launch.py`) can be saved
with the service below. Note that nvblox builds it in the session's `odom` frame, so it is **not
aligned with the saved RTAB-Map map**; use the `export_map.sh` mesh when you need map coordinates.

```bash
./scripts/dev.sh -- 'ros2 service call /nvblox_node/save_ply nvblox_msgs/srv/FilePath "{file_path: /maps/lab_nvblox.ply}"'
```

## 7. Record raw data and replay it

Use this to keep the raw sensor stream for later re-processing (including the intersection
detection) or to re-run mapping offline.

```bash
./scripts/record.sh lab_raw          # raw camera + IMU → bags/lab_raw; Ctrl-C to stop
./scripts/record.sh flight1 laptop px4:=true   # + every PX4 topic (/fmu/in/*, /fmu/out/*), same bag and clock
./scripts/record.sh flight1 orin_nano px4:=true   # same, on the Jetson
./scripts/record.sh flight1 laptop extra_regex:='/mavros/.*'   # other data (MAVROS, ...)
./scripts/replay.sh lab_raw lab_replay      # mapping from the bag, no camera → maps/lab_replay.db
./scripts/replay.sh lab_raw lab_replay laptop --rate 0.5   # slower replay on a busy machine
```

- PX4 topics need the matching `px4_msgs` package in the container (not installed yet; the camera
  topics record without it). Start the recording with a sharp shake of the drone so the camera IMU and
  PX4's IMU can be aligned in time later.
- A bag grows by about **85 MB per second** (a 2 minute walk is roughly 10 GB). Start walking
  right after the recorder says it is running, and stop it as soon as you finish.
- Bags recorded on the Jetson replay on the laptop (and the other way round).
- `replay.sh` needs a new map name each time and stops the whole stack when it finishes.

## 8. Other launches (for testing parts of the system)

Run inside the container through `./scripts/dev.sh -- '...'`, or with an interactive
`./scripts/dev.sh` shell. All of them take `platform:=laptop|orin_nano`.

| What | Command |
|---|---|
| camera + splitters only | `ros2 launch d455_mapping camera.launch.py` |
| camera + cuVSLAM (odometry only) | `ros2 launch d455_mapping vslam.launch.py rviz:=true` |
| camera + cuVSLAM + nvblox mesh | `ros2 launch d455_mapping nvblox.launch.py rviz:=true` |
| check the emitter / splitter (point the camera at something close and textured) | `ros2 run d455_mapping emitter_check --save /bags` |
| measure drift (move out and back to the start) | `ros2 run d455_mapping odom_drift_check --seconds 60` |
| measure GPU / RAM / CPU for 100 s | `./scripts/measure.sh 100 mapping` |

## 9. Stop and clean up

```bash
./scripts/stop.sh            # stops all launches; RTAB-Map saves its database first
./scripts/dev.sh --stop      # stops the container (do this after changing environment variables)
```

If something looks stuck or behaves oddly, run both and start again. Do not use `pkill -f` inside
`dev.sh -- '...'`: the pattern matches the command's own shell.

## 10. Limit the depth range (default 3 m)

One setting, `depth_max_m` at the top of `src/d455_mapping/config/laptop.yaml` (and `orin_nano.yaml`),
limits the depth used for **geometry**:

| Where | Effect |
|---|---|
| RTAB-Map | `Grid/RangeMax`: the 3D grid / cloud built from each keyframe |
| nvblox | integration range of the dense mesh |
| RViz map cloud | "Cloud max depth" of the RTAB-Map cloud display |
| `export_map.sh`, `localize.sh` saved cloud | default `--max-range` |

Why: the D455's depth gets noisy far away (here the raw topic reaches 35+ m; 1 % of the pixels are
beyond 16 m). On the `room1` map the unlimited export is **154 x 131 x 46 m wide** with stray far
points; limited to 3 m it is 12 x 10 x 4 m with 0.6 M instead of 1.8 M points.

How to change it: edit `depth_max_m` (0 = no limit), run
`python3 src/d455_mapping/tools/gen_rviz.py` (updates the RViz displays) and `./scripts/build.sh`,
then start again. Because the camera, the bags and the database keep full depth, you can change it
later and re-export or replay with another value.

Not affected: cuVSLAM (it uses the stereo IR images, not depth), and the loop-closure features.
Those have their own switch, `rtabmap.features_max_depth` (0 = off). Setting it to the same value
also drops far features, which can weaken loop closures and relocalization in big rooms, so map and
localize with the same value and check `map_info.sh` after mapping.

The camera itself cannot cut the depth here: the driver's `clip_distance` has no effect with
aligned depth (measured), and this driver build has no threshold filter. That is also why the raw
bags stay complete.

## 11. Maps from a drone bag (RGB-D camera + PX4 odometry)

For a bag recorded with the default RealSense launch plus `/fmu/out/vehicle_odometry` (sqlite3
`.db3`; no IR images or IMU needed, so cuVSLAM is not used). The bag is read in place; the PX4
odometry is decoded from the file, so `px4_msgs` is not needed. Topics are expected under
`/camera/camera/...` (override with `camera_ns:=/camera`).

```bash
# 1. first map (camera assumed to look straight forward from the PX4 body center)
./scripts/replay_px4.sh stup_indoor_1 indoor1                 # about 6 min for a 5 min bag

# 2. find the real camera mounting and time delay from the data (about 4 min)
./scripts/calibrate_rig.sh indoor1 stup_indoor_1              # prints cam_xyz / cam_rpy / time_offset_s

# 3. map again with that mounting (paste the values it printed)
./scripts/replay_px4.sh stup_indoor_1 indoor1_cal cam_xyz:='0.144 -0.021 0.023' cam_rpy:='-0.0059 0.1105 -0.0383' time_offset_s:=0.002

# 4. compare the two numerically (lower = cleaner), then look at them
./scripts/map_quality.sh --no-cloud indoor1 indoor1_cal
./scripts/export_map.sh indoor1_cal --prior && ./scripts/show_map.sh indoor1_cal
```

Why the map from a drone can look messy: PX4's odometry is accurate to centimeters locally but is
not as exact as RTAB-Map would assume, and the camera's mounting (where it sits on the drone, how
it is tilted) was unknown. `replay_px4.sh` therefore tells RTAB-Map to trust the odometry less,
refine every link between neighboring keyframes visually, and accept loop closures with fewer
matches (turn that off with `refine:=false`). `calibrate_rig.sh` finds the mounting by searching for
the values that make overlapping keyframes agree. **Check the printed mounting against the real
drone** (about how far in front of the PX4 and how much tilted); on `stup_indoor_1` it found the
camera 14 cm forward, 2 cm right, 2 cm up and pitched 6 degrees down, with a time delay of 2 ms.

Settings of `replay_px4.sh` (all optional): `--rate R`, `--start-offset S`, `--force`,
`cam_xyz:='x y z'` (m, in the PX4 body frame: x forward, y left, z up), `cam_rpy:='roll pitch yaw'`
(rad, pitch positive = looking down), `time_offset_s:=`, `smooth_resets:=true` (PX4 resets its
estimate now and then, e.g. a 0.43 m jump at 207 s in that bag; the jump is removed so the
trajectory stays continuous), `odom_lin_var:=0.05`, `odom_ang_var:=0.03`, `rviz:=false`.

To improve any existing map without replaying the bag (graph re-optimization on the stored
keyframes, about 80 s):

```bash
./scripts/refine_map.sh indoor1 indoor1_refined [--cam-tf "x y z roll pitch yaw"]
./scripts/map_quality.sh --no-cloud indoor1 indoor1_refined
```

What was measured on `stup_indoor_1` (keyframe depth mismatch, median, lower = cleaner):
as-is 52.8 mm, refinement 39.4-40.8 mm, calibrated mounting alone 39.4 mm, calibrated mounting +
refinement in the full pipeline **38.3 mm** (42 % of points within 3 cm, was 30 %). For
comparison the cuVSLAM map of a different room measured 26.1 mm. What did **not** help: ICP on scans
from depth (51.3 mm), SIFT features (43.9 mm, more loop closures but less accurate), recomputing
visual odometry from the stored keyframes (lost tracking for 38 % of them), and a second
refinement pass (no further gain). Details: docs/TEST_RESULTS.md.

## 12. Quick fixes

| Symptom | Fix |
|---|---|
| `rs-enumerate-devices` finds no camera | plug it directly into a laptop USB-C port, wait 5 s, then `./scripts/dev.sh --stop` and `./scripts/dev.sh --detach` |
| "Error starting device" / USB errors in `journalctl -k` | `./scripts/host_setup.sh --no-uvcvideo`, then replug the camera. If a USB-C port is dead after an `xHCI ... assume dead` message, reboot |
| nothing happens after `dev.sh` | `TERM=xterm ./scripts/dev.sh --detach` |
| RTAB-Map gets no input / depth missing | `emitter_ensure` normally fixes it at start; check the camera is plugged in and run `emitter_check` |
| cuVSLAM loses tracking | slower movement, more texture; avoid blank walls and glass |
| old processes confuse a new run | `./scripts/stop.sh`, then `./scripts/dev.sh --stop` and start again |
| map.sh says the map exists | choose a new name, or `--force` to overwrite |

More detail: [README.md](README.md) (setup, TF tree, topics, troubleshooting),
[docs/TEST_RESULTS.md](docs/TEST_RESULTS.md) (what was measured), and
[docs/JETSON.md](docs/JETSON.md) (Jetson Orin Nano).
