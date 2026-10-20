#!/usr/bin/env python3
"""Do the keyframes of a map agree with each other?  (lower mismatch = cleaner map)

    python3 map_consistency.py <export_dir> <name> [--pairs 1500] [--max-depth 3.0]

<export_dir> holds what `rtabmap-export --poses_camera --images_id --output <name>` writes
(<name>_camera_poses.txt, <name>_depth/<id>.png, <name>_calib/<id>.yaml). For pairs of keyframes
that look at the same place, the depth image of keyframe j is projected into keyframe i with the
MAP's poses and compared with i's own depth image. If the poses were perfect, both would agree up
to depth noise; odometry errors show up as a larger depth mismatch (double / smeared surfaces).
Reports the median absolute mismatch [mm], the share of points within 3 cm, and the number of
points compared. Only compare maps of the SAME scene made from the same keyframes.
"""
import argparse
import glob
import os
import re

import cv2
import numpy as np


def quat_to_rot(qx, qy, qz, qw):
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])


def load(export_dir, name):
    poses = {}
    for line in open(os.path.join(export_dir, f'{name}_camera_poses.txt')):
        if line.startswith('#') or not line.strip():
            continue
        v = line.split()
        t = np.eye(4)
        t[:3, :3] = quat_to_rot(*[float(x) for x in v[4:8]])
        t[:3, 3] = [float(x) for x in v[1:4]]
        poses[int(v[8])] = t
    return poses


def intrinsics(path):
    txt = open(path).read()
    m = re.search(r'camera_matrix:.*?data:\s*\[([^\]]+)\]', txt, re.S)
    d = [float(x) for x in m.group(1).replace('\n', ' ').split(',')]
    return d[0], d[4], d[2], d[5]            # fx fy cx cy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('export_dir'), ap.add_argument('name')
    ap.add_argument('--pairs', type=int, default=1500)
    ap.add_argument('--max-depth', type=float, default=3.0)
    ap.add_argument('--stride', type=int, default=6)
    ap.add_argument('--windows', nargs=2, metavar=('MAP_DB', 'BAG_DB3'),
                    help='also print the mismatch per 30 s window with the PX4 angular rate and speed')
    a = ap.parse_args()
    poses = load(a.export_dir, a.name)
    ids = sorted(poses)
    cache = {}

    def depth(i):
        if i not in cache:
            d = cv2.imread(os.path.join(a.export_dir, f'{a.name}_depth', f'{i}.png'), cv2.IMREAD_UNCHANGED)
            cache[i] = None if d is None else d.astype(np.float32) / 1000.0
        return cache[i]

    k = intrinsics(os.path.join(a.export_dir, f'{a.name}_calib', f'{ids[0]}.yaml'))
    centers = np.array([poses[i][:3, 3] for i in ids])
    fwd = np.array([poses[i][:3, 2] for i in ids])        # optical z axis in the map
    rng = np.random.default_rng(0)
    pairs = []
    for ai, i in enumerate(ids):
        d = np.linalg.norm(centers - centers[ai], axis=1)
        cos = fwd @ fwd[ai]
        for bi in np.where((d > 0.25) & (d < 1.5) & (cos > 0.8) & (np.abs(np.array(ids) - i) >= 3))[0]:
            pairs.append((i, ids[bi]))
    if len(pairs) > a.pairs:
        pairs = [pairs[x] for x in rng.choice(len(pairs), a.pairs, replace=False)]
    fx, fy, cx, cy = k
    res_all, per_pair, node_res = [], [], {}
    for i, j in pairs:
        di, dj = depth(i), depth(j)
        if di is None or dj is None:
            continue
        h, w = dj.shape
        v, u = np.mgrid[0:h:a.stride, 0:w:a.stride]
        z = dj[v, u]
        ok = (z > 0.2) & (z < a.max_depth)
        z, uu, vv = z[ok], u[ok], v[ok]
        if len(z) < 200:
            continue
        pj = np.stack([(uu - cx) * z / fx, (vv - cy) * z / fy, z, np.ones_like(z)])
        pi_ = np.linalg.inv(poses[i]) @ poses[j] @ pj
        zi = pi_[2]
        front = zi > 0.2
        ui = np.round(pi_[0][front] * fx / zi[front] + cx).astype(int)
        vi = np.round(pi_[1][front] * fy / zi[front] + cy).astype(int)
        inside = (ui >= 0) & (ui < di.shape[1]) & (vi >= 0) & (vi < di.shape[0])
        zi_f = zi[front][inside]
        dz = di[vi[inside], ui[inside]]
        good = (dz > 0.2) & (dz < a.max_depth)
        r = np.abs(zi_f[good] - dz[good])
        r = r[r < 0.15]                                   # drop occlusions / gross outliers
        if len(r) >= 200:
            res_all.append(r)
            per_pair.append(np.median(r))
            node_res.setdefault(i, []).append(np.median(r))
            node_res.setdefault(j, []).append(np.median(r))
    if not res_all:
        raise SystemExit('no usable keyframe pairs')
    r = np.concatenate(res_all) * 1000.0
    if a.windows:
        windows(a, node_res)
    print(f'{a.name}: {len(per_pair)} keyframe pairs, {len(r):,} points compared | depth mismatch [mm]: '
          f'median {np.median(r):.1f}, mean {r.mean():.1f}, 90th pct {np.percentile(r, 90):.1f}, '
          f'within 30 mm: {100 * (r < 30).mean():.0f} %, median of pair medians {np.median(per_pair) * 1000:.1f}')


def windows(a, node_res):
    import sqlite3
    import sys
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
    from d455_mapping.px4_bag_odom import convert, load_samples
    from scipy.spatial.transform import Rotation
    con = sqlite3.connect(f'file:{a.windows[0]}?mode=ro', uri=True)
    stamps = {i: t for i, t in con.execute('select id, stamp from Node')}
    smp, _ = convert(load_samples(a.windows[1], '/fmu/out/vehicle_odometry'))
    t = np.array([x[0] for x in smp]) / 1e9
    rots = Rotation.from_matrix(np.array([x[1] for x in smp]))
    pos = np.array([x[2] for x in smp])
    k = 10                                                       # 0.1 s at 100 Hz
    w = np.degrees((rots[:-k].inv() * rots[k:]).magnitude()) / (t[k:] - t[:-k])
    sp = np.linalg.norm(pos[k:] - pos[:-k], axis=1) / (t[k:] - t[:-k])
    tm = (t[:-k] + t[k:]) / 2
    t0 = min(stamps[i] for i in node_res)
    print('window[s]  keyframes  median mismatch[mm]  PX4 rotation [deg/s]  speed [m/s]')
    rows = []
    for lo in np.arange(0, max(stamps[i] for i in node_res) - t0 + 1, 30.0):
        ids = [i for i in node_res if lo <= stamps[i] - t0 < lo + 30]
        if len(ids) < 3:
            continue
        sel = (tm >= t0 + lo) & (tm < t0 + lo + 30)
        m = np.median([np.median(node_res[i]) for i in ids]) * 1000
        rows.append((lo, len(ids), m, w[sel].mean(), sp[sel].mean()))
        print(f'{lo:5.0f}-{lo + 30:<4.0f}  {len(ids):6d}  {m:15.1f}  {w[sel].mean():16.1f}  {sp[sel].mean():14.2f}')
    r = np.array(rows)
    if len(r) > 3:
        print(f'correlation of window mismatch with rotation rate {np.corrcoef(r[:, 2], r[:, 3])[0, 1]:+.2f}, '
              f'with speed {np.corrcoef(r[:, 2], r[:, 4])[0, 1]:+.2f}')


if __name__ == '__main__':
    main()
