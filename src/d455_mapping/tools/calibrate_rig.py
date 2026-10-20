#!/usr/bin/env python3
"""Find the camera mounting on the drone and the PX4 <-> camera time delay from the data itself.

    python3 calibrate_rig.py <export_dir> <name> <map.db> <bag.db3> [--pairs 500]

Inputs: the keyframes of a map made with scripts/replay_px4.sh (exported with
`rtabmap-export --images_id --output <name>`: depth PNGs + calibration), the map database (for the
keyframe timestamps) and the bag (PX4 odometry). The map's own optimized poses are NOT used: every
keyframe is placed with the raw PX4 pose at (keyframe time + delay) times the camera mounting
(base_link -> camera_link: x y z roll pitch yaw). For overlapping keyframe pairs the depth of one is
projected into the other; the mounting + delay that minimize that mismatch are searched (Powell).

Output: the best cam_xyz / cam_rpy / time_offset_s for scripts/replay_px4.sh, with the mismatch
before and after. Needs rotation in the data (the drone must have turned and moved around), and the
result is only as good as the scene's structure allows; check the improvement, not just the values.
"""
import argparse
import os
import sqlite3
import sys

import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation, Slerp

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from d455_mapping.px4_bag_odom import convert, load_samples  # noqa: E402

import cv2  # noqa: E402

R_LINK_OPT = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])   # optical axes in camera_link


def read_intrinsics(path):
    import re
    txt = open(path).read()
    m = re.search(r'camera_matrix:.*?data:\s*\[([^\]]+)\]', txt, re.S)
    d = [float(x) for x in m.group(1).replace('\n', ' ').split(',')]
    return d[0], d[4], d[2], d[5]


class Rig:

    def __init__(self, export_dir, name, db, bag_db3, topic='/fmu/out/vehicle_odometry'):
        self.dir, self.name = export_dir, name
        con = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
        self.stamps = {i: s for i, s in con.execute('select id, stamp from Node')}
        samples, _ = convert(load_samples(bag_db3, topic), smooth_resets=True)
        self.t = np.array([s[0] for s in samples]) / 1e9
        self.pos = np.array([s[2] for s in samples])
        self.slerp = Slerp(self.t, Rotation.from_matrix(np.array([s[1] for s in samples])))
        self.ids = sorted(i for i in self.stamps if os.path.exists(
            os.path.join(export_dir, f'{name}_depth', f'{i}.png')))
        self.ids = [i for i in self.ids if self.t[0] + 0.5 < self.stamps[i] < self.t[-1] - 0.5]
        self.k = read_intrinsics(os.path.join(export_dir, f'{name}_calib', f'{self.ids[0]}.yaml'))
        self._depth = {}

    def depth(self, i):
        if i not in self._depth:
            d = cv2.imread(os.path.join(self.dir, f'{self.name}_depth', f'{i}.png'), cv2.IMREAD_UNCHANGED)
            self._depth[i] = d.astype(np.float32) / 1000.0
        return self._depth[i]

    def cam_poses(self, ids, p):
        """p = [delay_s, roll, pitch, yaw, x, y, z]; returns {id: 4x4 T_world_optical}."""
        delay, rpy, xyz = p[0], p[1:4], p[4:7]
        tq = np.clip(np.array([self.stamps[i] for i in ids]) + delay, self.t[0], self.t[-1])
        rb = self.slerp(tq)
        pb = np.stack([np.interp(tq, self.t, self.pos[:, a]) for a in range(3)], axis=1)
        x = np.eye(4)
        x[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()      # roll, pitch, yaw (x, y, z)
        x[:3, 3] = xyz
        out = {}
        for n, i in enumerate(ids):
            tb = np.eye(4)
            tb[:3, :3], tb[:3, 3] = rb[n].as_matrix(), pb[n]
            t = tb @ x
            t[:3, :3] = t[:3, :3] @ R_LINK_OPT
            out[i] = t
        return out

    def select_pairs(self, n_pairs, seed=0):
        poses = self.cam_poses(self.ids, np.zeros(7))
        c = np.array([poses[i][:3, 3] for i in self.ids])
        f = np.array([poses[i][:3, 2] for i in self.ids])
        idarr = np.array(self.ids)
        pairs = []
        for a, i in enumerate(self.ids):
            ok = np.where((np.linalg.norm(c - c[a], axis=1) > 0.2) & (np.linalg.norm(c - c[a], axis=1) < 1.5)
                          & (f @ f[a] > 0.75) & (np.abs(idarr - i) >= 3))[0]
            pairs += [(i, self.ids[b]) for b in ok]
        rng = np.random.default_rng(seed)
        if len(pairs) > n_pairs:
            pairs = [pairs[x] for x in rng.choice(len(pairs), n_pairs, replace=False)]
        return pairs

    def mismatch(self, p, pairs, max_depth=3.0, stride=8, trunc=0.08, detail=False):
        need = sorted({i for pr in pairs for i in pr})
        poses = self.cam_poses(need, p)
        fx, fy, cx, cy = self.k
        res = []
        for i, j in pairs:
            di, dj = self.depth(i), self.depth(j)
            h, w = dj.shape
            v, u = np.mgrid[0:h:stride, 0:w:stride]
            z = dj[v, u]
            ok = (z > 0.2) & (z < max_depth)
            z, uu, vv = z[ok], u[ok], v[ok]
            if len(z) < 100:
                continue
            pj = np.stack([(uu - cx) * z / fx, (vv - cy) * z / fy, z, np.ones_like(z)])
            q = np.linalg.inv(poses[i]) @ poses[j] @ pj
            front = q[2] > 0.2
            ui = np.round(q[0][front] * fx / q[2][front] + cx).astype(int)
            vi = np.round(q[1][front] * fy / q[2][front] + cy).astype(int)
            inside = (ui >= 0) & (ui < di.shape[1]) & (vi >= 0) & (vi < di.shape[0])
            zi = q[2][front][inside]
            dz = di[vi[inside], ui[inside]]
            good = (dz > 0.2) & (dz < max_depth)
            r = np.abs(zi[good] - dz[good])
            r = r[r < 0.15]
            if len(r) >= 100:
                res.append(r)
        r = np.concatenate(res)
        if detail:
            return float(np.median(r) * 1000), float((r < 0.03).mean() * 100), len(r)
        return float(np.minimum(r, trunc).mean())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('export_dir'), ap.add_argument('name'), ap.add_argument('db'), ap.add_argument('bag_db3')
    ap.add_argument('--pairs', type=int, default=500)
    a = ap.parse_args()
    rig = Rig(a.export_dir, a.name, a.db, a.bag_db3)
    pairs = rig.select_pairs(a.pairs)
    print(f'{len(rig.ids)} keyframes, {len(pairs)} overlapping pairs')
    p0 = np.zeros(7)
    m0 = rig.mismatch(p0, pairs, detail=True)
    print(f'start (camera at the body center looking forward, delay 0): median mismatch {m0[0]:.1f} mm, '
          f'within 30 mm {m0[1]:.0f} %')
    names = ['delay_s', 'roll', 'pitch', 'yaw', 'x', 'y', 'z']
    scale = np.array([0.03, 0.05, 0.05, 0.05, 0.03, 0.03, 0.03])      # initial Powell steps

    def cost(q):
        if abs(q[0]) > 0.4 or np.any(np.abs(q[1:4]) > np.pi) or np.any(np.abs(q[4:7]) > 0.5):
            return 1.0
        return rig.mismatch(q, pairs)

    best = p0.copy()
    for rnd in range(2):
        r = minimize(cost, best, method='Powell', options={'xtol': 1e-3, 'ftol': 1e-5, 'maxiter': 6,
                                                         'direc': np.diag(scale / (1 + rnd))})
        best = r.x
        mm = rig.mismatch(best, pairs, detail=True)
        print(f'  pass {rnd + 1}: median mismatch {mm[0]:.1f} mm, within 30 mm {mm[1]:.0f} %  '
              + ', '.join(f'{n}={v:+.3f}' for n, v in zip(names, best)))
    m1 = rig.mismatch(best, pairs, detail=True)
    print(f'RESULT: median mismatch {m0[0]:.1f} -> {m1[0]:.1f} mm, within 30 mm {m0[1]:.0f} -> {m1[1]:.0f} %')
    d, rpy, xyz = best[0], best[1:4], best[4:7]
    print('replay_px4.sh arguments:')
    print(f"  cam_xyz:='{xyz[0]:.3f} {xyz[1]:.3f} {xyz[2]:.3f}' cam_rpy:='{rpy[0]:.4f} {rpy[1]:.4f} {rpy[2]:.4f}' "
          f"time_offset_s:={-d:.3f}")
    print(f'  (mounting roll/pitch/yaw in degrees: {np.degrees(rpy).round(1).tolist()}; '
          f'camera looks {np.degrees(rpy[1]):+.1f} deg pitch (+ = down), yaw {np.degrees(rpy[2]):+.1f})')


if __name__ == '__main__':
    main()
