#!/usr/bin/env python3
"""How 'clean' is a map cloud? Measures the thickness of locally flat patches.

    python3 map_thickness.py cloud.pcd [--radius 0.06] [--samples 40000]

For random points with enough neighbours within --radius, fit a plane (PCA) and report the
standard deviation of the points around it. Double or smeared surfaces caused by odometry errors
add spread, so LOWER is cleaner. The mean over ALL patches is the headline number (a filter on
flat patches would hide the messy ones); the flat share and the thickness of flat patches are
extra. Only relative comparisons between maps of the SAME scene are meaningful. Binary PCD only
(scripts/export_map.sh writes those).
"""
import argparse
import sys

import numpy as np
from scipy.spatial import cKDTree

TYPES = {('F', 4): np.float32, ('F', 8): np.float64, ('U', 1): np.uint8, ('U', 2): np.uint16,
         ('U', 4): np.uint32, ('I', 1): np.int8, ('I', 2): np.int16, ('I', 4): np.int32}


def read_xyz(path):
    with open(path, 'rb') as f:
        h = {}
        while True:
            k, _, v = f.readline().decode().strip().partition(' ')
            h[k.upper()] = v
            if k.upper() == 'DATA':
                break
        assert h['DATA'].strip() == 'binary', 'binary PCD needed'
        dt = np.dtype([(n, TYPES[(t, int(s))]) for n, t, s in
                       zip(h['FIELDS'].split(), h['TYPE'].split(), h['SIZE'].split())])
        d = np.frombuffer(f.read(), dtype=dt, count=int(h['POINTS']))
    xyz = np.c_[d['x'], d['y'], d['z']].astype(np.float64)
    return xyz[np.isfinite(xyz).all(axis=1)]


def patch_stats(xyz, radius=0.10, samples=100000, min_neighbors=40, seed=0):
    """-> (spread, flat) per well-sampled patch: spread = std of the points around the best plane
    [m], flat = patch is locally planar (smallest eigenvalue < 10 % of the middle one)."""
    tree = cKDTree(xyz)
    idx = np.random.default_rng(seed).choice(len(xyz), min(samples, len(xyz)), replace=False)
    spread, flat = [], []
    for i, nb in zip(idx, tree.query_ball_point(xyz[idx], radius)):
        if len(nb) < min_neighbors:
            continue
        p = xyz[nb] - xyz[nb].mean(0)
        w = np.linalg.eigvalsh(p.T @ p / len(nb))
        spread.append(np.sqrt(max(w[0], 0.0)))
        flat.append(w[0] < 0.1 * w[1])
    return np.array(spread), np.array(flat)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('pcd')
    ap.add_argument('--radius', type=float, default=0.10)
    ap.add_argument('--samples', type=int, default=100000)
    a = ap.parse_args()
    xyz = read_xyz(a.pcd)
    sp, fl = patch_stats(xyz, a.radius, a.samples)
    if len(sp) == 0:
        sys.exit('no well-sampled patches')
    sp = sp * 1000.0
    print(f'{a.pcd}: {len(xyz):,} points, {len(sp):,} patches (r={a.radius} m); '
          f'spread around local plane [mm]: mean {sp.mean():.1f}, median {np.median(sp):.1f}, '
          f'90th pct {np.percentile(sp, 90):.1f}; flat patches {100 * fl.mean():.0f} %, '
          f'thickness of those: median {np.median(sp[fl]) if fl.any() else float("nan"):.1f} mm')
