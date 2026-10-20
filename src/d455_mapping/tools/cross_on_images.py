#!/usr/bin/env python3
"""Run the cross detector's 2D stage on still images and write overlays (no ROS, no camera).

    python3 src/d455_mapping/tools/cross_on_images.py bags/_rgb_room1_*.png --out /tmp/cross_out
    python3 src/d455_mapping/tools/cross_on_images.py img.png --set beam_contrast=6 min_arm_px=30

Red lines = the two beam directions of each candidate, yellow circle = the crossing.
`--set name=value` overrides any DetectorParams field (same names as the ROS parameters).
Also writes a contact sheet `sheet.jpg` when several images are given.
"""
import argparse
import os
import sys
import time
from dataclasses import fields

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from d455_mapping.cross_logic import DetectorParams, GeometricCandidateDetector  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('images', nargs='+')
    ap.add_argument('--out', default='cross_out')
    ap.add_argument('--set', nargs='*', default=[], metavar='name=value')
    a = ap.parse_args()
    types = {f.name: type(f.default) for f in fields(DetectorParams)}
    kw = {}
    for item in a.set:
        k, v = item.split('=')
        if k not in types:
            sys.exit(f'unknown parameter {k}; valid: {", ".join(types)}')
        kw[k] = types[k](v)
    det = GeometricCandidateDetector(DetectorParams(**kw))
    os.makedirs(a.out, exist_ok=True)
    tiles = []
    for f in a.images:
        img = cv2.imread(f)
        if img is None:
            print('cannot read', f)
            continue
        t0 = time.perf_counter()
        cands = det.detect(img)
        ms = 1e3 * (time.perf_counter() - t0)
        vis = img.copy()
        for c in cands:
            for d in c.dirs:
                p, q = (c.pt + c.arm * d).astype(int), (c.pt - c.arm * d).astype(int)
                cv2.line(vis, tuple(map(int, p)), tuple(map(int, q)), (0, 0, 255), 2)
            cv2.circle(vis, tuple(map(int, c.pt)), 7, (0, 255, 255), 2)
        name = os.path.basename(f)
        cv2.putText(vis, f'{name[:28]} cands={len(cands)}', (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.imwrite(os.path.join(a.out, name), vis)
        tiles.append(cv2.resize(vis, (640, 480)))
        print(f'{name}: {len(cands)} candidate(s), {ms:.0f} ms')
    if len(tiles) > 1:
        while len(tiles) % 3:
            tiles.append(np.zeros_like(tiles[0]))
        sheet = np.vstack([np.hstack(tiles[i:i + 3]) for i in range(0, len(tiles), 3)])
        cv2.imwrite(os.path.join(a.out, 'sheet.jpg'), sheet, [cv2.IMWRITE_JPEG_QUALITY, 85])
    print('overlays in', a.out)


if __name__ == '__main__':
    main()
