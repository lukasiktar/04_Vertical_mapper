"""Verify the emitter on/off interleave end to end.

    ros2 run d455_mapping emitter_check [--seconds 4] [--save /bags]

Needs camera.launch.py (or any launch with the camera + splitter) running and something
textured within ~1.5 m in the lower-left of the IR image (projector dots only show up close).

Checks:
  1. raw IR1 frames alternate frame_emitter_mode 0/1 in the metadata,
  2. the splitter's infra_1 output (cuVSLAM input) contains exactly one emitter phase,
  3. end to end, independent of labels: the frames the splitter passes to cuVSLAM must have
     LESS projector-dot energy than the frames it drops. This catches firmware that reports
     the emitter flag inverted (D455 FW 5.15.1.55 + librealsense 2.55.1) and verifies the
     camera.invert_emitter_mode setting.
"""
import argparse
import json
import os
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from realsense2_camera_msgs.msg import Metadata
from sensor_msgs.msg import Image

RAW = '/camera/infra1/image_rect_raw'
META = '/camera/infra1/metadata'
SPLIT = '/camera/realsense_splitter_node/output/infra_1'


def stamp(msg):
    return msg.header.stamp.sec, msg.header.stamp.nanosec


def dot_energy(img, roi):
    y0, y1, x0, x1 = roi
    f = img[y0:y1, x0:x1].astype(np.float32)
    return float(np.mean(np.abs(f - cv2.GaussianBlur(f, (0, 0), 1.0))))


class Collector(Node):

    def __init__(self):
        super().__init__('emitter_check')
        self.meta, self.raw, self.split = {}, [], []
        self.create_subscription(Metadata, META, self._meta, 100)
        self.create_subscription(Image, RAW, self.raw.append, qos_profile_sensor_data)
        self.create_subscription(Image, SPLIT, self.split.append, qos_profile_sensor_data)

    def _meta(self, msg):
        self.meta[stamp(msg)] = json.loads(msg.json_data).get('frame_emitter_mode')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=4.0)
    ap.add_argument('--save', default='', help='directory for ir_emitter_{on,off}.png')
    ap.add_argument('--roi', default='', help='y0,y1,x0,x1 region with near texture '
                    '(default: lower-left sixth of the image)')
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = Collector()
    end = time.time() + args.seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    bridge = CvBridge()

    passed_stamps = {stamp(m) for m in node.split}
    frames = {0: [], 1: []}          # raw IR1 by metadata label
    passed, dropped = [], []         # raw IR1 by what the splitter did with it
    for msg in node.raw:
        img = bridge.imgmsg_to_cv2(msg, 'mono8')
        mode = node.meta.get(stamp(msg))
        if mode in (0, 1):
            frames[mode].append(img)
        (passed if stamp(msg) in passed_stamps else dropped).append(img)
    split_labels = sorted({node.meta.get(stamp(m)) for m in node.split}, key=str)
    ok = True
    inconclusive = False

    print(f'raw IR1 frames by metadata label: 0={len(frames[0])} 1={len(frames[1])}')
    if not frames[0] or not frames[1]:
        print('FAIL: no alternation (is depth_module.emitter_on_off enabled?)')
        ok = False
    print(f'splitter infra_1 (cuVSLAM input): {len(node.split)} frames, raw labels {split_labels}')
    if not node.split or len(split_labels) != 1:
        print('FAIL: splitter must pass exactly one emitter phase')
        ok = False

    if passed and dropped:
        h, w = passed[0].shape
        roi = tuple(int(v) for v in args.roi.split(',')) if args.roi else \
            (h * 5 // 6, h, 0, w // 3)
        e_pass = np.mean([dot_energy(f, roi) for f in passed])
        e_drop = np.mean([dot_energy(f, roi) for f in dropped])
        print(f'dot energy in roi {roi}: passed to cuVSLAM={e_pass:.2f}  dropped={e_drop:.2f}')
        if e_drop > 1.3 * e_pass:
            print('OK: cuVSLAM gets the dot-free (emitter OFF) frames')
        elif e_pass > 1.3 * e_drop:
            print('FAIL: cuVSLAM gets the frames WITH projector dots. Toggle '
                  'camera.invert_emitter_mode in config/<platform>.yaml')
            ok = False
        else:
            print('INCONCLUSIVE: no visible dots in the roi; point the lower-left of the '
                  'camera at a textured surface ~0.5-1.5 m away, or pass --roi y0,y1,x0,x1')
            inconclusive = True
        if args.save:
            os.makedirs(args.save, exist_ok=True)
            cv2.imwrite(os.path.join(args.save, 'ir_emitter_off.png'), passed[-1])
            cv2.imwrite(os.path.join(args.save, 'ir_emitter_on.png'), dropped[-1])
            print(f'saved {args.save}/ir_emitter_off.png (cuVSLAM input) and ir_emitter_on.png')
    elif ok:
        print('INCONCLUSIVE: no splitter output to compare')
        inconclusive = True

    result = 'FAIL' if not ok else ('INCONCLUSIVE' if inconclusive else 'PASS')
    print('RESULT:', result)
    node.destroy_node()
    rclpy.try_shutdown()
    raise SystemExit({'PASS': 0, 'FAIL': 1, 'INCONCLUSIVE': 2}[result])
