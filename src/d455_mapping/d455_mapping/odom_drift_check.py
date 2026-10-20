"""Measure cuVSLAM odometry drift for a there-and-back motion (Phase 4 test 2).

    ros2 run d455_mapping odom_drift_check [--seconds 60]

Start it with the camera at rest, move the camera (e.g. 2 m forward and back to the exact
start pose), hold still at the end. Ctrl-C or --seconds ends the recording. Reports path
length, furthest distance from the start, end-point error (absolute and % of path) and
tracking losses from /visual_slam/status.
"""
import argparse
import math
import time

import rclpy
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from nav_msgs.msg import Odometry
from rclpy.node import Node

ODOM = '/visual_slam/tracking/odometry'
STATUS = '/visual_slam/status'


class Recorder(Node):

    def __init__(self):
        super().__init__('odom_drift_check')
        self.poses = []          # (t, x, y, z)
        self.lost = 0
        self.status_msgs = 0
        self.create_subscription(Odometry, ODOM, self._odom, 50)
        self.create_subscription(VisualSlamStatus, STATUS, self._status, 50)

    def _odom(self, msg):
        p = msg.pose.pose.position
        self.poses.append((time.time(), p.x, p.y, p.z))

    def _status(self, msg):
        self.status_msgs += 1
        if msg.vo_state != 1:          # 1 = tracking (VisualSlamStatus.VO_STATE_SUCCESS)
            self.lost += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=60.0)
    args, _ = ap.parse_known_args()
    rclpy.init()
    node = Recorder()
    print(f'recording {ODOM} for up to {args.seconds:.0f} s - move the camera now '
          '(Ctrl-C to stop early)', flush=True)
    end = time.time() + args.seconds
    last_report = 0.0
    try:
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.poses and time.time() - last_report > 2.0:
                t, x, y, z = node.poses[-1]
                x0, y0, z0 = node.poses[0][1:]
                d = math.dist((x, y, z), (x0, y0, z0))
                print(f'  {t - node.poses[0][0]:5.1f} s  distance from start {d:5.2f} m',
                      flush=True)
                last_report = time.time()
    except KeyboardInterrupt:
        pass
    p = node.poses
    if len(p) < 2:
        print(f'FAIL: no odometry on {ODOM}')
        raise SystemExit(1)
    start, final = p[0][1:], p[-1][1:]
    path = sum(math.dist(a[1:], b[1:]) for a, b in zip(p, p[1:]))
    far = max(math.dist(q[1:], start) for q in p)
    err = math.dist(final, start)
    rate = (len(p) - 1) / (p[-1][0] - p[0][0])
    print(f'samples {len(p)} ({rate:.1f} Hz), duration {p[-1][0] - p[0][0]:.1f} s')
    print(f'path length          {path:6.3f} m')
    print(f'furthest from start  {far:6.3f} m')
    print(f'end-point error      {err:6.3f} m  ({100.0 * err / path if path > 0 else 0:.2f} % of path)')
    print(f'final offset xyz     [{final[0] - start[0]:+.3f}, {final[1] - start[1]:+.3f}, '
          f'{final[2] - start[2]:+.3f}] m (odom frame)')
    print(f'tracking-loss status messages: {node.lost} of {node.status_msgs}')
    node.destroy_node()
    rclpy.try_shutdown()
