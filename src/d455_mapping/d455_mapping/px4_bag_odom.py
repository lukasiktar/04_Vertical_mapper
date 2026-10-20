"""PX4 odometry from a rosbag2 *sqlite3* bag -> TF odom -> base_link (+ nav_msgs/Odometry).

For bags with /fmu/out/vehicle_odometry (px4_msgs/VehicleOdometry, PX4 >= 1.14) when px4_msgs is not
installed: the messages are decoded straight from the bag file, converted from PX4's NED / FRD to
ROS's ENU / FLU and published while `ros2 bag play --clock` runs (use_sim_time). Samples are
stamped with the bag's log (receive) time, which is the same clock as the camera in the same bag.

    ros2 run d455_mapping px4_bag_odom --ros-args -p use_sim_time:=true \\
        -p bag_db:=/bags/<bag>/<file>.db3

Parameters: bag_db, topic (/fmu/out/vehicle_odometry), odom_frame (odom), base_frame (base_link),
time_offset_s (0.0; added to every stamp, e.g. to remove a known latency),
smooth_resets (true; see below), odom_topic (/px4/odom).

Resets: when PX4's EKF resets its estimate (reset_counter increments) the pose jumps. With
smooth_resets the jump is removed (a rigid correction is folded in so the trajectory stays
continuous, which is what a graph-SLAM odometry input needs). The jumps are logged.
"""
import math
import sqlite3
import struct

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformBroadcaster

# NED -> ENU and FRD -> FLU basis changes (R_enu_flu = A @ R_ned_frd @ B)
A_ENU_NED = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])
B_FRD_FLU = np.diag([1., -1., -1.])


def quat_wxyz_to_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def matrix_to_quat_xyzw(m):
    t = np.trace(m)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w, x, y, z = 0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w, x, y, z = (m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w, x, y, z = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w, x, y, z = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def decode_vehicle_odometry(data):
    """Decode one CDR-serialized px4_msgs/VehicleOdometry (PX4 >= 1.14, 120 bytes)."""
    if len(data) != 120:
        raise ValueError(f'VehicleOdometry message of {len(data)} bytes, expected 120 '
                         '(PX4 >= 1.14 layout); install px4_msgs matching your firmware instead')
    p = bytes(data)[4:]                       # skip the 4-byte CDR encapsulation header
    timestamp, timestamp_sample = struct.unpack_from('<QQ', p, 0)
    pose_frame = p[16]
    position = struct.unpack_from('<3f', p, 20)
    q = struct.unpack_from('<4f', p, 32)      # w, x, y, z
    velocity = struct.unpack_from('<3f', p, 52)
    position_variance = struct.unpack_from('<3f', p, 76)
    orientation_variance = struct.unpack_from('<3f', p, 88)
    reset_counter = p[112]
    return dict(timestamp=timestamp, timestamp_sample=timestamp_sample, pose_frame=pose_frame,
                position=position, q=q, velocity=velocity, position_variance=position_variance,
                orientation_variance=orientation_variance, reset_counter=reset_counter)


def ned_frd_to_enu_flu(position, q_wxyz):
    """PX4 pose (NED position, FRD->NED quaternion) -> (R_enu_flu 3x3, t_enu)."""
    r = A_ENU_NED @ quat_wxyz_to_matrix(q_wxyz) @ B_FRD_FLU
    t = A_ENU_NED @ np.asarray(position, dtype=float)
    return r, t


def load_samples(db_path, topic):
    """[(log_time_ns, decoded dict)] sorted by time; checks the layout on the first samples."""
    con = sqlite3.connect(f'file:{db_path}?mode=ro', uri=True)
    row = con.execute('select id from topics where name=?', (topic,)).fetchone()
    if row is None:
        raise ValueError(f'topic {topic} not in {db_path}')
    samples = [(ts, decode_vehicle_odometry(d)) for ts, d in con.execute(
        'select timestamp, data from messages where topic_id=? order by timestamp', (row[0],))]
    if not samples:
        raise ValueError(f'no messages on {topic}')
    for _, s in samples[:200]:
        if abs(math.sqrt(sum(x * x for x in s['q'])) - 1.0) > 1e-3 or s['pose_frame'] != 1:
            raise ValueError('unexpected VehicleOdometry content (quaternion not normalized or '
                             'pose_frame != NED): wrong px4_msgs layout?')
    return samples


def convert(samples, smooth_resets=True):
    """-> list of (log_time_ns, R 3x3, t, dict) in ENU/FLU, with resets smoothed if requested."""
    out, rc, tc = [], np.eye(3), np.zeros(3)
    last_counter, last = samples[0][1]['reset_counter'], None
    jumps = []
    for ts, s in samples:
        r, t = ned_frd_to_enu_flu(s['position'], s['q'])
        if smooth_resets and s['reset_counter'] != last_counter and last is not None:
            # size of the jump (position / heading) before the correction is applied
            jump_m = float(np.linalg.norm(last[1] - (rc @ t + tc)))
            dr = last[0].T @ (rc @ r)
            jump_deg = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(dr) - 1) / 2))))
            # new estimate starts where the previous published pose ended
            rc = last[0] @ r.T
            tc = last[1] - rc @ t
            jumps.append((ts, jump_m, jump_deg, s['reset_counter']))
        last_counter = s['reset_counter']
        r2, t2 = rc @ r, rc @ t + tc
        last = (r2, t2)
        out.append((ts, r2, t2, s))
    return out, jumps


class Px4BagOdom(Node):

    def __init__(self):
        super().__init__('px4_bag_odom')
        db = self.declare_parameter('bag_db', '').value
        topic = self.declare_parameter('topic', '/fmu/out/vehicle_odometry').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.offset_ns = int(float(self.declare_parameter('time_offset_s', 0.0).value) * 1e9)
        smooth = bool(self.declare_parameter('smooth_resets', True).value)
        odom_topic = self.declare_parameter('odom_topic', '/px4/odom').value
        self.get_logger().info(f'reading {topic} from {db} ...')
        raw = load_samples(db, topic)
        raw_resets = sorted({s['reset_counter'] for _, s in raw})
        self.samples, jumps = convert(raw, smooth_resets=smooth)
        self.get_logger().info(
            f'px4 odometry ready: {len(self.samples)} samples, '
            f'{(raw[-1][0] - raw[0][0]) / 1e9:.1f} s, reset counters {raw_resets}'
            + (f', {len(jumps)} reset(s) smoothed' if smooth else ', resets NOT smoothed'))
        for ts, jm, jd, rcn in jumps:
            self.get_logger().info(f'  reset #{rcn} at bag t={(ts - raw[0][0]) / 1e9:.1f} s: '
                                   f'jump {jm:.2f} m / {jd:.1f} deg removed')
        self.tf = TransformBroadcaster(self)
        self.pub = self.create_publisher(Odometry, odom_topic, 20)
        self.i = None
        self.create_timer(0.01, self.tick)

    def tick(self):
        now = self.get_clock().now().nanoseconds
        if now <= 0:
            return                              # /clock has not started
        if self.i is None:                      # start near the current bag time (--start-offset)
            self.i = next((k for k, s in enumerate(self.samples)
                           if s[0] + self.offset_ns >= now - 500_000_000), len(self.samples))
        while self.i < len(self.samples) and self.samples[self.i][0] + self.offset_ns <= now:
            ts, r, t, s = self.samples[self.i]
            self.i += 1
            stamp = Time(nanoseconds=ts + self.offset_ns).to_msg()
            q = matrix_to_quat_xyzw(r)
            tfm = TransformStamped()
            tfm.header.stamp, tfm.header.frame_id = stamp, self.odom_frame
            tfm.child_frame_id = self.base_frame
            tfm.transform.translation.x, tfm.transform.translation.y, tfm.transform.translation.z = map(float, t)
            (tfm.transform.rotation.x, tfm.transform.rotation.y,
             tfm.transform.rotation.z, tfm.transform.rotation.w) = map(float, q)
            self.tf.sendTransform(tfm)
            od = Odometry()
            od.header = tfm.header
            od.child_frame_id = self.base_frame
            od.pose.pose.position.x, od.pose.pose.position.y, od.pose.pose.position.z = map(float, t)
            (od.pose.pose.orientation.x, od.pose.pose.orientation.y,
             od.pose.pose.orientation.z, od.pose.pose.orientation.w) = map(float, q)
            pv = (s['position_variance'][1], s['position_variance'][0], s['position_variance'][2])
            ov = (s['orientation_variance'][1], s['orientation_variance'][0], s['orientation_variance'][2])
            for k in range(3):
                od.pose.covariance[7 * k] = float(pv[k])
                od.pose.covariance[7 * (k + 3)] = float(ov[k])
            self.pub.publish(od)


def main():
    rclpy.init()
    node = Px4BagOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
