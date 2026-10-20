"""ROS 2 node: detect planar beam crosses in RGB, verify with aligned depth, publish them.

    ros2 run d455_mapping cross_detector --ros-args --params-file <share>/config/cross_detector.yaml

(normally started by cross_detector.launch.py / mapping.launch.py with_cross_detector:=true).
The logic lives in ``cross_logic`` (no ROS); this node only wires topics, TF and visualization.

Subscribes (defaults = our camera launch; aligned depth uses the color intrinsics and frame):
    color_topic  /camera/color/image_raw
    depth_topic  /camera/aligned_depth_splitter/output/depth   (emitter ON; without the
                 splitter: /camera/aligned_depth_to_color/image_raw)
    info_topic   /camera/color/camera_info
    localization_topic  /rtabmap/localization_pose  (only with require_localization)
Publishes:
    ~/live_markers  MarkerArray  green crosses of the current frame, in the camera frame
    ~/landmarks     MarkerArray  orange confirmed landmarks "cross N", in target_frame (map)
    ~/debug_image   Image        2D candidates red, depth-verified green with distance
"""
import os
import time
from dataclasses import fields

import cv2
import message_filters
import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, Imu
from visualization_msgs.msg import Marker, MarkerArray

from d455_mapping.cross_logic import (
    Camera, DetectorParams, GeometricCandidateDetector, LandmarkMap, LandmarkParams,
    VerifyParams, depth_to_meters, is_vertical_plane, quat_to_rot, transform_cross, verify_candidate)

LIVE_RGB = (0.1, 1.0, 0.1)
LANDMARK_RGB = (1.0, 0.55, 0.0)


class CrossDetectorNode(Node):
    def __init__(self):
        super().__init__('cross_detector')
        self.dp = self._declare_dataclass(DetectorParams)
        self.vp = self._declare_dataclass(VerifyParams)
        self.lp = self._declare_dataclass(LandmarkParams)
        param = self._declare
        color_topic = param('color_topic', '/camera/color/image_raw')
        depth_topic = param('depth_topic', '/camera/aligned_depth_splitter/output/depth')
        info_topic = param('info_topic', '/camera/color/camera_info')
        self.target_frame = param('target_frame', 'map')
        self.every_n = max(1, int(param('process_every_n', 1)))
        self.sync_slop = float(param('sync_slop_s', 0.04))
        self.tf_fallback_latest = bool(param('tf_fallback_latest', True))
        self.require_loc = bool(param('require_localization', False))
        loc_topic = param('localization_topic', '/rtabmap/localization_pose')
        self.loc_timeout = float(param('localization_timeout_s', 30.0))
        self.landmarks_file = param('landmarks_file', '')
        self.save_period = float(param('save_period_s', 10.0))
        self.marker_arm = float(param('marker_arm_m', 0.3))
        self.publish_hz = float(param('landmarks_publish_hz', 2.0))
        start_empty = bool(param('start_empty', False))
        self.vertical_tol = float(param('vertical_tol_deg', 20.0))   # 0 = off
        imu_topic = param('imu_topic', '/imu/data')
        self.imu_fallback_frame = param('imu_fallback_frame', 'camera_gyro_optical_frame')

        self.detector = GeometricCandidateDetector(self.dp)   # swap for a learned detector here
        self.rng = np.random.default_rng(0)
        self.bridge = CvBridge()
        self.cam = None
        self.frame_count = 0
        self.last_loc = None
        self.imu = None            # (orientation quaternion world<-imu, imu frame id)
        self.stats = {'frames': 0, 'detect': 0.0, 'verify': 0.0, 'total': 0.0, 'crosses': 0}

        self.landmarks = LandmarkMap(self.lp, self.target_frame)
        if start_empty and self.landmarks_file and os.path.exists(self.landmarks_file):
            os.replace(self.landmarks_file, self.landmarks_file + '.bak')   # belongs to the old map
            self.get_logger().warn(f'new map: previous {self.landmarks_file} moved to .bak')
        try:
            n = self.landmarks.load(self.landmarks_file)
        except ValueError as e:
            n = 0
            self.get_logger().error(str(e))
        self.get_logger().info(
            f'{n} landmark(s) loaded from {self.landmarks_file or "(no file)"}; depth {depth_topic}; '
            f'fusion into "{self.target_frame}"'
            + (' only while localized' if self.require_loc else ''))

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)

        self.create_subscription(CameraInfo, info_topic, self._on_info, qos_profile_sensor_data)
        if self.vertical_tol > 0:
            self.create_subscription(Imu, imu_topic, self._on_imu, qos_profile_sensor_data)
        if self.require_loc:
            self.create_subscription(PoseWithCovarianceStamped, loc_topic, self._on_loc, 10)
        subs = [message_filters.Subscriber(self, Image, color_topic, qos_profile=qos_profile_sensor_data),
                message_filters.Subscriber(self, Image, depth_topic, qos_profile=qos_profile_sensor_data)]
        self.sync = message_filters.ApproximateTimeSynchronizer(subs, queue_size=5, slop=self.sync_slop)
        self.sync.registerCallback(self._on_frames)

        self.live_pub = self.create_publisher(MarkerArray, '~/live_markers', 5)
        self.lm_pub = self.create_publisher(MarkerArray, '~/landmarks', 5)
        self.debug_pub = self.create_publisher(Image, '~/debug_image', 2)
        self.create_timer(1.0 / max(self.publish_hz, 0.1), self._on_slow_timer)
        if self.save_period > 0:
            self.create_timer(self.save_period, self._autosave)
        self.create_timer(10.0, self._log_stats)

    # -- parameters ---------------------------------------------------------------------------
    def _declare(self, name, default):
        # dynamic typing: YAML "4" for a double parameter is fine
        return self.declare_parameter(
            name, default, ParameterDescriptor(dynamic_typing=True)).value

    def _declare_dataclass(self, cls):
        return cls(**{f.name: type(f.default)(self._declare(f.name, f.default)) for f in fields(cls)})

    # -- inputs -------------------------------------------------------------------------------
    def _on_info(self, msg):
        k = msg.k
        self.cam = Camera(k[0], k[4], k[2], k[5])

    def _on_imu(self, msg):
        q = msg.orientation
        if q.x or q.y or q.z or q.w:
            self.imu = (quat_to_rot(q.x, q.y, q.z, q.w), msg.header.frame_id)

    def _up_in_frame(self, frame):
        """Unit 'up' (opposite to gravity) in ``frame``, from the Madgwick-filtered IMU, or None."""
        if self.imu is None:
            return None
        R_world_imu, imu_frame = self.imu
        tf = None
        # the united RealSense IMU stream is labelled camera_imu_optical_frame, which has no TF; its
        # data are in the gyro frame
        for name in (imu_frame, self.imu_fallback_frame):
            try:
                tf = self.tf_buffer.lookup_transform(frame, name, Time())
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                continue
        if tf is None:
            return None
        q = tf.transform.rotation
        return quat_to_rot(q.x, q.y, q.z, q.w) @ (R_world_imu.T @ np.array([0.0, 0.0, 1.0]))

    def _on_loc(self, _msg):
        self.last_loc = self._now()

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _fusion_allowed(self, now):
        if not self.require_loc:
            return True
        ok = self.last_loc is not None and now - self.last_loc <= self.loc_timeout
        if not ok:
            self.get_logger().info('not localized yet: detections are shown but not fused',
                                   throttle_duration_sec=10.0)
        return ok

    def _lookup(self, source_frame, stamp):
        """(R, T) of source_frame in target_frame at the image stamp, else latest, else None."""
        for when in (Time.from_msg(stamp), Time()) if self.tf_fallback_latest else (Time.from_msg(stamp),):
            try:
                tf = self.tf_buffer.lookup_transform(self.target_frame, source_frame, when,
                                                     timeout=Duration(seconds=0.05))
                q, t = tf.transform.rotation, tf.transform.translation
                return quat_to_rot(q.x, q.y, q.z, q.w), np.array([t.x, t.y, t.z])
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as e:
                err = e
        self.get_logger().warn(f'no TF {source_frame} -> {self.target_frame}: {err}',
                               throttle_duration_sec=5.0)
        return None

    # -- main callback ------------------------------------------------------------------------
    def _on_frames(self, color_msg, depth_msg):
        self.frame_count += 1
        if self.frame_count % self.every_n:
            return
        if self.cam is None:
            self.get_logger().warn('waiting for camera_info', throttle_duration_sec=5.0)
            return
        t0 = time.perf_counter()
        img = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
        depth = depth_to_meters(self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough'))
        if depth.shape != img.shape[:2]:
            self.get_logger().error(f'depth {depth.shape} and color {img.shape[:2]} sizes differ; '
                                    'use the aligned depth', throttle_duration_sec=5.0)
            return
        cands = self.detector.detect(img)
        t1 = time.perf_counter()
        results = [(c, *verify_candidate(c, depth, self.cam, self.vp, self.rng)) for c in cands]
        results = self._vertical_filter(results, color_msg.header.frame_id)
        crosses = [r[1] for r in results if r[1] is not None]
        t2 = time.perf_counter()

        frame = color_msg.header.frame_id
        self.live_pub.publish(self._live_markers(crosses, frame, color_msg.header.stamp))

        if crosses:
            now = self._now()
            if self._fusion_allowed(now):
                tf = self._lookup(frame, color_msg.header.stamp)
                if tf is not None:
                    self.landmarks.update([transform_cross(c, *tf) for c in crosses], now)
        if self.debug_pub.get_subscription_count() > 0:
            msg = self.bridge.cv2_to_imgmsg(self._debug_image(img, results), 'bgr8')
            msg.header = color_msg.header
            self.debug_pub.publish(msg)

        s = self.stats
        s['frames'] += 1
        s['detect'] += t1 - t0
        s['verify'] += t2 - t1
        s['total'] += time.perf_counter() - t0
        s['crosses'] += len(crosses)

    def _vertical_filter(self, results, frame):
        """The crosses are on vertical surfaces: their plane normal must be horizontal."""
        if self.vertical_tol <= 0 or not any(r[1] is not None for r in results):
            return results
        up = self._up_in_frame(frame)
        if up is None:
            self.get_logger().warn('no IMU gravity yet: vertical-plane filter inactive',
                                   throttle_duration_sec=10.0)
            return results
        return [(c, x, why) if x is None or is_vertical_plane(x.normal, up, self.vertical_tol)
                else (c, None, 'not_vertical') for c, x, why in results]

    # -- timers -------------------------------------------------------------------------------
    def _on_slow_timer(self):
        self.landmarks.prune(self._now())
        self.lm_pub.publish(self._landmark_markers())

    def _autosave(self):
        if self.landmarks_file and self.landmarks.dirty:
            n = self.landmarks.save(self.landmarks_file)
            self.get_logger().info(f'saved {n} cross(es) to {self.landmarks_file}')

    def _log_stats(self):
        s = self.stats
        if s['frames']:
            n = s['frames']
            self.get_logger().info(
                f'{n / 10.0:.1f} frames/s processed | per frame: detect {1e3 * s["detect"] / n:.1f} ms, '
                f'verify {1e3 * s["verify"] / n:.1f} ms, total {1e3 * s["total"] / n:.1f} ms | '
                f'{s["crosses"]} verified detection(s) | {len(self.landmarks.confirmed())} landmark(s)')
        if self.vertical_tol > 0 and self.cam is not None:
            up = self._up_in_frame('camera_color_optical_frame')
            self.get_logger().info('gravity "up" in the color optical frame (x right, y down, z forward): '
                                   + ('not available' if up is None else
                                      f'[{up[0]:+.2f} {up[1]:+.2f} {up[2]:+.2f}]  (level camera: [0 -1 0])'))
        self.stats = {k: 0 if k in ('frames', 'crosses') else 0.0 for k in s}

    def shutdown(self):
        if self.landmarks_file:
            n = self.landmarks.save(self.landmarks_file)
            self.get_logger().info(f'saved {n} cross(es) to {self.landmarks_file}')

    # -- visualization ------------------------------------------------------------------------
    def _cross_markers(self, c, frame, stamp, ns, idx, rgb, lifetime=0.0, label=None):
        out = []
        lines = Marker()
        lines.ns, lines.id, lines.type = ns + '_arms', idx, Marker.LINE_LIST
        lines.scale.x = 0.02
        for a in c.arms:
            for p in (c.center + self.marker_arm * a, c.center - self.marker_arm * a):
                lines.points.append(Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))
        ball = Marker()
        ball.ns, ball.id, ball.type = ns + '_center', idx, Marker.SPHERE
        ball.pose.position = Point(x=float(c.center[0]), y=float(c.center[1]), z=float(c.center[2]))
        ball.scale.x = ball.scale.y = ball.scale.z = 0.06
        out += [lines, ball]
        if label:
            text = Marker()
            text.ns, text.id, text.type = ns + '_label', idx, Marker.TEXT_VIEW_FACING
            text.pose.position = Point(x=float(c.center[0]), y=float(c.center[1]),
                                       z=float(c.center[2]) + self.marker_arm + 0.1)
            text.scale.z = 0.12
            text.text = label
            out.append(text)
        for m in out:
            m.header.frame_id, m.header.stamp = frame, stamp
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.color.r, m.color.g, m.color.b, m.color.a = (*rgb, 1.0)
            if m.type == Marker.TEXT_VIEW_FACING:
                m.color.r = m.color.g = m.color.b = 1.0
            if lifetime > 0:
                m.lifetime = Duration(seconds=lifetime).to_msg()
        return out

    def _live_markers(self, crosses, frame, stamp):
        arr = MarkerArray()
        for i, c in enumerate(crosses):   # short lifetime: gone when no longer detected
            arr.markers += self._cross_markers(c, frame, stamp, 'live', i, LIVE_RGB, lifetime=0.5)
        return arr

    def _landmark_markers(self):
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for lm in self.landmarks.confirmed():
            c = type('C', (), {'center': lm.center, 'arms': lm.arms})
            arr.markers += self._cross_markers(c, self.target_frame, stamp, 'landmark', lm.id,
                                               LANDMARK_RGB, label=f'cross {lm.id} ({lm.count} obs)')
        return arr

    @staticmethod
    def _debug_image(img, results):
        vis = img.copy()
        for cand, cross, reason in results:
            pt = cand.pt
            col = (0, 255, 0) if cross is not None else (0, 0, 255)
            for d in cand.dirs:   # 2D candidates red, verified ones green
                a, b = (pt + cand.arm * d).astype(int), (pt - cand.arm * d).astype(int)
                cv2.line(vis, tuple(int(v) for v in a), tuple(int(v) for v in b), col, 2 if cross is not None else 1)
            at = tuple(int(v) for v in pt)
            if cross is not None:
                cv2.circle(vis, at, 10, col, 2)
                cv2.putText(vis, f'{cross.distance:.2f} m  w {100 * cross.beam_width_m:.0f} cm  '
                            f'{cross.beam_angle_deg:.0f} deg', (at[0] + 12, at[1] - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)
            else:
                cv2.putText(vis, reason, (at[0] + 8, at[1] + 14),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1)
        return vis


def main():
    rclpy.init()
    node = CrossDetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
