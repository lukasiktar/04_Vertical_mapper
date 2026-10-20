"""ROS 2 node: pin detected crosses from RViz, draw 3D lines from the camera to the pins.

    ros2 run d455_mapping pins --ros-args -p pins_file:=/maps/lab_pins.json \
        -p crosses_file:=/maps/lab_crosses.json -p require_localization:=true

(normally started by mapping.launch.py / localization.launch.py with_pins:=true).

Select: RViz tool "Publish Point" (toolbar, shortcut P), click near a cross. The click is snapped to
the nearest cross center within ``snap_radius_m`` and pinned; clicking a pinned cross again removes
its pin. Pins are stored in ``target_frame`` (map) in ``pins_file`` and saved on every change.

Subscribes: /clicked_point (PointStamped), /cross_detector/landmarks (MarkerArray, cross centers;
            falls back to ``crosses_file`` when the detector is not running),
            /rtabmap/localization_pose (only with require_localization)
Publishes:  ~/markers    MarkerArray  pins (magenta), camera->pin lines (cyan) + distance labels
            ~/distances  std_msgs/String  JSON: distance and camera-frame offsets per pin
Service:    ~/clear      std_srvs/Trigger  remove all pins
"""
import json
import os

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import Point, PointStamped, PoseWithCovarianceStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from d455_mapping.cross_logic import quat_to_rot
from d455_mapping.pins_logic import PinStore, nearest_cross, pin_distances

PIN_RGB = (1.0, 0.2, 0.8)
LINE_RGB = (0.0, 0.9, 1.0)


class PinTool(Node):
    def __init__(self):
        super().__init__('pins')
        param = self._declare
        self.pins_file = param('pins_file', '')
        self.crosses_file = param('crosses_file', '')
        self.target_frame = param('target_frame', 'map')
        self.camera_frame = param('camera_frame', 'camera_color_optical_frame')
        self.snap_radius = float(param('snap_radius_m', 0.8))
        self.require_loc = bool(param('require_localization', False))
        self.loc_timeout = float(param('localization_timeout_s', 30.0))
        rate = float(param('rate_hz', 5.0))

        self.store = PinStore(self.target_frame)
        try:
            n = self.store.load(self.pins_file)
        except ValueError as e:
            n = 0
            self.get_logger().error(str(e))
        self.live_crosses = {}      # cross id -> (center, arms or None), from the detector's markers
        self.live_stamp = None
        self.last_loc = None
        self._last_log = None
        self.feedback = None        # (position, text, rgb, time): shown in RViz after every click

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)
        self.create_subscription(PointStamped, '/clicked_point', self._on_click, 10)
        self.create_subscription(MarkerArray, '/cross_detector/landmarks', self._on_landmarks, 5)
        if self.require_loc:
            self.create_subscription(PoseWithCovarianceStamped, '/rtabmap/localization_pose',
                                     lambda _m: setattr(self, 'last_loc', self._now()), 10)
        self.marker_pub = self.create_publisher(MarkerArray, '~/markers', 5)
        self.dist_pub = self.create_publisher(String, '~/distances', 5)
        self.create_subscription(Int32, '~/pin_cross', self._on_pin_cross, 5)
        self.create_service(Trigger, '~/clear', self._on_clear)
        self.create_timer(1.0 / max(rate, 0.5), self._tick)
        self.get_logger().info(
            f'{n} pin(s) loaded from {self.pins_file or "(no file)"}. In RViz: tool "Publish Point" '
            f'(P), click near a cross to pin or unpin it, or: ros2 topic pub --once /pins/pin_cross '
            f'std_msgs/msg/Int32 "{{data: <cross id>}}"'
            + ('; distances only while localized' if self.require_loc else ''))

    def _declare(self, name, default):
        return self.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True)).value

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # -- crosses to snap to -------------------------------------------------------------------
    def _on_landmarks(self, msg):
        centers, arms = {}, {}
        for m in msg.markers:
            if m.action != Marker.ADD:
                continue
            if m.ns == 'landmark_center':
                centers[m.id] = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
            elif m.ns == 'landmark_arms' and len(m.points) >= 4:     # LINE_LIST: +a0, -a0, +a1, -a1
                pts = np.array([[q.x, q.y, q.z] for q in m.points[:4]])
                arms[m.id] = np.array([pts[0] - pts[1], pts[2] - pts[3]])
                arms[m.id] /= np.linalg.norm(arms[m.id], axis=1)[:, None]
        if centers or not msg.markers:
            self.live_crosses = {i: (c, arms.get(i)) for i, c in centers.items()}
            self.live_stamp = self._now()

    def _crosses(self):
        """Confirmed crosses {id: (center, arms or None)}: the running detector's, else the
        detector's JSON file."""
        if self.live_stamp is not None and self._now() - self.live_stamp < 10.0 and self.live_crosses:
            return dict(self.live_crosses)
        try:
            with open(self.crosses_file, encoding='utf-8') as f:
                return {int(c['id']): (np.array(c['center'], float),
                                       np.array(c['arms'], float).reshape(2, 3) if c.get('arms') else None)
                        for c in json.load(f)['crosses']}
        except (OSError, ValueError, KeyError, TypeError):
            return {}

    # -- selection ----------------------------------------------------------------------------
    def _on_click(self, msg):
        p = np.array([msg.point.x, msg.point.y, msg.point.z])
        self.get_logger().info(f'click received at [{p[0]:.2f} {p[1]:.2f} {p[2]:.2f}] in {msg.header.frame_id or "?"}')
        if msg.header.frame_id not in ('', self.target_frame):
            try:
                tf = self.tf_buffer.lookup_transform(self.target_frame, msg.header.frame_id, Time(),
                                                     timeout=Duration(seconds=0.2))
            except tf2_ros.TransformException as e:
                self.get_logger().warn(f'click in {msg.header.frame_id}, no TF to {self.target_frame}: {e}')
                return
            q, t = tf.transform.rotation, tf.transform.translation
            p = quat_to_rot(q.x, q.y, q.z, q.w) @ p + np.array([t.x, t.y, t.z])
        self._pin_near(p)

    def _on_pin_cross(self, msg):
        crosses = self._crosses()
        if msg.data not in crosses:
            self._say(None, f'cross {msg.data} unknown (known: {sorted(crosses)})', (1.0, 0.3, 0.3))
            return
        self._toggle(msg.data, *crosses[msg.data])

    def _pin_near(self, p):
        crosses = self._crosses()
        if not crosses:
            self._say(p, 'no confirmed crosses to pin: is the cross detector running?', (1.0, 0.3, 0.3))
            return
        hit, dist = nearest_cross({i: c for i, (c, _) in crosses.items()}, p, self.snap_radius)
        if hit is None:
            self._say(p, f'click ignored: nearest cross is {dist:.2f} m away (limit {self.snap_radius:.2f} m); '
                         'click closer to an orange cross', (1.0, 0.8, 0.0))
            return
        self._toggle(hit[0], *crosses[hit[0]])

    def _toggle(self, cid, center, arms=None):
        action, pin = self.store.toggle_cross(cid, center, arms)
        self._say(center, f'{"pinned" if action == "added" else "removed pin of"} cross {cid} (pin {pin.id})',
                  (0.2, 1.0, 0.2) if action == 'added' else (1.0, 0.6, 0.2))
        if self.pins_file:
            self.store.save(self.pins_file)

    def _say(self, pos, text, rgb):
        """Log it and show it in RViz at ``pos`` for a few seconds, so a click never fails silently."""
        self.get_logger().info(text)
        if pos is not None:
            self.feedback = (np.asarray(pos, float), text, rgb, self._now())

    def _on_clear(self, _req, res):
        n = self.store.clear()
        if self.pins_file and n:
            self.store.save(self.pins_file)
        res.success, res.message = True, f'removed {n} pin(s)'
        return res

    # -- camera lines and distances -----------------------------------------------------------
    def _camera_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.target_frame, self.camera_frame, Time())
        except tf2_ros.TransformException:
            return None
        q, t = tf.transform.rotation, tf.transform.translation
        return quat_to_rot(q.x, q.y, q.z, q.w), np.array([t.x, t.y, t.z])

    def _localized(self):
        return (not self.require_loc) or (self.last_loc is not None
                                          and self._now() - self.last_loc <= self.loc_timeout)

    def _tick(self):
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for p in self.store.pins:
            arr.markers += self._pin_markers(p, stamp)
        if self.feedback is not None and self._now() - self.feedback[3] < 6.0:
            arr.markers += self._feedback_markers(stamp)
        pose = self._camera_pose()
        localized = self._localized()
        report = {'localized': localized, 'camera_frame': self.camera_frame, 'pins': []}
        if self.store.pins and pose is not None and localized:
            report['pins'] = pin_distances(self.store.pins, *pose)
            for d in report['pins']:
                arr.markers += self._line_markers(d, pose[1], stamp)
            self._log(report['pins'])
        elif self.store.pins and not localized:
            self.get_logger().info('not localized yet: pin distances unavailable', throttle_duration_sec=10.0)
        self.marker_pub.publish(arr)
        self.dist_pub.publish(String(data=json.dumps(report)))

    def _log(self, dists):
        now = self._now()
        if self._last_log is not None and now - self._last_log < 1.0:
            return
        self._last_log = now
        rows = [f'pin {d["id"]} (cross {d["cross_id"]}): {d["distance_m"]:.2f} m  '
                f'[right {d["camera_xyz"][0]:+.2f}, down {d["camera_xyz"][1]:+.2f}, '
                f'forward {d["camera_xyz"][2]:+.2f}]' for d in dists]
        self.get_logger().info('distances from the camera: ' + ' | '.join(rows))

    # -- markers ------------------------------------------------------------------------------
    def _base(self, kind, ns, idx, rgb, stamp, frame=None, lifetime=0.0):
        m = Marker()
        m.header.frame_id, m.header.stamp = frame or self.target_frame, stamp
        m.ns, m.id, m.type, m.action = ns, idx, kind, Marker.ADD
        m.pose.orientation.w = 1.0
        m.color.r, m.color.g, m.color.b, m.color.a = (*rgb, 1.0)
        if lifetime > 0:
            m.lifetime = Duration(seconds=lifetime).to_msg()
        return m

    def _cross_shape(self, ns, idx, center, arms, half, width, rgb, stamp, lifetime=0.0):
        """A small cross: the two beam directions of the pinned cross, or a 3D plus (map axes)
        when they are unknown."""
        dirs = list(arms) if arms is not None else list(np.eye(3))
        m = self._base(Marker.LINE_LIST, ns, idx, rgb, stamp, lifetime=lifetime)
        m.scale.x = width
        for d in dirs:
            for sgn in (1.0, -1.0):
                q = center + sgn * half * np.asarray(d)
                m.points.append(Point(x=float(q[0]), y=float(q[1]), z=float(q[2])))
        return m

    def _pin_markers(self, pin, stamp):
        x, y, z = (float(v) for v in pin.position)
        shape = self._cross_shape('pin', pin.id, pin.position, pin.arms, 0.08, 0.012, PIN_RGB, stamp)
        text = self._base(Marker.TEXT_VIEW_FACING, 'pin_label', pin.id, (1.0, 1.0, 1.0), stamp)
        text.pose.position = Point(x=x, y=y, z=z + 0.14)
        text.scale.z = 0.09
        text.text = f'pin {pin.id}' + (f' (cross {pin.cross_id})' if pin.cross_id is not None else '')
        return [shape, text]

    def _feedback_markers(self, stamp):
        pos, text, rgb, _ = self.feedback
        shape = self._cross_shape('pin_click', 0, pos, None, 0.05, 0.01, rgb, stamp, lifetime=1.0)
        label = self._base(Marker.TEXT_VIEW_FACING, 'pin_click_text', 0, rgb, stamp, lifetime=1.0)
        label.pose.position = Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]) + 0.2)
        label.scale.z = 0.09
        label.text = text
        return [shape, label]

    def _line_markers(self, d, cam_pos, stamp):
        pin = np.array(d['map_xyz'])
        line = self._base(Marker.LINE_STRIP, 'pin_line', d['id'], LINE_RGB, stamp, lifetime=1.0)
        line.scale.x = 0.015
        line.points = [Point(x=float(cam_pos[0]), y=float(cam_pos[1]), z=float(cam_pos[2])),
                       Point(x=float(pin[0]), y=float(pin[1]), z=float(pin[2]))]
        mid = 0.5 * (cam_pos + pin)
        text = self._base(Marker.TEXT_VIEW_FACING, 'pin_dist', d['id'], LINE_RGB, stamp, lifetime=1.0)
        text.pose.position = Point(x=float(mid[0]), y=float(mid[1]), z=float(mid[2]))
        text.scale.z = 0.12
        text.text = f'{d["distance_m"]:.2f} m'
        return [line, text]

    def shutdown(self):
        if self.pins_file and self.store.dirty:
            self.store.save(self.pins_file)


def main():
    rclpy.init()
    node = PinTool()
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
