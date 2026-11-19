"""ROS 2 node: vector from the drone's base_link to a pinned cross, as the map sees it.

    ros2 run d455_mapping cross_rel_base --ros-args -p pins_file:=/maps/<map>_pins.json

Takes the position of a pin (the pin tool's JSON, map frame) and base_link's pose in the map
(TF map -> base_link: RTAB-Map's map -> odom times cuVSLAM's odom -> base_link) and publishes the
vector from base_link to the cross in base_link axes (x forward, y left, z up). With px4_vio:=true
base_link is the PX4 body, so this is the drone-to-cross vector in the drone's own frame, which
does not depend on any world frame: the OptiTrack side (vanguard cross_rel_gt) computes the same
vector, so the two can be compared directly.

Parameters: pins_file (required), pin_id (-1 = lowest id), target_frame (map), base_frame
(base_link), require_localization (true: only while /rtabmap/localization_pose has a position
variance <= localization_max_variance, within localization_timeout_s), rate_hz.
Publishes:  /cross_rel/vslam           geometry_msgs/Vector3Stamped  (stamp = the TF's stamp)
            /cross_rel/vslam_distance  std_msgs/Float64              (length of the vector, m)
The pins file is read again whenever it changes (pin or unpin while this runs).
"""
import os

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3Stamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float64

from d455_mapping.cross_logic import quat_to_rot
from d455_mapping.cross_rel import load_pin_position, pin_in_base


class CrossRelBase(Node):
    def __init__(self):
        super().__init__('cross_rel_base')
        dyn = ParameterDescriptor(dynamic_typing=True)
        self.pins_file = str(self.declare_parameter('pins_file', '', dyn).value)
        self.pin_id = int(self.declare_parameter('pin_id', -1, dyn).value)
        self.target_frame = str(self.declare_parameter('target_frame', 'map', dyn).value)
        self.base_frame = str(self.declare_parameter('base_frame', 'base_link', dyn).value)
        self.require_loc = bool(self.declare_parameter('require_localization', True, dyn).value)
        self.loc_max_var = float(self.declare_parameter('localization_max_variance', 1.0, dyn).value)
        self.loc_timeout = float(self.declare_parameter('localization_timeout_s', 30.0, dyn).value)
        rate = float(self.declare_parameter('rate_hz', 15.0, dyn).value)

        self.pin = None             # (id, position in the map frame)
        self.pin_mtime = None
        self.last_loc = None
        self.last_stamp = None      # do not publish the same TF twice

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)
        if self.require_loc:
            self.create_subscription(PoseWithCovarianceStamped, '/rtabmap/localization_pose',
                                     self._on_loc, 10)
        self.vec_pub = self.create_publisher(Vector3Stamped, '/cross_rel/vslam', 10)
        self.dist_pub = self.create_publisher(Float64, '/cross_rel/vslam_distance', 10)
        self.create_timer(1.0 / max(rate, 1.0), self._tick)
        self.get_logger().info(
            f'pin {self.pin_id if self.pin_id >= 0 else "(lowest id)"} of {self.pins_file or "(no file)"}'
            f' -> /cross_rel/vslam in {self.base_frame}'
            + ('; only while localized' if self.require_loc else ''))

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_loc(self, msg):
        if msg.pose.covariance[0] <= self.loc_max_var:      # 9999 = RTAB-Map is lost
            self.last_loc = self._now()

    def _reload_pin(self):
        try:
            mtime = os.path.getmtime(self.pins_file)
        except OSError:
            mtime = None
        if mtime == self.pin_mtime and self.pin is not None:
            return
        pin = load_pin_position(self.pins_file, self.pin_id)
        if pin is not None and (self.pin is None or pin[0] != self.pin[0]
                                or np.linalg.norm(pin[1] - self.pin[1]) > 1e-6):
            self.get_logger().info(f'using pin {pin[0]} at map [{pin[1][0]:.3f} {pin[1][1]:.3f} '
                                   f'{pin[1][2]:.3f}]')
        self.pin, self.pin_mtime = pin, mtime

    def _tick(self):
        self._reload_pin()
        if self.pin is None:
            self.get_logger().warn(f'no pin to use in {self.pins_file or "(pins_file not set)"}: '
                                   'pin the cross first (pin tool)', throttle_duration_sec=5.0)
            return
        if self.require_loc and (self.last_loc is None
                                 or self._now() - self.last_loc > self.loc_timeout):
            self.get_logger().warn('not localized in the map yet: nothing published',
                                   throttle_duration_sec=5.0)
            return
        try:
            tf = self.tf_buffer.lookup_transform(self.target_frame, self.base_frame, Time())
        except tf2_ros.TransformException:
            return
        stamp = (tf.header.stamp.sec, tf.header.stamp.nanosec)
        if stamp == self.last_stamp:
            return
        self.last_stamp = stamp
        q, t = tf.transform.rotation, tf.transform.translation
        v = pin_in_base(quat_to_rot(q.x, q.y, q.z, q.w), np.array([t.x, t.y, t.z]), self.pin[1])
        out = Vector3Stamped()
        out.header.stamp = tf.header.stamp
        out.header.frame_id = self.base_frame
        out.vector.x, out.vector.y, out.vector.z = map(float, v)
        self.vec_pub.publish(out)
        self.dist_pub.publish(Float64(data=float(np.linalg.norm(v))))


def main(args=None):
    rclpy.init(args=args)
    node = CrossRelBase()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
