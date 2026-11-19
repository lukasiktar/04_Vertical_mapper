"""cuVSLAM odometry -> body-frame twist for the PX4 fusion EKF, dropped while cuVSLAM is not tracking.

cuVSLAM's pose and PX4's pose are in different frames (different origin and heading), so the EKF
fuses cuVSLAM only as velocity: the motion between consecutive cuVSLAM poses, in base_link.

Nothing is published (the EKF then runs on PX4 alone) while:
  * /visual_slam/status vo_state != 1 (camera too close to a surface, blank wall, glass, ...),
  * the first `resume_skip` poses after tracking comes back (cuVSLAM can jump on recovery),
  * the speed / turn rate between two poses is implausible (a jump that vo_state missed).
The covariance is fixed: cuVSLAM's own is sometimes all zero, which an EKF would take as
infinitely certain.

    ros2 run d455_mapping vslam_twist
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from d455_mapping.px4_bag_odom import quat_wxyz_to_matrix

VO_SUCCESS = 1


def rotation_vector(r):
    """Axis * angle [rad] of a rotation matrix."""
    angle = math.acos(max(-1.0, min(1.0, (np.trace(r) - 1.0) / 2.0)))
    v = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    s = math.sin(angle)
    return 0.5 * v if s < 1e-6 else v * (angle / (2.0 * s))


class VslamTwist(Node):

    def __init__(self):
        super().__init__('vslam_twist')
        odom_topic = self.declare_parameter('odom_topic', '/visual_slam/tracking/odometry').value
        status_topic = self.declare_parameter('status_topic', '/visual_slam/status').value
        twist_topic = self.declare_parameter('twist_topic', '/vslam/twist').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.lin_var = float(self.declare_parameter('linear_variance', 0.0025).value)
        self.ang_var = float(self.declare_parameter('angular_variance', 0.0025).value)
        self.max_speed = float(self.declare_parameter('max_speed', 5.0).value)
        self.max_rate = float(self.declare_parameter('max_rate', 6.0).value)
        self.max_dt = float(self.declare_parameter('max_dt', 0.5).value)
        self.resume_skip = int(self.declare_parameter('resume_skip', 5).value)

        self.tracking = None        # None until the first status message
        self.prev = None            # (t [s], R, p) of the previous pose
        self.skip = 0

        self.pub = self.create_publisher(TwistWithCovarianceStamped, twist_topic, 10)
        self.create_subscription(Odometry, odom_topic, self.on_odom, qos_profile_sensor_data)
        self.create_subscription(VisualSlamStatus, status_topic, self.on_status,
                                 qos_profile_sensor_data)
        self.create_timer(5.0, self.check_status)

    def restart(self):
        """Forget the previous pose and drop the next few: the next twist starts fresh."""
        self.prev = None
        self.skip = self.resume_skip

    def check_status(self):
        if self.tracking is None:
            self.get_logger().warn('No /visual_slam/status yet: fusing cuVSLAM with the jump '
                                   'check only', throttle_duration_sec=30.0)

    def on_status(self, msg):
        ok = msg.vo_state == VO_SUCCESS
        if ok != self.tracking:
            if ok:
                self.get_logger().info('cuVSLAM tracking -> fused with PX4')
            else:
                self.get_logger().warn(f'cuVSLAM not tracking (vo_state={msg.vo_state}) '
                                       '-> EKF on PX4 only')
        if not ok:
            self.restart()
        self.tracking = ok

    def on_odom(self, msg):
        if self.tracking is False:
            return
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        o, q = msg.pose.pose.position, msg.pose.pose.orientation
        r = quat_wxyz_to_matrix((q.w, q.x, q.y, q.z))
        p = np.array([o.x, o.y, o.z])
        prev, self.prev = self.prev, (t, r, p)
        if prev is None:
            return
        dt = t - prev[0]
        if dt <= 0.0 or dt > self.max_dt:
            return
        v = prev[1].T @ (p - prev[2]) / dt          # in the previous base_link
        w = rotation_vector(prev[1].T @ r) / dt
        speed, rate = float(np.linalg.norm(v)), float(np.linalg.norm(w))
        if speed > self.max_speed or rate > self.max_rate:
            self.get_logger().warn(f'cuVSLAM jump ({speed:.1f} m/s, {rate:.1f} rad/s) dropped',
                                   throttle_duration_sec=2.0)
            self.restart()
            self.prev = (t, r, p)
            return
        if self.skip > 0:
            self.skip -= 1
            return

        out = TwistWithCovarianceStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.base_frame
        out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z = map(float, v)
        out.twist.twist.angular.x, out.twist.twist.angular.y, out.twist.twist.angular.z = map(float, w)
        for k in range(3):
            out.twist.covariance[7 * k] = self.lin_var
            out.twist.covariance[7 * (k + 3)] = self.ang_var
        self.pub.publish(out)


def main():
    rclpy.init()
    node = VslamTwist()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
