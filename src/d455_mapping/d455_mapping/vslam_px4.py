"""cuVSLAM odometry -> /vslam/odom_px4 for PX4's EKF2 (external vision), dropped while not tracking.

The vanguard_ros2 container turns this into px4_msgs/VehicleOdometry
(`ros2 run vanguard vslam_to_px4_node`): the mapper has no px4_msgs and vanguard_ros2 has no
Isaac ROS messages, so the tracking check runs here and only a standard message crosses over.

Nothing is published (PX4 then flies on optical flow + IMU) while:
  * /visual_slam/status vo_state != 1 (camera too close to a surface, blank wall, glass, ...),
  * the first `resume_skip` poses after tracking comes back (cuVSLAM can jump on recovery),
  * the speed / turn rate between two poses is implausible (a jump that vo_state missed).
Each of these leaves a gap in the stream, which vslam_to_px4_node reports to PX4 as a reset.
The covariance is fixed: cuVSLAM's own is all zero, which an EKF would take as infinitely certain.

    ros2 run d455_mapping vslam_px4
"""
import numpy as np
import rclpy
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from d455_mapping.px4_bag_odom import quat_wxyz_to_matrix
from d455_mapping.vslam_twist import VO_SUCCESS, rotation_vector


class VslamPx4(Node):

    def __init__(self):
        super().__init__('vslam_px4')
        odom_topic = self.declare_parameter('odom_topic', '/visual_slam/tracking/odometry').value
        status_topic = self.declare_parameter('status_topic', '/visual_slam/status').value
        out_topic = self.declare_parameter('out_topic', '/vslam/odom_px4').value
        self.pos_var = float(self.declare_parameter('position_variance', 0.01).value)
        self.ang_var = float(self.declare_parameter('orientation_variance', 0.01).value)
        self.max_speed = float(self.declare_parameter('max_speed', 5.0).value)
        self.max_rate = float(self.declare_parameter('max_rate', 6.0).value)
        self.max_dt = float(self.declare_parameter('max_dt', 0.5).value)
        self.resume_skip = int(self.declare_parameter('resume_skip', 5).value)

        self.tracking = None        # None until the first status message
        self.prev = None            # (t [s], R, p) of the previous pose
        self.skip = 0
        self.count = 0

        self.pub = self.create_publisher(Odometry, out_topic, 10)
        self.create_subscription(Odometry, odom_topic, self.on_odom, qos_profile_sensor_data)
        self.create_subscription(VisualSlamStatus, status_topic, self.on_status,
                                 qos_profile_sensor_data)
        self.create_timer(5.0, self.check)
        self.get_logger().info(f'{odom_topic} -> {out_topic} (only while cuVSLAM is tracking)')

    def restart(self):
        """Forget the previous pose and drop the next few: the stream resumes after a gap."""
        self.prev = None
        self.skip = self.resume_skip

    def check(self):
        if self.tracking is None:
            self.get_logger().warn('No /visual_slam/status yet: sending cuVSLAM to PX4 with the '
                                   'jump check only', throttle_duration_sec=30.0)
        if self.count == 0 and self.tracking is not False:
            self.get_logger().warn('Nothing sent to PX4 in the last 5 s (is cuVSLAM running?)',
                                   throttle_duration_sec=30.0)
        self.count = 0

    def on_status(self, msg):
        ok = msg.vo_state == VO_SUCCESS
        if ok != self.tracking:
            if ok:
                self.get_logger().info('cuVSLAM tracking -> sent to PX4')
            else:
                self.get_logger().warn(f'cuVSLAM not tracking (vo_state={msg.vo_state}) '
                                       '-> PX4 on optical flow only')
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
        if prev is not None:
            dt = t - prev[0]
            if 0.0 < dt <= self.max_dt:
                speed = float(np.linalg.norm(p - prev[2])) / dt
                rate = float(np.linalg.norm(rotation_vector(prev[1].T @ r))) / dt
                if speed > self.max_speed or rate > self.max_rate:
                    self.get_logger().warn(f'cuVSLAM jump ({speed:.1f} m/s, {rate:.1f} rad/s) '
                                           'dropped', throttle_duration_sec=2.0)
                    self.restart()
                    self.prev = (t, r, p)
                    return
        if self.skip > 0:
            self.skip -= 1
            return

        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose.pose = msg.pose.pose
        for k in range(3):
            out.pose.covariance[7 * k] = self.pos_var
            out.pose.covariance[7 * (k + 3)] = self.ang_var
        self.pub.publish(out)
        self.count += 1


def main():
    rclpy.init()
    node = VslamPx4()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
