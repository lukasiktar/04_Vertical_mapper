"""Camera + Isaac ROS Visual SLAM (cuVSLAM) stereo-inertial odometry.

    ros2 launch d455_mapping vslam.launch.py platform:=laptop [rviz:=true]

cuVSLAM input : emitter-OFF IR1/IR2 (realsense_splitter) + /camera/imu (200 Hz)
cuVSLAM output: /visual_slam/tracking/odometry (frame odom -> child base_link),
                TF odom -> base_link. It does NOT publish map -> odom (RTAB-Map owns it)
                and its own SLAM/loop closure is disabled.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from d455_mapping import common as c


def setup(context):
    cfg = c.load_config(context)
    actions = c.camera_actions(cfg) + [c.map_to_odom_identity(cfg)]
    actions.append(c.container(
        cfg, c.camera_node(cfg) + c.splitter_nodes(cfg) + [c.vslam_node(cfg)]))
    if c.truthy(context.perform_substitution(LaunchConfiguration('rviz'))):
        actions.append(c.rviz_action(cfg, 'mapping'))
    return actions


def generate_launch_description():
    return LaunchDescription(c.common_args() + [
        DeclareLaunchArgument('rviz', default_value='false'),
        OpaqueFunction(function=setup)])
