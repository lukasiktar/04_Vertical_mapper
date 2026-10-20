"""Mapping: camera + cuVSLAM (odometry) + RTAB-Map (graph SLAM, map -> odom) [+ nvblox].

    ros2 launch d455_mapping mapping.launch.py platform:=laptop database_path:=/maps/lab.db

Bag replay (no camera):
    ros2 launch d455_mapping mapping.launch.py camera:=false use_sim_time:=true ...
    ros2 bag play /bags/<bag> --clock --qos-profile-overrides-path \\
        $(ros2 pkg prefix d455_mapping)/share/d455_mapping/config/bag_qos_overrides.yaml
"""
from launch import LaunchDescription
from launch.actions import OpaqueFunction

from d455_mapping import common as c


def generate_launch_description():
    return LaunchDescription(c.slam_args() + [
        OpaqueFunction(function=lambda ctx: c.build_slam(ctx, localization=False))])
