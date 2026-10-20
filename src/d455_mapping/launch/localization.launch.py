"""Localization against an existing RTAB-Map database (no map changes).

    ros2 launch d455_mapping localization.launch.py platform:=laptop database_path:=/maps/lab.db
"""
from launch import LaunchDescription
from launch.actions import OpaqueFunction

from d455_mapping import common as c


def generate_launch_description():
    return LaunchDescription(c.slam_args() + [
        OpaqueFunction(function=lambda ctx: c.build_slam(ctx, localization=True))])
