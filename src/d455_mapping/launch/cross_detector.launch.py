"""Cross detector only (camera, cuVSLAM and RTAB-Map are started elsewhere).

    ros2 launch d455_mapping cross_detector.launch.py platform:=laptop \\
        landmarks_file:=/maps/lab_crosses.json require_localization:=true

mapping.launch.py / localization.launch.py start it with ``with_cross_detector:=true``.
The camera/depth_fps arguments of the common set are accepted but unused here.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction

from d455_mapping import common as c


def setup(context):
    cfg = c.load_config(context)
    lc = c._lc
    return [c.cross_detector_node(
        cfg, context.perform_substitution(lc('landmarks_file')),
        require_localization=c.truthy(context.perform_substitution(lc('require_localization'))),
        start_empty=c.truthy(context.perform_substitution(lc('start_empty'))),
        extra_params=context.perform_substitution(lc('cross_params')))]


def generate_launch_description():
    return LaunchDescription(c.common_args() + c.cross_args() + [
        DeclareLaunchArgument('require_localization', default_value='false',
                              description='true: fuse only while /rtabmap/localization_pose is alive '
                                          '(localization mode)'),
        DeclareLaunchArgument('start_empty', default_value='false',
                              description='true: ignore (and back up to .bak) an existing landmark '
                                          'file, as in a new mapping run'),
        OpaqueFunction(function=setup)])
