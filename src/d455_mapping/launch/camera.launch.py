"""RealSense D455 driver + emitter on/off splitters.

    ros2 launch d455_mapping camera.launch.py platform:=laptop [depth_fps:=15]

Outputs (namespace /camera): infra1/2, depth, color, aligned_depth_to_color, imu (united,
linear interpolation), metadata; splitter outputs:
    /camera/realsense_splitter_node/output/{infra_1,infra_2}  emitter OFF  -> cuVSLAM
    /camera/realsense_splitter_node/output/depth              emitter ON   -> nvblox
    /camera/aligned_depth_splitter/output/depth               emitter ON   -> RTAB-Map
"""
from launch import LaunchDescription
from launch.actions import OpaqueFunction

from d455_mapping import common as c


def setup(context):
    cfg = c.load_config(context)
    actions = c.camera_actions(cfg)
    nodes = c.camera_node(cfg) + c.splitter_nodes(cfg)
    if nodes:
        actions.append(c.container(cfg, nodes))
    return actions


def generate_launch_description():
    return LaunchDescription(c.common_args() + [OpaqueFunction(function=setup)])
