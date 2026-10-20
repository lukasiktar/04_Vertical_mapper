"""Camera + cuVSLAM + Isaac ROS nvblox dense mesh (TSDF in the odom frame).

    ros2 launch d455_mapping nvblox.launch.py platform:=laptop [rviz:=true]

nvblox input: emitter-ON depth (+ depth camera_info), color (+ camera_info), pose from TF
odom -> camera frames (cuVSLAM). Mesh: /nvblox_node/mesh (nvblox_rviz_plugin).
Voxel size, range and rates come from config/<platform>.yaml (nvblox section).
Save the mesh:  ros2 service call /nvblox_node/save_ply nvblox_msgs/srv/FilePath "{file_path: /maps/mesh.ply}"
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from d455_mapping import common as c


def setup(context):
    cfg = c.load_config(context)
    actions = c.camera_actions(cfg) + [c.map_to_odom_identity(cfg)]
    actions.append(c.container(
        cfg, c.camera_node(cfg) + c.splitter_nodes(cfg) + [c.vslam_node(cfg),
                                                           c.nvblox_node(cfg)]))
    if c.truthy(context.perform_substitution(LaunchConfiguration('rviz'))):
        actions.append(c.rviz_action(cfg, 'mapping'))
    return actions


def generate_launch_description():
    return LaunchDescription(c.common_args() + [
        DeclareLaunchArgument('rviz', default_value='false'),
        OpaqueFunction(function=setup)])
