"""Offline map viewer (no camera): exported PCD -> /prior_map (PointCloud2, frame map),
optional mesh -> /prior_map_mesh (visualization_msgs/Marker MESH_RESOURCE).

    ros2 launch d455_mapping show_map.launch.py pcd:=/maps/lab/lab_cloud.pcd \\
        [mesh:=/maps/lab/lab_mesh.dae] [rviz:=true]
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from d455_mapping import common as c


def setup(context):
    pcd = context.perform_substitution(LaunchConfiguration('pcd'))
    mesh = context.perform_substitution(LaunchConfiguration('mesh'))
    actions = [Node(
        package='d455_mapping', executable='prior_map_publisher', name='prior_map_publisher',
        parameters=[{'pcd_path': pcd, 'frame_id': c.MAP_FRAME}], output='screen')]
    if mesh:
        actions.append(Node(
            package='d455_mapping', executable='mesh_marker_publisher',
            name='prior_map_mesh_publisher',
            parameters=[{'mesh_path': mesh, 'frame_id': c.MAP_FRAME}],
            remappings=[('mesh_marker', '/prior_map_mesh')], output='screen'))
    if c.truthy(context.perform_substitution(LaunchConfiguration('rviz'))):
        actions.append(Node(package='rviz2', executable='rviz2', name='rviz2',
                            arguments=['-d', c.share('rviz', 'show_map.rviz')], output='log'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('pcd', description='PCD file to publish on /prior_map'),
        DeclareLaunchArgument('mesh', default_value='', description='.dae/.stl mesh (optional)'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=setup)])
