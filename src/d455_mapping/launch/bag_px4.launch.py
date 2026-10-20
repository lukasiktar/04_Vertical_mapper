"""Map from a bag recorded WITHOUT cuVSLAM inputs: RGB-D camera + PX4 odometry.

    scripts/replay_px4.sh <bag> <map_name> [launch args]     (starts this launch and plays the bag)

What it runs (all with use_sim_time):
  * px4_bag_odom        PX4 /fmu/out/vehicle_odometry (read from the sqlite3 bag, NED/FRD) ->
                        TF odom -> base_link in ROS ENU/FLU
  * static TF           base_link -> camera_link (the camera's mounting on the drone!):
                        cam_xyz:='x y z' [m], cam_rpy:='roll pitch yaw' [rad], base_link is the
                        PX4 body frame FLU (x forward, y left, z up); the camera frame camera_link is
                        x forward, y left, z up as well
  * depth_image_proc    registers the unaligned depth to the color image (needs only the bag's
                        /tf_static for depth -> color)
  * rgbd_sync + rtabmap graph SLAM with that odometry (no cuVSLAM, no IR images needed)
Camera topics are read from <camera_ns> (default /camera/camera, the default RealSense launch).
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode

from d455_mapping import common as c


def floats(text, n, name):
    v = [float(x) for x in text.replace(',', ' ').split()]
    if len(v) != n:
        raise RuntimeError(f'{name} needs {n} numbers, got {text!r}')
    return v


def setup(context):
    arg = lambda k: context.perform_substitution(LaunchConfiguration(k))  # noqa: E731
    cfg = c.load_config(context)
    cfg['use_sim_time'] = True
    cfg['start_camera'] = False
    cfg['base_to_camera'] = {'xyz': floats(arg('cam_xyz'), 3, 'cam_xyz'),
                             'rpy': floats(arg('cam_rpy'), 3, 'cam_rpy')}
    ns = arg('camera_ns').rstrip('/')
    bag_db = arg('bag_db')
    if not os.path.isfile(bag_db):
        raise RuntimeError(f'bag_db {bag_db} not found (sqlite3 .db3 file of the bag)')
    db = arg('database_path')

    reg = ComposableNode(
        package='depth_image_proc', plugin='depth_image_proc::RegisterNode',
        name='register_depth', namespace='',
        parameters=[{'use_sim_time': True}],
        remappings=[('depth/image_rect', f'{ns}/depth/image_rect_raw'),
                    ('depth/camera_info', f'{ns}/depth/camera_info'),
                    ('rgb/camera_info', f'{ns}/color/camera_info'),
                    ('depth_registered/image_rect', f'{ns}/aligned_depth_to_color/image_raw'),
                    ('depth_registered/camera_info', f'{ns}/aligned_depth_to_color/camera_info')])
    topics = {'color': f'{ns}/color/image_raw', 'color_info': f'{ns}/color/camera_info',
              'depth': f'{ns}/aligned_depth_to_color/image_raw'}
    cfg['rtabmap']['use_imu'] = False                       # no IMU in these bags
    # PX4's odometry is good locally but not to the 1 mm RTAB-Map would assume: realistic
    # uncertainty lets the visual links correct it. Measured on a real bag (keyframe depth
    # mismatch 52.8 -> 35.8 mm): refine each link between neighbouring keyframes visually, accept
    # loop closures with fewer inliers, local bundle adjustment after a closure.
    cfg['rtabmap']['odom_linear_variance'] = float(arg('odom_lin_var'))
    cfg['rtabmap']['odom_angular_variance'] = float(arg('odom_ang_var'))
    extra = {'Mem/ImagePreDecimation': str(int(float(arg('image_decimation'))))}
    if c.truthy(arg('refine')):
        extra.update({'RGBD/NeighborLinkRefining': 'true', 'Vis/MinInliers': '12',
                      'RGBD/OptimizeMaxError': '6', 'RGBD/ProximityPathMaxNeighbors': '10',
                      'RGBD/LocalBundleOnLoopClosure': 'true'})
    rtab, rtab_actions = c.rtabmap_parts(
        cfg, db, localization=False, topics=topics, sync_interval=float(arg('sync_interval')),
        extra_params=extra)

    actions = [c.base_to_camera_tf(cfg),
               Node(package=c.PACKAGE, executable='px4_bag_odom', name='px4_bag_odom',
                    output='screen', emulate_tty=True,
                    parameters=[{'bag_db': bag_db, 'topic': arg('px4_topic'),
                                 'time_offset_s': float(arg('time_offset_s')),
                                 'smooth_resets': c.truthy(arg('smooth_resets')),
                                 'use_sim_time': True}]),
               c.container(cfg, [reg] + rtab)] + rtab_actions
    if c.truthy(arg('rviz')):
        actions.append(c.rviz_action(cfg, 'mapping'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('platform', default_value='laptop', choices=list(c.PLATFORMS)),
        DeclareLaunchArgument('bag_db', description='sqlite3 .db3 file of the bag (container path)'),
        DeclareLaunchArgument('database_path', default_value='/maps/map.db'),
        DeclareLaunchArgument('camera_ns', default_value='/camera/camera'),
        DeclareLaunchArgument('px4_topic', default_value='/fmu/out/vehicle_odometry'),
        DeclareLaunchArgument('cam_xyz', default_value='0 0 0',
                              description='camera position in the drone body frame (FLU) [m]'),
        DeclareLaunchArgument('cam_rpy', default_value='0 0 0',
                              description='camera orientation in the body frame: roll pitch yaw [rad]'),
        DeclareLaunchArgument('time_offset_s', default_value='0.0'),
        DeclareLaunchArgument('smooth_resets', default_value='true'),
        DeclareLaunchArgument('sync_interval', default_value='0.03',
                              description='color/depth approximate sync window [s]'),
        DeclareLaunchArgument('image_decimation', default_value='2',
                              description='RTAB-Map Mem/ImagePreDecimation (1280x720 -> /2)'),
        DeclareLaunchArgument('odom_lin_var', default_value='0.05',
                              description='odometry linear variance [m^2] given to RTAB-Map'),
        DeclareLaunchArgument('odom_ang_var', default_value='0.03',
                              description='odometry angular variance [rad^2]'),
        DeclareLaunchArgument('refine', default_value='true',
                              description='visual refinement of odometry links + more loop closures'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('camera', default_value='false'),
        DeclareLaunchArgument('depth_fps', default_value=''),
        OpaqueFunction(function=setup)])
