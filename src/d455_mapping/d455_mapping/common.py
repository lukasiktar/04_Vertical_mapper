"""Shared launch building blocks for d455_mapping.

Every launch file declares ``platform:=laptop|orin_nano`` and builds its nodes from
``config/<platform>.yaml`` through the functions below, so the node graph is defined once.

Frames (REP-105):
    map -> odom            RTAB-Map (mapping + localization). cuVSLAM does NOT publish it.
    odom -> base_link      cuVSLAM (smooth visual-inertial odometry)
    base_link -> camera_link   static, config ``base_to_camera`` (camera mounting)
    camera_link -> camera_*_optical_frame   realsense2_camera (factory extrinsics)
"""

import math
import os
from typing import Dict, List

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

PACKAGE = 'd455_mapping'
PLATFORMS = ('laptop', 'orin_nano')
CONTAINER = 'd455_container'

# Frames
MAP_FRAME = 'map'
ODOM_FRAME = 'odom'
BASE_FRAME = 'base_link'
CAMERA_NAME = 'camera'
CAMERA_LINK = f'{CAMERA_NAME}_link'

# Raw camera topics (realsense2_camera, namespace 'camera')
NS = f'/{CAMERA_NAME}'
T_IR1 = f'{NS}/infra1/image_rect_raw'
T_IR1_INFO = f'{NS}/infra1/camera_info'
T_IR1_META = f'{NS}/infra1/metadata'
T_IR2 = f'{NS}/infra2/image_rect_raw'
T_IR2_INFO = f'{NS}/infra2/camera_info'
T_IR2_META = f'{NS}/infra2/metadata'
T_DEPTH = f'{NS}/depth/image_rect_raw'
T_DEPTH_INFO = f'{NS}/depth/camera_info'
T_DEPTH_META = f'{NS}/depth/metadata'
T_COLOR = f'{NS}/color/image_raw'
T_COLOR_INFO = f'{NS}/color/camera_info'
T_COLOR_META = f'{NS}/color/metadata'
T_ALIGNED = f'{NS}/aligned_depth_to_color/image_raw'
T_ALIGNED_INFO = f'{NS}/aligned_depth_to_color/camera_info'
T_IMU = f'{NS}/imu'

# Splitter outputs (emitter-off IR, emitter-on depth)
SPLIT = f'{NS}/realsense_splitter_node/output'
SPLIT_ALIGNED = f'{NS}/aligned_depth_splitter/output'

# Downstream
T_ODOM = '/visual_slam/tracking/odometry'
T_IMU_FILTERED = '/imu/data'

RAW_TOPICS = [
    T_IR1, T_IR1_INFO, T_IR1_META, T_IR2, T_IR2_INFO, T_IR2_META,
    T_DEPTH, T_DEPTH_INFO, T_DEPTH_META,
    T_COLOR, T_COLOR_INFO, T_COLOR_META,
    T_ALIGNED, T_ALIGNED_INFO,
    T_IMU, f'{NS}/extrinsics/depth_to_color', f'{NS}/extrinsics/depth_to_infra1',
    f'{NS}/extrinsics/depth_to_infra2', f'{NS}/extrinsics/depth_to_accel',
    f'{NS}/extrinsics/depth_to_gyro',
    '/tf_static',
]


def share(*parts: str) -> str:
    return os.path.join(get_package_share_directory(PACKAGE), *parts)


def common_args() -> List[DeclareLaunchArgument]:
    return [
        DeclareLaunchArgument('platform', default_value='laptop',
                              choices=list(PLATFORMS),
                              description='Selects config/<platform>.yaml'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='true when replaying a bag with --clock'),
        DeclareLaunchArgument('camera', default_value='true',
                              description='Start the RealSense driver (false for bag replay)'),
        DeclareLaunchArgument('depth_fps', default_value='',
                              description='Override camera.depth_fps (e.g. 15 on the Jetson)'),
    ]


def truthy(value) -> bool:
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def load_config(context) -> Dict:
    platform = context.perform_substitution(_lc('platform'))
    with open(share('config', f'{platform}.yaml'), 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    cfg['platform'] = platform
    cfg['use_sim_time'] = truthy(context.perform_substitution(_lc('use_sim_time')))
    cfg['start_camera'] = truthy(context.perform_substitution(_lc('camera')))
    depth_fps = context.perform_substitution(_lc('depth_fps'))
    if depth_fps:
        cfg['camera']['depth_fps'] = int(depth_fps)
    return cfg


def _lc(name):
    from launch.substitutions import LaunchConfiguration
    return LaunchConfiguration(name)


def interleaved(cfg) -> bool:
    return bool(cfg['camera']['emitter_on_off'])


def vslam_image_topics(cfg):
    if interleaved(cfg):
        return f'{SPLIT}/infra_1', f'{SPLIT}/infra_2'
    return T_IR1, T_IR2


def nvblox_depth_topic(cfg) -> str:
    return f'{SPLIT}/depth' if interleaved(cfg) else T_DEPTH


def rtabmap_depth_topic(cfg) -> str:
    return f'{SPLIT_ALIGNED}/depth' if interleaved(cfg) else T_ALIGNED


# --------------------------------------------------------------------------------------------
# Camera
# --------------------------------------------------------------------------------------------
def camera_actions(cfg) -> list:
    """Static base_link -> camera_link (always, also for bag replay), plus the emitter check
    that restarts a stuck interleave (only with a live camera and emitter_on_off)."""
    actions = [base_to_camera_tf(cfg)]
    if cfg['start_camera'] and interleaved(cfg):
        actions.append(Node(
            package=PACKAGE, executable='emitter_ensure', name='emitter_ensure',
            output='screen', emulate_tty=True))
    return actions


def camera_node(cfg) -> List[ComposableNode]:
    """RealSense driver as a component (loaded into the shared container when camera:=true).

    Running it in the same process as the splitters / cuVSLAM / nvblox / RTAB-Map lets Fast DDS
    deliver the large depth and color images in-process instead of over the network stack
    (measured: depth/color dropped to 7-8 Hz with the driver as a separate process).
    """
    if not cfg['start_camera']:
        return []
    c = cfg['camera']
    params = {
        'camera_name': CAMERA_NAME,
        'serial_no': str(c['serial_no']),
        'usb_port_id': str(c['usb_port_id']),
        'initial_reset': bool(c['initial_reset']),
        'enable_infra1': True,
        'enable_infra2': True,
        'enable_depth': True,
        'enable_color': True,
        'depth_module.profile': f"{c['depth_width']}x{c['depth_height']}x{c['depth_fps']}",
        'rgb_camera.profile': f"{c['color_width']}x{c['color_height']}x{c['color_fps']}",
        'depth_module.emitter_enabled': 1,
        'depth_module.emitter_on_off': interleaved(cfg),
        'align_depth.enable': True,
        'enable_sync': False,
        'pointcloud.enable': False,
        'enable_gyro': True,
        'enable_accel': True,
        'gyro_fps': int(c['gyro_fps']),
        'accel_fps': int(c['accel_fps']),
        'unite_imu_method': 2,          # 2 = linear_interpolation (one /camera/imu topic)
        'publish_tf': True,
        'tf_publish_rate': 0.0,         # static extrinsics only
        'use_sim_time': cfg['use_sim_time'],
    }
    return [ComposableNode(
        package='realsense2_camera',
        plugin='realsense2_camera::RealSenseNodeFactory',
        name=CAMERA_NAME,
        namespace=CAMERA_NAME,
        parameters=[params],
    )]


def base_to_camera_tf(cfg) -> Node:
    xyz = [str(v) for v in cfg['base_to_camera']['xyz']]
    r, p, y = cfg['base_to_camera']['rpy']
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=['--x', xyz[0], '--y', xyz[1], '--z', xyz[2],
                   '--roll', str(r), '--pitch', str(p), '--yaw', str(y),
                   '--frame-id', BASE_FRAME, '--child-frame-id', CAMERA_LINK],
        parameters=[{'use_sim_time': cfg['use_sim_time']}],
    )


def map_to_odom_identity(cfg) -> Node:
    """Identity map -> odom for launches WITHOUT RTAB-Map (vslam/nvblox), so RViz's fixed
    frame `map` exists. Never use together with RTAB-Map (it owns map -> odom)."""
    return Node(
        package='tf2_ros', executable='static_transform_publisher', name='map_to_odom_identity',
        arguments=['--frame-id', MAP_FRAME, '--child-frame-id', ODOM_FRAME],
        parameters=[{'use_sim_time': cfg['use_sim_time']}])


def splitter_nodes(cfg) -> List[ComposableNode]:
    """Emitter on/off splitting (realsense_splitter from isaac_ros_nvblox).

    * realsense_splitter_node: emitter-OFF IR1/IR2 (-> cuVSLAM), emitter-ON depth (-> nvblox)
    * aligned_depth_splitter : emitter-ON aligned depth (-> RTAB-Map). realsense-ros 4.51
      stamps aligned depth with the same time as depth/metadata, so the exact-time sync
      inside the splitter matches them.
    """
    if not interleaved(cfg):
        return []
    qos = {'input_qos': 'SENSOR_DATA', 'output_qos': 'DEFAULT',
           'invert_emitter_mode': bool(cfg['camera'].get('invert_emitter_mode', False)),
           'use_sim_time': cfg['use_sim_time']}
    unused = f'{NS}/aligned_depth_splitter/unused'
    return [
        ComposableNode(
            package='realsense_splitter', plugin='nvblox::RealsenseSplitterNode',
            name='realsense_splitter_node', namespace=CAMERA_NAME, parameters=[qos],
            remappings=[
                ('input/infra_1', T_IR1), ('input/infra_1_metadata', T_IR1_META),
                ('input/infra_2', T_IR2), ('input/infra_2_metadata', T_IR2_META),
                ('input/depth', T_DEPTH), ('input/depth_metadata', T_DEPTH_META),
                ('input/pointcloud', f'{NS}/depth/color/points'),
                ('input/pointcloud_metadata', T_DEPTH_META),
            ]),
        ComposableNode(
            package='realsense_splitter', plugin='nvblox::RealsenseSplitterNode',
            name='aligned_depth_splitter', namespace=CAMERA_NAME, parameters=[qos],
            remappings=[
                ('input/depth', T_ALIGNED), ('input/depth_metadata', T_DEPTH_META),
                ('input/infra_1', f'{unused}/infra_1'),
                ('input/infra_1_metadata', f'{unused}/infra_1_metadata'),
                ('input/infra_2', f'{unused}/infra_2'),
                ('input/infra_2_metadata', f'{unused}/infra_2_metadata'),
                ('input/pointcloud', f'{unused}/pointcloud'),
                ('input/pointcloud_metadata', f'{unused}/pointcloud_metadata'),
            ]),
    ]


# --------------------------------------------------------------------------------------------
# cuVSLAM
# --------------------------------------------------------------------------------------------
def vslam_node(cfg) -> ComposableNode:
    v = cfg['vslam']
    ir1, ir2 = vslam_image_topics(cfg)
    ir_rate = cfg['camera']['depth_fps'] / (2.0 if interleaved(cfg) else 1.0)
    jitter = float(v['image_jitter_threshold_ms']) or float(math.ceil(1.5 * 1000.0 / ir_rate))
    params = {
        'num_cameras': 2,
        'rectified_images': True,
        'enable_image_denoising': False,
        'enable_imu_fusion': bool(v['enable_imu_fusion']),
        # IMU noise model from the Isaac ROS 3.2 RealSense tutorial (Bosch BMI0xx, D4xx IMU)
        'gyro_noise_density': 0.000244,
        'gyro_random_walk': 0.000019393,
        'accel_noise_density': 0.001862,
        'accel_random_walk': 0.003,
        'calibration_frequency': 200.0,
        'image_jitter_threshold_ms': jitter,
        'map_frame': MAP_FRAME,
        'odom_frame': ODOM_FRAME,
        'base_frame': BASE_FRAME,
        'imu_frame': f'{CAMERA_NAME}_gyro_optical_frame',
        'camera_optical_frames': [f'{CAMERA_NAME}_infra1_optical_frame',
                                  f'{CAMERA_NAME}_infra2_optical_frame'],
        # RTAB-Map owns map -> odom and loop closure; cuVSLAM is pure odometry here.
        'enable_localization_n_mapping': False,
        'publish_map_to_odom_tf': False,
        'publish_odom_to_base_tf': True,
        'enable_slam_visualization': bool(v['enable_observations_view']),
        'enable_observations_view': bool(v['enable_observations_view']),
        'enable_landmarks_view': False,
        'path_max_size': 1024,
        'verbosity': int(v['verbosity']),
        'use_sim_time': cfg['use_sim_time'],
    }
    return ComposableNode(
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        name='visual_slam_node',
        parameters=[params],
        remappings=[
            ('visual_slam/image_0', ir1), ('visual_slam/camera_info_0', T_IR1_INFO),
            ('visual_slam/image_1', ir2), ('visual_slam/camera_info_1', T_IR2_INFO),
            ('visual_slam/imu', T_IMU),
        ])


# --------------------------------------------------------------------------------------------
# nvblox
# --------------------------------------------------------------------------------------------
def depth_max_for(cfg, unlimited: float) -> float:
    """cfg['depth_max_m'] (0 = no limit -> `unlimited`, for consumers that need a number)."""
    d = float(cfg.get('depth_max_m', 0.0) or 0.0)
    return d if d > 0 else unlimited


def nvblox_node(cfg) -> ComposableNode:
    n = cfg['nvblox']
    params = {
        'global_frame': ODOM_FRAME,         # integrate with smooth cuVSLAM pose (odom->base_link)
        'use_tf_transforms': True,
        'num_cameras': 1,
        'use_lidar': False,
        'use_depth': True,
        'use_color': True,
        'mapping_type': 'static_tsdf',
        'voxel_size': float(n['voxel_size']),
        'input_qos': 'SENSOR_DATA',
        'integrate_depth_rate_hz': float(n['integrate_depth_rate_hz']),
        'integrate_color_rate_hz': float(n['integrate_color_rate_hz']),
        'update_mesh_rate_hz': float(n['update_mesh_rate_hz']),
        'publish_layer_rate_hz': float(n['publish_layer_rate_hz']),
        'update_esdf_rate_hz': 0.0,         # no ESDF/costmap needed (<= 0 disables)
        'publish_esdf_distance_slice': False,
        'decay_tsdf_rate_hz': 0.0,          # static map: never decay
        'map_clearing_radius_m': float(n['map_clearing_radius_m']),
        'map_clearing_frame_id': BASE_FRAME,
        'esdf_slice_bounds_visualization_attachment_frame_id': BASE_FRAME,
        'workspace_height_bounds_visualization_attachment_frame_id': BASE_FRAME,
        'layer_visualization_exclusion_height_m': 50.0,
        'layer_visualization_exclusion_radius_m': 100.0,
        'max_back_projection_distance': depth_max_for(cfg, 7.0),
        'static_mapper.projective_integrator_max_integration_distance_m':
            depth_max_for(cfg, 7.0),
        'static_mapper.layer_streamer_exclusion_height_m': 50.0,
        'static_mapper.layer_streamer_exclusion_radius_m': 100.0,
        'use_sim_time': cfg['use_sim_time'],
    }
    return ComposableNode(
        package='nvblox_ros', plugin='nvblox::NvbloxNode', name='nvblox_node',
        parameters=[params],
        remappings=[
            ('camera_0/depth/image', nvblox_depth_topic(cfg)),
            ('camera_0/depth/camera_info', T_DEPTH_INFO),
            ('camera_0/color/image', T_COLOR),
            ('camera_0/color/camera_info', T_COLOR_INFO),
        ])


def container(cfg, nodes: List[ComposableNode]) -> ComposableNodeContainer:
    return ComposableNodeContainer(
        name=CONTAINER, namespace='', package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=nodes,
        parameters=[{'use_sim_time': cfg['use_sim_time']}],
        output='screen', emulate_tty=True)


# --------------------------------------------------------------------------------------------
# RTAB-Map
# --------------------------------------------------------------------------------------------
def rtabmap_parts(cfg, database_path: str, localization: bool, topics=None,
                  sync_interval: float = 0.01, extra_params=None):
    """rgbd_sync + rtabmap components (shared container) and the Madgwick IMU filter process.

    Returns (components, actions). External odometry from cuVSLAM (or any odom -> base_link TF).
    `topics` overrides the input names {color, color_info, depth}; `sync_interval` is the approximate
    sync window between color and depth [s]; `extra_params` are added to the RTAB-Map parameters.
    """
    topics = topics or {}
    r = cfg['rtabmap']
    sim = cfg['use_sim_time']
    use_imu = bool(r['use_imu'])
    odom_tf = str(r.get('odom_source', 'tf')) == 'tf'
    q = int(r['queue_size'])
    actions = []

    if use_imu:
        actions.append(Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
            name='imu_filter', output='screen',
            parameters=[{'use_mag': False, 'publish_tf': False, 'world_frame': 'enu',
                         'use_sim_time': sim}],
            remappings=[('imu/data_raw', T_IMU), ('imu/data', T_IMU_FILTERED)]))

    components = [ComposableNode(
        package='rtabmap_sync', plugin='rtabmap_sync::RGBDSync', name='rgbd_sync',
        namespace='rtabmap',
        parameters=[{'approx_sync': True, 'approx_sync_max_interval': float(sync_interval),
                     'topic_queue_size': q, 'sync_queue_size': q, 'use_sim_time': sim}],
        remappings=[('rgb/image', topics.get('color', T_COLOR)),
                    ('rgb/camera_info', topics.get('color_info', T_COLOR_INFO)),
                    ('depth/image', topics.get('depth', rtabmap_depth_topic(cfg)))])]

    rtab_params = {
        'frame_id': BASE_FRAME,
        'map_frame_id': MAP_FRAME,
        # Odometry = cuVSLAM. 'tf' (default): look up odom -> base_link at each image stamp with
        # fixed variances. 'topic': /visual_slam/tracking/odometry incl. its covariance, which
        # cuVSLAM sometimes publishes as zero -> RTAB-Map aborts ("Linear information X should
        # not be null", seen with the camera at rest).
        'odom_frame_id': ODOM_FRAME if odom_tf else '',
        'odom_tf_linear_variance': float(r.get('odom_linear_variance', 0.001)),
        'odom_tf_angular_variance': float(r.get('odom_angular_variance', 0.001)),
        'publish_tf': True,               # RTAB-Map publishes map -> odom
        'subscribe_rgbd': True,
        'subscribe_depth': False,
        'subscribe_rgb': False,
        'subscribe_imu': use_imu,
        'approx_sync': True,
        'topic_queue_size': q,
        'sync_queue_size': q,
        'wait_for_transform': 0.2,
        'database_path': database_path,
        'use_sim_time': sim,
        # RTAB-Map library parameters are strings
        'Rtabmap/DetectionRate': str(r['detection_rate']),
        'Kp/MaxFeatures': str(r['kp_max_features']),
        'Vis/MaxFeatures': str(r['vis_max_features']),
        'Mem/STMSize': str(r['stm_size']),
        'RGBD/LinearUpdate': str(r['linear_update']),
        'RGBD/AngularUpdate': str(r['angular_update']),
        'RGBD/ProximityBySpace': 'true',
        'RGBD/OptimizeMaxError': '3.0',
        'Reg/Strategy': '0',              # visual registration for loop closures
        'Reg/Force3DoF': 'false',
        'Optimizer/Strategy': '1',        # g2o
        'Grid/3D': 'true',
        'Grid/RayTracing': 'false',
        'Grid/CellSize': str(r['grid_cell_size']),
        'Grid/RangeMax': str(depth_max_for(cfg, 0.0)),    # 0 = unlimited in RTAB-Map
        'Kp/MaxDepth': str(float(r.get('features_max_depth', 0.0))),
        'Vis/MaxDepth': str(float(r.get('features_max_depth', 0.0))),
        'Grid/DepthDecimation': str(r['cloud_decimation']),
        # Keep everything needed to project keyframe pixels into the map later:
        # RGB + depth + calibration are stored per node (Mem/BinDataKept default true).
        'Mem/BinDataKept': 'true',
        'Mem/NotLinkedNodesKept': 'false',
    }
    rtab_params.update(extra_params or {})
    if localization:
        rtab_params.update({
            'Mem/IncrementalMemory': 'false',
            'Mem/InitWMWithAllNodes': 'true',
        })
    else:
        rtab_params.update({'Mem/IncrementalMemory': 'true'})

    remaps = [('rgbd_image', 'rgbd_image')]
    if not odom_tf:
        remaps.append(('odom', T_ODOM))
    if use_imu:
        remaps.append(('imu', T_IMU_FILTERED))

    components.append(ComposableNode(
        package='rtabmap_slam', plugin='rtabmap_slam::CoreWrapper', name='rtabmap',
        namespace='rtabmap', parameters=[rtab_params], remappings=remaps))
    return components, actions


def rviz_action(cfg, name: str) -> Node:
    suffix = cfg['rviz'].get('config_suffix', '') or ''
    path = share('rviz', f'{name}{suffix}.rviz')
    if not os.path.exists(path):
        path = share('rviz', f'{name}.rviz')
    return Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
                arguments=['-d', path],
                parameters=[{'use_sim_time': cfg['use_sim_time']}])


# --------------------------------------------------------------------------------------------
# Cross detector
# --------------------------------------------------------------------------------------------
def crosses_file_for(database_path: str) -> str:
    """/maps/<map name>.db -> /maps/<map name>_crosses.json (next to the RTAB-Map database)."""
    base, _ = os.path.splitext(database_path)
    return f'{base}_crosses.json'


def pins_file_for(database_path: str) -> str:
    """/maps/<map name>.db -> /maps/<map name>_pins.json."""
    base, _ = os.path.splitext(database_path)
    return f'{base}_pins.json'


def pins_node(cfg, pins_file: str, crosses_file: str, require_localization: bool) -> Node:
    """Pins on cross centers (RViz "Publish Point" tool) + camera->pin lines and distances."""
    return Node(package=PACKAGE, executable='pins', name='pins', output='screen', emulate_tty=True,
                parameters=[{'pins_file': pins_file, 'crosses_file': crosses_file,
                             'require_localization': require_localization,
                             'camera_frame': f'{CAMERA_NAME}_color_optical_frame',
                             'use_sim_time': cfg['use_sim_time']}])


def cross_args() -> List[DeclareLaunchArgument]:
    return [
        DeclareLaunchArgument('landmarks_file', default_value='',
                              description='cross landmark JSON; empty = /maps/<map name>_crosses.json (mapping/localization '
                                          'launches) or no file (standalone)'),
        DeclareLaunchArgument('cross_params', default_value='',
                              description='extra ROS parameter YAML for the cross detector, applied '
                                          'last (tuning without editing the package config)'),
    ]


def cross_detector_node(cfg, landmarks_file: str, require_localization: bool, start_empty: bool,
                        extra_params: str = '') -> Node:
    """cross_detector.yaml defaults < platform `cross_detector:` block < launch-derived values
    < the optional `cross_params` file. Depth follows the same rule as RTAB-Map's."""
    params = [share('config', 'cross_detector.yaml'),
              dict(cfg.get('cross_detector') or {}),
              {'depth_topic': rtabmap_depth_topic(cfg),
               'landmarks_file': landmarks_file,
               'require_localization': require_localization,
               'start_empty': start_empty,
               'use_sim_time': cfg['use_sim_time']}]
    if extra_params:
        params.append(extra_params)
    return Node(package=PACKAGE, executable='cross_detector', name='cross_detector',
                output='screen', emulate_tty=True, parameters=params)


# --------------------------------------------------------------------------------------------
# mapping.launch.py / localization.launch.py
# --------------------------------------------------------------------------------------------
def slam_args() -> List[DeclareLaunchArgument]:
    return common_args() + cross_args() + [
        DeclareLaunchArgument('with_pins', default_value='false',
                              description='true/false: pin crosses from RViz and show camera->pin '
                                          'distances (needs the cross detector or a crosses file)'),
        DeclareLaunchArgument('pins_file', default_value='',
                              description='pin JSON; empty = /maps/<map name>_pins.json'),
        DeclareLaunchArgument('with_cross_detector', default_value='false',
                              description='true/false: run the cross detector (README "Cross detection")'),
        DeclareLaunchArgument('database_path', default_value='/maps/map.db'),
        DeclareLaunchArgument('nvblox', default_value='',
                              description='true/false; empty = platform default'),
        DeclareLaunchArgument('rviz', default_value='',
                              description='true/false; empty = platform default'),
        DeclareLaunchArgument('prior_map', default_value='',
                              description='localization only: PCD of the saved map to show in '
                                          'RViz as /prior_map (made by scripts/export_map.sh)'),
    ]


def build_slam(context, localization: bool) -> list:
    cfg = load_config(context)
    db = context.perform_substitution(_lc('database_path'))
    nvblox = context.perform_substitution(_lc('nvblox'))
    rviz = context.perform_substitution(_lc('rviz'))
    use_nvblox = truthy(nvblox) if nvblox else bool(cfg['nvblox']['enable'])
    use_rviz = truthy(rviz) if rviz else bool(cfg['rviz']['enable'])
    if localization and not os.path.isfile(db):
        raise RuntimeError(f'database_path {db} does not exist (create it with scripts/map.sh)')

    if not localization and os.path.exists(db):
        os.remove(db)               # new database each mapping run (scripts/map.sh guards this)
    actions = camera_actions(cfg)
    rtab_components, rtab_actions = rtabmap_parts(cfg, db, localization=localization)
    nodes = camera_node(cfg) + splitter_nodes(cfg) + [vslam_node(cfg)]
    if use_nvblox:
        nodes.append(nvblox_node(cfg))
    actions.append(container(cfg, nodes + rtab_components))
    actions += rtab_actions
    prior = context.perform_substitution(_lc('prior_map')) if localization else ''
    if prior:
        if os.path.isfile(prior):
            actions.append(Node(
                package=PACKAGE, executable='prior_map_publisher', name='prior_map_publisher',
                parameters=[{'pcd_path': prior, 'frame_id': MAP_FRAME}], output='screen'))
        else:
            print(f'[d455_mapping] prior_map {prior} not found; localizing without the saved cloud')
    crosses = context.perform_substitution(_lc('landmarks_file')) or crosses_file_for(db)
    if truthy(context.perform_substitution(_lc('with_pins'))):
        actions.append(pins_node(
            cfg, context.perform_substitution(_lc('pins_file')) or pins_file_for(db), crosses,
            require_localization=localization))
    if truthy(context.perform_substitution(_lc('with_cross_detector'))):
        actions.append(cross_detector_node(
            cfg, crosses, require_localization=localization, start_empty=not localization,
            extra_params=context.perform_substitution(_lc('cross_params'))))
    if use_rviz:
        actions.append(rviz_action(cfg, 'localization' if localization else 'mapping'))
    return actions
