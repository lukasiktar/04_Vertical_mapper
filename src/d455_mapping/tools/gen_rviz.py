#!/usr/bin/env python3
"""Generate rviz/*.rviz from one definition so all configs stay consistent.

    python3 src/d455_mapping/tools/gen_rviz.py      # rewrites src/d455_mapping/rviz/*.rviz

Local configs (laptop): raw color image, RTAB-Map MapCloud, nvblox mesh, cross detector markers + debug image.
*_remote configs (RViz on the laptop, robot = Jetson over Wi-Fi): compressed image,
no MapCloud/raw clouds (graph + paths + mesh only).
"""
import os

import yaml

RVIZ_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'rviz')


def topic(name, reliable=True, depth=5, transient=False):
    return {'Value': name, 'Depth': depth, 'History Policy': 'Keep Last',
            'Reliability Policy': 'Reliable' if reliable else 'Best Effort',
            'Durability Policy': 'Transient Local' if transient else 'Volatile'}


def grid():
    return {'Class': 'rviz_default_plugins/Grid', 'Name': 'Grid', 'Enabled': True,
            'Reference Frame': '<Fixed Frame>', 'Plane': 'XY', 'Cell Size': 1,
            'Plane Cell Count': 20, 'Color': '160; 160; 164', 'Alpha': 0.5, 'Value': True}


def tf():
    return {'Class': 'rviz_default_plugins/TF', 'Name': 'TF', 'Enabled': True,
            'Show Names': True, 'Show Axes': True, 'Show Arrows': False,
            'Marker Scale': 0.3, 'Update Interval': 0, 'Frame Timeout': 15, 'Value': True,
            'Frames': {'All Enabled': False,
                       **{f: {'Value': True} for f in
                          ('map', 'odom', 'base_link', 'camera_link',
                           'camera_color_optical_frame')}}}


def image(name, t, reliable=True, enabled=True):
    return {'Class': 'rviz_default_plugins/Image', 'Name': name, 'Enabled': enabled,
            'Topic': topic(t, reliable=reliable, depth=1), 'Normalize Range': True,
            'Max Value': 1, 'Min Value': 0, 'Median window': 5, 'Value': enabled}


def path(name, t, color, enabled=True):
    return {'Class': 'rviz_default_plugins/Path', 'Name': name, 'Enabled': enabled,
            'Topic': topic(t), 'Color': color, 'Line Style': 'Lines', 'Line Width': 0.03,
            'Alpha': 1, 'Buffer Length': 1, 'Pose Style': 'None', 'Value': enabled}


def map_cloud(enabled=True, decimation=4, max_depth=4.0):
    return {'Class': 'rtabmap_rviz_plugins/MapCloud', 'Name': 'RTAB-Map MapCloud',
            'Enabled': enabled, 'Topic': topic('/rtabmap/mapData', depth=1),
            'Cloud decimation': decimation, 'Cloud max depth (m)': max_depth,
            'Cloud min depth (m)': 0, 'Cloud voxel size (m)': 0.02,
            'Filter floor (m)': 0, 'Filter ceiling (m)': 0, 'Node filtering radius (m)': 0,
            'Style': 'Points', 'Size (Pixels)': 2, 'Size (m)': 0.01, 'Alpha': 1,
            'Color Transformer': 'RGB8', 'Position Transformer': 'XYZ',
            'Download map': False, 'Download graph': False, 'Value': enabled}


def map_graph(enabled=True):
    return {'Class': 'rtabmap_rviz_plugins/MapGraph', 'Name': 'RTAB-Map Graph',
            'Enabled': enabled, 'Topic': topic('/rtabmap/mapGraph', depth=1),
            'Alpha': 1, 'Color': '0; 255; 0', 'Value': enabled}


def nvblox_mesh(enabled=True):
    return {'Class': 'nvblox_rviz_plugin/NvbloxMesh', 'Name': 'nvblox Mesh',
            'Enabled': enabled, 'Cut Ceiling': False,
            'Topic': {**topic('/nvblox_node/mesh'), 'Filter size': 10}, 'Value': enabled}


def nvblox_color_layer(enabled=False):
    return {'Class': 'nvblox_rviz_plugin/NvbloxVoxelBlockLayer', 'Name': 'nvblox Color Voxels',
            'Enabled': enabled, 'Cut Ceiling': False,
            'Topic': {**topic('/nvblox_node/color_layer'), 'Filter size': 10},
            'Value': enabled}


def pose_cov(name, t, enabled=True):
    return {'Class': 'rviz_default_plugins/PoseWithCovariance', 'Name': name,
            'Enabled': enabled, 'Topic': topic(t), 'Shape': 'Arrow',
            'Color': '255; 25; 0', 'Shaft Length': 0.4, 'Head Length': 0.15,
            'Covariance': {'Value': True, 'Position': {'Value': True, 'Alpha': 0.3,
                                                          'Scale': 1, 'Color': '204; 51; 204'},
                           'Orientation': {'Value': False}},
            'Value': enabled}


def pointcloud(name, t, enabled=True, size=0.02, transient=False):
    return {'Class': 'rviz_default_plugins/PointCloud2', 'Name': name, 'Enabled': enabled,
            'Topic': topic(t, depth=1, transient=transient), 'Style': 'Points', 'Size (Pixels)': 2,
            'Size (m)': size, 'Alpha': 1, 'Decay Time': 0, 'Color Transformer': 'RGB8',
            'Position Transformer': 'XYZ', 'Use Fixed Frame': True, 'Value': enabled}


def marker(name, t, enabled=True):
    return {'Class': 'rviz_default_plugins/Marker', 'Name': name, 'Enabled': enabled,
            'Topic': topic(t, transient=True, depth=1), 'Namespaces': {}, 'Value': enabled}


def marker_array(name, t, enabled=True):
    # the node publishes volatile + reliable, so the display must not ask for transient local
    return {'Class': 'rviz_default_plugins/MarkerArray', 'Name': name, 'Enabled': enabled,
            'Topic': topic(t, depth=5), 'Namespaces': {}, 'Value': enabled}


def cross_displays(remote: bool):
    """Cross detector (with_cross_detector:=true): green live detections in the camera frame,
    orange confirmed landmarks in `map`, debug image (2D candidates red, verified green).
    The node only renders the debug image while it is subscribed, so remote leaves it off."""
    return [marker_array('Crosses (live)', '/cross_detector/live_markers'),
            marker_array('Crosses (landmarks)', '/cross_detector/landmarks'),
            image('Cross debug image', '/cross_detector/debug_image', reliable=False,
                  enabled=not remote)]


def pins_displays():
    """Pins (magenta), camera->pin lines and distances (cyan), from the `pins` node."""
    return [marker_array('Pins and distances', '/pins/markers')]


def config(displays, fixed='map'):
    return {
        'Panels': [
            {'Class': 'rviz_common/Displays', 'Name': 'Displays', 'Help Height': 0,
             'Tree Height': 600},
            {'Class': 'rviz_common/Views', 'Name': 'Views'},
            {'Class': 'rviz_common/Time', 'Name': 'Time', 'SyncMode': 0, 'SyncSource': ''},
        ],
        'Visualization Manager': {
            'Class': '', 'Name': 'root', 'Enabled': True,
            'Displays': displays,
            'Global Options': {'Fixed Frame': fixed, 'Background Color': '48; 48; 48',
                               'Frame Rate': 30},
            'Tools': [{'Class': 'rviz_default_plugins/Interact', 'Hide Inactive Objects': True},
                      {'Class': 'rviz_default_plugins/MoveCamera'},
                      {'Class': 'rviz_default_plugins/Select'},
                      # click a cross to pin/unpin it (topic /clicked_point, read by the `pins` node)
                      {'Class': 'rviz_default_plugins/PublishPoint', 'Single click': True,
                       'Topic': topic('/clicked_point', depth=5)},
                      {'Class': 'rviz_default_plugins/FocusCamera'},
                      {'Class': 'rviz_default_plugins/Measure', 'Line color': '128; 128; 0'}],
            'Transformation': {'Current': {'Class': 'rviz_default_plugins/TF'}},
            'Value': True,
            'Views': {'Current': {
                'Class': 'rviz_default_plugins/Orbit', 'Name': 'Current View',
                'Target Frame': '<Fixed Frame>', 'Distance': 8.0,
                'Focal Point': {'X': 0.0, 'Y': 0.0, 'Z': 0.0},
                'Pitch': 0.7, 'Yaw': 3.6, 'Near Clip Distance': 0.01,
                'Focal Shape Fixed Size': True, 'Focal Shape Size': 0.05,
                'Invert Z Axis': False, 'Value': 'Orbit (rviz)'}, 'Saved': None},
        },
        'Window Geometry': {'Width': 1600, 'Height': 1000, 'Hide Left Dock': False,
                            'Hide Right Dock': True, 'X': 0, 'Y': 0},
    }


def slam_displays(remote: bool, localization: bool, depth_max: float = 4.0):
    d = [grid(), tf()]
    if remote:
        d.append(image('Color (compressed)', '/camera/color/image_raw/compressed',
                       reliable=False))
    else:
        d.append(image('Color', '/camera/color/image_raw'))
        d.append(image('IR1 emitter-off (cuVSLAM input)',
                       '/camera/realsense_splitter_node/output/infra_1', enabled=False))
    d.append(path('cuVSLAM odom path', '/visual_slam/tracking/vo_path', '255; 170; 0'))
    d.append(path('RTAB-Map path', '/rtabmap/mapPath', '0; 255; 255'))
    d.append(map_graph())
    d.append(map_cloud(enabled=not remote, decimation=8 if remote else 4, max_depth=depth_max))
    d.append(nvblox_mesh(enabled=True))
    d.append(nvblox_color_layer(enabled=False))
    if localization:
        d.append(pose_cov('Localization pose', '/rtabmap/localization_pose'))
        if not remote:
            # whole saved map (scripts/localize.sh publishes it once, latched)
            d.append(pointcloud('Saved map (prior cloud)', '/prior_map', size=0.015, transient=True))
    d += cross_displays(remote)
    d += pins_displays()
    return d


def depth_max(platform):
    """depth_max_m of config/<platform>.yaml (0/absent -> 4 m viewer default)"""
    path = os.path.join(RVIZ_DIR, '..', 'config', f'{platform}.yaml')
    with open(path, encoding='utf-8') as f:
        return float(yaml.safe_load(f).get('depth_max_m') or 0.0) or 4.0


def main():
    dl, do = depth_max('laptop'), depth_max('orin_nano')
    files = {
        'mapping.rviz': config(slam_displays(remote=False, localization=False, depth_max=dl)),
        'mapping_remote.rviz': config(slam_displays(remote=True, localization=False, depth_max=do)),
        'localization.rviz': config(slam_displays(remote=False, localization=True, depth_max=dl)),
        'localization_remote.rviz': config(slam_displays(remote=True, localization=True, depth_max=do)),
        'show_map.rviz': config([grid(), tf(), pointcloud('Prior map', '/prior_map', transient=True),
                                 marker('Prior map mesh', '/prior_map_mesh')]),
    }
    os.makedirs(RVIZ_DIR, exist_ok=True)
    for name, cfg in files.items():
        with open(os.path.join(RVIZ_DIR, name), 'w', encoding='utf-8') as f:
            f.write('# Generated by tools/gen_rviz.py -- edit that file, not this one.\n')
            yaml.safe_dump(cfg, f, sort_keys=False, default_flow_style=False)
        print('wrote', name)


if __name__ == '__main__':
    main()
