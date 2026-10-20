"""Record raw camera + IMU topics (MCAP) for offline re-processing.

    ros2 launch d455_mapping record.launch.py platform:=orin_nano name:=lab_run1 [camera:=true]

Drone data: add `px4:=true` to also record every /fmu/* topic (PX4 uXRCE-DDS; needs the px4_msgs
package matching the firmware in the container), or `extra_topics:=a,b` / `extra_regex:='/mavros/.*'`
for anything else. They land in the same bag, stamped with the same computer clock as the camera.

Records BEFORE the splitters (raw IR with emitter metadata, depth, aligned depth, color,
IMU, extrinsics, /tf_static) so splitting, cuVSLAM, RTAB-Map and nvblox can be re-run
offline. Output: /bags/<name> (= ./bags/<name> on the host).

Replay into mapping (any platform, e.g. a Jetson bag on the laptop):
    ros2 launch d455_mapping mapping.launch.py camera:=false use_sim_time:=true \\
        database_path:=/maps/<name>.db
    ros2 bag play /bags/<name> --clock --qos-profile-overrides-path \\
        $(ros2 pkg prefix d455_mapping)/share/d455_mapping/config/bag_qos_overrides.yaml
(scripts/replay.sh wraps both.)
"""
import re
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from d455_mapping import common as c


def setup(context):
    cfg = c.load_config(context)
    name = context.perform_substitution(LaunchConfiguration('name')) or \
        datetime.now().strftime('d455_%Y%m%d_%H%M%S')
    out_dir = context.perform_substitution(LaunchConfiguration('out_dir'))
    cmd = ['ros2', 'bag', 'record', '-s', 'mcap', '-o', f'{out_dir}/{name}']
    rec = cfg['record']
    if int(rec['max_bag_size_bytes']) > 0:
        cmd += ['-b', str(int(rec['max_bag_size_bytes']))]
    if rec.get('compression'):
        cmd += ['--compression-mode', 'file', '--compression-format', rec['compression']]
    # ros2 bag record IGNORES explicit topics when --regex is given, so with extra data everything
    # (camera topics + extras) goes into one regex; without extras the explicit list is used.
    extra = [t.strip() for t in
             context.perform_substitution(LaunchConfiguration('extra_topics')).split(',')
             if t.strip()]
    regexes = [r for r in [
        '/fmu/.*' if c.truthy(context.perform_substitution(LaunchConfiguration('px4'))) else '',
        context.perform_substitution(LaunchConfiguration('extra_regex'))] if r]
    if extra or regexes:
        parts = [re.escape(t) for t in c.RAW_TOPICS + extra] + regexes
        cmd += ['--regex', '|'.join(f'({p})' for p in parts)]
    else:
        cmd += c.RAW_TOPICS
    actions = []
    if cfg['start_camera']:
        actions = c.camera_actions(cfg) + [c.container(cfg, c.camera_node(cfg))]
    actions.append(ExecuteProcess(cmd=cmd, output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription(c.common_args() + [
        DeclareLaunchArgument('name', default_value=''),
        DeclareLaunchArgument('out_dir', default_value='/bags'),
        DeclareLaunchArgument('px4', default_value='false',
                              description='also record every /fmu/* topic (PX4 uXRCE-DDS, in + out)'),
        DeclareLaunchArgument('extra_topics', default_value='',
                              description='comma-separated extra topics, e.g. /mavros/local_position/odom'),
        DeclareLaunchArgument('extra_regex', default_value='',
                              description="regex of extra topics, e.g. '/mavros/.*'"),
        OpaqueFunction(function=setup)])
