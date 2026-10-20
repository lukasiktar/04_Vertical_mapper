"""Make sure the D455 emitter on/off interleave is really running.

The camera keeps its emitter state between sessions, and the driver's startup set of
depth_module.emitter_on_off can leave it stuck "on" (every frame labelled frame_emitter_mode=1,
so the splitter forwards no depth and no emitter-off IR). Toggling the parameter off and on
while streaming restarts the interleave. This node checks the metadata labels after startup and
does that toggle, up to --retries times. It exits when the labels alternate (or gives up with an
error), so it can simply be launched next to the camera.

    ros2 run d455_mapping emitter_ensure
"""
import argparse
import json
import time

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from realsense2_camera_msgs.msg import Metadata

META = '/camera/infra1/metadata'
CAMERA = '/camera/camera'
PARAM = 'depth_module.emitter_on_off'


class Ensure(Node):

    def __init__(self):
        super().__init__('emitter_ensure')
        self.labels = []
        self.create_subscription(Metadata, META, self._meta, 100)
        self.client = self.create_client(SetParameters, f'{CAMERA}/set_parameters')

    def _meta(self, msg):
        self.labels.append(json.loads(msg.json_data).get('frame_emitter_mode'))

    def sample(self, seconds):
        self.labels.clear()
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return set(self.labels), len(self.labels)

    def set_param(self, value):
        param = Parameter(name=PARAM, value=ParameterValue(
            type=ParameterType.PARAMETER_BOOL, bool_value=value))
        req = SetParameters.Request(parameters=[param])
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        res = future.result()
        return bool(res and res.results and res.results[0].successful)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--retries', type=int, default=4)
    ap.add_argument('--startup-timeout', type=float, default=60.0)
    args, _ = ap.parse_known_args()
    rclpy.init()
    node = Ensure()
    log = node.get_logger()

    # wait for the camera to stream metadata
    deadline = time.time() + args.startup_timeout
    seen = set()
    while time.time() < deadline:
        seen, n = node.sample(1.5)
        if n >= 10 and node.client.service_is_ready():
            break
    else:
        log.error(f'no frames on {META} within {args.startup_timeout:.0f} s; giving up')
        raise SystemExit(1)

    for attempt in range(args.retries + 1):
        seen, n = node.sample(1.5)
        if {0, 1} <= seen:
            log.info(f'emitter interleave OK (labels {sorted(seen)}, {n} frames)'
                     + (f' after {attempt} toggle(s)' if attempt else ''))
            node.destroy_node()
            rclpy.try_shutdown()
            return
        if attempt == args.retries:
            break
        log.warn(f'emitter labels {sorted(seen)} do not alternate; toggling {PARAM} '
                 f'(attempt {attempt + 1}/{args.retries})')
        node.set_param(False)
        node.sample(1.5)
        node.set_param(True)
        node.sample(1.5)
    log.error(f'emitter interleave NOT running after {args.retries} toggles (labels {sorted(seen)}). '
              'Re-plug the camera, or set camera.emitter_on_off: false')
    raise SystemExit(1)
