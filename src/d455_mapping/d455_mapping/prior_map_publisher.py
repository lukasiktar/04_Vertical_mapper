"""Publish a saved point cloud (binary PCD from scripts/export_map.sh) ONCE as a latched
PointCloud2, so RViz shows the whole saved map, e.g. while localizing.

    ros2 run d455_mapping prior_map_publisher --ros-args -p pcd_path:=/maps/lab/lab_cloud.pcd

Topic /prior_map, frame `map`, QoS transient_local + reliable (a viewer that connects later still
gets the cloud; nothing is re-sent). Only x, y, z and rgb are published (16 bytes per point).
Parameters: pcd_path, frame_id (map), topic (/prior_map), max_points (0 = all; otherwise a regular
subsampling to at most this many points, useful over Wi-Fi).
"""
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField

_NP_TYPES = {('F', 4): np.float32, ('F', 8): np.float64, ('U', 1): np.uint8, ('U', 2): np.uint16,
             ('U', 4): np.uint32, ('I', 1): np.int8, ('I', 2): np.int16, ('I', 4): np.int32}


def read_pcd(path):
    """Return a structured numpy array of the points in a binary PCD file."""
    with open(path, 'rb') as f:
        header = {}
        while True:
            line = f.readline().decode('ascii', 'ignore').strip()
            if not line:
                raise ValueError(f'{path}: truncated PCD header')
            if line.startswith('#'):
                continue
            key, _, value = line.partition(' ')
            header[key.upper()] = value
            if key.upper() == 'DATA':
                break
        if header['DATA'].strip() != 'binary':
            raise ValueError(f"{path}: DATA {header['DATA']!r} not supported (need binary; "
                             'scripts/export_map.sh writes binary PCDs)')
        names = header['FIELDS'].split()
        sizes = [int(v) for v in header['SIZE'].split()]
        types = header['TYPE'].split()
        counts = [int(v) for v in header.get('COUNT', ' '.join('1' for _ in names)).split()]
        if any(c != 1 for c in counts):
            raise ValueError(f'{path}: multi-count fields not supported')
        dtype = np.dtype([(n, _NP_TYPES[(t, s)]) for n, t, s in zip(names, types, sizes)])
        n = int(header['POINTS'])
        return np.frombuffer(f.read(n * dtype.itemsize), dtype=dtype, count=n)


class PriorMapPublisher(Node):

    def __init__(self):
        super().__init__('prior_map_publisher')
        path = os.path.abspath(self.declare_parameter('pcd_path', '').value)
        frame = self.declare_parameter('frame_id', 'map').value
        topic = self.declare_parameter('topic', '/prior_map').value
        max_points = int(self.declare_parameter('max_points', 0).value)
        pts = read_pcd(path)
        ok = np.isfinite(pts['x']) & np.isfinite(pts['y']) & np.isfinite(pts['z'])
        pts = pts[ok]
        if max_points > 0 and len(pts) > max_points:
            pts = pts[np.linspace(0, len(pts) - 1, max_points).astype(np.int64)]
        out = np.empty(len(pts), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<f4')])
        for name in ('x', 'y', 'z'):
            out[name] = pts[name]
        out['rgb'] = pts['rgb'] if 'rgb' in pts.dtype.names else np.float32(0)
        msg = PointCloud2()
        msg.header.frame_id = frame
        msg.height, msg.width = 1, len(out)
        msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate(('x', 'y', 'z', 'rgb'))]
        msg.is_bigendian, msg.point_step = False, 16
        msg.row_step, msg.is_dense = 16 * len(out), True
        msg.data = out.tobytes()
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self._pub = self.create_publisher(PointCloud2, topic, qos)
        msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(msg)
        self._msg = msg           # keep alive: the middleware serves it to late subscribers
        self.get_logger().info(f'published {len(out)} points from {path} on {topic} '
                               f'(frame {frame}, once, latched)')


def main():
    rclpy.init()
    node = PriorMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
