"""Publish a mesh file (.dae/.stl) as a latched visualization_msgs/Marker MESH_RESOURCE."""
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker


class MeshMarkerPublisher(Node):

    def __init__(self):
        super().__init__('mesh_marker_publisher')
        path = os.path.abspath(self.declare_parameter('mesh_path', '').value)
        frame_id = self.declare_parameter('frame_id', 'map').value
        if not os.path.isfile(path):
            raise FileNotFoundError(f'mesh_path {path!r} does not exist')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub = self.create_publisher(Marker, 'mesh_marker', qos)
        m = Marker()
        m.header.frame_id = frame_id
        m.ns = 'prior_map'
        m.type = Marker.MESH_RESOURCE
        m.action = Marker.ADD
        m.mesh_resource = f'file://{path}'
        m.mesh_use_embedded_materials = True
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 1.0
        # Used only if the mesh has no embedded colors/materials.
        m.color.r = m.color.g = m.color.b = 0.8
        m.color.a = 1.0
        self._marker = m
        self.create_timer(2.0, self._publish)
        self._publish()
        self.get_logger().info(f'publishing {path} in frame {frame_id}')

    def _publish(self):
        self._marker.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(self._marker)


def main():
    rclpy.init()
    node = MeshMarkerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
