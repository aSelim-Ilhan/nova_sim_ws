"""
Gazebo Classic bridge: /control/wrench_cmd → /nova/body_wrench topic
libgazebo_ros_force.so plugin bu topic'i dinleyip base_link'e uygular.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench


class GazeboBridge(Node):
    def __init__(self):
        super().__init__('gazebo_bridge')
        self._sub = self.create_subscription(
            Wrench, '/control/wrench_cmd', self._cb, 10)
        self._pub = self.create_publisher(
            Wrench, '/nova/gazebo_ros_force', 10)
        self.get_logger().info('Gazebo bridge hazır → /nova/gazebo_ros_force')

    def _cb(self, msg: Wrench):
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboBridge()
    rclpy.spin(node)
    rclpy.shutdown()
