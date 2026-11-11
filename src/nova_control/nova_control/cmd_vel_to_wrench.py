import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Wrench

# Twist linear/angular birimleri → Newton/Nm dönüşüm katsayıları
FORCE_SCALE  = 20.0   # N   per unit linear velocity command
TORQUE_SCALE = 3.0    # N·m per unit angular velocity command


class CmdVelToWrench(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_wrench')
        self._sub = self.create_subscription(
            Twist, '/cmd_vel', self._cb, 10)
        self._pub = self.create_publisher(
            Wrench, '/control/wrench_cmd', 10)

    def _cb(self, msg: Twist):
        w = Wrench()
        w.force.x  = msg.linear.x  * FORCE_SCALE
        w.force.y  = msg.linear.y  * FORCE_SCALE
        w.force.z  = msg.linear.z  * FORCE_SCALE
        w.torque.x = msg.angular.x * TORQUE_SCALE
        w.torque.y = msg.angular.y * TORQUE_SCALE
        w.torque.z = msg.angular.z * TORQUE_SCALE
        self._pub.publish(w)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToWrench()
    rclpy.spin(node)
    rclpy.shutdown()
