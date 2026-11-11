import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from nova_msgs.msg import ThrusterCommand


def _rotation(roll, pitch, yaw):
    cr, sr = np.cos(roll),  np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw),   np.sin(yaw)
    Rx = np.array([[1,  0,   0 ], [0,  cr, -sr], [0,  sr,  cr]])
    Ry = np.array([[cp, 0,  sp ], [0,   1,   0 ], [-sp, 0, cp]])
    Rz = np.array([[cy,-sy,  0 ], [sy,  cy,  0 ], [0,   0,  1]])
    return Rz @ Ry @ Rx


def build_tam(thruster_poses):
    """
    thruster_poses: list of (x, y, z, roll, pitch, yaw)
    Returns 6×N allocation matrix B where F_body = B @ T
    Rows: [Fx, Fy, Fz, Tx, Ty, Tz]
    """
    B = np.zeros((6, len(thruster_poses)))
    for i, (x, y, z, roll, pitch, yaw) in enumerate(thruster_poses):
        d = _rotation(roll, pitch, yaw) @ np.array([1.0, 0.0, 0.0])
        r = np.array([x, y, z])
        B[:3, i] = d
        B[3:, i] = np.cross(r, d)
    return B


# Thruster poses: (x, y, z, roll, pitch, yaw) — matches URDF exactly
# X=ileri, Y=sol, Z=yukarı  |  T1-T4: yatay 45°  |  T5-T8: dikey
THRUSTER_POSES = [
    ( 0.220, -0.150, -0.025,  0.0,  0.0,     -2.3562),  # T1 ön-sağ
    ( 0.220,  0.150, -0.025,  0.0,  0.0,      2.3562),  # T2 ön-sol
    (-0.220,  0.150, -0.025,  0.0,  0.0,      0.7854),  # T3 arka-sol
    (-0.220, -0.150, -0.025,  0.0,  0.0,     -0.7854),  # T4 arka-sağ
    ( 0.100, -0.180,  0.035,  0.0,  1.5708,   0.0   ),  # T5 dikey ön-sağ
    ( 0.100,  0.180,  0.035,  0.0,  1.5708,   0.0   ),  # T6 dikey ön-sol
    (-0.100, -0.180,  0.035,  0.0,  1.5708,   0.0   ),  # T7 dikey arka-sağ
    (-0.100,  0.180,  0.035,  0.0,  1.5708,   0.0   ),  # T8 dikey arka-sol
]

PWM_NEUTRAL = 1500.0
PWM_RANGE   = 400.0   # +/- microseconds
FORCE_MAX   = 40.0    # Newton, T200 max ~50 N


class ThrusterAllocator(Node):
    def __init__(self):
        super().__init__('thruster_allocator')
        self._B     = build_tam(THRUSTER_POSES)
        self._B_inv = np.linalg.pinv(self._B)
        self.get_logger().info(
            f'TAM built — shape {self._B.shape}, rank {np.linalg.matrix_rank(self._B)}')

        self._sub = self.create_subscription(
            Wrench, '/control/wrench_cmd', self._wrench_cb, 10)
        self._pub = self.create_publisher(
            ThrusterCommand, '/thrusters/commands', 10)

    def _wrench_cb(self, msg: Wrench):
        wrench = np.array([
            msg.force.x,  msg.force.y,  msg.force.z,
            msg.torque.x, msg.torque.y, msg.torque.z,
        ])
        forces = self._B_inv @ wrench
        forces = np.clip(forces, -FORCE_MAX, FORCE_MAX)
        pwm = PWM_NEUTRAL + (forces / FORCE_MAX) * PWM_RANGE

        out = ThrusterCommand()
        out.header.stamp = self.get_clock().now().to_msg()
        out.pwm_values = pwm.astype(np.float32).tolist()
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ThrusterAllocator()
    rclpy.spin(node)
    rclpy.shutdown()
