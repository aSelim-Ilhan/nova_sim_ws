"""
Sualtı hidrodinamik sürükleme kuvveti.
F_drag = -0.5 * rho * Cd * A * v * |v|  (quadratic drag)

Gazebo'dan model hızını alır, body frame'de drag hesaplar,
/nova/gazebo_ros_force topic'ine ekler.
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from gazebo_msgs.srv import GetEntityState

# Su yoğunluğu (tuzlu su)
RHO = 1025.0

# Araç boyutları: 0.625 × 0.5 × 0.35 m
# Drag katsayısı × kesit alanı her eksen için
# F = 0.5 * RHO * (Cd*A) * v * |v|
CDA = {
    'x': 0.10 * (0.500 * 0.350),   # surge : Cd=0.10, A=0.175 m²
    'y': 0.80 * (0.625 * 0.350),   # sway  : Cd=0.80, A=0.219 m²
    'z': 0.80 * (0.625 * 0.500),   # heave : Cd=0.80, A=0.313 m²
    'rx': 0.012,                     # roll
    'ry': 0.018,                     # pitch
    'rz': 0.018,                     # yaw
}

ENTITY   = 'nova::base_link'
HZ       = 50


def _quat_to_rot(q):
    """Quaternion → 3×3 rotasyon matrisi (world→body için transpoz alınır)."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y-w*z),   2*(x*z+w*y)],
        [  2*(x*y+w*z), 1-2*(x*x+z*z),   2*(y*z-w*x)],
        [  2*(x*z-w*y),   2*(y*z+w*x), 1-2*(x*x+y*y)],
    ])


def _drag(v, cda):
    if not math.isfinite(v):
        return 0.0
    return -0.5 * RHO * cda * v * abs(v)


class Hydrodynamics(Node):
    def __init__(self):
        super().__init__('hydrodynamics')

        self._cli = self.create_client(GetEntityState, '/get_entity_state')
        while not self._cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Gazebo state servisi bekleniyor...')

        self._pub = self.create_publisher(Wrench, '/nova/drag_wrench', 10)

        # Force plugin iki ayrı topic'i toplayamaz — combine edelim
        self._thrust_sub = self.create_subscription(
            Wrench, '/control/wrench_cmd', self._thrust_cb, 10)
        self._force_pub = self.create_publisher(
            Wrench, '/nova/gazebo_ros_force', 10)

        self._thrust = Wrench()
        self._timer  = self.create_timer(1.0 / HZ, self._tick)
        self.get_logger().info('Hydrodynamics node hazır')

    def _thrust_cb(self, msg):
        self._thrust = msg

    def _tick(self):
        req = GetEntityState.Request()
        req.name             = ENTITY
        req.reference_frame  = ''   # world frame
        future = self._cli.call_async(req)
        future.add_done_callback(self._state_cb)

    def _state_cb(self, future):
        try:
            resp = future.result()
        except Exception:
            return
        if not resp.success:
            return

        pose  = resp.state.pose
        twist = resp.state.twist

        v_world = np.array([twist.linear.x,  twist.linear.y,  twist.linear.z])
        w_world = np.array([twist.angular.x, twist.angular.y, twist.angular.z])

        # NaN veya aşırı değerleri filtrele (Gazebo bazen geçersiz state döner)
        if (not np.all(np.isfinite(v_world)) or not np.all(np.isfinite(w_world))
                or np.linalg.norm(v_world) > 20.0
                or np.linalg.norm(w_world) > 50.0):
            return

        q = pose.orientation
        if not all(math.isfinite(v) for v in [q.x, q.y, q.z, q.w]):
            return

        # World frame hız → body frame
        R = _quat_to_rot(q)
        if not np.all(np.isfinite(R)):
            return

        v_body = R.T @ v_world   # world → body  (R.T = world→body)
        w_body = R.T @ w_world

        # Quadratic drag (body frame)
        fd_body_f = np.array([
            _drag(v_body[0], CDA['x']),
            _drag(v_body[1], CDA['y']),
            _drag(v_body[2], CDA['z']),
        ])
        fd_body_t = np.array([
            _drag(w_body[0], CDA['rx']),
            _drag(w_body[1], CDA['ry']),
            _drag(w_body[2], CDA['rz']),
        ])

        # Thrust is in body frame → rotate to world frame  (R = body→world)
        thrust_f = np.array([self._thrust.force.x,  self._thrust.force.y,  self._thrust.force.z])
        thrust_t = np.array([self._thrust.torque.x, self._thrust.torque.y, self._thrust.torque.z])

        # Su yüzeyi kısıtı: z > 0 ise aşağı iten restorasyon kuvveti (world frame)
        z_pos = pose.position.z
        SURFACE_Z = 5.0
        surface_force_z = -200.0 * max(0.0, z_pos - SURFACE_Z) - 50.0 * max(0.0, v_world[2]) * float(z_pos > SURFACE_Z - 0.5)

        # Combine in world frame and publish
        total_f = R @ (thrust_f + fd_body_f)
        total_t = R @ (thrust_t + fd_body_t)
        total_f[2] += surface_force_z

        total = Wrench()
        total.force.x,  total.force.y,  total.force.z  = total_f
        total.torque.x, total.torque.y, total.torque.z = total_t

        self._force_pub.publish(total)

        spd = np.linalg.norm(v_body)
        if spd > 0.01:
            self.get_logger().info(
                f'v={spd:.2f}m/s  dx={fd_body_f[0]:.1f}N tx={self._thrust.force.x:.1f}N  dz={fd_body_f[2]:.1f}N tz={self._thrust.force.z:.1f}N',
                throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = Hydrodynamics()
    rclpy.spin(node)
    rclpy.shutdown()
