import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Wrench
from nova_msgs.msg import DvlData, PressureData


class PidController(Node):
    """
    Axis-decoupled PID controllers for depth, heading and surge.
    Outputs a Wrench command consumed by ThrusterAllocator.
    """

    def __init__(self):
        super().__init__('pid_controller')

        # PID gains (tune per axis)
        self.declare_parameter('depth.kp', 20.0)
        self.declare_parameter('depth.ki', 0.1)
        self.declare_parameter('depth.kd', 5.0)
        self.declare_parameter('heading.kp', 15.0)
        self.declare_parameter('heading.ki', 0.05)
        self.declare_parameter('heading.kd', 3.0)
        self.declare_parameter('surge.kp', 10.0)
        self.declare_parameter('surge.ki', 0.0)
        self.declare_parameter('surge.kd', 2.0)

        self._depth_setpoint   = 0.0
        self._heading_setpoint = 0.0
        self._surge_setpoint   = 0.0

        self._depth_err_i  = 0.0
        self._heading_err_i = 0.0
        self._surge_err_i  = 0.0
        self._prev_depth_err   = 0.0
        self._prev_heading_err = 0.0
        self._prev_surge_err   = 0.0

        self._current_depth   = 0.0
        self._current_heading = 0.0
        self._current_surge   = 0.0

        self.create_subscription(PressureData, '/pressure/data', self._pressure_cb, 10)
        self.create_subscription(DvlData, '/dvl/data', self._dvl_cb, 10)
        self.create_subscription(Float64, '/setpoint/depth',   self._depth_sp_cb, 10)
        self.create_subscription(Float64, '/setpoint/heading', self._heading_sp_cb, 10)
        self.create_subscription(Float64, '/setpoint/surge',   self._surge_sp_cb, 10)

        self._wrench_pub = self.create_publisher(Wrench, '/control/wrench_cmd', 10)
        self.create_timer(0.02, self._control_loop)  # 50 Hz

    def _pressure_cb(self, msg): self._current_depth = msg.depth_m
    def _dvl_cb(self, msg):      self._current_surge = msg.velocity.x
    def _depth_sp_cb(self, msg):   self._depth_setpoint   = msg.data
    def _heading_sp_cb(self, msg): self._heading_setpoint = msg.data
    def _surge_sp_cb(self, msg):   self._surge_setpoint   = msg.data

    def _pid(self, err, err_i, prev_err, kp, ki, kd, dt=0.02):
        err_i += err * dt
        derr   = (err - prev_err) / dt
        return kp * err + ki * err_i + kd * derr, err_i, err

    def _control_loop(self):
        depth_err   = self._depth_setpoint   - self._current_depth
        heading_err = self._heading_setpoint - self._current_heading
        surge_err   = self._surge_setpoint   - self._current_surge

        fz, self._depth_err_i,   self._prev_depth_err   = self._pid(
            depth_err,   self._depth_err_i,   self._prev_depth_err,
            self.get_parameter('depth.kp').value,
            self.get_parameter('depth.ki').value,
            self.get_parameter('depth.kd').value)

        tz, self._heading_err_i, self._prev_heading_err = self._pid(
            heading_err, self._heading_err_i, self._prev_heading_err,
            self.get_parameter('heading.kp').value,
            self.get_parameter('heading.ki').value,
            self.get_parameter('heading.kd').value)

        fx, self._surge_err_i,   self._prev_surge_err   = self._pid(
            surge_err,   self._surge_err_i,   self._prev_surge_err,
            self.get_parameter('surge.kp').value,
            self.get_parameter('surge.ki').value,
            self.get_parameter('surge.kd').value)

        wrench = Wrench()
        wrench.force.x  = fx
        wrench.force.z  = fz
        wrench.torque.z = tz
        self._wrench_pub.publish(wrench)


def main(args=None):
    rclpy.init(args=args)
    node = PidController()
    rclpy.spin(node)
    rclpy.shutdown()
