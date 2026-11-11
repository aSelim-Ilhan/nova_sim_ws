"""
NOVA AUV Teleop
  W/S  : ileri / geri
  A/D  : sol yengeç / sağ yengeç
  Q/E  : sola dön / sağa dön (yaw)
  Y/H  : yukarı / aşağı
  Space: dur
"""
import sys
import tty
import termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

SPEED_DEFAULT = 1.0
SPEED_STEP    = 0.1
SPEED_MIN     = 0.1
SPEED_MAX     = 5.0

BANNER = """
━━━━━━━━━━━━━━━━━━━━━━━━━━
  NOVA AUV Teleop
━━━━━━━━━━━━━━━━━━━━━━━━━━
  W        : İleri
  S        : Geri
  A        : Sol yengeç
  D        : Sağ yengeç
  Q        : Sola dön (yaw)
  E        : Sağa dön (yaw)
  Y        : Yukarı
  H        : Aşağı
  T        : Hız artır
  G        : Hız azalt
  Space    : Dur
  Ctrl+C   : Çıkış
━━━━━━━━━━━━━━━━━━━━━━━━━━
"""


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class TeleopNode(Node):
    def __init__(self):
        super().__init__('nova_teleop')
        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        print(BANNER)

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        speed = SPEED_DEFAULT
        print(f'  Hız: {speed:.1f} m/s')
        try:
            while rclpy.ok():
                key = get_key(settings).lower()
                if key == '\x03':
                    break

                twist = Twist()
                action = ''

                if key == 't':
                    speed = min(speed + SPEED_STEP, SPEED_MAX)
                    action = f'Hız artır → {speed:.1f}'
                elif key == 'g':
                    speed = max(speed - SPEED_STEP, SPEED_MIN)
                    action = f'Hız azalt → {speed:.1f}'
                elif key == 'w':
                    twist.linear.x  =  speed; action = 'İleri'
                elif key == 's':
                    twist.linear.x  = -speed; action = 'Geri'
                elif key == 'a':
                    twist.linear.y  =  speed; action = 'Sol yengeç'
                elif key == 'd':
                    twist.linear.y  = -speed; action = 'Sağ yengeç'
                elif key == 'y':
                    twist.linear.z  =  speed; action = 'Yukarı'
                elif key == 'h':
                    twist.linear.z  = -speed; action = 'Aşağı'
                elif key == 'q':
                    twist.angular.z =  speed; action = 'Sola dön'
                elif key == 'e':
                    twist.angular.z = -speed; action = 'Sağa dön'
                elif key == ' ':
                    action = 'Dur'

                print(f'\r  {action:<25} [hız: {speed:.1f}]', end='', flush=True)
                self._pub.publish(twist)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            self._pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    node.run()
    rclpy.shutdown()
