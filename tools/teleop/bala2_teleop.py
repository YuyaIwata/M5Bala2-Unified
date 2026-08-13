#!/usr/bin/env python3
"""Keyboard teleoperation for M5Bala2-Unified.

teleop_twist_keyboard is not enough for this robot: it cannot arm the controller
(/enable) and it latches the last Twist forever, which on a balancing robot means
a keystroke keeps it driving after you stop touching the keyboard.

Here the velocity expires shortly after the last keypress, so holding a key drives
and letting go stops. The robot's own /cmd_vel timeout (500ms) is the second layer
of that same protection.

Keys
  w / s     forward / backward
  a / d     turn left / right (counter-clockwise / clockwise, REP-103)
  space     stop moving (stays balancing)
  e         enable the controller
  q         disable the controller
  , / .     decrease / increase the speed magnitude
  < / >     decrease / increase the turn magnitude
  Ctrl-C    disable and quit
"""

import os
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Bool, Float32

# How long a keypress keeps the robot moving. Long enough that the terminal's own
# auto-repeat keeps a held key alive, short enough that releasing it stops.
KEY_HOLD_S = 0.7
PUBLISH_HZ = 20.0
STATUS_HZ = 5.0

MOVE_KEYS = {
    'w': (1.0, 0.0),
    's': (-1.0, 0.0),
    'a': (0.0, 1.0),
    'd': (0.0, -1.0),
}


class Teleop(Node):
    def __init__(self):
        super().__init__('bala2_teleop')
        best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.enable_pub = self.create_publisher(Bool, 'enable', 10)
        self.center_pub = self.create_publisher(Float32, 'center_angle', 10)
        self.create_subscription(Vector3, 'control', self._on_control, best_effort)
        self.create_subscription(Vector3, 'yaw', self._on_yaw, best_effort)

        self.linear_scale = 0.3
        self.angular_scale = 0.3
        self.linear = 0.0
        self.angular = 0.0
        self.last_key_time = 0.0

        self.angle = 0.0
        self.pwm = 0.0
        self.enabled = False
        self.yaw = 0.0
        self.last_control_time = 0.0

    def _on_control(self, msg):
        self.angle, self.pwm = msg.x, msg.y
        self.enabled = msg.z > 0.5
        self.last_control_time = time.time()

    def _on_yaw(self, msg):
        self.yaw = msg.x

    def set_enabled(self, value):
        # Repeat it: a single message can be lost while discovery settles, and a
        # missed enable looks exactly like a broken robot.
        for _ in range(3):
            self.enable_pub.publish(Bool(data=value))
            time.sleep(0.02)

    def stop_moving(self):
        self.linear = 0.0
        self.angular = 0.0
        self.cmd_pub.publish(Twist())

    def handle_key(self, key):
        if key in MOVE_KEYS:
            lin, ang = MOVE_KEYS[key]
            self.linear = lin * self.linear_scale
            self.angular = ang * self.angular_scale
            self.last_key_time = time.time()
        elif key == ' ':
            self.stop_moving()
        elif key == 'e':
            self.set_enabled(True)
        elif key == 'q':
            self.stop_moving()
            self.set_enabled(False)
        elif key == ',':
            self.linear_scale = max(0.05, self.linear_scale - 0.05)
        elif key == '.':
            self.linear_scale = min(1.0, self.linear_scale + 0.05)
        elif key == '<':
            self.angular_scale = max(0.05, self.angular_scale - 0.05)
        elif key == '>':
            self.angular_scale = min(1.0, self.angular_scale + 0.05)

    def publish_cmd(self):
        if time.time() - self.last_key_time > KEY_HOLD_S:
            self.linear = 0.0
            self.angular = 0.0
        msg = Twist()
        msg.linear.x = self.linear
        msg.angular.z = self.angular
        self.cmd_pub.publish(msg)

    def status_line(self):
        if time.time() - self.last_control_time > 1.0:
            state = '\033[31mNO LINK\033[0m'
        elif self.enabled:
            state = '\033[32mRUN \033[0m'
        else:
            state = '\033[33mSTOP\033[0m'
        return (f'{state}  angle {self.angle:+6.2f}  pwm {self.pwm:+5.0f}  '
                f'yaw {self.yaw:+7.1f}  cmd {self.linear:+.2f}/{self.angular:+.2f}  '
                f'scale {self.linear_scale:.2f}/{self.angular_scale:.2f}   ')


def main():
    print(__doc__)
    print('待機中: /control を受信するまで状態は NO LINK と表示されます。\n')

    rclpy.init()
    node = Teleop()

    stdin_is_tty = sys.stdin.isatty()
    if not stdin_is_tty:
        print('標準入力が端末ではありません。container run に -t を付けてください。',
              file=sys.stderr)
        return 1

    old_attrs = termios.tcgetattr(sys.stdin)
    next_pub = time.time()
    next_status = time.time()
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.005)

            if select.select([sys.stdin], [], [], 0)[0]:
                node.handle_key(sys.stdin.read(1))

            now = time.time()
            if now >= next_pub:
                next_pub = now + 1.0 / PUBLISH_HZ
                node.publish_cmd()
            if now >= next_status:
                next_status = now + 1.0 / STATUS_HZ
                sys.stdout.write('\r' + node.status_line())
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attrs)
        print('\n終了します。制御を無効化します。')
        node.stop_moving()
        node.set_enabled(False)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
