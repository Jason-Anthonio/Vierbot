#!/usr/bin/env python3

import sys
import termios
import tty
import select
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


HELP = """
WASD Teleop (minibot)
---------------------
Controls:
  w/s : forward/backward
  a/d : turn left/right
  space or k : stop

Speed tuning during runtime:
  q/z : increase/decrease BOTH linear & angular by 10%
  e/c : increase/decrease angular only by 10%
  r/f : increase/decrease linear only by 10%

ESC or Ctrl+C to quit

Parameters (ros2):
  speed (float, default 0.25)  - linear speed (m/s)
  turn  (float, default 0.50)  - angular speed (rad/s)
  repeat_rate (float, default 10.0) - publish rate while key held (Hz)
  topic (string, default 'joy_vel') - output topic for TwistStamped (for twist_mux)
"""


class KeyboardReader:
    def __init__(self):
        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self, timeout=0.1):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def restore(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


class TeleopWASD(Node):
    def __init__(self):
        super().__init__('teleop_wasd')

        # Parameters
        self.declare_parameter('speed', 0.20)
        self.declare_parameter('turn', 0.50)
        self.declare_parameter('repeat_rate', 10.0)
        self.declare_parameter('topic', 'cmd_vel')

        self.speed = float(self.get_parameter('speed').value)
        self.turn = float(self.get_parameter('turn').value)
        self.repeat_rate = float(self.get_parameter('repeat_rate').value)
        self.topic = str(self.get_parameter('topic').value)

        self.pub = self.create_publisher(TwistStamped, self.topic, 10)

        self.keyboard = KeyboardReader()
        self.key = ''
        self.stop_requested = False

        # Current commanded velocities
        self.target_lin = 0.0
        self.target_ang = 0.0

        # Key polling thread
        self.key_thread = threading.Thread(target=self._key_loop, daemon=True)
        self.key_thread.start()

        # Publish timer
        self.timer = self.create_timer(1.0 / max(self.repeat_rate, 1e-3), self._publish_twist)

        self.get_logger().info(HELP)
        self._print_status()

    def _print_status(self):
        self.get_logger().info(f"currently: speed {self.speed:.2f} turn {self.turn:.2f}")

    def _key_loop(self):
        try:
            while rclpy.ok() and not self.stop_requested:
                key = self.keyboard.get_key(timeout=0.1)
                if not key:
                    continue

                # Quit
                if key in ['\x03', '\x1b']:
                    self.stop_requested = True
                    break

                # Movement (WASD)
                if key in ['w', 'W']:
                    self.target_lin = self.speed
                elif key in ['s', 'S']:
                    self.target_lin = -self.speed
                elif key in ['a', 'A']:
                    self.target_ang = self.turn
                elif key in ['d', 'D']:
                    self.target_ang = -self.turn
                # Stop
                elif key in [' ', 'k', 'K']:
                    self.target_lin = 0.0
                    self.target_ang = 0.0
                # Speed scaling
                elif key in ['q', 'Q']:
                    self.speed *= 1.1
                    self.turn *= 1.1
                    self._print_status()
                elif key in ['z', 'Z']:
                    self.speed *= 0.9
                    self.turn *= 0.9
                    self._print_status()
                elif key in ['r', 'R']:
                    self.speed *= 1.1
                    self._print_status()
                elif key in ['f', 'F']:
                    self.speed *= 0.9
                    self._print_status()
                elif key in ['e', 'E']:
                    self.turn *= 1.1
                    self._print_status()
                elif key in ['c', 'C']:
                    self.turn *= 0.9
                    self._print_status()

        except Exception as e:
            self.get_logger().error(f"Key loop error: {e}")
        finally:
            try:
                self.keyboard.restore()
            except Exception:
                pass

    def _publish_twist(self):
        # Publish TwistStamped at fixed rate
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        # No fixed frame needed for TwistStamped
        msg.twist.linear.x = float(self.target_lin)
        msg.twist.angular.z = float(self.target_ang)
        self.pub.publish(msg)

    def destroy_node(self):
        self.stop_requested = True
        time.sleep(0.05)
        super().destroy_node()


def main(argv=None):
    rclpy.init(args=argv)
    node = TeleopWASD()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
