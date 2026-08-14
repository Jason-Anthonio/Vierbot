#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Control Your VierBot / Minibot!
---------------------------
Moving around:
   w
a  s  d
   x

w/x : increase/decrease linear velocity (~ 0.05 m/s)
a/d : increase/decrease angular velocity (~ 0.1 rad/s)
space key, s : force stop

CTRL-C to quit
"""

moveBindings = {
    'w': (1, 0),
    'a': (0, 1),
    'd': (0, -1),
    'x': (-1, 0),
    's': (0, 0),
    ' ': (0, 0),
}

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = rclpy.create_node('teleop_wasd')
    pub = node.create_publisher(Twist, '/cmd_vel_keyboard', 10)

    speed = 0.25
    turn = 0.8
    x = 0.0
    th = 0.0

    try:
        print(msg)
        while True:
            key = getKey(settings)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
            elif key == '':
                break
            else:
                x = 0.0
                th = 0.0

            twist = Twist()
            twist.linear.x = x * speed
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = th * turn
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
