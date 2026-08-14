import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistUnstamper(Node):
    def __init__(self):
        super().__init__('twist_unstamper')
        self.sub = self.create_subscription(TwistStamped, 'cmd_vel_in', self.callback, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel_out', 10)

    def callback(self, msg: TwistStamped):
        self.pub.publish(msg.twist)

def main(args=None):
    rclpy.init(args=args)
    node = TwistUnstamper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
