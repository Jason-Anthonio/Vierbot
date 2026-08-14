import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistStamper(Node):
    def __init__(self):
        super().__init__('twist_stamper')
        self.declare_parameter('frame_id', 'base_link')
        self.frame_id = self.get_parameter('frame_id').value

        self.sub = self.create_subscription(Twist, 'cmd_vel_in', self.twist_callback, 10)
        self.pub = self.create_publisher(TwistStamped, 'cmd_vel_out', 10)

    def twist_callback(self, msg: Twist):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self.frame_id
        stamped.twist = msg
        self.pub.publish(stamped)

def main(args=None):
    rclpy.init(args=args)
    node = TwistStamper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
