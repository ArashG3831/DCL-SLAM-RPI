import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class TwistStamper(Node):
    def __init__(self):
        super().__init__('twist_stamper')

        # Subscribe to un-stamped velocity commands
        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel_unstamped',
            self.cmd_callback,
            10
        )

        # Publish stamped velocity commands for the controller
        self.pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )

        self.get_logger().info('TwistStamper node started: /cmd_vel_unstamped -> /cmd_vel (TwistStamped)')

    def cmd_callback(self, msg: Twist):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = 'base_link'  # or '' if you prefer
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
