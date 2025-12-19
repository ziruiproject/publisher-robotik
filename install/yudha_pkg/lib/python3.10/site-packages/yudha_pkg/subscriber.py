import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class YudhaSubscriber(Node):
    def __init__(self):
        super().__init__("yudha_subscriber")
        self.subscriber_ = self.create_subscription(Twist, "yudha_chatter", self.listener_callback, 10)

    def listener_callback(self, msg: Twist):
        self.get_logger().info(f'Angular: {msg.angular}')
        self.get_logger().info(f'Angular: {msg.linear}')


def main(args=None):
    rclpy.init(args=args)
    node = YudhaSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
