import rclpy
from rclpy.node import Node

from custom_message.msg import Gps

class GpsSubscriber(Node):
    def __init__(self):
        super().__init__('graph_node')
        self.subscription = self.create_subscription(
            Gps,
            'gps_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f"{msg.lat}")


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = GpsSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()