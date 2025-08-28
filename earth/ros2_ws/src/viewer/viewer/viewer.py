import rclpy
from rclpy.node import Node

from custom_message.msg import Rotation
from geometry_msgs.msg import PoseStamped

class ViewerNode(Node):
    def __init__(self):
        super().__init__('viewer_node')
        self.create_subscription(
            Rotation,
            'rotation_topic',
            self.rotation_callback,
            10)
        self.publisher = self.create_publisher(
            PoseStamped,
            'pose_topic',
            10
        )


    def rotation_callback(self, Rotation_msg):
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'
        # pose_msg.header.stamp = self.get_clock().now().to_msg()

        pose_msg.pose.position.x = 0.0
        pose_msg.pose.position.y = 0.0
        pose_msg.pose.position.z = 0.0

        pose_msg.pose.orientation.x = Rotation_msg.x
        pose_msg.pose.orientation.y = Rotation_msg.y
        pose_msg.pose.orientation.z = Rotation_msg.z
        pose_msg.pose.orientation.w = Rotation_msg.w

        self.publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = ViewerNode()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


# GPSの緯度と軽度はlogとして表示
# 状態は2D図？
