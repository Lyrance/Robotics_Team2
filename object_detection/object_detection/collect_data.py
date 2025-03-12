import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageDepthSubscriber(Node):
    def __init__(self, name):
        super().__init__(name)
        self.sub_color = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.listener_callback_color, 10
        )
        # self.sub_depth = self.create_subscription(
        #     Image, '/camera/camera/depth/image_rect_raw', self.listener_callback_depth, 10
        # )
        self.cv_bridge = CvBridge()
        self.depth_image = None

        self.color_count = 0
        self.depth_count = 0

    def listener_callback_color(self, data):
        # self.get_logger().info('Receiving color video frame')
        color_image = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')

        file_name = f"/home/mscrobotics2425laptop21/ros2_test/saved_image_1/color_{self.color_count}.jpg"
        cv2.imwrite(file_name, color_image)
        self.get_logger().info('Saved color image as {}'.format(file_name))
        self.color_count += 1

    def listener_callback_depth(self, data):
        # self.get_logger().info('Receiving depth video frame')
        depth_image = self.cv_bridge.imgmsg_to_cv2(data, '16UC1')

        file_name = f"/home/mscrobotics2425laptop21/ros2_test/saved_image_1/far_depth_{self.depth_count}.png"
        cv2.imwrite(file_name, depth_image)
        self.get_logger().info('Saved depth image as {}'.format(file_name))
        self.depth_count += 1


def main(args=None):
    rclpy.init(args=args)
    node = ImageDepthSubscriber("image_depth_sub")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
