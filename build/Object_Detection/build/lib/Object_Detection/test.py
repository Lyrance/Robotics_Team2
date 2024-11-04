import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Float32MultiArray  # Used for publishing the center and distance

lower_red = np.array([0, 90, 128])
upper_red = np.array([180, 255, 255])


class ImageDepthSubscriber(Node):
    def __init__(self, name):
        super().__init__(name)
        self.sub_color = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.listener_callback_color, 10
        )
        self.sub_depth = self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw', self.listener_callback_depth, 10
        )
        self.cv_bridge = CvBridge()
        self.depth_image = None
        self.pub = self.create_publisher(Float32MultiArray, 'object_center_distance', 10)

    def listener_callback_color(self, data):
        self.get_logger().info('Receiving color video frame')
        color_image = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')
        self.object_detect(color_image)

    def listener_callback_depth(self, data):
        self.get_logger().info('Receiving depth video frame')
        self.depth_image = self.cv_bridge.imgmsg_to_cv2(data, '16UC1')

    def object_detect(self, image):
        if self.depth_image is None:
            return

        hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask_red = cv2.inRange(hsv_img, lower_red, upper_red)
        contours, _ = cv2.findContours(mask_red, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

        for cnt in contours:
            if cnt.shape[0] < 150:
                continue

            # (x, y, w, h) = cv2.boundingRect(cnt)
            # center_x, center_y = int(x + w / 2), int(y + h / 2)
            #
            # # Ensure the depth value is read as uint16 and convert to float
            # distance_mm = self.depth_image[center_y, center_x].astype(float)
            #
            # # Convert from mm to meters if necessary
            # distance_meters = distance_mm / 1000.0
            #
            # # Publishing the center and distance, ensuring all values are float
            # msg = Float32MultiArray()
            # msg.data = [float(center_x), float(center_y), float(distance_meters)]
            # self.pub.publish(msg)

            cv2.drawContours(image, [cnt], -1, (0, 255, 0), 2)
            # cv2.circle(image, (center_x, center_y), 5, (0, 255, 0), -1)

        cv2.imshow("object", image)
        cv2.waitKey(10)


def main(args=None):
    rclpy.init(args=args)
    node = ImageDepthSubscriber("image_depth_sub")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()