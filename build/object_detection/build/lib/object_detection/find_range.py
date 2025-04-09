import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorHSVNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.bridge = CvBridge()
        self.current_frame = None

        # 订阅图像话题
        self.sub_color = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.callback, 10
        )

        # 创建 Trackbars 窗口
        cv2.namedWindow("Trackbars", cv2.WINDOW_NORMAL)
        cv2.createTrackbar("LH", "Trackbars", 35, 179, lambda x: None)
        cv2.createTrackbar("LS", "Trackbars", 100, 255, lambda x: None)
        cv2.createTrackbar("LV", "Trackbars", 50, 255, lambda x: None)
        cv2.createTrackbar("UH", "Trackbars", 85, 179, lambda x: None)
        cv2.createTrackbar("US", "Trackbars", 255, 255, lambda x: None)
        cv2.createTrackbar("UV", "Trackbars", 255, 255, lambda x: None)

    def callback(self, msg):
        self.current_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def spin_with_trackbar(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            if self.current_frame is not None:
                # 获取 Trackbar 值
                lh = cv2.getTrackbarPos("LH", "Trackbars")
                ls = cv2.getTrackbarPos("LS", "Trackbars")
                lv = cv2.getTrackbarPos("LV", "Trackbars")
                uh = cv2.getTrackbarPos("UH", "Trackbars")
                us = cv2.getTrackbarPos("US", "Trackbars")
                uv = cv2.getTrackbarPos("UV", "Trackbars")

                lower = np.array([lh, ls, lv])
                upper = np.array([uh, us, uv])

                hsv = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, lower, upper)
                result = cv2.bitwise_and(self.current_frame, self.current_frame, mask=mask)

                cv2.imshow("Original", self.current_frame)
                cv2.imshow("Mask", mask)
                cv2.imshow("Result", result)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = ColorHSVNode("color_hsv_node")

    try:
        node.spin_with_trackbar()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

