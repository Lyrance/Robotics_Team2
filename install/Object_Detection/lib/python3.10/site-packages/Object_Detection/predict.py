import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

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

        self.model = YOLO('/home/mscrobotics2425laptop21/Team2_Workspace/src/Object_Detection/Object_Detection/Detection/weights/best.pt')

    def listener_callback_color(self, data):
        # self.get_logger().info('Receiving color video frame')
        color_image = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')
        self.dectection(color_image)

    def listener_callback_depth(self, data):
        # self.get_logger().info('Receiving depth video frame')
        self.depth_image = self.cv_bridge.imgmsg_to_cv2(data, '16UC1')

    def dectection(self, color_image):
        if self.depth_image is None:
            return

        # Perform object detection
        results = self.model.predict(source=color_image, save=False, save_txt=False, conf=0.5)

        # Loop through each result and draw bounding boxes
        for result in results:
            if result.boxes is not None:  # Ensure there are boxes to process
                boxes = result.boxes.xyxy.numpy()  # Get the box coordinates
                confidences = result.boxes.conf.numpy()  # Get the confidence scores
                class_ids = result.boxes.cls.numpy().astype(int)  # Get class indices

                for i in range(len(boxes)):
                    box = boxes[i]  # Each box is in [x1, y1, x2, y2] format
                    x1, y1, x2, y2 = box  # Unpack the coordinates
                    cv2.rectangle(color_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)  # Draw rectangle

                    # Get the label and confidence
                    label_idx = class_ids[i]  # Class index
                    conf = confidences[i]  # Confidence score
                    label = result.names[label_idx]  # Get the class name

                    # Calculate position for the label
                    label_position = (int(x1), int(y1) - 10)  # Slightly above the box

                    # Put the label and confidence on the image
                    cv2.putText(color_image, f'{label} {conf:.2f}', label_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (0, 255, 0), 2)

        # Display the image
        cv2.imshow("object", color_image)
        cv2.waitKey(1)  # Use 1 to update the window


def main(args=None):
    rclpy.init(args=args)
    node = ImageDepthSubscriber("image_depth_sub")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
