import numpy as np
import rclpy
from imgviz.external.transformations import rotation_matrix
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from package_with_interfaces.srv import ObjectGrab


class ImageDepthSubscriber(Node):
    def __init__(self, name):
        super().__init__(name)
        self.sub_color = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.listener_callback_color, 10
        )
        self.sub_depth = self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw', self.listener_callback_depth, 10
        )

        self.cv_bridge = CvBridge()
        self.depth_image = None

        #create service client
        self.client = self.create_client(ObjectGrab, 'object_grab_service')
        # wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('wait for robotic arm service ...')

        self.model = YOLO('/home/mscrobotics2425laptop16/Robotics_Team2/src/object_detection/object_detection/weights/train3/best.pt')

        self.timer = self.create_timer(0.5, self.send_latest_detection)

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
        results = self.model.predict(source=color_image, save=False, save_txt=False, conf=0.7)

        detected_objects = []

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

                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)
                    if 0 <= center_x < self.depth_image.shape[1] and 0 <= center_y < self.depth_image.shape[0]:
                        distance = self.depth_image[center_y][center_x].astype(float)/10
                        detected_objects.append((center_x, center_y, distance/100.0))   
                    else:
                        distance = 0.0

                    # Put the label and confidence on the image
                    cv2.putText(color_image, f'{label} {conf:.2f} Dis: {distance:.2f} cm', label_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (0, 255, 0), 2)
                    cv2.circle(color_image, (center_x, center_y), 5, (0, 0, 255), -1)

        if detected_objects:
            nearest = min(detected_objects, key=lambda x: x[2])
            self.latest_detected_object = {'x': nearest[0], 'y': nearest[1], 'z': nearest[2], 'detected': True}
        else:
            self.latest_detected_object = {'x': 0, 'y': 0, 'z': 0, 'detected': False}

        # Display the image
        cv2.imshow("object", color_image)
        cv2.waitKey(1)  # Use 1 to update the window

    def send_latest_detection(self):
            # self.get_logger().info(f"Passing YOLO coordinates: {self.latest_detected_object}")
            self.call_service()

    def call_service(self):

            x = self.latest_detected_object['x']
            y = self.latest_detected_object['y']
            z = self.latest_detected_object['z']
            detected = self.latest_detected_object['detected']
            request = ObjectGrab.Request()
            request.object.y = float(x)
            request.object.z= float(y)
            request.object.x = float(z)
            request.object.detected = bool(detected)
            # self.get_logger().info(f"Passing YOLO coordinates: {x}, {y}, {z}, {detected}")

            future = self.client.call_async(request)
            future.add_done_callback(self.handle_response)
        
    def handle_response(self, future):
            try:
                response = future.result()
                if response.success:
                    self.get_logger().info('grab success')
                else:
                    self.get_logger().info('grab fail')
            except Exception as e:
                self.get_logger().info(f'Service call failed {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageDepthSubscriber("image_depth_sub")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

# launch camera command:
# ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=848x480x30 rgb_camera.color_profile:=848x480x30 align_depth.enable:=true pointcloud.enable:=true