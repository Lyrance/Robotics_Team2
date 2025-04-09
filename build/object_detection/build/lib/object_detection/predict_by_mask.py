'''
1.获取物体mask,通过mask计算出物体中心
2.将中心点从像素坐标转移到基坐标系
3.计算机械臂的抓取位姿(或者关节角度)

YOLO 坐标系（像素级）
原点 (0,0) 在图像的 左上角。
X 轴（横向）：向右 为正方向（从左到右）。
Y 轴（纵向）：向下 为正方向（从上到下）。

'''
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

from package_with_interfaces.srv import ObjectGrab


class ImageDepthSubscriber(Node):
    def __init__(self, name):
        super().__init__(name)

        # 订阅彩色 & 深度图像
        self.sub_color = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.callback_color, 10
        )
        self.sub_depth = self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw', self.callback_depth, 10
        )

        # 订阅 /rover_status 获取当前状态
        self.sub_status = self.create_subscription(
            String, '/rover_status', self.status_callback, 10
        )

        self.cv_bridge = CvBridge()
        self.depth_image = None

        # 创建服务客户端 -> /object_grab_service
        self.client = self.create_client(ObjectGrab, 'object_grab_service')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('wait for robotic arm service ...')

        # 加载 YOLO 模型
        self.model = YOLO('/home/mscrobotics2425laptop16/Robotics_Team2/src/object_detection/object_detection/weights/train3/best.pt')

        # 每0.5s检查是否需要发送坐标
        self.timer = self.create_timer(0.5, self.send_latest_detection)

        # 存储最近检测到的目标信息
        # (center_x_px, center_y_px, distance_m, detected)
        self.latest_detected_object = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'detected': False}

        self.in_progress = False  # 表示是否正在抓取
        self.current_state = "GRAB"  # 默认为 INITIATE

    def status_callback(self, msg: String):
        """
        订阅 /rover_status, 用于判断只有在 'GRAB' 状态下才进行检测
        """
        self.current_state = msg.data
        self.get_logger().info(f"[ImageDepthSubscriber] Current State: {self.current_state}")

    def callback_depth(self, data):
        """接收并存储深度图像"""
        self.depth_image = self.cv_bridge.imgmsg_to_cv2(data, '16UC1')

    def callback_color(self, data):
        if self.current_state != "GRAB" or self.depth_image is None:
            return

        color_image = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')
        results = self.model.predict(source=color_image, save=False, conf=0.7)
        detected_objects = []

        for result in results:
            if result.boxes is None:
                continue

            boxes = result.boxes.xyxy.cpu().numpy().astype(int)
            confidences = result.boxes.conf.cpu().numpy()
            class_ids = result.boxes.cls.cpu().numpy().astype(int)
            masks = result.masks.data.cpu().numpy() if result.masks else []

            h, w = color_image.shape[:2]

            for i in range(len(boxes)):
                if i < len(masks):
                    
                    mask = masks[i].squeeze()
                    mask = (mask > 0.5).astype(np.uint8) * 255  # 转为uint8二值图像
                    
                    # 缩放到原始尺寸
                    mask_resized = cv2.resize(mask, (w, h), interpolation=cv2.INTER_NEAREST)
                    
                    if mask_resized.dtype == np.uint8 and len(mask_resized.shape) == 2:
                        cv2.imshow("Resized Mask", mask_resized)
                        cv2.waitKey(1)
                else:
                    mask_resized = np.zeros((h, w), dtype=np.uint8)

                # ---- 2. 计算质心（优先使用掩码）----
                x1, y1, x2, y2 = boxes[i]
                if np.any(mask_resized > 0):
                    # 合并所有轮廓并计算质心
                    contours, _ = cv2.findContours(mask_resized, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    if contours:
                        merged_contour = np.concatenate(contours)
                        M = cv2.moments(merged_contour)
                        if M["m00"] > 0:
                            center_x = int(M["m10"] / M["m00"])
                            center_y = int(M["m01"] / M["m00"])
                            self.get_logger().info(f"使用掩码质心: ({center_x}, {center_y})")
                        else:
                            center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
                            self.get_logger().info(f"使用边界框中心: ({center_x}, {center_y})")
                else:
                    center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
                    self.get_logger().info(f"使用边界框中心: ({center_x}, {center_y})")

                # ---- 3. 获取深度信息 ----
                distance_m = 0.0
                if (0 <= center_x < self.depth_image.shape[1] and 
                    0 <= center_y < self.depth_image.shape[0]):
                    distance_m = self.depth_image[center_y, center_x] * 0.001  # mm -> m

                # ---- 4. 可视化（调试时可启用）----
                label = f"{result.names[class_ids[i]]} {confidences[i]:.2f}"
                cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(color_image, (center_x, center_y), 5, (0, 0, 255), -1)
                cv2.putText(color_image, f"{label} {distance_m:.2f}m", 
                            (x1, max(y1 - 10, 0)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                detected_objects.append((center_x, center_y, distance_m))

        # ---- 5. 更新最近物体信息 ----
        if detected_objects:
            nearest = min(detected_objects, key=lambda x: x[2])  # 按距离排序
            self.latest_detected_object = {
                'x': nearest[0],     # 像素坐标X
                'y': nearest[1],     # 像素坐标Y
                'z': nearest[2],     # 距离（米）
                'angle': self.compute_angle_with_mask(mask_resized),
                'detected': True,
                'timestamp': data.header.stamp  # ROS时间戳（可选）
            }
        else:
            self.latest_detected_object = {
                'x': 0, 'y': 0, 'z': 0,'angle': 0,'detected': False
            }

        # 显示结果（实际部署时可关闭）
        cv2.imshow("YOLO Detection", color_image)
        cv2.waitKey(1)

    def compute_angle_with_mask(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # 如果根本没检测到轮廓，则返回一个默认值，避免后面报错
        if not contours:
            return 0.0  # 或者 (0.0, (0.0, 0.0)) 视需求而定

        max_area = 0
        best_rect = None
        
        for contour in contours:
            rect = cv2.minAreaRect(contour)  # rect 是 ((cx, cy), (w, h), angle)
            (cx, cy), (w, h), angle = rect
            area = w * h
            if area > max_area:
                max_area = area
                best_rect = rect

        # 如果循环完 best_rect 依旧是 None，说明没找到合适轮廓
        if best_rect is None:
            return 0.0  # 或 (0.0, (0.0, 0.0))

        # 真正做角度修正
        (cx, cy), (width, height), angle = best_rect
        if width > height:
            angle = -(90 - angle)

        return angle


        


    def send_latest_detection(self):
        """
        每0.5s检查是否要调用 /object_grab_service
        条件:
         1) latest_detected_object['detected'] == True
         2) 0.18 < 距离 <0.25
         3) 当前不在抓取中 (in_progress=False)
         4) 当前状态是 GRAB
        """
        dist = self.latest_detected_object['z']
        # 
        if(self.in_progress == False and dist < 0.25 and dist >0.18 and self.current_state == "GRAB" and self.latest_detected_object['detected']):
            self.get_logger().info(f"Distance <0.3m => sending service request: {self.latest_detected_object}")
            self.in_progress = True
            self.call_service()

    def call_service(self):
        # 这里将 (x,y,z) 分别对应到 request.object.x, .y, .z
        # 也可根据你的 srv 定义做改动
        x_px = self.latest_detected_object['x']  # 像素坐标
        y_px = self.latest_detected_object['y']
        dist_m = self.latest_detected_object['z']
        angle = self.latest_detected_object['angle']
        detected = self.latest_detected_object['detected']

        request = ObjectGrab.Request()
        request.object.y = float(x_px)
        request.object.z = float(y_px)
        request.object.x = float(dist_m)
        request.object.angle = float(angle)
        request.object.detected = bool(detected)

        future = self.client.call_async(request)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Grab success -> setting in_progress = False')
                
            else:
                self.get_logger().info('Grab fail -> setting in_progress = False')
                self.in_progress = False
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')
        # 无论成功或失败，都可以再次发送
        #self.in_progress = False

def main(args=None):
    """
    Main entry point for the image depth subscriber node.
    
    This function initializes the ROS2 system, creates an ImageDepthSubscriber node,
    and spins it until interrupted. It handles cleanup by destroying the node and
    shutting down ROS2 when done.
    
    Args:
    
        args: Command line arguments passed to the ROS2 system (optional)
    
    """
    rclpy.init(args=args)
    node = ImageDepthSubscriber("image_depth_sub")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


# launch camera command:
# ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=848x480x30 rgb_camera.color_profile:=848x480x30 align_depth.enable:=true pointcloud.enable:=true