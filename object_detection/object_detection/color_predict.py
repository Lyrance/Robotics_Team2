import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

from package_with_interfaces.msg import ObjectInformation
from package_with_interfaces.srv import SetRoverState

class ColorPredictNode(Node):
    def __init__(self, name="color_predict_node"):
        super().__init__(name)

        self.cv_bridge = CvBridge()
        self.depth_image = None

        # 当前状态
        self.current_state = "INITIATE"

        # locked_color: 如果已锁定某颜色, 只检测那一个
        self.locked_color = None

        # locked_object: (px_x, px_y, dist_m, detected, color_name)
        # 这里 (px_x, px_y) 是像素坐标; dist_m 为米距离
        self.locked_object = (0.0, 0.0, 0.0, False, None)

        # 订阅摄像头图像 + 深度图 + 状态
        self.sub_color = self.create_subscription(Image, '/camera/camera/color/image_raw', self.callback_color, 10)
        self.sub_depth = self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.callback_depth, 10)
        self.sub_status = self.create_subscription(String, '/rover_status', self.status_callback, 10)

        # 只发布 /target_color
        self.target_pub = self.create_publisher(ObjectInformation, '/target_color', 10)

        # 创建服务客户端 -> /set_rover_state
        self.state_client = self.create_client(SetRoverState, '/set_rover_state')
        while not self.state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for rover_controller ...")

        # 定时器: 每0.5秒发布一次检测结果
        self.timer = self.create_timer(0.5, self.publish_latest_detection)

    def status_callback(self, msg: String):
        """当 /rover_status 发布状态变化时执行"""
        self.current_state = msg.data
        self.get_logger().info(f"[ColorPredictNode] Current State: {self.current_state}")

        # 如果回到 INITIATE, 解锁颜色
        if self.current_state == "INITIATE":
            self.locked_color = None
            self.locked_object = (0.0,0.0,0.0,False,None)
            self.get_logger().info("Reset locked_color to None")

    def callback_depth(self, msg: Image):
        """Callback function for depth image subscription.
        
        Stores the depth image from ROS message into class member. The depth image is 
        converted from ROS Image message to OpenCV format with 16-bit unsigned integer 
        values representing depth in millimeters.
        
        Args:
            msg (Image): ROS Image message containing depth data
        
        """
        """存储深度图 (uint16, 单位:mm)"""
        self.depth_image = self.cv_bridge.imgmsg_to_cv2(msg, '16UC1')

    def callback_color(self, msg: Image):
        """当前状态在 [INITIATE, EXPLORE, NAV, GRAB, RELEASE, FOUND] 时执行检测"""
        if self.depth_image is None:
            return

        if self.current_state in ["INITIATE","EXPLORE","NAV","GRAB","RELEASE","FOUND"]:
            color_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.detect_nearest_color(color_image)

    def detect_nearest_color(self, color_image):
        """
        检测最近的颜色物体 (像素坐标).
        如果 locked_color=None, 检测全部颜色 => 锁定距离最近者
        若已锁定, 只检测 locked_color.
        最终将 (px_x, px_y, dist_m, detected, color_name) 存到 self.locked_object
        """
        if self.depth_image is None:
            return

        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        color_ranges = {
            "green":  ([32, 100, 25],  [90, 255, 216]),
            "yellow": ([19, 148, 50],  [65, 255, 255]),
            "blue":   ([97, 213, 50],  [127, 255, 212])
        }

        # 如果已锁定, 只检测该颜色; 否则检测全部
        colors_to_check = [self.locked_color] if self.locked_color else list(color_ranges.keys())

        nearest_distance = float('inf')
        nearest_color = None
        nearest_px = (0,0)

        for color_name in colors_to_check:
            if color_name not in color_ranges:
                continue
            lower_vals, upper_vals = color_ranges[color_name]
            lower = np.array(lower_vals, dtype=np.uint8)
            upper = np.array(upper_vals, dtype=np.uint8)

            mask = cv2.inRange(hsv, lower, upper)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for c in contours:
                area = cv2.contourArea(c)
                if area < 3000:  # 过滤小区域
                    continue
                M = cv2.moments(c)
                if M["m00"] == 0:
                    continue
                px_x = int(M["m10"] / M["m00"])
                px_y = int(M["m01"] / M["m00"])

                # 计算深度
                dist_m = 0.0
                if 0<=px_x<self.depth_image.shape[1] and 0<=px_y<self.depth_image.shape[0]:
                    dist_m = self.depth_image[px_y, px_x] * 0.001

                if dist_m>0.0 and dist_m<nearest_distance:
                    nearest_distance = dist_m
                    nearest_color = color_name
                    nearest_px = (px_x, px_y)

        # 可视化
        if nearest_color:
            px_x, px_y = nearest_px
            cv2.circle(color_image, (px_x, px_y), 5, (0,0,255), -1)
            text = f"{nearest_color} Dist={nearest_distance:.2f}m"
            cv2.putText(color_image, text, (px_x+10, px_y-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

            # 如果没锁定颜色, 那就锁定
            if self.locked_color is None:
                self.locked_color = nearest_color
                self.get_logger().info(f"Lock color: {nearest_color}")

            self.locked_object = (float(px_x), float(px_y), nearest_distance, True, nearest_color)
        else:
            self.locked_object = (0.0, 0.0, 0.0, False, None)

        cv2.imshow("NearestColorDetection", color_image)
        cv2.waitKey(1)

        self.state_switch_logic()

    def state_switch_logic(self):
        """
        示例: INITIATE->EXPLORE, EXPLORE->FOUND
        """
        px_x, px_y, dist_m, detected, c = self.locked_object

        if self.current_state == "EXPLORE":
            # 若检测到物体, => EXPLORE
            if detected and c:
                if self.current_state != "EXPLORE":
                    self.call_set_state("FOUND")

        elif self.current_state == "NAV":
            # 如果距离 <0.3 => FOUND
            if detected and dist_m<0.3 and dist_m>0.2:
                if self.current_state != "NAV":
                    self.call_set_state("GRAB")

        # 根据需求可再添加 NAV->GRAB等

    def call_set_state(self, new_state):
        """避免重复请求相同状态"""
        if new_state == self.current_state:
            self.get_logger().info(f"No need to switch: already in {new_state}")
            return

        if not self.state_client.service_is_ready():
            self.get_logger().warn("Service not ready!")
            return

        req = SetRoverState.Request()
        req.new_state = new_state
        fut = self.state_client.call_async(req)
        fut.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info(f"State switch to {resp.success} success!")
            else:
                self.get_logger().warn("State switch fail or not allowed!")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")

    def publish_latest_detection(self):
        """
        每0.5秒 发布 locked_object => /target_color
        如果detect=false => (0,0,0,false)
        """
        px_x, px_y, dist_m, detected, color_name = self.locked_object
        msg = ObjectInformation()
        msg.x = px_x     # 像素坐标 X
        msg.y = px_y     # 像素坐标 Y
        msg.z = dist_m   # 距离(米)
        msg.detected = detected

        self.get_logger().info(f"Publish locked_color={self.locked_color}: px={msg.x:.1f},py={msg.y:.1f}, z={msg.z:.2f}, det={msg.detected}")
        self.target_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ColorPredictNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__=="__main__":
    main()