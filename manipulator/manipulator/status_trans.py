import rclpy
from rclpy.node import Node
from numpy import pi
from package_with_interfaces.srv import ObjectGrab
from package_with_interfaces.msg import ObjectInformation  # 用于接收颜色检测消息

from interbotix_common_modules.common_robot.robot import (
    create_interbotix_global_node,
    robot_shutdown,
    robot_startup,
)
from interbotix_perception_modules.armtag import InterbotixArmTagInterface
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

import numpy as np
from math import cos, sin, pi
import cv2

# 相机内参
color_intr = {
    "ppx": 429.7027587890625,
    "ppy": 247.5892333984375,
    "fx": 606.6625366210938,
    "fy": 606.3907470703125,
}
depth_intr = {
    "ppx": 426.28863525390625,
    "ppy": 235.5947265625,
    "fx": 431.85052490234375,
    "fy": 431.85052490234375,
}

# 从像素坐标到相机坐标系转换（对单位做变换）
def transform_pixel_to_camera(x, y, z, color_intr):
    res = np.array([[x], [y], [z], [1]])
    distance = x  # 假设 x 表示距离（单位：m）
    res[1] = (y - color_intr["ppx"]) * distance / color_intr["fx"]
    res[2] = (z - color_intr["ppy"]) * distance / color_intr["fy"]
    return res

# 从相机坐标系到基坐标系转换
def transform_camera_to_base(coordinate_pixel_to_camera):
    # 取巧办法
    # translation_dis_x = 0.15
    # translation_dis_y = 0
    # translation_dis_z = -0.05
    # TODO: "Need to be modified"

    H = np.eye(4)  # 4x4单位矩阵 H为基坐标系到相机坐标系的转化
    H[0, 3] = -0.01 # TODO chage for testing, right value is -0.07
    H[1, 3] =  0.011
    H[2, 3] =  -0.05 # radius + margin
    
    #H = np.array([[ 0.076519, -0.996796 ,-0.023283 , 0.081215],[-0.026184  ,0.021334 ,-0.999429 , 0.026931], [ 0.996724,  0.077085, -0.024467, -0.02966 ],[ 0.      ,  0. ,       0.      ,  1.      ]])


    res = H @ coordinate_pixel_to_camera
    return res.flatten()  # 返回一维数组

# 机械臂逆运动学计算（根据 PX150 参数）
def inverse_kinematics(xx, yy, zz):
    L1 = 0.10457  # 底座高度
    L2 = 0.158    # 肩关节到肘关节
    L3 = 0.15     # 肘关节到腕关节
    L4 = 0.158575 # 腕关节到末端执行器
    x, y, z = xx, yy, zz
    theta1 = np.arctan2(y, x)
    wrist_x = np.sqrt(x**2 + y**2)
    wrist_z = z + L4 - L1
    theta2 = np.arctan2(wrist_x, wrist_z) - np.arccos((wrist_x**2 + wrist_z**2 + L2**2 - L3**2) / (2 * np.sqrt(wrist_x**2 + wrist_z**2) * L2)) - np.arctan2(1, 3)
    theta3 = pi / 2 + np.arctan2(1, 3) - np.arccos((L3**2 + L2**2 - (wrist_x**2 + wrist_z**2)) / (2 * L3 * L2))
    theta4 = np.arccos((L3**2 + L2**2 - (wrist_x**2 + wrist_z**2)) / (2 * L3 * L2)) - theta2 - np.arctan2(1, 3)
    theta5 = 0
    return np.array([theta1, theta2, theta3, theta4, theta5])

class ServerOfPerceptionAndGrasp(Node):
    def __init__(self, name):
        super().__init__(name)
        self.srv = self.create_service(ObjectGrab, 'object_grab_service', self.callback)
        
        # 创建颜色检测订阅器，用于获取 /target_color 消息中的距离信息
        self.create_subscription(ObjectInformation, '/target_color', self.color_info_callback, 10)
        self.latest_target_distance = None  # 存储从颜色检测中获得的距离

        # 机械臂参数初始化
        self.ROBOT_MODEL = 'px150'
        self.ROBOT_NAME = self.ROBOT_MODEL
        self.REF_FRAME = 'camera_color_optical_frame'
        self.ARM_TAG_FRAME = f'{self.ROBOT_NAME}/ar_tag_link'
        self.ARM_BASE_FRAME = f'{self.ROBOT_NAME}/base_link'

        # 创建全局节点和机械臂接口
        self.global_node = create_interbotix_global_node()
        self.bot = InterbotixManipulatorXS(
            robot_model=self.ROBOT_MODEL,
            robot_name=self.ROBOT_NAME,
            node=self.global_node,
        )
        robot_startup(self.global_node)

        # 设置初始姿态（sleep_pose）和打开夹爪
        self.bot.arm.set_ee_pose_components(x=0.15, y=0, z=0.16, moving_time=1.5)
        self.bot.gripper.release()

        # 状态机初始状态为 “grab”
        self.current_state = "grab"
        self.get_logger().info("Initial state: grab")

    def color_info_callback(self, msg: ObjectInformation):
        # 假设 msg.z 表示从颜色检测得到的目标距离（单位：米）
        self.latest_target_distance = msg.z
        self.get_logger().info(f"Received target distance from color detection: {self.latest_target_distance:.2f} m")

    def callback(self, request, response):
        """
        根据当前状态执行不同操作：
          - grab：执行抓取操作，如果成功则进入 return 状态；
          - return：执行回返动作（移动到放置盒子前），状态切换到 release；
          - release：获取颜色检测的距离信息，用该距离设置放置位姿，再执行释放，
                     然后切换到 explore 状态；
          - explore：当前状态下不执行抓取。
        """
        if self.current_state == "grab":
            # 坐标转换及抓取目标判断
            x = request.object.x
            y = request.object.y
            z = request.object.z
            angle = request.object.angle
            angle_rad = np.deg2rad(angle)
            self.get_logger().info(
                f"Receiving YOLO coordinates: x: {x}, y: {y}, z: {z}, angle (rad): {angle_rad}"
            )
            coordinate_pixel_to_camera = transform_pixel_to_camera(x, y, z, color_intr)
            base_coordinate = transform_camera_to_base(coordinate_pixel_to_camera)
            new_x, new_y, new_z = base_coordinate[0], base_coordinate[1], base_coordinate[2]

            new_x = base_coordinate[0] + 0.0026
            new_y = -base_coordinate[1] - 0.063
            new_z = base_coordinate[2] - 0.020

            joints_position = inverse_kinematics(new_x, new_y, new_z)
            joints_position[4] = angle_rad  # 根据物体角度调整抓取位姿

            self.get_logger().info(
                f"Transformed coordinates: x: {new_x:.3f}, y: {new_y:.3f}, z: {new_z:.3f}"
            )
            detected = request.object.detected

            if detected and (new_x > 0.1):
                if self.perception_and_grasp(joints_position):
                    self.get_logger().info("Grasp succeeded. Entering return state...")
                    self.current_state = "return"
                    response.success = True
                else:
                    self.get_logger().info("Grasp failed. Retrying...")
                    response.success = False
            else:
                response.success = False

        elif self.current_state == "return":
            # 执行回返动作
            if self.move_to_return_pose():
                self.get_logger().info("Reached return pose. Entering release state...")
                self.current_state = "release"
                response.success = True
            else:
                self.get_logger().info("Failed to reach return pose. Retrying return action...")
                response.success = False

        elif self.current_state == "release":
            # --- 执行释放操作 ---
            # 这里使用从 /target_color 接收到的距离信息
            if self.latest_target_distance is None:
                self.get_logger().warn("No target distance received from color detection. Cannot move to proper release pose.")
                response.success = False
            else:
                dist = self.latest_target_distance  # 使用从颜色检测得到的距离作为目标 x 坐标
                # 根据实际需求，你可能需要对 dist 进行校正或转换
                self.bot.arm.set_ee_pose_components(x=dist, y=0.0, z=-0.05, moving_time=2.0)
                self.bot.gripper.release()
                self.get_logger().info("Gripper released the object. Changing state to explore.")
                self.current_state = "explore"
                response.success = True

        elif self.current_state == "explore":
            self.get_logger().info("Current state is explore. No grasp action executed.")
            response.success = False

        else:
            response.success = False

        return response

    def perception_and_grasp(self, joints_position):
        """
        在 "grab" 状态下尝试抓取：
         1. 先打开夹爪；
         2. 移动机械臂至抓取位姿；
         3. 执行抓取（闭合夹爪），然后返回保持抓持状态的位姿。
        """
        try:
            self.bot.gripper.release()
            if self.bot.arm.set_joint_positions(joints_position):
                self.bot.gripper.grasp()
                self.get_logger().info("Grasp executed successfully.")
                self.bot.arm.set_ee_pose_components(x=0.15, y=0, z=0.16, moving_time=1.5)
                return True
            else:
                self.get_logger().info("Failed to move to grasp position.")
                return False
        except Exception as e:
            self.get_logger().error(f"Grasp operation error: {str(e)}")
            return False

    def move_to_return_pose(self):
        """
        执行回返动作：将机械臂移动到预定义的放置位姿（目标盒子正前方）。
        """
        try:
            self.bot.arm.set_ee_pose_components(x=0.3, y=0, z=0.2, moving_time=2.0)
            self.get_logger().info("Moving to return pose (ready to place object)...")
            return True
        except Exception as e:
            self.get_logger().error(f"Error in moving to return pose: {str(e)}")
            return False

def main(args=None):
    rclpy.init(args=args)
    node = ServerOfPerceptionAndGrasp('ServerOfPerceptionAndGrasp_status')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        robot_shutdown(node.global_node)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
