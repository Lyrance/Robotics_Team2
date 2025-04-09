"""
ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=px150

ros2 run manipulator status_trans
"""

import rclpy
from rclpy.node import Node
from numpy import pi
from package_with_interfaces.srv import ObjectGrab

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

# 从像素坐标到相机坐标系转换（注意：这里对单位做了变换）
def transform_pixel_to_camera(x, y, z, color_intr):
    res = np.array([[x], [y], [z], [1]])
    distance = x  # 假设 x 表示距离（单位：m）
    res[1] = (y - color_intr["ppx"]) * distance / color_intr["fx"]
    res[2] = (z - color_intr["ppy"]) * distance / color_intr["fy"]
    return res

# 从相机坐标系到基坐标系转换（这里以简化的刚体变换方法实现）
def transform_camera_to_base(coordinate_pixel_to_camera):
    H = np.eye(4)
    H[0, 3] = 0.03   # 根据实际情况调整
    H[1, 3] =  0.147
    H[2, 3] =  0.05
    res = np.linalg.inv(H) @ coordinate_pixel_to_camera
    return res.flatten()  # 返回一维数组

def inverse_kinematics(xx, yy, zz):
    L1 = 0.10457
    L2 = 0.158
    L3 = 0.15
    L4 = 0.158575
    x, y, z = xx, yy, zz
    theta1 = np.arctan2(y, x)
    wrist_x = np.sqrt(x**2 + y**2)
    wrist_z = z + L4 - L1
    theta2 = np.arctan2(wrist_x, wrist_z) - np.arccos((wrist_x**2 + wrist_z**2 + L2**2 - L3**2) / (2 * np.sqrt(wrist_x**2 + wrist_z**2) * L2)) - np.arctan2(1, 3)
    theta3 = np.pi / 2 + np.arctan2(1, 3) - np.arccos((L3**2 + L2**2 - (wrist_x**2 + wrist_z**2)) / (2 * L3 * L2))
    theta4 = np.arccos((L3**2 + L2**2 - (wrist_x**2 + wrist_z**2)) / (2 * L3 * L2)) - theta2 - np.arctan2(1, 3)
    theta5 = 0
    return np.array([float(theta1), float(theta2), float(theta3), float(theta4), float(theta5)])


class ServerOfPerceptionAndGrasp(Node):
    def __init__(self, name):
        super().__init__(name)
        self.srv = self.create_service(ObjectGrab, 'object_grab_service', self.callback)
        
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

        # 设置初始姿态（sleep_pose）和打开夹爪（确保空闲状态）
        self.bot.arm.set_ee_pose_components(x=0.15, y=0, z=0.16, moving_time=1.5)
        self.bot.gripper.release()

        # 增加状态机变量，初始状态为抓取状态 "grab"
        self.current_state = "grab"
        self.get_logger().info("Initial state: grab")

    def callback(self, request, response):
        """
        根据当前状态执行不同操作：
        - grab 状态：转换接收的像素坐标，并调用 grasp 操作，
                      如果 grasp 成功则进入 return 状态，否则保持状态等待后续请求重试；
        - return 状态：执行“回返”动作（移动到目标盒子前），成功后进入 release 状态；
        - release 状态：执行释放操作（夹爪打开），并将状态切换到 explore；
        - explore 状态：表示任务进行中，无需执行抓取；
        """
        if self.current_state == "grab":
            # --- 坐标转换及抓取目标判断 ---
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
            # --- 回返动作，移动到放置目标盒子前 ---
            if self.move_to_return_pose():
                self.get_logger().info("Reached return pose. Entering release state...")
                self.current_state = "release"
                response.success = True
            else:
                self.get_logger().info("Failed to reach return pose. Retrying return action...")
                response.success = False

        elif self.current_state == "release":
            # --- 执行释放操作 ---
            self.bot.gripper.release()
            self.get_logger().info("Gripper released the object. Changing state to explore.")
            self.current_state = "explore"
            response.success = True

        elif self.current_state == "explore":
            # 在 explore 状态下，本服务端不执行抓取操作
            self.get_logger().info("Current state is explore. No grasp action executed.")
            response.success = False

        else:
            response.success = False

        return response

    def perception_and_grasp(self, joints_position):
        """
        在 "grab" 状态下尝试抓取操作：
         1. 先打开夹爪（release）以备抓取；
         2. 移动机械臂至抓取位姿；
         3. 执行抓取操作（grasp），抓取后不再释放，接着将机械臂移至 sleep_pose 保持夹持状态；
         如果抓取失败则返回 False，等待后续重试。
        """
        try:
            # 打开夹爪，确保无物体干扰
            self.bot.gripper.release()
            # 移动至目标抓取位姿
            if self.bot.arm.set_joint_positions(joints_position):
                # 执行抓取：闭合夹爪
                self.bot.gripper.grasp()
                self.get_logger().info("Grasp executed successfully.")
                # 移动到 sleep_pose（保持抓持状态，不释放）
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
        执行回返动作：
         - 将机械臂移动到预定义的放置位姿（目标盒子正前方）；
         - 此处模拟小车已行驶到位，实际使用时应结合底盘控制；
         - 若移动成功返回 True，否则返回 False。
        """
        try:
            # 此处定义放置物体时机械臂的目标位姿，数值需要根据实际标定调整。
            self.bot.arm.set_ee_pose_components(x=0.3, y=0, z=0.2, moving_time=2.0)
            self.get_logger().info("Moving to return pose (ready to place object)...")
            return True
        except Exception as e:
            self.get_logger().error(f"Error in moving to return pose: {str(e)}")
            return False

def main(args=None):
    rclpy.init(args=args)
    node = ServerOfPerceptionAndGrasp('ServerOfPerceptionAndGrasp')
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
