#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from numpy import pi
from math import acos, atan2
import numpy as np
import cv2

# 导入接口类型，注意包名与消息名需要与你工程中保持一致
from package_with_interfaces.srv import ObjectGrab
from package_with_interfaces.msg import ObjectInformation

# 导入 Interbotix 机械臂接口（根据你的实际情况调整）
from interbotix_common_modules.common_robot.robot import (
    create_interbotix_global_node,
    robot_shutdown,
    robot_startup,
)
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

#####################
# 坐标转换、运动学函数
#####################

def transform_pixel_to_camera(x, y, z, color_intr):
    res = np.array([[x], [y], [z], [1]])
    distance = x  # 假设 x 表示距离（单位：m）
    res[1] = (y - color_intr["ppx"]) * distance / color_intr["fx"]
    res[2] = (z - color_intr["ppy"]) * distance / color_intr["fy"]
    return res

def transform_camera_to_base(coordinate_pixel_to_camera):
    H = np.eye(4)  # 示例转换矩阵，根据实际标定调整
    H[0, 3] = 0.03
    H[1, 3] = 0.147
    H[2, 3] = 0.05
    res = np.linalg.inv(H) @ coordinate_pixel_to_camera
    return res.flatten()  # 返回一维数组

def inverse_kinematics(xx, yy, zz):
    L1 = 0.10457  # 底座高度
    L2 = 0.158    # 肩到肘
    L3 = 0.15     # 肘到腕
    L4 = 0.158575 # 腕到末端执行器
    x, y, z = xx, yy, zz
    theta1 = np.arctan2(y, x)
    wrist_x = np.sqrt(x**2 + y**2)
    wrist_z = z + L4 - L1
    # 注意：acos 的参数需在[-1,1]之间，实际使用时请保证参数合理
    theta2 = atan2(wrist_x, wrist_z) - acos((wrist_x**2 + wrist_z**2 + L2**2 - L3**2) / (2 * L2 * np.sqrt(wrist_x**2 + wrist_z**2))) - atan2(1, 3)
    theta3 = pi / 2 + atan2(1, 3) - acos((L3**2 + L2**2 - (wrist_x**2 + wrist_z**2)) / (2 * L3 * L2))
    theta4 = acos((L3**2 + L2**2 - (wrist_x**2 + wrist_z**2)) / (2 * L3 * L2)) - theta2 - atan2(1, 3)
    theta5 = 0
    return np.array([float(theta1), float(theta2), float(theta3), float(theta4), float(theta5)])

# 相机内参
color_intr = {
    "ppx": 429.7027587890625,
    "ppy": 247.5892333984375,
    "fx": 606.6625366210938,
    "fy": 606.3907470703125,
}

##############################################
# Demo 节点：集成机械臂抓取及颜色检测（箱子颜色检查）
##############################################

class DemoManipulatorWithColorCheck(Node):
    def __init__(self, name):
        super().__init__(name)
        # 创建服务，响应 ObjectGrab 请求（用于状态转移）
        self.srv = self.create_service(ObjectGrab, 'object_grab_service', self.callback)
        # 订阅目标颜色发布（假设 ObjectInformation 包含 color 字段）
        self.target_sub = self.create_subscription(ObjectInformation, '/target_color', self.target_color_callback, 10)

        # 创建 Interbotix 全局节点以及机械臂对象
        self.global_node = create_interbotix_global_node()
        self.bot = InterbotixManipulatorXS(robot_model='px150', robot_name='px150', node=self.global_node)
        robot_startup(self.global_node)

        # 初始化机械臂为预定义姿态（sleep_pose）和夹爪打开
        self.bot.arm.set_ee_pose_components(x=0.15, y=0, z=0.16, moving_time=1.5)
        self.bot.gripper.release()

        # 状态机初始状态为 "grab"
        self.current_state = "grab"
        self.get_logger().info("Initial state: grab")

        # 存储物块（被抓物体）的颜色，抓取成功后锁定
        self.object_color = None
        # 存储检测到的放置箱子颜色（来自颜色检测节点的 /target_color）
        self.detected_box_color = None

    def target_color_callback(self, msg: ObjectInformation):
        # 这里假设 msg 包含 color 字段，代表当前检测到的箱子颜色
        if hasattr(msg, 'color'):
            self.detected_box_color = msg.color
            self.get_logger().info(f"Detected box color: {self.detected_box_color}")
        else:
            self.get_logger().warn("接收到的 ObjectInformation 中无 color 字段")

    def callback(self, request, response):
        """
        根据当前状态执行操作：
          - grab：利用目标信息进行抓取，抓取成功后保存物块颜色（模拟使用当前检测的颜色），然后转为 return 状态。
          - return：执行回返动作（移动到放置箱子前的位置），状态转为 release。
          - release：在执行释放前，检查检测到的箱子颜色是否与物块颜色一致，若一致则释放，若不一致则保持抓持。
          - explore：不执行抓取操作。
        """
        if self.current_state == "grab":
            # 进行坐标转换及逆运动学计算
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
            joints_position[4] = angle_rad

            self.get_logger().info(
                f"Transformed coordinates: x: {new_x:.3f}, y: {new_y:.3f}, z: {new_z:.3f}"
            )
            detected = request.object.detected
            if detected and (new_x > 0.1):
                if self.perception_and_grasp(joints_position):
                    # 抓取成功后锁定物块颜色（这里假设抓取时颜色检测节点已有输出）
                    self.object_color = self.detected_box_color
                    self.get_logger().info(f"Grasp succeeded. Object color locked as: {self.object_color}")
                    self.get_logger().info("Entering return state...")
                    self.current_state = "return"
                    response.success = True
                else:
                    self.get_logger().info("Grasp failed. Retrying...")
                    response.success = False
            else:
                response.success = False

        elif self.current_state == "return":
            if self.move_to_return_pose():
                self.get_logger().info("Reached return pose. Entering release state...")
                self.current_state = "release"
                response.success = True
            else:
                self.get_logger().info("Failed to reach return pose. Retrying...")
                response.success = False

        elif self.current_state == "release":
            # 释放前先检查箱子颜色是否与物块颜色匹配
            if self.object_color is None:
                self.get_logger().warn("Object color not locked. Cannot verify box color. Holding object.")
                response.success = False
            elif self.detected_box_color is None:
                self.get_logger().warn("No box color detected. Holding object.")
                response.success = False
            elif self.detected_box_color.lower() == self.object_color.lower():
                self.get_logger().info("Box color matches object color. Executing release.")
                self.bot.gripper.release()
                self.current_state = "explore"
                response.success = True
            else:
                self.get_logger().info(f"Box color ({self.detected_box_color}) DOES NOT match object color ({self.object_color}). Holding object.")
                response.success = False

        elif self.current_state == "explore":
            self.get_logger().info("Current state is explore. No grasp action executed.")
            response.success = False
        else:
            response.success = False

        return response

    def perception_and_grasp(self, joints_position):
        """抓取操作：先打开夹爪，运动至抓取位姿后执行抓取，再回到 sleep_pose 保持抓持状态。"""
        try:
            self.bot.gripper.release()
            if self.bot.arm.set_joint_positions(joints_position):
                self.bot.gripper.grasp()
                self.get_logger().info("Grasp executed successfully.")
                # 移回保持抓持状态的位姿
                self.bot.arm.set_ee_pose_components(x=0.15, y=0, z=0.16, moving_time=1.5)
                return True
            else:
                self.get_logger().info("Failed to move to grasp position.")
                return False
        except Exception as e:
            self.get_logger().error(f"Grasp operation error: {str(e)}")
            return False

    def move_to_return_pose(self):
        """将机械臂移动到预定义放置箱子前的位姿"""
        try:
            self.bot.arm.set_ee_pose_components(x=0.3, y=0, z=0.2, moving_time=2.0)
            self.get_logger().info("Moving to return pose (ready to place object)...")
            return True
        except Exception as e:
            self.get_logger().error(f"Error in moving to return pose: {str(e)}")
            return False

def main(args=None):
    rclpy.init(args=args)
    node = DemoManipulatorWithColorCheck("demo_manipulator_with_color_check")
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
