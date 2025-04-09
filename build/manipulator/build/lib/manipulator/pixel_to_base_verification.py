"""
ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=px150

ros2 run manipulator ServerOfPerceptionAndGrasp
"""

import rclpy
from rclpy.node import Node
from numpy import pi
# TODO: 根据实际情况导入正确的 GoalPosition 服务消息类型
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

color_intr = {"ppx": 429.7027587890625, "ppy": 247.5892333984375, "fx": 606.6625366210938, "fy": 606.3907470703125}  # RGB相机内参 ppx, ppy = cx, cy
depth_intr = {"ppx": 426.28863525390625 , "ppy": 235.5947265625, "fx": 431.85052490234375, "fy": 431.85052490234375}  # 深度相机内参

#从像素坐标到相机坐标系的转换
def transform_pixel_to_camera(x, y, z,color_intr): #pixel coordinates(x,y), z represent distance
    res = np.array([[x],[y],[z],[1]])
    # yolo给出的坐标原点在图像的左上方
    # 单位变换
    res[0] = (x - color_intr["ppx"])* z / color_intr["fx"]
    res[1] = (y - color_intr["ppy"])* z / color_intr["fy"]
    res[2] = z # distance in meter 世界坐标系的x轴为distance
    # 单位为m

    return res

#从相机坐标系到ee坐标系的转换
def transform_camera_to_base(coordinate_pixel_to_camera):
    # 取巧办法
    # translation_dis_x = 0.15
    # translation_dis_y = 0
    # translation_dis_z = -0.05
    # TODO: "Need to be modified"

    H = np.eye(4)  # 4x4单位矩阵 H为基坐标系到相机坐标系的转化
    H[0, 3] = 0.07
    H[1, 3] = 0
    H[2, 3] = 0.05 + 0.0045 # radius + margin
    
    res = np.linalg.inv(H) @ coordinate_pixel_to_camera

    return res # 基坐标系下的坐标

def inverse_kinematics(xx, yy, zz):
    """计算 PX150 机械臂的逆运动学解"""
    L1 = 0.10457  # 底座高度
    L2 = 0.158    # 肩关节到肘关节
    L3 = 0.15     # 肘关节到腕关节
    L4 = 0.158575 # 腕关节到末端执行器

    x, y, z = xx, yy, zz
    theta1 = np.arctan2(y, x)  # 计算底座角度

    wrist_x = np.sqrt(x**2 + y**2)
    wrist_z = z + L4 - L1

    theta2 = np.arctan2(wrist_x, wrist_z) - np.arccos((wrist_x**2 + wrist_z**2 + L2**2 - L3**2) / (2 * np.sqrt(wrist_x**2 + wrist_z**2) * L2)) - np.arctan2(1, 3)

    theta3 = pi / 2 + np.arctan2(1, 3) - np.arccos((L3**2 + L2**2 - (wrist_x**2 + wrist_z**2)) / (2 * L3 * L2))
    
    theta4 = np.arccos((L3**2 + L2**2 - (wrist_x**2 + wrist_z**2)) / (2 * L3 * L2)) - theta2 - np.arctan2(1, 3)
    
    theta5 = 0
    
    return np.array([theta1, theta2, theta3, theta4, theta5])

class TEST_ServerOfPerceptionAndGrasp(Node):
    def __init__(self,name):
        super().__init__(name)
        self.srv = self.create_service(ObjectGrab, 'object_grab_service', self.callback) #TODO message type needed to be confirmed

        # Arm Parameters
        self.ROBOT_MODEL = 'px150'
        self.ROBOT_NAME = self.ROBOT_MODEL
        self.REF_FRAME = 'camera_color_optical_frame'
        self.ARM_TAG_FRAME = f'{self.ROBOT_NAME}/ar_tag_link'
        self.ARM_BASE_FRAME = f'{self.ROBOT_NAME}/base_link'

        # Arm Initialization

        # Create a global node to serve as the backend for each API component
        self.global_node = create_interbotix_global_node()
        # Initialize the arm module along with the pointcloud and armtag modules
        self.bot = InterbotixManipulatorXS(
            robot_model=self.ROBOT_MODEL,
            robot_name=self.ROBOT_NAME,
            node=self.global_node,
        )
        # self.pcl = InterbotixPointCloudInterface(
        #     node_inf=self.global_node,
        # )
        # armtag = InterbotixArmTagInterface(
        #     ref_frame=self.REF_FRAME,
        #     arm_tag_frame=self.ARM_TAG_FRAME,
        #     arm_base_frame=self.ARM_BASE_FRAME,
        #     node_inf=self.global_node,
        # )

        # Start up the API
        robot_startup(self.global_node)

        # set initial arm and gripper pose
        self.bot.arm.set_ee_pose_components(x=0.15,y = 0, z=0.16,moving_time=1.5)
        self.bot.gripper.release()

        # # get the ArmTag pose
        # armtag.find_ref_to_arm_base_transform()
        # self.bot.arm.set_ee_pose_components(x=0.3, z=0.2)

    def callback(self,request,response):
        """
        接收请求，获取点云中各个目标簇的位置
        获取request后,确认可执行再调用perception_and_grasp()

        TODO
        写一个客户端,用于发送一个request包含success(是否可执行),clusters(目标位置簇)
        """

        x = request.object.x
        y = request.object.y
        z = request.object.z
        self.get_logger().info(f"Receiving YOLO coordinates: x: {x}, y: {y}, z: {z}")
        # pixel to camera
        coordinate_pixel_to_camera = transform_pixel_to_camera(x, y, z,color_intr)
        base_coordinate = transform_camera_to_base(coordinate_pixel_to_camera)

        x = base_coordinate[0]
        y = base_coordinate[1]
        z = base_coordinate[2] 
        self.get_logger().info(f"Transformed coordinates: x: {x}, y: {y}, z: {z}")

        detected = request.object.detected

        self.get_logger().info(f"Passing YOLO coordinates: x: {x}, y: {y}, z: {z}")

        if detected:
            with open("transform.csv", "a") as f:
                f.write(f"{x},{y},{z}\n")
        else:
            response.success = False

        return response
        
    def perception_and_grasp(self,x, y, z):
        """
        TODO:
        The parameters of positions needed to be repalced by measuring in real world
        """
        # for cluster in clusters:
        #     x, y, z = cluster['position']
        try:
            self.bot.gripper.release()
            jiont_position = inverse_kinematics(x, y, z)
            self.bot.arm.set_joint_positions(jiont_position)

            # self.bot.arm.set_ee_pose_components(x=x, y=y, z=z + 0.05, roll = -3.14)
            # self.bot.arm.set_ee_pose_components(x=x, y=y, z=z, roll = -3.14)
            if(self.bot.arm.set_joint_positions(jiont_position)):
                self.bot.gripper.grasp()
                self.bot.arm.set_ee_pose_components(x=0.15,y = 0, z=0.16) # sleep pose
                self.bot.gripper.release()

            self.get_logger().info("grasp success")
            return True

        except Exception as e:
            self.get_logger().error(f"grasp fail: {str(e)}")
            return False
def main(args = None):
    rclpy.init(args=args)
    node = TEST_ServerOfPerceptionAndGrasp('TEST_ServerOfPerceptionAndGrasp')
    rclpy.spin(node)
    robot_shutdown(node.global_node)
    Node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    