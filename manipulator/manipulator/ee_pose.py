from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_xs_modules.xs_robot.core import InterbotixRobotXSCore
import numpy as np
from math import cos, sin, pi

"""
This script makes the end-effector go to a specific pose by defining the pose components

To get started, open a terminal and type:

    ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=px150

Then change to this directory and type:

    python3 ee_pose.py
"""
import cv2
import numpy as np  

# Get the center of the object by using mask
# (u, v) = 图像坐标（像素单位）--> (x, y) (def dectection function) (get!)
# Z = 深度图中的深度值（米或毫米）--> (z) distance = self.depth_image[center_y][center_x].astype(float)/10
# (cx, cy) = 相机的光心（Principal Point）(get!)
# fx, fy = 相机焦距（Focal Length）(get!)

color_intr = {"ppx": 429.7027587890625, "ppy": 247.5892333984375, "fx": 606.6625366210938, "fy": 606.3907470703125}  # RGB相机内参 ppx, ppy = cx, cy
depth_intr = {"ppx": 426.28863525390625 , "ppy": 235.5947265625, "fx": 431.85052490234375, "fy": 431.85052490234375}  # 深度相机内参

#从像素坐标到相机坐标系的转换
def transform_pixel_to_camera(x, y, z,color_intr): #pixel coordinates(x,y), x represent distance, y,z need to be transform
    res = np.array([[x],[y],[z],[1]])
    # yolo给出的坐标原点在图像的左上方
    # 单位变换
    res[0] = x  # distance in meter 世界坐标系的x轴为distance
    distance = x
    res[1] = (y - color_intr["ppx"])* distance / color_intr["fx"] # 单位与distance的单位一致
    res[2] = (z - color_intr["ppy"])* distance / color_intr["fy"]
    
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
    H[0, 3] = -0.07
    H[1, 3] = 0
    H[2, 3] = -0.05 - 0.0045 # radius + margin
    
    res = np.linalg.inv(H) @ coordinate_pixel_to_camera

    return res # 基坐标系下的坐标

def inverse_kinematics(xx, yy, zz):
    """计算 PX150 机械臂的逆运动学解"""
    L1 = 0.10457  # 底座高度 upper_arm height
    L2 = 0.158    # 肩关节到肘关节 
    L3 = 0.15     # 肘关节到腕关节 length from wrist to forearm
    L4 = 0.158575 # 腕关节到末端执行器 length from ee_gripper to wrist

    x, y, z = xx, yy, zz
    theta1 = np.arctan2(y, x)  # 计算底座角度

    wrist_x = np.sqrt(x**2 + y**2)
    wrist_z = z + L4 - L1

    theta2 = np.arctan2(wrist_x, wrist_z) - np.arccos((wrist_x**2 + wrist_z**2 + L2**2 - L3**2) / (2 * np.sqrt(wrist_x**2 + wrist_z**2) * L2)) - np.arctan2(1, 3)

    theta3 = pi / 2 + np.arctan2(1, 3) - np.arccos((L3**2 + L2**2 - (wrist_x**2 + wrist_z**2)) / (2 * L3 * L2))
    
    theta4 = np.arccos((L3**2 + L2**2 - (wrist_x**2 + wrist_z**2)) / (2 * L3 * L2)) - theta2 - np.arctan2(1, 3)
    
    theta5 = 0
    
    return np.array([theta1, theta2, theta3, theta4, theta5])


def main():
    bot = InterbotixManipulatorXS(
        robot_model='px150',
        group_name='arm',
        gripper_name='gripper',
    )

    robot_startup()
    
    
    # bot.arm.go_to_sleep_pose() 

    #bot.arm.set_ee_pose_components(x=0.15,y = 0, z=0.16,moving_time=1.5) # sleep pose

    #bot.arm.go_to_sleep_pose()# sleep pose joint positions: [0.0, -1.7688999531169733, 1.5249832864229877, 0.7793333450953166, 0.0010999999940395355]

    # bot.arm.set_ee_pose_components(x=0.108, y=0.0650865, z=0.05515118)
    # # bot.arm.set_ee_cartesian_trajectory(x=0.108, y=0.0650865, z=-0.05515118) # TODO: 要改为用逆运动学来控制
    
    joint_positions = [0.    ,    0.43706908, 0.44541088, 0.68831637, 0.        ]
    # right answer:
    # [0.0, -0.029999999205271403, 0.025833332538604738, 0.013333333532015483, 0.0]

    # [0.000, 0.066, 0.449, 0.073, 0.438] by ai
    # ['0.000', '0.066', '0.449', '-0.073', '0.438'] by ai(2nd try)

    # ['0.000', '0.066', '0.449', '-0.515', '0.009'] self fixed
    # ['0.000', '0.066', '0.449', '-0.440', '0.066'] self fixed (2nd try)

    original_data_from_cam = [0.17800000309944153,552,178]

    pixel_pose = transform_pixel_to_camera(original_data_from_cam[0],original_data_from_cam[1],original_data_from_cam[2],color_intr)

    base_pose = transform_camera_to_base(pixel_pose)

    print(base_pose)

    base_pose_to_jiont_pose = inverse_kinematics(base_pose[0],base_pose[1],base_pose[2])

    bot.arm.set_joint_positions(base_pose_to_jiont_pose)

    # target = [0.3540,0.0000,0.2552,0,0.00917,0]
    # bot.arm.set_ee_pose_components(x=target[0], y=target[1], z=target[2], roll=target[3], pitch=target[4], yaw=target[5])

    print(bot.arm.get_joint_positions())
    print(bot.arm.get_ee_pose())

    robot_shutdown()


if __name__ == '__main__':
    main()