import numpy as np
from math import cos, sin, pi

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

home_pose = inverse_kinematics(0.2480,0.0000,0.0552)
print(home_pose)

print([0.3540,0.0000,0.2552,0,0.00917,0])