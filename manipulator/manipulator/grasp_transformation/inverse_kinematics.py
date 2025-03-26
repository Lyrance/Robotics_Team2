import numpy as np
from math import cos, sin, atan2, sqrt, acos, pi

# DH参数表 (单位：米/弧度)
dh_params = [
    {'a':0,       'alpha':0,      'd':0.06566},  # 关节1
    {'a':0,       'alpha':-pi/2,  'd':0.03891},  # 关节2
    {'a':0.05,    'alpha':0,      'd':0.15},     # 关节3
    {'a':0.15,    'alpha':0,      'd':0},        # 关节4
    {'a':0.065,   'alpha':pi/2,   'd':0}         # 关节5
]
# dh_params = [
#     {'a':0,       'alpha':0,      'd':0.06566},  # 关节1
#     {'a':0,       'alpha':-pi/2,  'd':0.03891},  # 关节2
#     {'a':0.05,    'alpha':-pi/2,  'd':0.15},     # 关节3
#     {'a':0.15,    'alpha':-pi/2,  'd':0},        # 关节4
#     {'a':0.065,   'alpha':pi/2,   'd':0}         # 关节5
# ]

def inverse_kinematics(target_pose):
    """
    逆运动学求解函数
    输入: 
        target_pose = [x, y, z, roll, pitch, yaw] (单位：米/弧度)
    输出: 
        joints = [θ1, θ2, θ3, θ4, θ5] (弧度) 或 None (无解)
    """
    x, y, z, roll, pitch, yaw = target_pose
    
    # 步骤1: 计算末端执行器旋转矩阵
    R = euler_to_rotation_matrix(roll, pitch, yaw)
    
    # 步骤2: 计算腕部中心位置 (Wrist Center)
    L = 0.09357  # 从关节5到末端执行器的长度（根据URDF） 0.043 0.09357 
    wrist_center = np.array([x, y, z]) - R @ np.array([L, 0, 0])
    
    # 步骤3: 求解关节1 (θ1)
    theta1 = atan2(wrist_center[1], wrist_center[0])
    
    # 步骤4: 求解关节2和关节3 (θ2, θ3)
    # 将问题投影到关节2坐标系平面
    d1 = dh_params[0]['d']  # 0.06566
    d2 = dh_params[1]['d']  # 0.03891
    x_proj = sqrt(wrist_center[0]**2 + wrist_center[1]**2)  # 修正：不再减去d1
    z_proj = wrist_center[2] - (d1 + d2)  # 修正：累计d1+d2
    
    # 几何参数
    a2 = dh_params[2]['a']   # 0.05
    d3 = dh_params[2]['d']   # 0.15
    a3 = dh_params[3]['a']   # 0.15
    
    # 使用余弦定理
    numerator = x_proj**2 + z_proj**2 - a2**2 - (d3 + a3)**2
    denominator = 2*a2*(d3 + a3)
    if denominator == 0: return None
    D = numerator / denominator
    
    if abs(D) > 1: return None  # 无解
    
    theta3 = acos(D)
    
    # 两种可能的解（elbow-up/elbow-down）
    theta2 = atan2(z_proj, x_proj) - atan2((d3 + a3)*sin(theta3), a2 + (d3 + a3)*cos(theta3))
    theta3_alt = -theta3
    theta2_alt = atan2(z_proj, x_proj) - atan2((d3 + a3)*sin(theta3_alt), a2 + (d3 + a3)*cos(theta3_alt))
    
    # 步骤5: 求解关节4和关节5 (θ4, θ5)
    # 计算前三个关节的变换矩阵
    T03 = forward_kinematics([theta1, theta2, theta3, 0, 0])
    R03 = T03[:3, :3]
    R35 = R03.T @ R  # 修正旋转矩阵计算
    
    # 提取Z-Y欧拉角（根据实际机械臂结构）
    theta5 = atan2(-R35[2, 0], sqrt(R35[0, 0]**2 + R35[1, 0]**2))
    if abs(cos(theta5)) > 1e-6:
        theta4 = atan2(R35[1, 0]/cos(theta5), R35[0, 0]/cos(theta5))
    else:
        # 奇异位形处理
        theta4 = 0
    
    # 步骤6: 验证关节限制（修正后的检查）
    solution = [theta1, theta2, theta3, theta4, theta5]
    if check_joint_limits(solution):
        return solution
    else:
        # 尝试另一种解
        solution_alt = [theta1, theta2_alt, theta3_alt, theta4, theta5]
        if check_joint_limits(solution_alt):
            return solution_alt
        return None

def check_joint_limits(joints):
    """ 检查关节角度是否在URDF定义的范围内（修正关节2的转换） """
    # URDF关节限制（转换为弧度）
    limits = [
        (-pi + 1e-5, pi - 1e-5),          # 关节1
        (np.radians(-113), np.radians(111)), # 关节2（URDF角度）
        (np.radians(-120), np.radians(95)),  # 关节3
        (np.radians(-100), np.radians(123)), # 关节4
        (-pi + 1e-5, pi - 1e-5)          # 关节5
    ]
    
    # 转换关节2的DH角度到URDF角度
    theta2_urdf = joints[1] + pi/2
    
    # 检查各关节限制
    if not (limits[0][0] <= joints[0] <= limits[0][1]):
        return False
    if not (limits[1][0] <= theta2_urdf <= limits[1][1]):
        return False
    if not (limits[2][0] <= joints[2] <= limits[2][1]):
        return False
    if not (limits[3][0] <= joints[3] <= limits[3][1]):
        return False
    if not (limits[4][0] <= joints[4] <= limits[4][1]):
        return False
    return True

# 其余辅助函数保持不变（forward_kinematics、euler_to_rotation_matrix等）
def forward_kinematics(joint_angles):
    """
    正运动学计算函数（计算到第3关节的变换矩阵 T03）
    
    参数:
        joint_angles : [θ1, θ2, θ3, θ4, θ5] 各关节角度（弧度）
                      这里只使用前三个角度
    返回:
        T03 : 4x4齐次变换矩阵（从基座到第3关节）
    """
    # 提取前三关节角度
    theta1, theta2, theta3 = joint_angles[:3]
    
    T = np.eye(4)
    for i in range(3):
        a = dh_params[i]['a']
        alpha = dh_params[i]['alpha']
        d = dh_params[i]['d']
        # 对应关节角
        theta = [theta1, theta2, theta3][i]
        
        A = np.array([
            [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta)],
            [sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
            [0,           sin(alpha),             cos(alpha),            d],
            [0,           0,                      0,                     1]
        ])
        T = T @ A
    return T

def euler_to_rotation_matrix(roll, pitch, yaw):
    """ 欧拉角转旋转矩阵（ZYX顺序） """
    Rz = np.array([[cos(yaw), -sin(yaw), 0],
                   [sin(yaw),  cos(yaw), 0],
                   [0,         0,        1]])
    Ry = np.array([[cos(pitch), 0, sin(pitch)],
                   [0,          1, 0],
                   [-sin(pitch),0, cos(pitch)]])
    Rx = np.array([[1, 0,         0],
                   [0, cos(roll), -sin(roll)],
                   [0, sin(roll), cos(roll)]])
    return Rz @ Ry @ Rx

# 示例使用
if __name__ == "__main__":
    # 示例目标位姿：位置 (0.108, 0.0650865, 0.05515118)，姿态 (roll, pitch, yaw) = (0, 0, 0)
    target = [0.3540,0.0000,0.2552,0,0.00917,0]
    solution = inverse_kinematics(target)
    
    if solution:
        print("逆运动学解 (弧度):", [f"{angle:.3f}" for angle in solution])
        print("转换为角度:", [f"{np.degrees(angle):.1f}°" for angle in solution])
    else:
        print("无法找到可行解")
