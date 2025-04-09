import numpy as np
from scipy.spatial.transform import Rotation as R

# 四元数：[x, y, z, w] 顺序！
quat = [0.5195315973224763, -0.4922606708607389, 0.46842276392100884, 0.5180216844379146]

# 平移向量
trans = [0.08121504019744888, 0.026930675029414246, -0.029660294622912864]

# 计算旋转矩阵
rot_matrix = R.from_quat(quat).as_matrix()  # 输出 3x3 旋转矩阵

# 构造齐次变换矩阵
H = np.eye(4)
H[0:3, 0:3] = rot_matrix
H[0:3, 3] = trans

# 打印变换矩阵
np.set_printoptions(precision=6, suppress=True)
print("Homogeneous Transformation Matrix H:\n", H)

res = np.array([[ 0.076519, -0.996796 ,-0.023283 , 0.081215],[-0.026184  ,0.021334 ,-0.999429 , 0.026931], [ 0.996724,  0.077085, -0.024467, -0.02966 ],[ 0.      ,  0. ,       0.      ,  1.      ]])

print(res)