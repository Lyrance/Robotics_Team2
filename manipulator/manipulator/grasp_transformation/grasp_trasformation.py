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

def base_to_ee(coordinate_base_to_ee):
    # 不需要完成，arm的函数只需要一个在基坐标系下的点
    pass


# x: 0.17800000309944153, y: 552.0, z: 238.0
camera_point = transform_pixel_to_camera(0.17800000309944153,552,178,color_intr)
print(camera_point)
camera_point = transform_camera_to_base(camera_point)
print(camera_point)


#Receiving YOLO coordinates: x: 0.17299999296665192, y: 518.0, z: 175.0
#[INFO] [1742937281.113684724] [TEST_ServerOfPerceptionAndGrasp]: Transformed coordinates: x: [-123.97365854], y: [78.03859868], z: [174.9455]
