import cv2
import numpy as np

# 读取归一化坐标文件
def read_coordinates(file_path):
    coordinates = []
    with open(file_path, 'r') as f:
        for line in f:
            data = line.strip().split()
            # 忽略第一个 class_id
            points = list(map(float, data[1:]))
            # 将点对存入列表
            coordinates.append([(points[i], points[i + 1]) for i in range(0, len(points), 2)])
    return coordinates

# 将归一化的坐标转换为图像坐标
def denormalize_coordinates(points, img_width, img_height):
    return [(int(x * img_width), int(y * img_height)) for x, y in points]

# 在图像上绘制点
def draw_points_on_image(image_path, coordinates):
    # 读取图像
    image = cv2.imread(image_path)
    img_height, img_width = image.shape[:2]

    # 反归一化并绘制点
    for points in coordinates:
        denormalized_points = denormalize_coordinates(points, img_width, img_height)
        for point in denormalized_points:
            cv2.circle(image, point, radius=3, color=(0, 0, 255), thickness=-1)  # 绘制红色小圆点

    # 显示结果
    cv2.imshow('Image with Points', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# 主函数
if __name__ == "__main__":
    # 设置文件路径
    coordinates_file_path = "bridge_yellow_color_1.0.txt"  # 这是存储归一化坐标的文件
    image_path = "bridge_yellow_color_1.0.jpg"  # 这是你想要绘制点的图像

    # 读取坐标并绘制
    coordinates = read_coordinates(coordinates_file_path)
    draw_points_on_image(image_path, coordinates)
