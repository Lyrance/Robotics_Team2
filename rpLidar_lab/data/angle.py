import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import re

def parse_distance_from_filename(filename):
    # 从文件名中提取距离值，单位为厘米
    match = re.match(r'angle_1m_([\d\.]+)cm_distance\.csv', filename)
    if match:
        distance_cm = float(match.group(1))
        return distance_cm
    else:
        raise ValueError(f"文件名 {filename} 不符合预期格式。")

def calculate_probabilities(file_list):
    distances = []
    probabilities = []

    for file in file_list:
        # 提取距离并添加到列表
        distance_cm = parse_distance_from_filename(file)
        distances.append(distance_cm)

        # 读取 CSV 文件
        data = pd.read_csv(file)
        total_rows = len(data)
        valid_rows = 0

        for ranges in data['ranges']:
            # 将 ranges 从字符串转换为浮点数列表
            ranges_list = [float(x) if x != 'inf' else np.inf for x in ranges.split(',')]
            # 获取前30个数
            first_30 = ranges_list[0:50]
            # 如果前30个数中包含 inf，则该行为有效行
            if np.inf in first_30:
                valid_rows += 1

        # 计算有效行的概率
        probability = valid_rows / total_rows
        probabilities.append(probability)

    return distances, probabilities

# 文件列表
file_list = [
    'angle_1m_1.2cm_distance.csv',
    'angle_1m_1.5cm_distance.csv',
    'angle_1m_1.7cm_distance.csv',
    'angle_1m_2.0cm_distance.csv',
    'angle_1m_2.3cm_distance.csv',
    'angle_1m_2.6cm_distance.csv',
    'angle_1m_2.8cm_distance.csv',
    'angle_1m_3.0cm_distance.csv',
    'angle_1m_3.5cm_distance.csv',
    'angle_1m_4.5cm_distance.csv',
    'angle_1m_5.5cm_distance.csv'
]

# 计算概率
distances, probabilities = calculate_probabilities(file_list)

# 根据距离对数据进行排序
distances, probabilities = zip(*sorted(zip(distances, probabilities)))

# 输出具体数值
print("距离(cm)\t概率")
for distance, probability in zip(distances, probabilities):
    print(f"{distance:.1f}\t\t{probability:.2%}")

# 绘制概率随距离变化的曲线图
plt.figure(figsize=(10, 6))
plt.plot(distances, probabilities, marker='o')
plt.title('有效行概率随距离的变化')
plt.xlabel('距离 (cm)')
plt.ylabel('概率')
plt.ylim(0, 1)  # 概率范围为0到1
plt.grid(True)
plt.show()