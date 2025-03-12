import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def calculate_non_inf_probability(file_list, distances):
    probabilities = []

    for file in file_list:
        data = pd.read_csv(file)
        non_inf_counts = []

        for ranges in data['ranges']:
            # 将 ranges 从字符串转换为浮点数列表
            ranges_list = [float(x) if x != 'inf' else np.inf for x in ranges.split(',')]
            # 获取前10个值
            front_10 = ranges_list[10:20]
            # 统计非 inf 值的数量
            non_inf_count = sum(1 for x in front_10 if x != np.inf)
            non_inf_counts.append(non_inf_count)

        # 计算非 inf 值的概率
        total_entries = len(non_inf_counts) * 10
        total_non_inf = sum(non_inf_counts)
        probability = (total_non_inf / total_entries) * 100  # 转换为百分比
        probabilities.append(probability)

    return probabilities

# 文件列表和对应的距离
file_list = [
    '7m_distance.csv',
    '8m_distance.csv',
    '9m_distance.csv',
    '10m_distance.csv',
]
distances = [12, 14, 15, 16]

# 计算概率
probabilities = calculate_non_inf_probability(file_list, distances)
print(probabilities)

# 绘制曲线图
plt.figure(figsize=(8, 6))
plt.plot(distances, probabilities, marker='o')
plt.title('前10个 ranges 非 inf 值的概率随距离变化图')
plt.xlabel('距离 (cm)')
plt.ylabel('非 inf 值概率 (%)')
plt.ylim(0, 100)
plt.xticks(distances)
plt.grid(True)
plt.show()