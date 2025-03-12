import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import re

def parse_distance_from_filename(filename):
    # 从文件名中提取距离值，单位转换为米
    match = re.match(r'(\d+)(cm|m)_distance\.csv', filename)
    if match:
        value, unit = match.groups()
        distance = float(value)
        if unit == 'cm':
            distance /= 100  # 将厘米转换为米
        return distance
    else:
        raise ValueError(f"文件名 {filename} 不符合预期格式。")

def calculate_measured_values(file_list):
    actual_distances = []
    measured_values = []
    errors = []

    for file in file_list:
        actual_distance = parse_distance_from_filename(file)
        actual_distances.append(actual_distance)
        data = pd.read_csv(file)
        averages = []

        for ranges in data['ranges']:
            # 将 ranges 从字符串转换为浮点数列表
            ranges_list = [float(x) if x != 'inf' else np.nan for x in ranges.split(',')]
            # 提取前 10 个非 NaN 值
            front_10 = [x for x in ranges_list[10:20] if not np.isnan(x)]
            if front_10:
                avg_measurement = np.mean(front_10)
                averages.append(avg_measurement)
        if averages:
            measured_value = np.mean(averages)
            measured_values.append(measured_value)
            error = measured_value - actual_distance
            errors.append(error)
        else:
            measured_values.append(np.nan)
            errors.append(np.nan)

    return actual_distances, measured_values, errors

# 文件列表
file_list = [
    '15cm_distance.csv',
    '16cm_distance.csv',
    '30cm_distance.csv',
    '50cm_distance.csv',
    '1m_distance.csv',
    '2m_distance.csv',
    '3m_distance.csv',
    '4m_distance.csv',
    '5m_distance.csv',
    '6m_distance.csv',
    '7m_distance.csv',
]

# 计算测量值和误差
actual_distances, measured_values, errors = calculate_measured_values(file_list)

# 绘制误差曲线图
plt.figure(figsize=(10, 6))
plt.plot(actual_distances, errors, marker='o')
plt.title('测量误差随实际距离的变化')
plt.xlabel('实际距离 (m)')
plt.ylabel('误差 (测量值 - 实际值) (m)')
plt.grid(True)
plt.show()

# 输出结果
for actual, measured, error in zip(actual_distances, measured_values, errors):
    print(f"实际距离: {actual:.2f} m, 测量值: {measured:.2f} m, 误差: {error:.2f} m")