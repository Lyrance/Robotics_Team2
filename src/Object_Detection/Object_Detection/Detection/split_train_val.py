# coding:utf-8

import os
import random
import shutil

# 数据集划分比例
train_percent = 0.8
val_percent = 0.1

data_dir = '.'
image_path = os.path.join(data_dir, 'image')
label_path = os.path.join(data_dir, 'label')
output_path = os.path.join(data_dir, 'dataset')

# 获取图像和标注文件
image_files = sorted([f for f in os.listdir(image_path) if f.endswith(('.jpg', '.png', '.jpeg'))])
label_files = sorted([f for f in os.listdir(label_path) if f.endswith('.txt')])

if len(image_files) != len(label_files):
    raise ValueError("The number of image files and label files do not match.")

num_files = len(image_files)
indices = list(range(num_files))
random.shuffle(indices)

train_end = int(num_files * train_percent)
val_end = train_end + int(num_files * val_percent)

train_indices = indices[:train_end]
val_indices = indices[train_end:val_end]
test_indices = indices[val_end:]

# 创建输出文件夹结构
for split in ['images/train', 'images/val', 'images/test', 'labels/train', 'labels/val', 'labels/test']:
    split_path = os.path.join(output_path, split)
    if not os.path.exists(split_path):
        os.makedirs(split_path)

# 函数用于复制文件
def copy_files(file_indices, split):
    for i in file_indices:
        image_src = os.path.join(image_path, image_files[i])
        label_src = os.path.join(label_path, label_files[i])

        image_dst = os.path.join(output_path, f'images/{split}', image_files[i])
        label_dst = os.path.join(output_path, f'labels/{split}', label_files[i])

        shutil.copy2(image_src, image_dst)
        shutil.copy2(label_src, label_dst)

# 复制训练集、验证集和测试集的文件
copy_files(train_indices, 'train')
copy_files(val_indices, 'val')
copy_files(test_indices, 'test')
