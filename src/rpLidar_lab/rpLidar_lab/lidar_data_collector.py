# lidar_data_collector.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import csv
import os

class LidarDataCollector(Node):
    def __init__(self):
        super().__init__('lidar_data_collector')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.scan_data = []
        self.frame_count = 0
        self.max_frames = 100  # 每个位置采集的帧数
        self.data_directory = 'data'  # 数据保存目录
        self.saved_data = False
        if not os.path.exists(self.data_directory):
            os.makedirs(self.data_directory)
        self.experiment_name = 'distance'

    def listener_callback(self, msg):
        self.scan_data.append({
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment,
            'ranges': msg.ranges
        })
        self.frame_count += 1
        if self.frame_count >= self.max_frames and not self.saved_data:
            self.save_data()
            self.saved_data = True
            self.get_logger().info('Data collection complete for current position.')
            rclpy.shutdown()

    def save_data(self):
        filename = os.path.join(self.data_directory, f'angle_1m_1.2cm_{self.experiment_name}.csv')
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            header = ['frame', 'angle_min', 'angle_max', 'angle_increment', 'ranges']
            writer.writerow(header)
            for i, data in enumerate(self.scan_data):
                writer.writerow([
                    i,
                    data['angle_min'],
                    data['angle_max'],
                    data['angle_increment'],
                    ','.join(map(str, data['ranges']))
                ])
        self.get_logger().info(f'Data saved to {filename}')

def main(args=None):
    rclpy.init(args=args)
    node = LidarDataCollector()
    rclpy.spin(node)

if __name__ == '__main__':
    main()