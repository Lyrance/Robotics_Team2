#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

import numpy as np
import math

# TF 相关
import tf2_ros
from geometry_msgs.msg import TransformStamped

class ExplorerNode(Node):
    def __init__(self):
        super().__init__('explorer')
        self.get_logger().info("Explorer Node Started")

        # ----------- TF 相关初始化 -----------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ----------- 订阅地图 -----------
        self.map_sub = self.create_subscription(
            OccupancyGrid, 
            '/map',
            self.map_callback,
            10
        )

        # ----------- Action Client (Nav2) -----------
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ----------- 成员变量 -----------
        self.visited_frontiers = set()       # 记录已经访问过的前沿
        self.map_data = None                 # 接收 OccupancyGrid 后存储
        self.robot_position = (0, 0)         # 机器人在栅格中的行列坐标 (row, col)
        self.initial_position = None         # 机器人初始位置 (同样是栅格坐标)
        self.is_returning = False            # 是否在返回初始位置

        # ----------- 定时器 -----------
        # 1) 每 1 秒更新一次机器人位姿
        self.pose_timer = self.create_timer(1.0, self.update_robot_position)
        # 2) 每 5 秒执行一次探索
        self.explore_timer = self.create_timer(5.0, self.explore)
        # 3) 每 120 秒返回初始位置
        self.return_timer = self.create_timer(120.0, self.return_to_initial_position)

    # ------------------- 通过 TF 获取机器人位置 -------------------
    def update_robot_position(self):
        """
        每隔 1 秒执行一次：尝试获取 'map' -> 'base_footprint' 的 TF，更新机器人在栅格中的坐标 (row, col)。
        """
        if self.map_data is None:
            return  # 还没接收到地图，不做处理

        # 地图的分辨率和原点
        resolution = self.map_data.info.resolution
        origin_x   = self.map_data.info.origin.position.x
        origin_y   = self.map_data.info.origin.position.y

        try:
            # 从 TF Buffer 中查询变换: map -> base_footprint
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                'map',               # source frame
                'base_footprint',         # target frame
                now, 
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            # 提取平移部分 (x, y, _)
            x = transform.transform.translation.x
            y = transform.transform.translation.y

            # 将世界坐标 (x,y) 转换成栅格坐标 (row, col)
            # 注意 int(...) 会向下取整，可根据需要取 round(...)。
            col = int((x - origin_x) / resolution)
            row = int((y - origin_y) / resolution)

            self.robot_position = (row, col)
            # 下面可做一些日志打印，方便调试
            # self.get_logger().info(f"Robot grid position: row={row}, col={col}")

        except Exception as e:
            # 如果超时没获取到变换，或者其他错误
            self.get_logger().warning(f"Failed to get TF map->base_footprint: {e}")

    # ------------------- 地图回调 -------------------
    def map_callback(self, msg):
        self.map_data = msg
        self.get_logger().info("Map received")

    # ------------------- 导航到指定 (x, y) 世界坐标 -------------------
    def navigate_to(self, x, y):
        """
        发送导航目标给 Nav2 (NavigateToPose)
        """
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        # orientation 这里简单设置 w=1
        goal_msg.pose.orientation.w = 1.0

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_msg

        self.get_logger().info(f"Navigating to goal: x={x:.2f}, y={y:.2f}")

        # 等待 Action Server 就绪
        self.nav_to_pose_client.wait_for_server()
        send_goal_future = self.nav_to_pose_client.send_goal_async(nav_goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        处理目标响应
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning("Goal rejected!")
            return

        self.get_logger().info("Goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_complete_callback)

    def navigation_complete_callback(self, future):
        """
        处理导航完成的结果
        """
        try:
            result = future.result().result
            self.get_logger().info(f"Navigation completed with result: {result}")
        except Exception as e:
            self.get_logger().error(f"Navigation failed: {e}")

    # ------------------- 前沿相关 -------------------
    def find_frontiers(self, map_array):
        """
        在 occupancy grid (numpy 数组) 中查找前沿。 
        前沿定义：本网格=0，邻居中有 -1(未知)。
        """
        frontiers = []
        rows, cols = map_array.shape

        for r in range(1, rows - 1):
            for c in range(1, cols - 1):
                if map_array[r, c] == 0:  # 可行走区域
                    # 判断周围(3x3)是否包含 -1
                    neighbors = map_array[r-1:r+2, c-1:c+2].flatten()
                    if -1 in neighbors:
                        frontiers.append((r, c))

        self.get_logger().info(f"Found {len(frontiers)} frontiers")
        return frontiers

    def is_valid_frontier(self, frontier, map_array):
        """
        检查前沿是否有效：避免越界、太靠近障碍物等
        """
        r, c = frontier
        rows, cols = map_array.shape

        # 1) 越界检查
        if r <= 1 or r >= rows - 2 or c <= 1 or c >= cols - 2:
            return False

        # 2) 周围是否包含占据=100的障碍
        sub_map = map_array[r-1:r+2, c-1:c+2]
        if np.any(sub_map == 100):
            return False

        return True

    def choose_frontier(self, frontiers, map_array):
        """
        选择离机器人位置最近的有效前沿
        """
        robot_row, robot_col = self.robot_position
        min_distance = float('inf')
        chosen_frontier = None

        for frontier in frontiers:
            if frontier in self.visited_frontiers:
                continue
            if not self.is_valid_frontier(frontier, map_array):
                continue

            # 计算欧几里得距离
            distance = math.sqrt((robot_row - frontier[0])**2 + (robot_col - frontier[1])**2)
            if distance < min_distance:
                min_distance = distance
                chosen_frontier = frontier

        if chosen_frontier:
            self.visited_frontiers.add(chosen_frontier)
            self.get_logger().info(f"Chosen frontier: {chosen_frontier}")
        else:
            self.get_logger().warning("No valid frontier found")

        return chosen_frontier

    # ------------------- 主探索逻辑，每 5 秒执行一次 -------------------
    def explore(self):
        # 如果已经在返回初始位置，不再继续探索
        if self.is_returning:
            return

        # 如果尚未收到地图
        if self.map_data is None:
            self.get_logger().warning("No map data available")
            return

        # 若初始位置还没设置，则保存当前机器人栅格位置作为初始位置
        if self.initial_position is None:
            self.initial_position = self.robot_position
            self.get_logger().info(f"Initial position saved: {self.initial_position}")

        # 将 map_data 转成 numpy array
        map_array = np.array(self.map_data.data).reshape(
            (self.map_data.info.height, self.map_data.info.width)
        )

        # 找前沿
        frontiers = self.find_frontiers(map_array)
        if not frontiers:
            self.get_logger().info("No frontiers found. Exploration complete!")
            return

        # 选最近的前沿
        chosen_frontier = self.choose_frontier(frontiers, map_array)
        if not chosen_frontier:
            self.get_logger().warning("No valid frontiers to explore")
            return

        # 前沿 (r, c) 转成世界坐标 (x, y)
        resolution = self.map_data.info.resolution
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y

        r, c = chosen_frontier
        goal_x = c * resolution + origin_x
        goal_y = r * resolution + origin_y

        # 发送导航目标
        self.navigate_to(goal_x, goal_y)

    def return_to_initial_position(self):
        """
        每 120 秒执行一次，看是否需要返回初始位置
        """
        if self.initial_position is None:
            self.get_logger().warning("Initial position not set!")
            return

        if self.is_returning:
            return

        # 开始返回
        self.is_returning = True
        goal_x, goal_y = self.convert_grid_to_world(self.initial_position)
        self.get_logger().info(f"Returning to initial position: x={goal_x:.2f}, y={goal_y:.2f}")
        self.navigate_to(goal_x, goal_y)

    def convert_grid_to_world(self, grid_pos):
        """
        辅助函数，把 (row, col) 栅格坐标转成世界坐标 (x, y)
        """
        r, c = grid_pos
        resolution = self.map_data.info.resolution
        origin_x   = self.map_data.info.origin.position.x
        origin_y   = self.map_data.info.origin.position.y

        x = c * resolution + origin_x
        y = r * resolution + origin_y
        return (x, y)

    def shutdown_robot(self):
        self.get_logger().info("Shutting down robot exploration")


def main(args=None):
    rclpy.init(args=args)
    explorer_node = ExplorerNode()

    try:
        explorer_node.get_logger().info("Starting exploration...")
        rclpy.spin(explorer_node)
    except KeyboardInterrupt:
        explorer_node.get_logger().info("Exploration stopped by user")
    finally:
        explorer_node.destroy_node()
        rclpy.shutdown()
