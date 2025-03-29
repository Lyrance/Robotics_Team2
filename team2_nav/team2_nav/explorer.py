#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import String
import numpy as np
import math
import tf2_ros
from geometry_msgs.msg import TransformStamped
from package_with_interfaces.msg import ObjectInformation
from package_with_interfaces.srv import SetRoverState
import math
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

class ExplorerNode(Node):
    def __init__(self):
        super().__init__('explorer')
        self.get_logger().info("Explorer Node Started")

        self.rover_status_sub = self.create_subscription(
            String,
            '/rover_status',
            self.rover_status_callback,
            10
        )
        self.rover_status = None

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
        self.initial_oritation = None

        # ----------- 定时器 -----------
        # 1) 每 1 秒更新一次机器人位姿
        self.pose_timer = self.create_timer(1.0, self.update_robot_position)
        # 2) 每 5 秒执行一次探索
        self.explore_timer = self.create_timer(5.0, self.explore)

        # 创建服务客户端 -> /set_rover_state
        self.state_client = self.create_client(SetRoverState, '/set_rover_state')
        while not self.state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for rover_controller ...")

        # 订阅 /target_color
        self.target_color_sub = self.create_subscription(
            ObjectInformation,
            '/target_color',
            self.target_color_callback,
            10
        )
        self.target_color_sub  # 确保不被垃圾回收

        # 初始状态为未发现目标
        self.target_detected = False

    # ------------------- 通过 TF 获取机器人位置 -------------------
    def update_robot_position(self):
        """
        每隔 1 秒执行一次：尝试获取 'map' -> 'base_footprint' 的 TF,更新机器人在栅格中的坐标 (row, col)。
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

    def rover_status_callback(self, msg):
        self.rover_status = msg.data
        self.get_logger().info(f"Current status: {self.rover_status}")

    # ------------------- 导航到指定 (x, y) 世界坐标 -------------------
    def navigate_to(self, x, y, orientation = 1.0):
        """
        发送导航目标给 Nav2 (NavigateToPose)
        """
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        # orientation 这里简单设置 w=1
        goal_msg.pose.orientation.w = orientation

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

    def call_set_state(self, new_state):
        """避免重复请求相同状态"""
        if new_state == self.current_state:
            self.get_logger().info(f"No need to switch: already in {new_state}")
            return

        if not self.state_client.service_is_ready():
            self.get_logger().warn("Service not ready!")
            return

        req = SetRoverState.Request()
        req.new_state = new_state
        fut = self.state_client.call_async(req)
        fut.add_done_callback(self.response_callback)

    # ------------------- 主探索逻辑，每 5 秒执行一次 -------------------
    def explore(self):

        # 如果尚未收到地图
        if self.map_data is None:
            self.get_logger().warning("No map data available")
            return

        # 将 map_data 转成 numpy array
        map_array = np.array(self.map_data.data).reshape(
            (self.map_data.info.height, self.map_data.info.width)
        )

        if self.rover_status == "INITIATE":
            # 保存当前机器人栅格位置作为初始
            self.initial_position = self.robot_position
            self.initial_oritation = self.map_data.info.origin.orientation.w
            self.get_logger().info(f"Initial position saved: {self.initial_position}")

        elif self.rover_status == "GRAB" or self.rover_status == "RELEASE":
            self.get_logger().info("Status requires holding position. No exploration.")
            return

        elif self.rover_status == "EXPLORE":
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

        elif self.rover_status == "RETURN":
            # 调用返回初始位置的逻辑
            if np.abs(self.initial_position[0]-self.robot_position[0]) + np.abs(self.initial_position[1]-self.robot_position[1]) and \
                np.abs(self.initial_oritation-self.map_data.info.origin.orientation.w):
                self.get_logger().info("Reached the initial position!")
                self.call_set_state("RELEASE")
            else:
                self.return_to_initial_position()

        elif self.rover_status == "FOUND":
            # 保留该状态的处理，稍后补充
            pass

        else:
            # 如果状态未知，或者未设置，发出警告
            self.get_logger().warning("Unrecognized status or status not set.")

    def return_to_initial_position(self):
        """
        返回初始位置
        """
        if self.initial_position is None:
            self.get_logger().warning("Initial position not set!")
            return

        # 开始返回
        goal_x, goal_y = self.convert_grid_to_world(self.initial_position)
        self.get_logger().info(f"Returning to initial position: x={goal_x:.2f}, y={goal_y:.2f}")
        self.navigate_to(goal_x, goal_y, self.initial_oritation)

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



    def target_color_callback(self, msg):
        if self.rover_status != "FOUND":
            return

        if not msg.detected:
            return

        x = msg.x       # 像素横坐标
        z = msg.z       # 与目标的距离
        center_x = 424  # 相机分辨率的一半(848/2)

        # 如果 z < 0.2，就认为到达目标
        if z < 0.2:
            self.get_logger().info("Target is very close, stop.")
            return

        # 判断 x 偏差，决定旋转方向或直行
        # 例如给定一个阈值 10 像素
        offset_threshold = 10  
        if x < center_x - offset_threshold:
            # 目标在左边，原地左转一点
            self.rotate_in_place_using_nav2("left", 5.0)
        elif x > center_x + offset_threshold:
            # 目标在右边，原地右转一点
            self.rotate_in_place_using_nav2("right", 5.0)
        else:
            # 目标大致居中，可以尝试向前推进一小步
            self.step_forward_using_nav2(distance=0.2)

    def rotate_in_place_using_nav2(self, rotate_direction: str = "left", rotate_angle_deg: float = 5.0):
        """
        让机器人在原地旋转一个固定的小角度，并调用 Nav2 的 navigate_to_pose。
        rotate_direction: "left" 或 "right"
        rotate_angle_deg: 旋转角度（度）
        """
        # 1) 获取当前机器人在 map 下的位姿
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                'map',             
                'base_footprint',   
                now, 
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
        except Exception as e:
            self.get_logger().warning(f"Failed to get TF map->base_footprint: {e}")
            return

        # 当前平移
        current_x = transform.transform.translation.x
        current_y = transform.transform.translation.y
        
        # 当前朝向（四元数 -> 欧拉角）
        q = transform.transform.rotation
        (roll, pitch, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # 2) 计算新的 yaw
        sign = 1.0 if rotate_direction == "left" else -1.0
        rotate_angle_rad = math.radians(rotate_angle_deg)
        new_yaw = yaw + sign * rotate_angle_rad  # 左转为正，右转为负

        # 3) 转回四元数
        new_q = quaternion_from_euler(roll, pitch, new_yaw)

        # 4) 构造新目标
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = current_x
        goal_msg.pose.position.y = current_y
        goal_msg.pose.orientation.x = new_q[0]
        goal_msg.pose.orientation.y = new_q[1]
        goal_msg.pose.orientation.z = new_q[2]
        goal_msg.pose.orientation.w = new_q[3]

        # 5) 发给 Nav2
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_msg

        self.nav_to_pose_client.wait_for_server()
        send_goal_future = self.nav_to_pose_client.send_goal_async(nav_goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def step_forward_using_nav2(self, distance=0.2):
        now = rclpy.time.Time()
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                now,
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
        except Exception as e:
            self.get_logger().warning(f"Failed to get TF map->base_footprint: {e}")
            return

        current_x = transform.transform.translation.x
        current_y = transform.transform.translation.y

        q = transform.transform.rotation
        (roll, pitch, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # 在当前yaw方向前方distance米
        goal_x = current_x + distance * math.cos(yaw)
        goal_y = current_y + distance * math.sin(yaw)

        # 仍保持当前 yaw
        new_q = quaternion_from_euler(roll, pitch, yaw)

        # 构造 PoseStamped
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = goal_x
        goal_msg.pose.position.y = goal_y
        goal_msg.pose.orientation.x = new_q[0]
        goal_msg.pose.orientation.y = new_q[1]
        goal_msg.pose.orientation.z = new_q[2]
        goal_msg.pose.orientation.w = new_q[3]

        # 发送给 Nav2
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_msg
        self.nav_to_pose_client.wait_for_server()
        send_goal_future = self.nav_to_pose_client.send_goal_async(nav_goal)
        send_goal_future.add_done_callback(self.goal_response_callback)




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
