import rclpy
from rclpy.node import Node
from package_with_interfaces.srv import ObjectGrab  # 导入自定义服务

class ObjectGrabServer(Node):
    def __init__(self):
        super().__init__('object_grab_server')
        # 创建服务端
        self.srv = self.create_service(ObjectGrab, 'object_grab_service', self.handle_object_grab_request)
        self.get_logger().info('机械臂服务已启动，等待抓取请求...')

    def handle_object_grab_request(self, request, response):
        # 接收来自客户端的物块坐标和检测状态
        x = request.object.x
        y = request.object.y
        z = request.object.z
        detected = request.object.detected

        # 输出接收到的坐标和状态
        if detected:
            self.get_logger().info(f'收到抓取请求: x={x:.2f}, y={y:.2f}, z={z:.2f}，检测状态: 已检测到')
        else:
            self.get_logger().info('收到抓取请求: 未检测到物块')

        # 判断抓取条件（示例条件: x, y, z 都不为 0 且已检测到）
        if detected and x != 0.0 and y != 0.0 and z != 0.0:
            # 模拟抓取成功
            self.get_logger().info('模拟抓取成功')
            response.success = True
        else:
            # 模拟抓取失败
            self.get_logger().info('模拟抓取失败')
            response.success = False

        return response

def main(args=None):
    rclpy.init(args=args)
    node = ObjectGrabServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
