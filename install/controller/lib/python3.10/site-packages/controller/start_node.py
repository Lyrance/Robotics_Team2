import rclpy
from rclpy.node import Node
from package_with_interfaces.srv import SetRoverState

class SwitchInitiateToExplore(Node):
    def __init__(self):
        super().__init__("switch_initiate_to_explore")

        # 创建客户端，请求 /set_rover_state 服务
        self.client = self.create_client(SetRoverState, '/set_rover_state')
        
        # 等待服务可用
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /set_rover_state service...')

        self.get_logger().info("Service is ready, sending request to switch INITIATE -> EXPLORE...")

        # 构造请求：希望切换到 "EXPLORE"
        request = SetRoverState.Request()
        request.new_state = "EXPLORE"

        # 异步发送请求
        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        # 获取服务调用结果
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Switched state from INITIATE to EXPLORE: success!")
            else:
                self.get_logger().warn("Switched state from INITIATE to EXPLORE: failed or not allowed!")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")

        # 收到响应后，节点退出
        self.get_logger().info("Node will shut down now.")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = SwitchInitiateToExplore()

    # 如果回调会shutdown，这里spin一下就够了
    rclpy.spin(node)

    # 不再调用 rclpy.shutdown()，去掉finally块或把它留空
    node.destroy_node()


if __name__=="__main__":
    main()
