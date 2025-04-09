#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time

# 请确保这里导入的 ObjectGrab 消息类型与服务端代码一致
from package_with_interfaces.srv import ObjectGrab

class GraspTestClient(Node):
    def __init__(self):
        super().__init__('grasp_test_client')
        # 创建服务客户端，对应服务端的服务名称需一致
        self.cli = self.create_client(ObjectGrab, 'object_grab_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 "object_grab_service" 服务中...')
        self.get_logger().info('已连接 "object_grab_service" 服务。')

    def send_request(self, state_desc: str, x: float, y: float, z: float, angle: float, detected: bool) -> bool:
        """
        构造一个请求并同步等待响应。

        参数：
          - state_desc：状态描述，用于日志标识请求阶段
          - x, y, z：模拟接收到的目标信息（注意 x 在抓取阶段需要大于 0.1）
          - angle：目标物体角度（单位：度）
          - detected：是否检测到物体

        返回：
          - 服务响应中 success 字段的值
        """
        req = ObjectGrab.Request()
        # 显式转换为 float，防止类型错误
        req.object.x = float(x)
        req.object.y = float(y)
        req.object.z = float(z)
        req.object.angle = float(angle)
        req.object.detected = detected

        self.get_logger().info(
            f"[{state_desc}] 发送请求：x={x}, y={y}, z={z}, angle={angle}, detected={detected}"
        )
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            success = future.result().success
            self.get_logger().info(f"[{state_desc}] 接收到响应: success = {success}")
            return success
        else:
            self.get_logger().error("调用服务失败")
            return False

def main(args=None):
    rclpy.init(args=args)
    test_client = GraspTestClient()
    
    # --------------- 测试流程说明 ------------------
    # 根据服务端状态机：
    #   1. 初始状态 "grab"：传入模拟有效抓取数据（x > 0.1 且 detected True），成功后状态转为 "return"。
    #   2. 下一次请求，状态处于 "return"，服务端执行回返动作后进入 "release"。
    #   3. 再下一次调用，状态为 "release"，服务端执行释放动作后进入 "explore"。
    #   4. 后续调用，处于 "explore" 状态下服务端不执行抓取。
    # -------------------------------------------------
    
    # 1. 模拟抓取状态请求（grab）
    result = test_client.send_request("grab", 0.2, 250.0, 250.0, 30.0, True)
    time.sleep(1.0)
    
    # 2. 模拟回返状态请求（return）
    result = test_client.send_request("return", 0.0, 0.0, 0.0, 0.0, False)
    time.sleep(1.0)
    
    # 3. 模拟释放状态请求（release）
    result = test_client.send_request("release", 0.0, 0.0, 0.0, 0.0, False)
    time.sleep(1.0)
    
    # 4. 模拟探索状态请求（explore）
    result = test_client.send_request("explore", 0.0, 0.0, 0.0, 0.0, False)
    
    test_client.get_logger().info("状态机测试结束。")
    test_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
