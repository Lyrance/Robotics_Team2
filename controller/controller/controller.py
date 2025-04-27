import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from package_with_interfaces.srv import SetRoverState

# 定义所有状态
ALL_STATES = ["INITIATE", "EXPLORE", "FOUND", "GRAB", "RETURN", "RELEASE"]

class RoverController(Node):
    def __init__(self, name="rover_controller"):
        super().__init__(name)
        # 当前状态初始化为 INITIATE
        self.current_state = "EXPLORE"

        # 发布状态的话题
        self.state_pub = self.create_publisher(String, "/rover_status", 10)

        # 创建服务 (用于外部请求改变状态)
        self.srv = self.create_service(
            SetRoverState,
            "/set_rover_state",
            self.handle_set_state
        )
        # 定时器：周期性发布当前状态
        self.timer = self.create_timer(1.0, self.publish_state)

    def publish_state(self):
        """定时器回调：发布当前状态"""
        msg = String()
        msg.data = self.current_state
        self.state_pub.publish(msg)
        self.get_logger().info(f"[RoverController] Current state: {self.current_state}")

    def handle_set_state(self, request, response):
        """服务回调：尝试切换状态"""
        new_state = request.new_state
        if new_state not in ALL_STATES:
            self.get_logger().error(f"Invalid state request: {new_state}")
            response.success = False
            return response

        # 检查状态跳转是否允许
        allowed = self.check_transition(self.current_state, new_state)
        if allowed:
            self.current_state = new_state
            self.get_logger().info(f"State changed to: {new_state}")
            response.success = True
        else:
            self.get_logger().warn(f"Transition from {self.current_state} to {new_state} not allowed!")
            response.success = False

        return response

    def check_transition(self, old, new):
        """
        允许的跳转关系可自行定义。
        下面示例：INITIATE -> EXPLORE -> FOUND -> GRAB -> RETURN -> RELEASE -> INITIATE
        """
        valid_transitions = {
            "INITIATE": ["EXPLORE"],
            "EXPLORE":  ["FOUND"],
            "FOUND":    ["GRAB"],
            "GRAB":     ["RETURN"],
            "RETURN":   ["RELEASE"],
            "RELEASE":  ["EXPLORE"]
        }
        return (new in valid_transitions.get(old, []))

def main(args=None):
    rclpy.init(args=args)
    node = RoverController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
