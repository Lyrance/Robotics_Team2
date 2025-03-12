import rclpy
from rclpy.node import Node
from package_with_interfaces.srv import ObjectGrab  

class ObjectGrabServer(Node):
    def __init__(self):
        super().__init__('object_grab_server')
        
        self.srv = self.create_service(ObjectGrab, 'object_grab_service', self.handle_object_grab_request)
        self.get_logger().info('arm activated...')

    def handle_object_grab_request(self, request, response):
        
        x = request.object.x
        y = request.object.y
        z = request.object.z
        detected = request.object.detected

        
        if detected:
            self.get_logger().info(f'recieve the request: x={x:.2f}, y={y:.2f}, z={z:.2f}，state: true')
        else:
            self.get_logger().info('recieve the request: state: false')

        
        if detected and x != 0.0 and y != 0.0 and z != 0.0:
            
            self.get_logger().info('grab success')
            response.success = True
        else:
            
            self.get_logger().info('grab fail')
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
