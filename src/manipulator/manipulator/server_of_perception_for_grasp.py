import rclpy
from rclpy.node import Node

# TODO: 根据实际情况导入正确的 GoalPosition 服务消息类型
from package_interfaces.srv import ObjectGrab

from interbotix_common_modules.common_robot.robot import (
    create_interbotix_global_node,
    robot_shutdown,
    robot_startup,
)
from interbotix_perception_modules.armtag import InterbotixArmTagInterface
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

class ServerOfPerceptionAndGrasp(Node):
    def __init__(self,name):
        super().__init__(name)
        self.srv = self.create_service(ObjectGrab, 'object_grab_service', self.callback) #TODO message type needed to be confirmed

        # Arm Parameters
        self.ROBOT_MODEL = 'px150'
        self.ROBOT_NAME = self.ROBOT_MODEL
        self.REF_FRAME = 'camera_color_optical_frame'
        self.ARM_TAG_FRAME = f'{self.ROBOT_NAME}/ar_tag_link'
        self.ARM_BASE_FRAME = f'{self.ROBOT_NAME}/base_link'

        # Arm Initialization

        # Create a global node to serve as the backend for each API component
        self.global_node = create_interbotix_global_node()
        # Initialize the arm module along with the pointcloud and armtag modules
        self.bot = InterbotixManipulatorXS(
            robot_model=self.ROBOT_MODEL,
            robot_name=self.ROBOT_NAME,
            node=self.global_node,
        )
        self.pcl = InterbotixPointCloudInterface(
            node_inf=self.global_node,
        )
        armtag = InterbotixArmTagInterface(
            ref_frame=self.REF_FRAME,
            arm_tag_frame=self.ARM_TAG_FRAME,
            arm_base_frame=self.ARM_BASE_FRAME,
            node_inf=self.global_node,
        )

        # Start up the API
        robot_startup(self.global_node)

        # set initial arm and gripper pose
        self.bot.arm.go_to_sleep_pose()
        self.bot.gripper.release()

        # get the ArmTag pose
        armtag.find_ref_to_arm_base_transform()
        self.bot.arm.set_ee_pose_components(x=0.3, z=0.2)

    def callback(self,request,response):
        """
        接收请求，获取点云中各个目标簇的位置
        获取request后,确认可执行再调用perception_and_grasp()

        TODO
        写一个客户端,用于发送一个request包含success(是否可执行),clusters(目标位置簇)
        """
        # get the cluster positions
        # sort them from max to min 'x' position w.r.t. the ARM_BASE_FRAME
        # success, clusters = self.pcl.get_cluster_positions(
        #     ref_frame=self.ARM_BASE_FRAME,
        #     sort_axis='x'
        #     reverse=True
        

        # clusters = request.clusters
        # success = request.success

        # response = success
        # if success:
        #     self.perception_and_grasp(clusters)
        #     self.get_logger().info('Get cluster positions sucessfully!')
        #     return response
        # else:
        #     return response

        x = request.object.x
        y = request.object.y
        z = request.object.z
        detected = request.object.detected

        if detected:
            self.perception_and_grasp(x, y, z)
            response.success = True
        else:
            response.success = False
        
    def perception_and_grasp(self,x, y, z):
        """
        TODO:
        The parameters of positions needed to be repalced by measuring in real world
        """
        # for cluster in clusters:
        #     x, y, z = cluster['position']
        try:
            
            self.bot.arm.set_ee_pose_components(x=x, y=y, z=z + 0.05, pitch=0.5)
            self.bot.arm.set_ee_pose_components(x=x, y=y, z=z, pitch=0.5)
            self.bot.gripper.grasp()

            
            self.bot.arm.set_ee_pose_components(x=x, y=y, z=z + 0.05, pitch=0.5)
            self.bot.arm.set_ee_pose_components(x=0.3, z=0.2)
            self.bot.gripper.release()

            
            self.bot.arm.set_ee_pose_components(x=0.3, z=0.2)
            self.bot.arm.go_to_sleep_pose()

            self.get_logger().info("grab success")
            return True

        except Exception as e:
            self.get_logger().error(f"grab fail: {str(e)}")
            return False
        #     print(x, y, z)
        #     self.bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.05, pitch=0.5)
        #     self.bot.arm.set_ee_pose_components(x=x, y=y, z=z, pitch=0.5)
        #     self.bot.gripper.grasp()
        #     self.bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.05, pitch=0.5)
        #     self.bot.arm.set_ee_pose_components(x=0.3, z=0.2)
        #     self.bot.gripper.release()

        # self.bot.arm.set_ee_pose_components(x=0.3, z=0.2)
        # self.bot.arm.go_to_sleep_pose()

def main(args = None):
    rclpy.init(args=args)
    node = ServerOfPerceptionAndGrasp('ServerOfPerceptionAndGrasp')
    rclpy.spin(node)
    robot_shutdown(node.global_node)
    Node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    