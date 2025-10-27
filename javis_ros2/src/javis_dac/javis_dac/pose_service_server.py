import rclpy
from rclpy.node import Node
from javis_dac.srv import ChangeArmPose
from geometry_msgs.msg import Pose

class PoseService(Node):
    def __init__(self):
        super().__init__('pose_service_server')
        self.srv = self.create_service(ChangeArmPose, 'dobby1/arm/change_pose', self.callback)
        self.get_logger().info('🦾 Pose Change Service Ready')

    def callback(self, request, response):
        self.get_logger().info(f"Pose change request: type={request.pose_type}")
        response.success = True
        response.message = "Pose changed successfully"
        response.pose_type = request.pose_type
        return response

def main(args=None):
    rclpy.init(args=args)
    node = PoseService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
