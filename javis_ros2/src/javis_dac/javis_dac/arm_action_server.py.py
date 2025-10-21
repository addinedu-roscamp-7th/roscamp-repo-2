import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from javis_interfaces.action import PickBook
from pymycobot.mycobot import MyCobot

PORT = "/dev/ttyUSB0"
BAUD = 1000000
SPEED = 25

class PickBookActionServer(Node):
    def __init__(self):
        super().__init__('pick_book_action_server')
        self._action_server = ActionServer(
            self,
            PickBook,
            'pinky1/arm/pick_book',  # DMC에서 지정한 액션 네임
            self.execute_callback)
        self.mc = MyCobot(PORT, BAUD)
        self.mc.send_angles([0, 0, 0, 0, 0, 0], 25)
        self.get_logger().info('✅ PickBook Action Server Ready')

    async def execute_callback(self, goal_handle):
        self.get_logger().info(f"📚 PickBook goal received: {goal_handle.request.book_id}")

        feedback = PickBook.Feedback()
        feedback.status = "Moving to pose..."
        feedback.current_action = "MoveToTarget"
        goal_handle.publish_feedback(feedback)

        # Pose 값 적용
        pose = goal_handle.request.book_pose
        x, y, z = pose.position.x, pose.position.y, pose.position.z
        self.mc.send_coords([x, y, z, 180, 0, 0], SPEED, 1)

        self.get_logger().info('🦾 Move completed')
        goal_handle.succeed()

        result = PickBook.Result()
        result.success = True
        result.book_id = goal_handle.request.book_id
        result.message = "PickBook completed successfully!"
        return result

def main(args=None):
    rclpy.init(args=args)
    node = PickBookActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
