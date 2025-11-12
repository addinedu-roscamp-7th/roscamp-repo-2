import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from javis_interfaces.action import PickBook
from geometry_msgs.msg import Pose
class PickBookActionClient(Node):
    def __init__(self):
        super().__init__('pick_book_action_client')
        self._client = ActionClient(self, PickBook, 'dobby2/arm/pick_book')
    def send_goal(self):
        # 서버 준비 대기
        self.get_logger().info(':hourglass_flowing_sand: Waiting for PickBook action server...')
        self._client.wait_for_server()
        self.get_logger().info(':white_check_mark: PlaceBook action server connected.')
        # Goal 생성
        goal_msg = PickBook.Goal()
        goal_msg.book_id = "6"
        # 포즈는 지금 스켈레톤이라 의미 없음 (dummy pose)
        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = 0.0
        pose.orientation.w = 1.0
        goal_msg.book_pose = pose
        goal_msg.carrier_slot_id = 0
        # Goal 전송
        self.get_logger().info(f":rocket: Sending PickBook goal: {goal_msg.book_id}")
        self._send_goal_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(':x: Goal rejected by server.')
            rclpy.shutdown()
            return
        self.get_logger().info(':dart: Goal accepted, waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # 서버에서 [STATUS] ... 형식으로 current_action을 보냄
        self.get_logger().info(f":satellite_antenna: Feedback: {feedback.current_action}")
    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(
            f":white_check_mark: Result: success={result.success}, message='{result.message}'"
        )
        rclpy.shutdown()
def main(args=None):
    rclpy.init(args=args)
    node = PickBookActionClient()
    node.send_goal()
    rclpy.spin(node)
if __name__ == '__main__':
    main()