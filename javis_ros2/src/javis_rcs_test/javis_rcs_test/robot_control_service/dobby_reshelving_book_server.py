import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from javis_interfaces.action import ReshelvingBook
import time

class DobbyReshelvingBookServer(Node):
    def __init__(self):
        super().__init__('dobby_reshelving_book_server')
        self._server = ActionServer(
            self,
            ReshelvingBook,
            'reshelving_book',
            execute_callback = self.execute_callback,
            goal_callback = self.goal_callback,
            cancel_callback = self.cancel_callback
        )

    def cancel_callback(self):

        return CancelResponse.ACCEPT
    
    def goal_callback(self, goal_request: ReshelvingBook.Goal):
        self.get_logger().info(f'작업 아이디 : {goal_request.task_id}, 책상 위치 : [location: {goal_request.return_desk_location}, pose: {goal_request.return_desk_pose}]')
        return GoalResponse.ACCEPT
    
    async def execute_callback(self, goal_handle):
        goal = goal_handle.request
        feedback_msg = ReshelvingBook.Feedback()
        self.get_logger().info('도서 재배치 시작')

        count = 0

        while count < 10:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = ReshelvingBook.Result()
                result.task_id = goal.task_id
                result.succes = False
                result.message = '재배치 작업 취소'
                result.failed_book_ids = ["마법천자문 3권", "코믹 메이플 60권"]
                result.books_processed = 3
                result.total_distance_m = 2.5
                result.total_time_sec = 1.29
                return result
            count += 1
            feedback_msg.progress_percent = count * 10
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.5)

        goal_handle.succeed()

        result = ReshelvingBook.Result()
        result.task_id = goal.task_id
        result.success = True
        result.message = '5권중 2권 성공'
        result.books_processed = 2
        result.failed_book_ids = ["마법천자문 3권", "코믹 메이플 60권", "why책 로봇편"]
        result.total_distance_m = count * 0.2
        result.total_time_sec = count * 0.025
        self.get_logger().info('도서 재배치가 끝났습니다.')
        return result
    
def main():
    rclpy.init()

    node = DobbyReshelvingBookServer()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown
