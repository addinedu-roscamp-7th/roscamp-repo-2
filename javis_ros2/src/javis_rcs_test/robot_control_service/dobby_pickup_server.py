import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from javis_interfaces.action import PickupBook
import time

class DobbyPickupServer(Node):
    def __init__(self, namespace=''):
        super().__init__('dobby_pickup_server', namespace=namespace)
        self.pickup_server = ActionServer(
            self,
            PickupBook,
            'pickup_book',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

    def goal_callback(self, goal_request: PickupBook.Goal):
        self.get_logger().info(f'도서 픽업 요청, book_id:{goal_request.book_id}, storage_id:{goal_request.storage_id}')

        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    
    async def execute_callback(self, goal_handle):
        goal = goal_handle.request
        feedback_msg = PickupBook.Feedback()
        self.get_logger().info('픽업 시작')
        count = 0
        
  

        while count < 10:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = PickupBook.Result()
                result.book_id = goal.book_id
                result.storage_id = goal.storage_id
                result.success = False
                result.message = '작업 취소'
                result.total_distance_m = count * 0.5
                result.total_time_sec = 0.5
                self.get_logger().info('작업이 취소되었습니다.')
                return result
            count += 1
            feedback_msg.progress_percent = count * 10
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.5)

        goal_handle.succeed()
        result = PickupBook.Result()
        result.book_id = goal.book_id
        result.storage_id = goal.storage_id
        result.success = True
        result.message = '픽업 완료'
        result.total_distance_m = count * 1.0
        result.total_time_sec = count * 0.2
        self.get_logger().info('픽업을 완료했습니다.')
        return result

    
    
def main():
    rclpy.init()

    node = DobbyPickupServer(namespace='dobby2/main')

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()