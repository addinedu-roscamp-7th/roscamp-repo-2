import rclpy
from rclpy.node import Node
from javis_interfaces.action import CleanSeat
from rclpy.action import ActionServer, GoalResponse, CancelResponse
import time

class CleanSeatServer(Node):
    def __init__(self, namespace = ''):
        super().__init__('clean_seat_server', namespace=namespace)
        self._server = ActionServer(
            self,
            CleanSeat,
            'clean_seat',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        
        )

    def goal_callback(self, goal_request: CleanSeat.Goal):
        self.get_logger().info(f"좌석 정리 요청, task_id: {goal_request.task_id}, seat_id: {goal_request.seat_id}, 좌석위치:[location : {goal_request.return_desk_location}, pose: {goal_request.return_desk_pose}], 쓰레기 위치: [location: {goal_request.bin_location}, pose: {goal_request.bin_pose}]")
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT
    
    async def execute_callback(self, goal_handle):
        goal = goal_handle.request
        feedback_msg = CleanSeat.Feedback()
        self.get_logger().info('좌석 정리 시작!!!')
        count = 0

        while count < 10:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = CleanSeat.Result()
                result.task_id = goal.task_id
                result.seat_id = goal.seat_id
                result.success = False
                result.message = '작업 취소'
                result.trash_collected_count = 4
                result.trash_types = ["캔", "플라스틱", "유리", "일반쓰레기"]
                result.total_distance_m = count * 0.05
                result.total_time_sec = count * 0.1
                self.get_logger().info("작업이 취소되었습니다.")
                return result
            count += 1
            feedback_msg.progress_percent = count * 10
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.5)

        goal_handle.succeed()
        result = CleanSeat.Result()
        result.task_id = goal.task_id
        result.seat_id = goal.seat_id
        result.success = True
        result.message = "좌석 정리 완료"
        result.trash_collected_count = 4
        result.trash_types = ["캔", "플라스틱", "유리", "일반쓰레기"]
        result.total_distance_m = count * 0.05
        result.total_time_sec = count * 0.1
        self.get_logger().info("좌석 정리 완료")
        return result
    
def main():
    rclpy.init()

    node = CleanSeatServer(namespace='dobby1/main')

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()