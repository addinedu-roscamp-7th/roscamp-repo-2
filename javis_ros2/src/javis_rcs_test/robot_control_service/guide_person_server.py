import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from javis_interfaces.action import GuidePerson
import time

class GuidePersonServer(Node):
    def __init__(self, namespace: str = ''):
        super().__init__('guide_person_server', namespace=namespace)
        self._server = ActionServer(
            self,
            GuidePerson,
            'guide_person',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

    def goal_callback(self, goal_request: GuidePerson.Goal):
        self.get_logger().info(f'길안내 요청: 목적지 위치:{goal_request.dest_location}')

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('요청 거절당함')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        goal = goal_handle.request
        feedback_msg = GuidePerson.Feedback()
        self.get_logger().info('길안내 시작')
        count = 0

        while count < 10:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = GuidePerson.Result()
                result.success = False
                result.error_code = "test code"
                result.total_distance_m = 0.909
                result.total_total_time_sec = 0.421
                self.get_logger().info('작업이 취소되었습니다.')
                return result
            count += 1
            feedback_msg.distance_remaining_m = count * 0.2
            feedback_msg.person_detected = True
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.5)

        goal_handle.succeed()
        result = GuidePerson.Result()
        result.success = True
        result.error_code = 1
        result.total_distance_m = 9.09
        result.total_time_sec = 4.21
        result.message = "길안내 완료"
        self.get_logger().info('길안내를 완료했습니다.')
        return result


def main():

    rclpy.init()

    node = GuidePersonServer(namespace='dobby2/main')

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()