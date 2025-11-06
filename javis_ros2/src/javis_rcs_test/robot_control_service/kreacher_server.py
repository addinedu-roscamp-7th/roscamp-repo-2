
import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

from javis_interfaces.action import PerformTask

class KreacherActionServer(Node):
    """
    Kreacher의 'PerformTask' 액션 요청을 처리하는 서버 노드입니다.
    작업을 시뮬레이션하고, 피드백과 결과를 클라이언트에게 반환합니다.
    """
    def __init__(self):
        # 노드 이름은 'kreacher_action_server'로, 네임스페이스는 'kreacher/action'으로 설정합니다.
        # 이렇게 하면 최종 액션 서버 이름이 '/kreacher/action/perform_task'가 됩니다.
        super().__init__('kreacher_action_server', namespace='kreacher/action')

        self.action_server = ActionServer(
            self,
            PerformTask,
            'perform_task',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info("Kreacher 'PerformTask' 액션 서버가 준비되었습니다.")

    def goal_callback(self, goal_request):
        """새로운 목표 요청을 수락할지 결정합니다."""
        menu = ""
        if goal_request.order_id == 1:
            menu = "핫아메리카노"
        elif goal_request.order_id == 2:
            menu = "아이스아메리카노"
        
        
        self.get_logger().info(f"새로운 작업 요청 수신: 주문번호={goal_request.order_id}, 메뉴={menu}, 얼음양: {goal_request.quantity}")
        # 모든 요청을 일단 수락합니다.
        return GoalResponse.ACCEPT

    def cancel_callback(self, cancel_request):
        """클라이언트의 작업 취소 요청을 처리합니다."""
        self.get_logger().info("작업 취소 요청 수신.")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """실제 액션 작업을 수행합니다."""
        order_id = goal_handle.request.order_id
        menu_id = goal_handle.request.menu_id
        quantity = goal_handle.request.quantity
        
        self.get_logger().info(f"작업시작 order_id={order_id}")

        # 피드백 메시지 생성
        feedback_msg = PerformTask.Feedback()
        feedback_msg.order_id = order_id

        # 5초간 1초마다 진행률을 20%씩 올리며 피드백을 보냅니다.
        for i in range(5):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("작업이 취소되었습니다.")
                return PerformTask.Result()

            progress = (i + 1) * 20
            feedback_msg.progress_percentage = float(progress)
            self.get_logger().info(f"피드백 전송: 진행률 {progress}%")
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        # 작업 성공으로 설정
        goal_handle.succeed()

        # 결과 메시지 생성 및 반환
        result = PerformTask.Result()
        result.success = True
        result.message = f"음료 제조 완료 (주문 ID: {order_id})"
        result.order_id = order_id
        result.pick_up_num = 1 # 예시 픽업 번호
        
        self.get_logger().info(f"작업 완료. 결과: {result.message}")
        return result

def main(args=None):
    rclpy.init(args=args)
    kreacher_server = KreacherActionServer()
    rclpy.spin(kreacher_server)
    kreacher_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
