import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from javis_interfaces.action import PerformTask
from javis_interfaces.msg import DobbyState # DobbyState 메시지 임포트
from rclpy.task import Future
import sys

# kreacher의 작업 가능 상태를 나타내는 전역 변수
kreacher_state = True 

class KreacherPerform(Node):
    """
    Orchestrator로부터 PerformTask 액션 요청을 받아 수행하고,
    자신의 상태를 토픽으로 발행하는 노드.
    """
    def __init__(self, namespace: str):
        super().__init__('kreacher_perform', namespace=namespace)

        self._client = ActionClient(self, PerformTask, 'perform_task')
        self.task_done_future: Future | None = None
    
    def send_goal(self,
                        order_id: int,
                        menu_id: int,
                        quantity: int
                        ) -> Future :
        global kreacher_state
        goal_msg = PerformTask.Goal()
        goal_msg.order_id = order_id
        goal_msg.menu_id = menu_id
        goal_msg.quantity = quantity
        kreacher_state = False
        self.get_logger().info("Kreacher(KC) 액션 서버를 기다리는 중...")
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Kreacher 액션 서버가 응답하지 않습니다.')
            kreacher_state = True # 작업 실패 시 상태를 True로 변경
            raise RuntimeError("Action server not available within timeout.")
        
        return self._client.send_goal_async(
            goal_msg, feedback_callback=self.perform_callback
        )
    
    def perform_callback(self, feedback):
        """액션 피드백을 수신했을 때 호출되는 콜백 함수."""
        fb = feedback.feedback
        self.get_logger().info(f'Kreacher perform feedback: 주문번호: {fb.order_id}진행률: {fb.progress_percentage}%')
        

    def goal_response_callback(self, future: Future):
        """서버의 목표 수락 여부를 처리하는 콜백 함수."""
        global kreacher_state
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Kreacher goal rejected')
            kreacher_state = True # 작업이 거절되면 다시 True로 변경
            return

        self.get_logger().info('Kreacher goal accepted. Waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future: Future):
        global kreacher_state
        result = future.result().result
        self.get_logger().info(f'작업 완료 결과: {result.message}')
        if self.task_done_future is not None and not self.task_done_future.done():
            self.task_done_future.set_result({'order_id':result.order_id, 'pick_up_num':result.pick_up_num,'success': result.success, 'message': result.message})
        kreacher_state = True # 작업 완료 후 상태를 True로 변경
    
    def run_task(self, order_id: int, menu_id: int, quantity: int, **kwargs) -> Future:
        """
        지정된 order_id에 대한 perform_task 작업 실행.
        """
        # ✅ Node에는 create_future가 없으므로, rclpy.task.Future로 직접 생성
        self.task_done_future = Future()

        # kwargs로부터 Pose2D / Pose 생성

        # Goal 전송 후, goal 응답 완료 콜백 체인 연결
        goal_future = self.send_goal(order_id=order_id, menu_id=menu_id, quantity=quantity)
        goal_future.add_done_callback(self.goal_response_callback)

        return self.task_done_future

def get_kreacher_state():
    """kreacher의 현재 상태를 반환합니다."""
    return kreacher_state


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 4:
        print("Usage: ros2 run javis_rcs kreacher_perform")
        return

   
    order_id = int(sys.argv[1])
    menu_id = int(sys.argv[2])
    quantity = int(sys.argv[3])

    # ✅ __init__ 시그니처 수정에 맞게 사용
    node = KreacherPerform(namespace='kreacher/action')

    try:
        node.get_logger().info(f"Starting kreacher task for order_id: {order_id}, menu_id: {menu_id}, quantity: {quantity}")
    except KeyboardInterrupt:
        node.get_logger().info("KreacherPerform 노드가 종료됩니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()