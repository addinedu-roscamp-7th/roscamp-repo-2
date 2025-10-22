import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from javis_interfaces.action import PerformTask
from rclpy.task import Future
import argparse
import sys

class KreacherPerformTask(Node):
    """Kreacher의 PerformTask 액션을 요청하고 결과를 처리하는 클라이언트 노드입니다."""
    def __init__(self, namespace: str):
        super().__init__('kreacher_perform_task_client', namespace=namespace)

        # --- 파라미터 선언 ---
        self.declare_parameter('action_server_timeout', 10.0)

        # --- 파라미터 값 가져오기 ---
        self.action_server_timeout = self.get_parameter('action_server_timeout').get_parameter_value().double_value

        # 액션 클라이언트는 노드 생성 시 전달된 네임스페이스를 자동으로 상속받습니다.
        self._client = ActionClient(self, PerformTask, 'perform_task')
        self.goal_handle = None
        self.get_logger().info(f"Kreacher client node started, targeting namespace: '{namespace}'")

    def send_goal(self, member_id: int):
    
        goal_msg = PerformTask.Goal()
        goal_msg.member_id = member_id

        self.get_logger().info("Kreacher 액션 서버를 기다리는 중...")
        if not self._client.wait_for_server(timeout_sec=self.action_server_timeout):
            self.get_logger().error('Kreacher 액션 서버가 응답하지 않습니다. 노드를 종료합니다.')
            # 서버가 없으면 더 이상 진행할 수 없으므로 종료
            rclpy.shutdown()
            return

        self.get_logger().info("Kreacher 액션 서버를 찾았습니다. 목표를 전송합니다...")
        send_goal_future = self._client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        # 목표 전송 후 응답 처리를 위해 콜백을 등록합니다.
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def feedback_callback(self, feedback):
        """액션 실행 중 서버로부터 피드백을 수신했을 때 호출됩니다."""
        fb = feedback.feedback
        self.get_logger().info(f'feedback = 상태:{fb.status} 진행률: {fb.progress_percentage}%')

    def goal_response_callback(self, future: Future):
        """서버가 목표를 수락했는지 여부를 처리합니다."""
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('Kreacher 작업이 거절되었습니다.')
            # 작업이 거절되었으므로 노드를 종료합니다.
            rclpy.shutdown()
            return

        self.get_logger().info('Kreacher 작업이 수락되었습니다. 결과를 기다립니다...')
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future: Future):
        """액션의 최종 결과를 처리합니다."""
        result = future.result().result
        self.get_logger().info(f'작업 완료 결과 =  success: {result.success}, message: {result.message}, pick_up_num: {result.pick_up_num}')
        
        # 모든 작업이 완료되었으므로 노드를 종료합니다.
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    # --- 커맨드 라인 인자 파싱 ---
    parser = argparse.ArgumentParser(description='Kreacher PerformTask 액션 클라이언트')
    parser.add_argument('member_id', type=int, help='작업을 요청할 멤버의 ID')
    # rclpy가 사용하는 ROS 관련 인자를 제외하고 파싱합니다.
    parsed_args = parser.parse_args(args=rclpy.utilities.remove_ros_args(args=sys.argv)[1:])

    # Kreacher 서버의 네임스페이스를 명시적으로 전달합니다.
    namespace = 'kreacher/action'
    perform_task_node = KreacherPerformTask(namespace=namespace)
    
    # 파싱된 member_id로 목표 전송
    perform_task_node.send_goal(member_id=parsed_args.member_id)

    try:
        # rclpy.shutdown()이 호출될 때까지 노드를 실행합니다.
        rclpy.spin(perform_task_node)
    except KeyboardInterrupt:
        perform_task_node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        # 노드가 종료되기 전에 goal_handle을 취소하려고 시도할 수 있습니다.
        if perform_task_node.goal_handle:
            perform_task_node.get_logger().info('Canceling the goal...')
            perform_task_node.goal_handle.cancel_goal_async()
        
        perform_task_node.destroy_node()
        rclpy.shutdown()