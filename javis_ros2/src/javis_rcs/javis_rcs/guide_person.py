import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from javis_interfaces.action import GuidePerson
from rclpy.task import Future
from geometry_msgs.msg import Pose2D


class GuidePersonClient(Node):
    """Dobby의 GuidePerson 액션을 요청하고 결과를 처리하는 클라이언트 노드입니다."""
    def __init__(self, namespace: str):
        super().__init__('guide_person_client', namespace=namespace)

        # --- 파라미터 선언 ---
        self.declare_parameter('action_server_timeout', 10.0)

        # --- 파라미터 값 가져오기 ---
        self.action_server_timeout = self.get_parameter('action_server_timeout').get_parameter_value().double_value

        # 액션 클라이언트는 노드 생성 시 전달된 네임스페이스를 자동으로 상속받습니다.
        self._client = ActionClient(self, GuidePerson, 'guide_person')
        self.goal_handle = None
        self.get_logger().info(f"GuidePerson client node started, targeting namespace: '{namespace}'")

    def send_goal(self, dest_location: Pose2D):
       
        goal_msg = GuidePerson.Goal()
        goal_msg.dest_location = dest_location

        self.get_logger().info("Dobby(DMC) 액션 서버를 기다리는 중...")
        if not self._client.wait_for_server(timeout_sec=self.action_server_timeout):
            self.get_logger().error('Dobby(DMC) 액션 서버가 응답하지 않습니다. 노드를 종료합니다.')
            rclpy.shutdown()
            return

        self.get_logger().info("Dobby(DMC) 액션 서버를 찾았습니다. 목표를 전송합니다...")
        send_goal_future = self._client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def feedback_callback(self, feedback):
        """액션 실행 중 서버로부터 피드백을 수신했을 때 호출됩니다."""
        fb = feedback.feedback
        self.get_logger().info(
            f'Feedback: 남은 거리={fb.distance_remaining_m:.2f}m, '
            f'사람 감지 여부={fb.person_detected}'
        )

    def goal_response_callback(self, future: Future):
        """서버가 목표를 수락했는지 여부를 처리합니다."""
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('길안내 요청이 거부되었습니다.')
            rclpy.shutdown()
            return

        self.get_logger().info('길안내 요청이 수락되었습니다. 결과를 기다립니다...')
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future: Future):
        """액션의 최종 결과를 처리합니다."""
        result = future.result().result
        self.get_logger().info(f"작업 완료 결과: success={result.success}, message='{result.message}'")
        
        # 모든 작업이 완료되었으므로 노드를 종료합니다.
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    # 서버가 'dobby1/main' 네임스페이스에서 실행되므로 클라이언트도 동일하게 설정합니다.
    namespace = 'dobby1/main'
    guide_person_node = GuidePersonClient(namespace=namespace)

    destination = Pose2D(x=1.2, y=5.3, theta=3.14)
    guide_person_node.get_logger().info(f"목표 위치 {destination.x}, {destination.y}로 길안내를 요청합니다.")
    guide_person_node.send_goal(dest_location=destination)

    try:
        rclpy.spin(guide_person_node)
    except KeyboardInterrupt:
        guide_person_node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        guide_person_node.destroy_node()
        rclpy.shutdown()
