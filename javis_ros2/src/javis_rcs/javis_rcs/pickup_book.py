import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from javis_interfaces.action import PickupBook
from rclpy.task import Future
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Pose
from flask import Flask, request, jsonify, current_app
import threading
import requests # HTTP 요청을 보내기 위해 추가

app = Flask(__name__) # Flask 앱 인스턴스 생성

class PickupClient(Node):
    """
    Flask 서버를 통해 PickupBook 액션을 요청받고,
    Dobby에게 작업을 지시한 후 결과를 외부 서버로 알리는 노드.
    """
    def __init__(self, namespace: str):
        super().__init__('pickup_client', namespace=namespace)

        # --- 파라미터 선언 ---
        self.declare_parameter('flask_port', 8001)
        self.declare_parameter('action_server_timeout', 10.0)
        self.declare_parameter('completion_report_url', 'http://localhost:8001/app/books')

        # --- 파라미터 값 가져오기 ---
        flask_port = self.get_parameter('flask_port').get_parameter_value().integer_value
        self.action_server_timeout = self.get_parameter('action_server_timeout').get_parameter_value().double_value
        self.completion_report_url = self.get_parameter('completion_report_url').get_parameter_value().string_value

        # 액션 클라이언트는 노드 생성 시 전달된 네임스페이스를 자동으로 상속받습니다.
        # 따라서 별도의 네임스페이스 지정 없이 생성합니다.
        self.dobby_client = ActionClient(self, PickupBook, 'pickup_book')

        # --- Flask 서버 스레드 실행 ---
        self.flask_thread = threading.Thread(target=self._run_flask_server, args=(flask_port,))
        self.flask_thread.daemon = True
        self.flask_thread.start()
        self.get_logger().info(f"Flask server started on port {flask_port}")
        self.get_logger().info(f"Pickup action client is targeting namespace: '{namespace}'")
    
    def _run_flask_server(self, port):
        """Flask 서버를 실행하는 내부 메서드."""
        # Flask 앱 컨텍스트에 ROS 노드 인스턴스를 저장합니다.
        app.config['ros_node'] = self
        app.run(host='0.0.0.0', port=port, debug=False, use_reloader=False)
    
    def pickup_send_goal(self,
                        book_id: str,
                        storage_id: int,
                        book_pick_pose: Pose,
                        storage_approach_location: Pose2D,
                        storage_slot_pose: Pose,
                        shelf_approach_location: Pose2D,
                        ) -> Future :
        """
        PickupBook 액션 목표(goal)를 DMC에 전송합니다.

        Args:
            book_id: 픽업할 책의 ID.
            storage_id: 보관함 ID.
            book_pick_pose: 책을 집을 때의 팔의 포즈.
            storage_approach_location: 보관함 접근 위치.
            storage_slot_pose: 보관함 슬롯 포즈.
            shelf_approach_location: 책장 접근 위치.

        Returns:
            서버로 목표를 비동기 전송하는 Future 객체.
        """
        pickup_goal_msg = PickupBook.Goal()
        pickup_goal_msg.book_id = book_id
        pickup_goal_msg.shelf_approach_location = shelf_approach_location
        pickup_goal_msg.book_pick_pose = book_pick_pose
        pickup_goal_msg.storage_id = storage_id
        pickup_goal_msg.storage_approach_location = storage_approach_location
        pickup_goal_msg.storage_slot_pose = storage_slot_pose

        self.get_logger().info("Dobby(DMC) 액션 서버를 기다리는 중...")
        if not self.dobby_client.wait_for_server(timeout_sec=self.action_server_timeout):
            self.get_logger().error('Dobby(DMC) 액션 서버가 응답하지 않습니다.')
            return None

        return self.dobby_client.send_goal_async(
            pickup_goal_msg, feedback_callback=self.dobby_pickup_callback
        )
    
    def dobby_pickup_callback(self, feedback):
        """액션 피드백을 수신했을 때 호출되는 콜백 함수."""
        fb = feedback.feedback
        self.get_logger().info(f'dobby pickup feedback: {fb.progress_percent}%')

    def goal_response_callback(self, future: Future):
        """서버의 목표 수락 여부를 처리하는 콜백 함수."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Pickup goal rejected')
            return

        self.get_logger().info('Pickup goal accepted. Waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future: Future):
        """액션의 최종 결과를 처리하고 외부 서버로 보고하는 콜백 함수."""
        result = future.result().result
        self.get_logger().info(f'Pickup result received: success={result.success}, message="{result.message}"')
        
        # 작업 완료 후 book_id를 GET 방식으로 전송
        if result.success:
            try:
                # book_id를 쿼리 파라미터로 전송
                response = requests.get(self.completion_report_url, params={'book_id': result.book_id})
                response.raise_for_status() # 2xx 응답이 아니면 예외 발생
                self.get_logger().info(
                    f"Successfully sent completion for book_id '{result.book_id}' to {self.completion_report_url}. "
                    f"Response: {response.text}"
                )
            except requests.exceptions.RequestException as e:
                self.get_logger().error(f"Failed to send completion for book_id '{result.book_id}': {e}")


# Flask 라우트를 클래스 외부에서 정의합니다.
@app.route('/robot/pickup', methods=['POST'])
def pickup_handler():
    # app.config에서 ROS 노드 인스턴스를 가져옵니다.
    node = current_app.config['ros_node']
    try:
        data = request.get_json()
        if not data:
            return jsonify({'status': 'error', 'message': 'Invalid JSON'}), 400

        node.get_logger().info(f'Received JSON data: {data}')

        # --- JSON 데이터 파싱 ---
        book_id = data.get('book_id', "default_task_name")
        book_info = data.get('book_info', {})
        location = data.get('location', {})
        storage_info = data.get('storage_info', {})
        storage_id = storage_info.get('storage_id', 0)
        shelf_info = data.get('shelf_info', {})
        shelf_location = shelf_info.get('shelf_location', {})
        shelf_approach_location = Pose2D(x=shelf_location.get('x', 0.0), y=shelf_location.get('y', 0.0), theta=shelf_location.get('facing', 0.0))
        book_pick_pose = Pose(x=book_info.get('x', 0.0), y=book_info.get('y', 0.0), z=book_info.get('z', 0.0))
        storage_approach_location = Pose2D(x=storage_info.get('location_x', 0.0), y=storage_info.get('location_y', 0.0), theta=storage_info.get('location_facing', 0.0))
        storage_slot_pose = Pose(x=storage_info.get('pose_x', 0.0), y=storage_info.get('pose_y', 0.0), z=storage_info.get('pose_z', 0.0))
        node.get_logger().info(f'Parsed pickup parameters: book_id={book_id}, storage_id={storage_id}')
        # --- ROS 액션 목표 전송 ---
        pickup_goal_future = node.pickup_send_goal(
            book_id=book_id,
            storage_id=storage_id,
            book_pick_pose=book_pick_pose,
            storage_approach_location=storage_approach_location,
            storage_slot_pose=storage_slot_pose,
            shelf_approach_location=shelf_approach_location
        )
        
        if pickup_goal_future is None:
            return jsonify({'status': 'error', 'message': 'Failed to send goal to action server.'}), 503

        # 목표 전송 후, 응답을 비동기적으로 처리하기 위해 콜백을 추가합니다.
        pickup_goal_future.add_done_callback(node.goal_response_callback)

        # 여기서 액션 결과를 기다리지 않고 바로 응답합니다.
        # 실제 작업 결과는 콜백을 통해 비동기적으로 처리됩니다.
        return jsonify({'status': 'success', 'message': 'Pickup goal sent.'})

    except Exception as e:
        node.get_logger().error(f'Error processing /robot/pickup request: {e}')
        return jsonify({'status': 'error', 'message': str(e)}), 500
    


def main():
    rclpy.init()

    # 노드를 생성할 때 네임스페이스를 명시적으로 전달합니다.
    # 서버가 'dobby1/main' 네임스페이스에서 실행되므로 클라이언트도 동일하게 설정합니다.
    namespace = 'dobby1/main'
    pickup_node = PickupClient(namespace=namespace)
    try:
        rclpy.spin(pickup_node)
    except KeyboardInterrupt:
        pickup_node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        pickup_node.destroy_node()
        rclpy.shutdown()
