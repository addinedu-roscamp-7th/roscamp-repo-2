import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from javis_interfaces.action import PickupBook
from rclpy.task import Future
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Pose
from flask import Flask, request, jsonify, current_app
import threading

app = Flask(__name__) # Flask 앱 인스턴스 생성
namespace = 'dobby1/main'

class PickupClient(Node):
    def __init__(self, namespace='', flask_port=8001):
        super().__init__('pickup_client', namespace=namespace)
        self.dobby_client = ActionClient(self, PickupBook, 'pickup_book')
        self.flask_thread = threading.Thread(target=self._run_flask_server, args=(flask_port,))
        self.flask_thread.daemon = True
        self.flask_thread.start()
        self.get_logger().info(f'Flask Server running on port {flask_port}')
    
    def _run_flask_server(self, port):
        # Flask 앱 컨텍스트에 ROS 노드 인스턴스를 저장합니다.1
        app.config['ros_node'] = self
        app.run(host='0.0.0.0', port=port, debug=False, use_reloader=False)
    
    def pickup_send_goal(self,
                        book_id: str,
                        shelf_approach_location: Pose2D,
                        book_pick_pose: Pose,
                        storage_id: int,
                        storage_approach_location: Pose2D,
                        storage_slot_pose: Pose
                        ) -> Future :
        pickup_goal_msg = PickupBook.Goal()
        pickup_goal_msg.book_id = book_id
        pickup_goal_msg.shelf_approach_location = shelf_approach_location
        pickup_goal_msg.book_pick_pose = book_pick_pose
        pickup_goal_msg.storage_id = storage_id
        pickup_goal_msg.storage_approach_location = storage_approach_location
        pickup_goal_msg.storage_slot_pose = storage_slot_pose

        self.get_logger().info("도비 서버 기다리는 중....")
        self.dobby_client.wait_for_server()

        return self.dobby_client.send_goal_async(
            pickup_goal_msg, feedback_callback=self.dobby_pickup_callback
        )
    
    def dobby_pickup_callback(self, feedback):
        fb = feedback.feedback
        self.get_logger().info(f'dobby pickup feedback: {fb.progress_percent}%')

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: {msg.data}')
        namespace = msg.data
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

        # JSON 데이터로부터 필요한 정보를 추출합니다.
        book_id = data.get('task_name', "default_task_name")
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
        
        # ROS 액션 목표 전송
        pickup_goal_future = node.pickup_send_goal(
            book_id=book_id,
            storage_id=storage_id,
            book_pick_pose=book_pick_pose,
            storage_approach_location=storage_approach_location,
            storage_slot_pose=storage_slot_pose,
            shelf_approach_location=shelf_approach_location
        )
        
        # 여기서 액션 결과를 기다리지 않고 바로 응답합니다.
        # 결과는 ROS 로그를 통해 비동기적으로 확인됩니다.
        return jsonify({'status': 'success', 'message': 'Pickup goal sent.'})

    except Exception as e:
        node.get_logger().error(f'Error processing /robot/pickup request: {e}')
        return jsonify({'status': 'error', 'message': str(e)}), 500

def main():
    rclpy.init()

    pickup_node = PickupClient(namespace=namespace)
    rclpy.spin(pickup_node)
    rclpy.shutdown()
