import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from javis_interfaces.action import PerformTask
from rclpy.task import Future
from geometry_msgs.msg import Pose2D, Point, Quaternion, Pose
from flask import Flask, request, jsonify, current_app
import threading
import requests # HTTP 요청을 보내기 위해 추가
import sys

app = Flask(__name__) # Flask 앱 인스턴스 생성

class KreacherPerformTask(Node):
    """
    Flask 서버를 통해 PickupBook 액션을 요청받고,
    Dobby에게 작업을 지시한 후 결과를 외부 서버로 알리는 노드.
    """
    def __init__(self, namespace: str):
        super().__init__('perform_task', namespace=namespace)

        self._client = ActionClient(self, PerformTask, 'perform_task')
        self.task_done_future: Future | None = None
    
    
    
    def send_goal(self,
                        member_id: int
                        ) -> Future :
      
        goal_msg = PerformTask.Goal()
        goal_msg.member_id = member_id
        self.get_logger().info("Kreacher Controller(KC) 액션 서버를 기다리는 중...")
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Dobby(DMC) 액션 서버가 응답하지 않습니다.')
            raise RuntimeError("Action server not available within timeout.")

        return self._client.send_goal_async(
            goal_msg, feedback_callback=self.pickup_callback
        )
    
    def pickup_callback(self, feedback):
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
        result = future.result().result
        self.get_logger().info(f'작업 완료 결과: {result.message}')
        if self.task_done_future is not None and not self.task_done_future.done():
            self.task_done_future.set_result({'success': result.success, 'message': result.message})

    def run_task(self, book_id: str, **kwargs) -> Future:
        """
        지정된 seat_id에 대한 clean_seat 작업 실행.
        작업 완료 시 결과를 되돌려줄 Future를 반환합니다.
        외부(예: Orchestrator)에서 spin_until_future_complete로 기다리면 됩니다.
        """
        # ✅ Node에는 create_future가 없으므로, rclpy.task.Future로 직접 생성
        self.task_done_future = Future()

        # kwargs로부터 Pose2D / Pose 생성
        book_id = str(book_id)
        storage_id = int(kwargs.get('storage_id'))
        shelf_approach_location = self._make_pose2d(kwargs.get('shelf_approafch_location', {}))
        book_pick_pose = self._make_pose(kwargs.get('book_pick_pose', {}))
        storage_approach_location = self._make_pose2d(kwargs.get('storage_approach_locaiton', {}))
        storage_slot_pose = self._make_pose(kwargs.get('storage_slot_pose', {}))

        # Goal 전송 후, goal 응답 완료 콜백 체인 연결
        goal_future = self.send_goal(book_id=book_id,storage_id=storage_id, shelf_approach_location=shelf_approach_location, book_pick_pose=book_pick_pose, storage_approach_location=storage_approach_location, storage_slot_pose=storage_slot_pose)
        goal_future.add_done_callback(self.goal_response_callback)

        return self.task_done_future
    
    def _make_pose2d(self, d: dict) -> Pose2D:
        p = Pose2D()
        p.x = float(d.get('x', 0.0))
        p.y = float(d.get('y', 0.0))
        p.theta = float(d.get('theta', 0.0))
        return p

    def _make_pose(self, d: dict) -> Pose:
        """
        d = {
          'position': {'x':..., 'y':..., 'z':...},
          'orientation': {'x':..., 'y':..., 'z':..., 'w':...}
        }
        각각 없으면 기본값 사용
        """
        pose = Pose()
        pos = d.get('position', {})
        ori = d.get('orientation', {})

        pose.position = Point(
            x=float(pos.get('x', 0.0)),
            y=float(pos.get('y', 0.0)),
            z=float(pos.get('z', 0.0)),
        )
        pose.orientation = Quaternion(
            x=float(ori.get('x', 0.0)),
            y=float(ori.get('y', 0.0)),
            z=float(ori.get('z', 0.0)),
            w=float(ori.get('w', 1.0)),
        )
        return pose
    


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 3:
        print("Usage: ros2 run javis_rcs pickup_book <robot_namespace> <book_id>")
        return

    robot_namespace = sys.argv[1]
    book_id = sys.argv[2]

    # ✅ __init__ 시그니처 수정에 맞게 사용
    node = PickupBook(namespace=f'{robot_namespace}/main')

    try:
        node.get_logger().info(f"Starting pickup_book task for book_id: {book_id}")
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("PickupBook 노드가 종료됩니다.")

    finally:
        node.destroy_node()
        rclpy.shutdown()