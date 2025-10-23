import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import sys
from javis_interfaces.action import CleanSeat as CS
from rclpy.task import Future
from geometry_msgs.msg import Pose, Point, Pose2D, Quaternion
import threading


class CleanSeat(Node):
    def __init__(self, namespace: str = ''):
        """
        Node에는 create_future가 없으므로, 필요한 Future는 rclpy.task.Future로 직접 생성합니다.
        """
        super().__init__('clean_seat', namespace=namespace)
        # 네임스페이스 상대 경로 'clean_seat' 액션 서버를 사용 (예: /<ns>/clean_seat 로 해석됨)
        self._client = ActionClient(self, CS, 'clean_seat')
        self.task_done_future: Future | None = None

    # ⚠️ 굳이 async로 둘 필요가 없습니다. ActionClient는 Future를 돌려주므로 그대로 반환하면 됩니다.
    def send_goal(
        self,
        seat_id: int,
        return_desk_location: Pose2D,
        return_desk_pose: Pose,
        bin_location: Pose2D,
        bin_pose: Pose
    ) -> Future:
        goal_msg = CS.Goal()
        goal_msg.seat_id = seat_id
        goal_msg.return_desk_location = return_desk_location
        goal_msg.return_desk_pose = return_desk_pose
        goal_msg.bin_location = bin_location
        goal_msg.bin_pose = bin_pose

        self.get_logger().info("도비 액션 서버 연결 대기 중...")
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("도비 액션 서버에 연결할 수 없습니다.")
            raise RuntimeError("Action server not available within timeout.")

        # goal 응답 Future 반환 (goal handle가 담김)
        return self._client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

    def feedback_callback(self, feedback):
        fb = feedback.feedback
        self.get_logger().info(f'feedback = 진행률: {fb.progress_percent}%')

    def run_task(self, seat_id: int, **kwargs) -> Future:
        """
        지정된 seat_id에 대한 clean_seat 작업 실행.
        작업 완료 시 결과를 되돌려줄 Future를 반환합니다.
        외부(예: Orchestrator)에서 spin_until_future_complete로 기다리면 됩니다.
        """
        # ✅ Node에는 create_future가 없으므로, rclpy.task.Future로 직접 생성
        self.task_done_future = Future()

        # kwargs로부터 Pose2D / Pose 생성
        rd_loc = self._make_pose2d(kwargs.get('return_desk_location', {}))
        rd_pose = self._make_pose(kwargs.get('return_desk_pose', {}))
        bin_loc = self._make_pose2d(kwargs.get('bin_location', {}))
        bin_ps = self._make_pose(kwargs.get('bin_pose', {}))

        # Goal 전송 후, goal 응답 완료 콜백 체인 연결
        goal_future = self.send_goal(seat_id, rd_loc, rd_pose, bin_loc, bin_ps)
        goal_future.add_done_callback(self._goal_response_callback)

        return self.task_done_future

    # --- 콜백 체인: goal 응답 → result 응답 → 최종 Future 완료 ---

    def _goal_response_callback(self, future: Future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('좌석 정리 작업 거절됨')
            # ✅ 최종 Future 완료 처리
            if self.task_done_future is not None and not self.task_done_future.done():
                self.task_done_future.set_result({'success': False, 'message': 'Goal rejected'})
            return

        self.get_logger().info('좌석 정리 작업 수락됨. 결과 대기 중...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future: Future):
        result = future.result().result
        self.get_logger().info(f'작업 완료 결과: {result.message}')
        if self.task_done_future is not None and not self.task_done_future.done():
            self.task_done_future.set_result({'success': result.success, 'message': result.message})

    # --- 유틸 생성기 ---

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
    """독립 실행을 위한 main 함수."""
    rclpy.init(args=args)

    if len(sys.argv) < 3:
        print("Usage: ros2 run javis_rcs clean_seat <robot_namespace> <seat_id>")
        return

    robot_namespace = sys.argv[1]
    seat_id = int(sys.argv[2])

    node = CleanSeat(namespace=f'{robot_namespace}/main')
    try:
        node.get_logger().info(f"Starting clean_seat task for seat_id: {seat_id}")
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("CleanSeat 노드가 종료됩니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
