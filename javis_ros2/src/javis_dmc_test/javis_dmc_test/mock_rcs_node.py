'''Mock RCS Node - TEST_GUI에서 작업 Goal을 DMC에 전송.'''

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, Pose2D

from javis_interfaces.action import (
    PickupBook,
    GuidePerson,
    ReshelvingBook,
    CleanSeat,
)
from javis_dmc_test_msgs.srv import SendTask


class MockRCSNode(Node):
    '''TEST_GUI에서 작업 Goal을 DMC에 전송하는 Mock RCS.'''

    def __init__(self):
        super().__init__('mock_rcs_node')

        self.declare_parameter('robot_namespace', 'dobby1')
        self.namespace = self.get_parameter('robot_namespace').value

        # Action Clients (DMC의 Action Server로 Goal 전송)
        self.pickup_client = ActionClient(
            self,
            PickupBook,
            f'/{self.namespace}/main/pickup_book',
        )

        self.guiding_client = ActionClient(
            self,
            GuidePerson,
            f'/{self.namespace}/main/guide_person',
        )

        self.reshelving_client = ActionClient(
            self,
            ReshelvingBook,
            f'/{self.namespace}/main/reshelving_book',
        )

        self.clean_seat_client = ActionClient(
            self,
            CleanSeat,
            f'/{self.namespace}/main/clean_seat',
        )

        # 제어 서비스 (TEST_GUI가 호출)
        self.create_service(
            SendTask,
            '/test/mock_rcs/send_task',
            self._on_send_task,
        )

        self.get_logger().info(f'Mock RCS 노드 초기화 완료 (namespace: {self.namespace})')

    def _on_send_task(self, request, response):
        '''TEST_GUI로부터 작업 전송 요청 처리.'''
        task_type = request.task_type.lower()

        self.get_logger().info(f'작업 전송 요청: {task_type}')

        if task_type == 'pickup':
            success = self.send_pickup_task()
        elif task_type == 'guiding':
            success = self.send_guiding_task()
        elif task_type == 'reshelving':
            success = self.send_reshelving_task()
        elif task_type == 'clean_seat':
            success = self.send_clean_seat_task()
        else:
            response.success = False
            response.message = f'Unknown task type: {task_type}'
            return response

        response.success = success
        response.message = f'{task_type} task sent' if success else f'Failed to send {task_type} task'
        return response

    def send_pickup_task(self, book_id: str = 'TEST_BOOK_001', storage_id: int = 1):
        '''도서 픽업 작업 전송.'''
        if not self.pickup_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('PickupBook 액션 서버가 준비되지 않았습니다')
            return False

        goal = PickupBook.Goal()
        goal.book_id = book_id
        goal.storage_id = storage_id

        # 서가 접근 위치 (2D)
        goal.shelf_approach_location = Pose2D()
        goal.shelf_approach_location.x = 3.0
        goal.shelf_approach_location.y = 2.0
        goal.shelf_approach_location.theta = 0.0

        # 책 집기 위치 (3D)
        goal.book_pick_pose = Pose()
        goal.book_pick_pose.position.x = 3.1
        goal.book_pick_pose.position.y = 2.0
        goal.book_pick_pose.position.z = 0.8
        goal.book_pick_pose.orientation.w = 1.0

        # 보관함 접근 위치 (2D)
        goal.storage_approach_location = Pose2D()
        goal.storage_approach_location.x = 0.5
        goal.storage_approach_location.y = 0.0
        goal.storage_approach_location.theta = 1.57

        # 보관함 슬롯 위치 (3D)
        goal.storage_slot_pose = Pose()
        goal.storage_slot_pose.position.x = 0.5
        goal.storage_slot_pose.position.y = 0.0
        goal.storage_slot_pose.position.z = 0.3
        goal.storage_slot_pose.orientation.w = 1.0

        self.get_logger().info(f'도서 픽업 작업 전송: book_id={book_id}, storage_id={storage_id}')
        future = self.pickup_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)
        return True

    def send_guiding_task(self, dest_x: float = 5.0, dest_y: float = 3.0):
        '''길 안내 작업 전송.'''
        if not self.guiding_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('GuidePerson 액션 서버가 준비되지 않았습니다')
            return False

        goal = GuidePerson.Goal()
        goal.dest_location = Pose2D()
        goal.dest_location.x = dest_x
        goal.dest_location.y = dest_y
        goal.dest_location.theta = 0.0

        self.get_logger().info(f'길 안내 작업 전송: dest=({dest_x}, {dest_y})')
        future = self.guiding_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)
        return True

    def send_reshelving_task(self, return_desk_id: int = 1):
        '''반납 정리 작업 전송.'''
        if not self.reshelving_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('ReshelvingBook 액션 서버가 준비되지 않았습니다')
            return False

        goal = ReshelvingBook.Goal()
        goal.return_desk_id = return_desk_id

        # 반납대 위치 (2D)
        goal.return_desk_location = Pose2D()
        goal.return_desk_location.x = 2.0
        goal.return_desk_location.y = 1.0
        goal.return_desk_location.theta = 0.0

        # 반납대 좌표 (3D)
        goal.return_desk_pose = Pose()
        goal.return_desk_pose.position.x = 2.0
        goal.return_desk_pose.position.y = 1.0
        goal.return_desk_pose.position.z = 0.7
        goal.return_desk_pose.orientation.w = 1.0

        self.get_logger().info(f'반납 정리 작업 전송: return_desk_id={return_desk_id}')
        future = self.reshelving_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)
        return True

    def send_clean_seat_task(self, seat_id: int = 1):
        '''좌석 청소 작업 전송.'''
        if not self.clean_seat_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('CleanSeat 액션 서버가 준비되지 않았습니다')
            return False

        goal = CleanSeat.Goal()
        goal.seat_id = seat_id

        # 좌석 위치 (2D)
        goal.return_desk_location = Pose2D()
        goal.return_desk_location.x = 4.0
        goal.return_desk_location.y = 2.0
        goal.return_desk_location.theta = 0.0

        # 좌석 좌표 (3D)
        goal.return_desk_pose = Pose()
        goal.return_desk_pose.position.x = 4.0
        goal.return_desk_pose.position.y = 2.0
        goal.return_desk_pose.position.z = 0.75
        goal.return_desk_pose.orientation.w = 1.0

        # 쓰레기통 위치 (2D)
        goal.bin_location = Pose2D()
        goal.bin_location.x = 1.0
        goal.bin_location.y = 0.0
        goal.bin_location.theta = 0.0

        # 쓰레기통 좌표 (3D)
        goal.bin_pose = Pose()
        goal.bin_pose.position.x = 1.0
        goal.bin_pose.position.y = 0.0
        goal.bin_pose.position.z = 0.5
        goal.bin_pose.orientation.w = 1.0

        self.get_logger().info(f'좌석 청소 작업 전송: seat_id={seat_id}')
        future = self.clean_seat_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)
        return True

    def _goal_response_callback(self, future):
        '''Goal 응답 콜백.'''
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('작업 Goal이 거부되었습니다')
            return

        self.get_logger().info('작업 Goal이 수락되었습니다')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        '''작업 결과 콜백.'''
        result = future.result().result

        # 결과 메시지 추출
        if hasattr(result, 'success'):
            success_msg = f'success={result.success}'
        else:
            success_msg = 'no success field'

        if hasattr(result, 'message'):
            msg_content = f', message={result.message}'
        else:
            msg_content = ''

        self.get_logger().info(f'작업 결과: {success_msg}{msg_content}')


def main(args=None):
    '''Mock RCS 노드 실행 엔트리 포인트.'''
    rclpy.init(args=args)
    node = MockRCSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('사용자 중단')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
