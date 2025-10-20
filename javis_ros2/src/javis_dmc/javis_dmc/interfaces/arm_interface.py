'''JAVIS DMC의 로봇 팔 제어 인터페이스.'''

from abc import abstractmethod
from concurrent.futures import Future
from types import SimpleNamespace
from typing import List, Optional
from geometry_msgs.msg import Pose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from javis_interfaces.action import (
    CollectReturnedBooks,
    CollectTrash,
    DisposeTrash,
    PickBook,
    PlaceBook,
)
from javis_interfaces.srv import ChangeArmPose

from .base_interface import BaseInterface


class ArmInterface(BaseInterface):
    '''DAC와 연동되는 로봇 팔 제어 추상 인터페이스.'''

    @abstractmethod
    def pick_book(self, book_id: str, book_pose: Pose, carrier_slot_id: int) -> Future:
        '''도서를 집어 내부 캐리어에 적재한다.'''

    @abstractmethod
    def place_book(
        self,
        book_id: str,
        carrier_slot_id: int,
        target_pose: Pose,
        target_id: int,
        target_type: int,
    ) -> Future:
        '''캐리어에 있는 도서를 목표 위치로 이동시킨다.'''

    @abstractmethod
    def collect_books(self, book_ids: List[str], book_poses: List[Pose]) -> Future:
        '''여러 도서를 순차적으로 수거한다.'''

    @abstractmethod
    def collect_trash(self, trash_pose: Pose) -> Future:
        '''지정된 위치의 쓰레기를 수거한다.'''

    @abstractmethod
    def dispose_trash(self, bin_pose: Pose) -> Future:
        '''수거한 쓰레기를 쓰레기통에 배출한다.'''

    @abstractmethod
    def change_pose(self, pose_type: str) -> bool:
        '''사전 정의된 팔 자세로 전환한다.'''


class RosArmInterface(ArmInterface):
    '''ROS 2 액션/서비스 기반 로봇 팔 인터페이스 구현.'''

    _POSE_TYPE_MAP = {
        'observe': 0,
        'home': 1,
        'custom': 2,
        'carry': 1,
        'ready': 0,
    }

    def __init__(self, node, namespace: str = ''):
        super().__init__(node, namespace)
        self._cb_group = ReentrantCallbackGroup()
        self._pick_client: Optional[ActionClient] = None
        self._place_client: Optional[ActionClient] = None
        self._collect_client: Optional[ActionClient] = None
        self._collect_trash_client: Optional[ActionClient] = None
        self._dispose_trash_client: Optional[ActionClient] = None
        self._pose_client = None

    def initialize(self) -> bool:
        '''ROS 2 액션/서비스 클라이언트를 초기화한다.'''
        self._pick_client = self._create_action_client(PickBook, 'arm/pick_book')
        self._place_client = self._create_action_client(PlaceBook, 'arm/place_book')
        self._collect_client = self._create_action_client(CollectReturnedBooks, 'arm/collect_returned_books')
        self._collect_trash_client = self._create_action_client(CollectTrash, 'arm/collect_trash')
        self._dispose_trash_client = self._create_action_client(DisposeTrash, 'arm/dispose_trash')
        self._pose_client = self.node.create_client(
            ChangeArmPose,
            self._create_topic_name('arm/change_pose'),
            callback_group=self._cb_group,
        )

        for name, client in [
            ('arm/pick_book', self._pick_client),
            ('arm/place_book', self._place_client),
            ('arm/collect_returned_books', self._collect_client),
            ('arm/collect_trash', self._collect_trash_client),
            ('arm/dispose_trash', self._dispose_trash_client),
        ]:
            if client and not client.wait_for_server(timeout_sec=1.0):
                self.logger.warn(f'{name} 액션 서버가 아직 준비되지 않았습니다.')

        if not self._pose_client.wait_for_service(timeout_sec=1.0):
            self.logger.warn('arm/change_pose 서비스가 아직 준비되지 않았습니다.')

        self._set_initialized(True)
        return True

    def pick_book(self, book_id: str, book_pose: Pose, carrier_slot_id: int) -> Future:
        '''도서를 집어 내부 캐리어에 적재한다.'''
        goal = PickBook.Goal()
        goal.book_id = book_id
        goal.carrier_slot_id = carrier_slot_id
        goal.book_pose = book_pose
        return self._send_goal(self._pick_client, goal)

    def place_book(
        self,
        book_id: str,
        carrier_slot_id: int,
        target_pose: Pose,
        target_id: int,
        target_type: int,
    ) -> Future:
        '''캐리어에 있는 도서를 목표 위치로 이동시킨다.'''
        goal = PlaceBook.Goal()
        goal.book_id = book_id
        goal.carrier_slot_id = carrier_slot_id
        if target_type == 0:
            goal.storage_box_id = target_id
        else:
            goal.storage_box_id = target_id
        goal.storage_box_pose = target_pose
        return self._send_goal(self._place_client, goal)

    def collect_books(self, book_ids: List[str], book_poses: List[Pose]) -> Future:
        '''여러 도서를 순차적으로 수거한다.'''
        goal = CollectReturnedBooks.Goal()
        goal.book_ids = book_ids
        goal.carrier_slot_ids = list(range(len(book_ids)))
        goal.book_poses = book_poses
        return self._send_goal(self._collect_client, goal)

    def collect_trash(self, trash_pose: Pose) -> Future:
        '''지정된 위치의 쓰레기를 수거한다.'''
        goal = CollectTrash.Goal()
        goal.trash_pose = trash_pose
        goal.trash_type = 'general'
        return self._send_goal(self._collect_trash_client, goal)

    def dispose_trash(self, bin_pose: Pose) -> Future:
        '''수거한 쓰레기를 쓰레기통에 배출한다.'''
        goal = DisposeTrash.Goal()
        goal.trash_bin_pose = bin_pose
        return self._send_goal(self._dispose_trash_client, goal)

    def change_pose(self, pose_type: str) -> bool:
        '''사전 정의된 팔 자세로 전환한다.'''
        if self._pose_client is None:
            return False
        pose_value = self._POSE_TYPE_MAP.get(pose_type, 1)
        request = ChangeArmPose.Request()
        request.pose_type = pose_value
        request.target_pose = Pose()
        self._pose_client.call_async(request)
        return True

    def shutdown(self) -> None:
        '''생성된 자원을 정리한다.'''
        for client in (
            self._pick_client,
            self._place_client,
            self._collect_client,
            self._collect_trash_client,
            self._dispose_trash_client,
        ):
            if client is not None:
                client.destroy()
        if self._pose_client is not None:
            self._pose_client.destroy()
        super().shutdown()

    def _create_action_client(self, action_type, suffix: str) -> ActionClient:
        '''액션 클라이언트를 생성한다.'''
        return ActionClient(
            self.node,
            action_type,
            self._create_topic_name(suffix),
            callback_group=self._cb_group,
        )

    def _send_goal(self, client, goal) -> Future:
        '''액션 Goal을 전송한다.'''
        result_future: Future = Future()

        if client is None:
            result_future.set_result(SimpleNamespace(success=False))
            return result_future

        if not client.wait_for_server(timeout_sec=0.5):
            message = '액션 서버가 준비되지 않았습니다.'
            self.logger.error(message)
            result_future.set_result(SimpleNamespace(success=False, message=message))
            return result_future

        goal_future = client.send_goal_async(goal)

        def _goal_response(fut):
            goal_handle = fut.result()
            if not goal_handle.accepted:
                message = '액션 Goal이 거절되었습니다.'
                self.logger.warn(message)
                result_future.set_result(SimpleNamespace(success=False, message=message))
                return

            result = goal_handle.get_result_async()

            def _result_ready(res_future):
                try:
                    result_msg = res_future.result().result
                except Exception as exc:
                    result_future.set_exception(exc)
                    return
                result_future.set_result(result_msg)

            result.add_done_callback(_result_ready)

        goal_future.add_done_callback(_goal_response)
        return result_future
