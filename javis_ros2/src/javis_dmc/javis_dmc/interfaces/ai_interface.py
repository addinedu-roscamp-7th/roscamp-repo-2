'''JAVIS DMC의 AI 인식 인터페이스.'''

from abc import abstractmethod
from typing import Any, Callable, Dict, List, Optional

import rclpy
from geometry_msgs.msg import Pose
from rclpy.callback_groups import ReentrantCallbackGroup

from javis_interfaces.msg import TrackingStatus
from javis_interfaces.srv import (
    ChangeTrackingMode,
    CheckStorageBox,
    DetectBook,
    DetectTrash,
    IdentifyBookShelf,
    VerifyBookPosition,
)

from .base_interface import BaseInterface


class BookPose:
    '''도서 인식 결과를 표현하는 데이터 구조.'''

    def __init__(self, book_id: int, pose: Pose, confidence: float):
        self.book_id = book_id
        self.pose = pose
        self.confidence = confidence


class BoxStatus:
    '''보관함 상태 정보를 표현하는 데이터 구조.'''

    def __init__(self, box_id: int, is_empty: bool, book_count: int):
        self.box_id = box_id
        self.is_empty = is_empty
        self.book_count = book_count


class ShelfInfo:
    '''책장 정보를 표현하는 데이터 구조.'''

    def __init__(self, shelf_id: str, pose: Pose, capacity: int, current_books: int):
        self.shelf_id = shelf_id
        self.pose = pose
        self.capacity = capacity
        self.current_books = current_books


class TrashInfo:
    '''쓰레기 탐지 정보를 표현하는 데이터 구조.'''

    def __init__(self, trash_type: str, pose: Pose, confidence: float):
        self.trash_type = trash_type
        self.pose = pose
        self.confidence = confidence


class AIInterface(BaseInterface):
    '''AIS와 통신하는 AI 비전 추상 인터페이스.'''

    @abstractmethod
    def detect_book(self, book_id: str) -> Optional[BookPose]:
        '''특정 도서를 탐지하고 위치를 반환한다.'''

    @abstractmethod
    def check_storage_box(self, box_id: int) -> BoxStatus:
        '''보관함 상태를 조회한다.'''

    @abstractmethod
    def verify_book_position(self, book_id: str, expected_pose: Pose) -> bool:
        '''도서가 예상 위치에 있는지 검증한다.'''

    @abstractmethod
    def identify_bookshelf(self, shelf_area: Pose) -> Optional[ShelfInfo]:
        '''책장을 식별하고 정보를 분석한다.'''

    @abstractmethod
    def detect_trash(self, seat_pose: Pose) -> List[TrashInfo]:
        '''좌석 주변의 쓰레기를 탐지한다.'''

    @abstractmethod
    def change_tracking_mode(self, mode: str) -> bool:
        '''추적 모드를 변경한다.'''

    @abstractmethod
    def subscribe_tracking_status(self, callback: Callable[[Dict[str, Any]], None]) -> bool:
        '''추적 상태 업데이트를 구독한다.'''


class RosAIInterface(AIInterface):
    '''ROS 2 서비스/토픽 기반 AI 인터페이스 구현.'''

    def __init__(self, node, namespace: str = ''):
        super().__init__(node, namespace)
        self._cb_group = ReentrantCallbackGroup()
        self._detect_book_client = None
        self._check_storage_client = None
        self._verify_book_client = None
        self._identify_shelf_client = None
        self._detect_trash_client = None
        self._change_mode_client = None
        self._tracking_sub = None
        self._tracking_callback: Optional[Callable[[Dict[str, Any]], None]] = None
        self._service_timeout = 2.0

    def initialize(self) -> bool:
        '''AI 관련 서비스 및 토픽을 초기화한다.'''
        self._detect_book_client = self._create_client(DetectBook, 'ai/detect_book')
        self._check_storage_client = self._create_client(CheckStorageBox, 'ai/check_storage_box')
        self._verify_book_client = self._create_client(VerifyBookPosition, 'ai/verify_book_position')
        self._identify_shelf_client = self._create_client(IdentifyBookShelf, 'ai/identify_bookshelf')
        self._detect_trash_client = self._create_client(DetectTrash, 'ai/detect_trash')
        self._change_mode_client = self._create_client(ChangeTrackingMode, 'ai/change_tracking_mode')

        self._tracking_sub = self.node.create_subscription(
            TrackingStatus,
            self._create_topic_name('ai/tracking/status'),
            self._on_tracking_status,
            10,
            callback_group=self._cb_group,
        )

        for name, client in [
            ('ai/detect_book', self._detect_book_client),
            ('ai/check_storage_box', self._check_storage_client),
            ('ai/verify_book_position', self._verify_book_client),
            ('ai/identify_bookshelf', self._identify_shelf_client),
            ('ai/detect_trash', self._detect_trash_client),
            ('ai/change_tracking_mode', self._change_mode_client),
        ]:
            self._wait_for_service(client, name)

        self._set_initialized(True)
        return True

    def detect_book(self, book_id: str) -> Optional[BookPose]:
        '''특정 도서를 탐지한다.'''
        if self._detect_book_client is None:
            return None
        request = DetectBook.Request()
        request.book_id = book_id
        response = self._call_service(self._detect_book_client, request)
        if response is None or not response.detected:
            return None
        return BookPose(book_id=response.book_id, pose=response.book_pose, confidence=response.confidence)

    def check_storage_box(self, box_id: int) -> BoxStatus:
        '''보관함 상태를 조회한다.'''
        if self._check_storage_client is None:
            return BoxStatus(box_id, False, 0)
        request = CheckStorageBox.Request()
        request.storage_box_id = box_id
        response = self._call_service(self._check_storage_client, request)
        if response is None:
            return BoxStatus(box_id, False, 0)
        return BoxStatus(box_id=response.storage_box_id, is_empty=response.is_empty, book_count=0)

    def verify_book_position(self, book_id: str, expected_pose: Pose) -> bool:
        '''도서 위치를 검증한다.'''
        del expected_pose
        if self._verify_book_client is None:
            return False
        request = VerifyBookPosition.Request()
        request.book_id = book_id
        response = self._call_service(self._verify_book_client, request)
        if response is None:
            return False
        return response.is_correct_position

    def identify_bookshelf(self, shelf_area: Pose) -> Optional[ShelfInfo]:
        '''책장을 식별한다.'''
        del shelf_area
        if self._identify_shelf_client is None:
            return None
        request = IdentifyBookShelf.Request()
        response = self._call_service(self._identify_shelf_client, request)
        if response is None or not response.detected:
            return None
        return ShelfInfo(
            shelf_id=response.bookshelf_id,
            pose=response.bookshelf_pose,
            capacity=len(response.available_slots),
            current_books=0,
        )

    def detect_trash(self, seat_pose: Pose) -> List[TrashInfo]:
        '''좌석 주변 쓰레기를 탐지한다.'''
        if self._detect_trash_client is None:
            return []
        request = DetectTrash.Request()
        request.seat_location = seat_pose
        response = self._call_service(self._detect_trash_client, request)
        if response is None or not response.trash_found:
            return []
        infos: List[TrashInfo] = []
        for trash_type, trash_pose, confidence in zip(
            response.trash_types,
            response.trash_poses,
            response.confidence_scores,
        ):
            infos.append(TrashInfo(trash_type=trash_type, pose=trash_pose, confidence=confidence))
        return infos

    def change_tracking_mode(self, mode: str) -> bool:
        '''추적 모드를 변경한다.'''
        if self._change_mode_client is None:
            return False
        request = ChangeTrackingMode.Request()
        mode_map = {
            'registration': ChangeTrackingMode.Request.REGISTRATION_MODE,
            'tracking': ChangeTrackingMode.Request.TRACKING_MODE,
            'idle': ChangeTrackingMode.Request.IDLE_MODE,
        }
        request.mode = mode_map.get(mode, ChangeTrackingMode.Request.IDLE_MODE)
        response = self._call_service(self._change_mode_client, request)
        if response is None:
            return False
        return response.success

    def subscribe_tracking_status(self, callback: Callable[[Dict[str, Any]], None]) -> bool:
        '''추적 상태 업데이트 콜백을 등록한다.'''
        self._tracking_callback = callback
        return True

    def shutdown(self) -> None:
        '''생성된 자원을 정리한다.'''
        for client in (
            self._detect_book_client,
            self._check_storage_client,
            self._verify_book_client,
            self._identify_shelf_client,
            self._detect_trash_client,
            self._change_mode_client,
        ):
            if client is not None:
                client.destroy()
        if self._tracking_sub is not None:
            self.node.destroy_subscription(self._tracking_sub)
            self._tracking_sub = None
        super().shutdown()

    def _create_client(self, srv_type, suffix: str):
        '''서비스 클라이언트를 생성한다.'''
        return self.node.create_client(
            srv_type,
            self._create_topic_name(suffix),
            callback_group=self._cb_group,
        )

    def _wait_for_service(self, client, name: str) -> None:
        '''서비스 준비 상태를 확인한다.'''
        if client is None:
            return
        if not client.wait_for_service(timeout_sec=1.0):
            self.logger.warn(f'{name} 서비스가 아직 준비되지 않았습니다.')

    def _call_service(self, client, request):
        '''서비스를 호출하고 결과를 반환한다.'''
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=self._service_timeout)
        if not future.done():
            self.logger.error('서비스 응답이 시간 안에 도착하지 않았습니다.')
            return None
        try:
            return future.result()
        except Exception as exc:
            self.logger.error(f'서비스 호출 실패: {exc}')
            return None

    def _on_tracking_status(self, msg: TrackingStatus) -> None:
        '''추적 상태 메시지를 전달한다.'''
        if self._tracking_callback is None:
            return
        payload = {
            'person_detected': msg.person_detected,
            'person_pose': msg.person_pose,
            'distance': msg.distance_to_person,
            'confidence': msg.confidence,
            'is_lost': msg.is_lost,
            'time_since_last_seen': msg.time_since_last_seen,
        }
        self._tracking_callback(payload)
