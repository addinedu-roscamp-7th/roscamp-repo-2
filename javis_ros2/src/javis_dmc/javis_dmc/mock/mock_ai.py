'''JAVIS DMC AI 인터페이스 Mock 구현 모듈.'''

import random
import time
from typing import Any, Callable, Dict, List, Optional
from geometry_msgs.msg import Pose, Point, Quaternion
from ..interfaces.ai_interface import AIInterface, BookPose, BoxStatus, ShelfInfo, TrashInfo
from .mock_base import MockBase



class MockAIInterface(AIInterface, MockBase):
    '''테스트 시나리오에서 사용할 AI 인터페이스 Mock 구현.'''

    def __init__(self, node, namespace: str = '') -> None:
        AIInterface.__init__(self, node, namespace)
        MockBase.__init__(self, node, namespace)

        self._tracking_mode = 'disabled'
        self._person_detected = False
        self._tracking_callbacks: List[Callable[[Dict[str, Any]], None]] = []

        self._mock_books: Dict[str, BookPose] = {
            'ISBN-123': BookPose('ISBN-123', self._create_mock_pose(1.0, 2.0), 0.95),
            'ISBN-456': BookPose('ISBN-456', self._create_mock_pose(2.0, 3.0), 0.88),
        }

    def initialize(self) -> bool:
        '''Mock AI 인터페이스를 초기화한다.'''
        self.start_topic_publishing('tracking_status', rate_hz=10.0)
        self.logger.info('Mock AI interface initialized')
        self._set_initialized(True)
        return True

    def detect_book(self, book_id: str) -> Optional[BookPose]:
        '''Mock 도서 검출 동작을 수행한다.'''
        self.logger.info(f'Mock detect_book: {book_id}')
        response = self.get_mock_response('detect_book')

        if response.delay > 0:
            time.sleep(response.delay)

        if response.success and book_id in self._mock_books:
            detected_book = self._mock_books[book_id]
            detected_book.confidence = max(0.1, detected_book.confidence + random.uniform(-0.1, 0.1))
            self.logger.info(
                f'Mock detected book {book_id} with confidence {detected_book.confidence:.2f}',
            )
            return detected_book

        self.logger.info(f'Mock book {book_id} not detected')
        return None

    def check_storage_box(self, box_id: int) -> BoxStatus:
        '''Mock 수거함 상태를 조회한다.'''
        self.logger.info(f'Mock check_storage_box: {box_id}')
        response = self.get_mock_response('check_storage_box')

        if response.delay > 0:
            time.sleep(response.delay)

        is_empty = random.choice([True, False]) if response.success else True
        book_count = 0 if is_empty else random.randint(1, 5)

        status = BoxStatus(box_id, is_empty, book_count)
        self.logger.info(f'Mock box {box_id}: empty={is_empty}, books={book_count}')
        return status

    def verify_book_position(self, book_id: str, expected_pose: Pose) -> bool:
        '''Mock 도서 위치 검증을 수행한다.'''
        self.logger.info(f'Mock verify_book_position: {book_id}')
        response = self.get_mock_response('verify_book_position')

        if response.delay > 0:
            time.sleep(response.delay)

        verified = response.success and random.random() > 0.2
        self.logger.info(f'Mock book {book_id} verification: {verified}')
        return verified

    def identify_bookshelf(self, shelf_area: Pose) -> Optional[ShelfInfo]:
        '''Mock 서가 식별 동작을 수행한다.'''
        self.logger.info(
            f'Mock identify_bookshelf at ({shelf_area.position.x:.2f}, {shelf_area.position.y:.2f})',
        )
        response = self.get_mock_response('identify_bookshelf')

        if response.delay > 0:
            time.sleep(response.delay)

        if not response.success:
            return None

        shelf_info = ShelfInfo(
            shelf_id=f'SHELF_{int(shelf_area.position.x)}_{int(shelf_area.position.y)}',
            pose=shelf_area,
            capacity=20,
            current_books=random.randint(5, 18),
        )
        self.logger.info(f'Mock identified shelf {shelf_info.shelf_id}')
        return shelf_info

    def detect_trash(self, seat_pose: Pose) -> List[TrashInfo]:
        '''Mock 좌석 쓰레기 검출을 수행한다.'''
        self.logger.info(
            f'Mock detect_trash at seat ({seat_pose.position.x:.2f}, {seat_pose.position.y:.2f})',
        )
        response = self.get_mock_response('detect_trash')

        if response.delay > 0:
            time.sleep(response.delay)

        trash_list: List[TrashInfo] = []
        if response.success:
            trash_count = random.randint(0, 3)
            trash_types = ['paper', 'bottle', 'food_wrapper', 'cup']
            for _ in range(trash_count):
                trash_pose = self._create_mock_pose(
                    seat_pose.position.x + random.uniform(-0.5, 0.5),
                    seat_pose.position.y + random.uniform(-0.5, 0.5),
                )
                trash = TrashInfo(
                    trash_type=random.choice(trash_types),
                    pose=trash_pose,
                    confidence=random.uniform(0.7, 0.95),
                )
                trash_list.append(trash)

        self.logger.info(f'Mock detected {len(trash_list)} trash items')
        return trash_list

    def change_tracking_mode(self, mode: str) -> bool:
        '''Mock 추적 모드 변경을 수행한다.'''
        self.logger.info(f'Mock change_tracking_mode: {mode}')
        response = self.get_mock_response('change_tracking_mode')

        if response.delay > 0:
            time.sleep(response.delay)

        if response.success:
            self._tracking_mode = mode
            if mode in ('disabled', 'idle'):
                self._person_detected = False
            elif mode in ('registration', 'follow', 'tracking'):
                self._person_detected = True

        return response.success

    def subscribe_tracking_status(self, callback: Callable[[Dict[str, Any]], None]) -> bool:
        '''Mock 추적 상태 콜백을 등록한다.'''
        self.logger.info('Mock subscribe_tracking_status')
        self._tracking_callbacks.append(callback)
        return True

    def _create_mock_pose(self, x: float, y: float, z: float = 0.0) -> Pose:
        '''Mock Pose 데이터를 생성한다.'''
        pose = Pose()
        pose.position = Point(x=x, y=y, z=z)
        pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        return pose

    def _publish_topic_data(self, topic_name: str) -> None:
        '''Mock 토픽 데이터를 퍼블리시한다.'''
        if topic_name != 'tracking_status':
            return

        if random.random() < 0.1:
            self._person_detected = not self._person_detected

        tracking_status = {
            'tracking_id': self._tracking_id,
            'person_detected': self._person_detected,
            'confidence': random.uniform(0.7, 0.95) if self._person_detected else 0.0,
            'position': {
                'x': random.uniform(-2.0, 2.0),
                'y': random.uniform(-2.0, 2.0),
            } if self._person_detected else None,
            'mode': self._tracking_mode,
        }

        for callback in self._tracking_callbacks:
            try:
                callback(tracking_status)
            except Exception as exc:  # pragma: no cover - logging only
                self.logger.error(f'Error in tracking callback: {exc}')
