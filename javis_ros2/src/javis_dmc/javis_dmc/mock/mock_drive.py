'''JAVIS DMC 주행 인터페이스 Mock 구현 모듈.'''

import threading
import time
from concurrent.futures import Future
from typing import Optional
from geometry_msgs.msg import Pose, Point, Quaternion
from types import SimpleNamespace
from ..interfaces.drive_interface import DriveInterface
from .mock_base import MockBase



class MockDriveInterface(DriveInterface, MockBase):
    '''테스트 시나리오용 주행 인터페이스 Mock 구현.'''

    def __init__(self, node, namespace: str = '') -> None:
        DriveInterface.__init__(self, node, namespace)
        MockBase.__init__(self, node, namespace)

        self._current_pose = Pose()
        self._current_pose.position = Point(x=0.0, y=0.0, z=0.0)
        self._current_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self._follow_mode_enabled = False
        self._patrol_active = False
        self._patrol_thread: Optional[threading.Thread] = None
        self._last_pose_log_time = 0.0

    def initialize(self) -> bool:
        '''Mock 주행 인터페이스를 초기화한다.'''
        self.start_topic_publishing('current_pose', rate_hz=10.0)
        self._set_initialized(True)
        return True

    def move_to_target(self, pose: Pose, location_name: str = '') -> Future:
        '''Mock 목적지 이동 동작을 수행한다.'''
        self.logger.info(
            f'Mock move_to_target: {location_name} at ({pose.position.x}, {pose.position.y})',
        )

        additional_data = {
            'final_pose': pose,
            'distance_traveled': 5.0,
            'location_name': location_name,
        }

        future = self.create_mock_future('move_to_target', additional_data)

        def update_pose_callback(fut):
            result = fut.result()
            if result.success:
                self._current_pose = pose

        future.add_done_callback(update_pose_callback)
        return future

    def start_patrol(self, route: dict) -> Future:
        '''Mock 순찰 동작을 시작한다.'''
        future: Future = Future()
        response = self.get_mock_response('start_patrol')

        def _run():
            if response.delay > 0:
                time.sleep(response.delay)
            if not self._patrol_active:
                result = SimpleNamespace(
                    success=False,
                    error_code='cancelled',
                    message='mock patrol cancelled',
                    completed_loops=0,
                )
                future.set_result(result)
                return
            if response.success:
                pause = float(route.get('loop_pause_sec', 0.0)) if isinstance(route, dict) else 0.1
                time.sleep(max(0.1, pause))
                result = SimpleNamespace(
                    success=True,
                    error_code='',
                    message='mock patrol complete',
                    completed_loops=1,
                )
                future.set_result(result)
            else:
                result = SimpleNamespace(
                    success=False,
                    error_code=response.error_code,
                    message='mock patrol failure',
                    completed_loops=0,
                )
                future.set_result(result)
            self._patrol_active = False

        self.logger.info('Mock patrol 시작')
        self._patrol_active = True
        self._patrol_thread = threading.Thread(target=_run, daemon=True)
        self._patrol_thread.start()
        return future

    def cancel_patrol(self, reason: str = 'user_request') -> bool:
        '''Mock 순찰을 취소한다.'''
        if not self._patrol_active:
            return False
        self.logger.info(f'Mock patrol 취소: {reason}')
        self._patrol_active = False
        return True

    def is_patrol_active(self) -> bool:
        '''Mock 순찰 진행 여부를 반환한다.'''
        return self._patrol_active

    def enable_follow_mode(self, tracking_topic: str, destination: Pose, follow_distance: float = 1.5) -> bool:
        '''Mock 추종 모드를 활성화한다.'''
        response = self.get_mock_response('enable_follow_mode')

        if response.delay > 0:
            time.sleep(response.delay)

        if response.success:
            self._follow_mode_enabled = True
            self.logger.info(
                f'Mock follow mode enabled: topic={tracking_topic}, distance={follow_distance}m',
            )
        else:
            self.logger.warn('Mock follow mode enable failed')

        return response.success

    def disable_follow_mode(self) -> bool:
        '''Mock 추종 모드를 비활성화한다.'''
        response = self.get_mock_response('disable_follow_mode')

        if response.delay > 0:
            time.sleep(response.delay)

        if response.success:
            self._follow_mode_enabled = False
            self.logger.info('Mock follow mode disabled')
        else:
            self.logger.warn('Mock follow mode disable failed')

        return response.success

    def stop(self, reason: str = 'emergency_stop') -> bool:
        '''Mock 긴급 정지 동작을 수행한다.'''
        response = self.get_mock_response('stop')

        if response.delay > 0:
            time.sleep(response.delay)

        if response.success:
            self.logger.info(f'Mock stop executed: reason={reason}')
        else:
            self.logger.warn('Mock emergency stop failed')

        return response.success

    def resume(self, reason: str = 'resume_navigation') -> bool:
        '''Mock 주행 재개 동작을 수행한다.'''
        response = self.get_mock_response('resume')

        if response.delay > 0:
            time.sleep(response.delay)

        if response.success:
            self.logger.info(f'Mock resume executed: reason={reason}')
        else:
            self.logger.warn('Mock resume failed')

        return response.success

    def rotate_in_place(self, angle: float) -> bool:
        '''Mock 제자리 회전 동작을 수행한다.'''
        self.logger.info(f'Mock rotate_in_place: {angle} degrees')
        response = self.get_mock_response('rotate_in_place')

        if response.delay > 0:
            time.sleep(response.delay)

        return response.success

    def get_current_pose(self) -> Optional[Pose]:
        '''Mock 현재 자세를 반환한다.'''
        return self._current_pose

    def _publish_topic_data(self, topic_name: str) -> None:
        '''Mock 토픽 데이터를 퍼블리시한다.'''
        if topic_name != 'current_pose':
            return

        now = time.time()
        if now - self._last_pose_log_time > 5.0:
            self.logger.debug(
                f'Mock current pose: ({self._current_pose.position.x:.2f}, {self._current_pose.position.y:.2f})',
            )
            self._last_pose_log_time = now

    def shutdown(self) -> None:
        '''Mock 주행 인터페이스 자원을 정리한다.'''
        self.cleanup()
        super().shutdown()
