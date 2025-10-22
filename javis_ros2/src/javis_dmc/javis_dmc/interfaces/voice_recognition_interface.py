'''voice_recognition_controller와의 연동 인터페이스.'''

from dataclasses import dataclass
from threading import Lock
from typing import Callable, Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from std_msgs.msg import String
from std_srvs.srv import SetBool

from .base_interface import BaseInterface


@dataclass
class DialogEvent:
    '''음성 인식 컨트롤러에서 수신한 대화 이벤트.'''

    text: str
    confidence: float
    raw_text: str

    @property
    def is_empty(self) -> bool:
        '''내용이 비어 있는지 여부.'''
        return not self.text.strip()


class VoiceRecognitionInterface(BaseInterface):
    '''음성 인식 컨트롤러와의 공통 동작을 정의한다.'''

    def register_dialog_callback(self, callback: Callable[[DialogEvent], None]) -> None:
        '''대화 이벤트 콜백을 등록한다.'''
        raise NotImplementedError

    def set_listening_mode(self, enabled: bool) -> bool:
        '''LISTENING 모드 전환을 요청한다.'''
        raise NotImplementedError

    def stop(self) -> None:
        '''음성 스트림을 종료한다.'''
        raise NotImplementedError


class RosVoiceRecognitionInterface(VoiceRecognitionInterface):
    '''ROS 2 기반 음성 인식 인터페이스 구현.'''

    def __init__(self, node, namespace: str = ''):
        super().__init__(node, namespace)
        self._cb_group = ReentrantCallbackGroup()
        self._subscription = None
        self._service_client = None
        self._dialog_callback: Optional[Callable[[DialogEvent], None]] = None
        self._latest_event: Optional[DialogEvent] = None
        self._event_lock = Lock()

    def initialize(self) -> bool:
        '''토픽 및 서비스를 초기화한다.'''
        topic = self._create_topic_name('voice_recognition_controller/stt_result')
        service = self._create_topic_name('voice_recognition_controller/set_listening_mode')

        self._subscription = self.node.create_subscription(
            String,
            topic,
            self._on_dialog_event,
            10,
            callback_group=self._cb_group,
        )

        self._service_client = self.node.create_client(
            SetBool,
            service,
            callback_group=self._cb_group,
        )

        if not self._service_client.wait_for_service(timeout_sec=2.0):
            self.logger.warn('set_listening_mode 서비스가 준비되지 않았습니다.')

        self._set_initialized(True)
        return True

    def shutdown(self) -> None:
        '''구독 및 서비스를 정리한다.'''
        if self._subscription is not None:
            self.node.destroy_subscription(self._subscription)
            self._subscription = None

        if self._service_client is not None:
            self.node.destroy_client(self._service_client)
            self._service_client = None

        super().shutdown()

    def register_dialog_callback(self, callback: Callable[[DialogEvent], None]) -> None:
        '''대화 이벤트 콜백을 등록한다.'''
        self._dialog_callback = callback

    def set_listening_mode(self, enabled: bool) -> bool:
        '''LISTENING 모드 토글을 요청한다.'''
        if self._service_client is None:
            self.logger.error('set_listening_mode 서비스 클라이언트가 초기화되지 않았습니다.')
            return False

        request = SetBool.Request()
        request.data = bool(enabled)
        future = self._service_client.call_async(request)

        rclpy.spin_until_future_complete(self.node, future)

        if not isinstance(future, Future) or future.result() is None:
            self.logger.error('set_listening_mode 서비스 호출에 실패했습니다.')
            return False

        response = future.result()
        if not response.success:
            self.logger.warn(f'set_listening_mode 실패: {response.message}')
            return False

        return True

    def stop(self) -> None:
        '''LISTENING 모드를 빠르게 종료한다.'''
        self.set_listening_mode(False)

    def get_latest_event(self) -> Optional[DialogEvent]:
        '''가장 최근 이벤트를 반환한다.'''
        with self._event_lock:
            return self._latest_event

    def _on_dialog_event(self, msg: String) -> None:
        '''STT 결과 콜백을 처리한다.'''
        event = DialogEvent(
            text=msg.data.strip(),
            confidence=0.6,
            raw_text=msg.data,
        )

        with self._event_lock:
            self._latest_event = event

        if self._dialog_callback is not None:
            self._dialog_callback(event)
