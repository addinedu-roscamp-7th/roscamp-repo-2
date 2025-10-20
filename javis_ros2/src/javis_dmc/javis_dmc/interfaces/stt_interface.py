'''JAVIS DMC의 음성 인식(STT) 인터페이스.'''

from abc import abstractmethod
from threading import Lock
from typing import Callable, Optional

from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String

from .base_interface import BaseInterface


class STTResult:
    '''음성 인식 결과를 표현하는 데이터 구조.'''
    def __init__(self, text: str, confidence: float, language: str):
        self.text = text
        self.confidence = confidence
        self.language = language


class STTInterface(BaseInterface):
    '''ROS 2 기반 음성 인식 구독을 담당하는 추상 인터페이스.'''

    @abstractmethod
    def subscribe_stt_result(self, callback: Callable[[STTResult], None]) -> bool:
        '''음성 인식 결과를 구독한다.'''

    @abstractmethod
    def get_latest_result(self) -> Optional[STTResult]:
        '''가장 최근의 음성 인식 결과를 반환한다.'''

    @abstractmethod
    def set_wake_word(self, wake_word: str) -> bool:
        '''웨이크 워드를 설정한다.'''

    @abstractmethod
    def start_listening(self) -> bool:
        '''지속적인 음성 인식을 시작한다.'''

    @abstractmethod
    def stop_listening(self) -> bool:
        '''지속적인 음성 인식을 중단한다.'''


class RosSTTInterface(STTInterface):
    '''ROS 토픽 기반 음성 인식 인터페이스 구현.'''

    def __init__(self, node, namespace: str = ''):
        super().__init__(node, namespace)
        self._cb_group = ReentrantCallbackGroup()
        self._result_callback: Optional[Callable[[STTResult], None]] = None
        self._latest_result: Optional[STTResult] = None
        self._result_lock = Lock()
        self._stt_sub = None
        self._listening = False
        self._wake_word = '도비야'

    def initialize(self) -> bool:
        '''STT 결과 토픽을 구독한다.'''
        self._stt_sub = self.node.create_subscription(
            String,
            self._create_topic_name('stt/result'),
            self._on_stt_result,
            10,
            callback_group=self._cb_group,
        )
        self._set_initialized(True)
        return True

    def subscribe_stt_result(self, callback: Callable[[STTResult], None]) -> bool:
        '''STT 결과 콜백을 설정한다.'''
        self._result_callback = callback
        return True

    def get_latest_result(self) -> Optional[STTResult]:
        '''최근 STT 결과를 반환한다.'''
        with self._result_lock:
            return self._latest_result

    def set_wake_word(self, wake_word: str) -> bool:
        '''웨이크 워드를 설정한다.'''
        self._wake_word = wake_word
        return True

    def start_listening(self) -> bool:
        '''지속적인 청취 상태를 활성화한다.'''
        self._listening = True
        return True

    def stop_listening(self) -> bool:
        '''지속적인 청취 상태를 비활성화한다.'''
        self._listening = False
        return True

    def shutdown(self) -> None:
        '''생성된 자원을 정리한다.'''
        if self._stt_sub is not None:
            self.node.destroy_subscription(self._stt_sub)
            self._stt_sub = None
        super().shutdown()

    def _on_stt_result(self, msg: String) -> None:
        '''STT 결과 메시지를 처리한다.'''
        if not self._listening:
            return
        confidence = 0.5
        text = msg.data
        if self._wake_word and text.startswith(self._wake_word):
            text = text[len(self._wake_word) :].strip()
            confidence = 0.8
        result = STTResult(text=text, confidence=confidence, language='ko-KR')
        with self._result_lock:
            self._latest_result = result
        if self._result_callback is not None:
            self._result_callback(result)
