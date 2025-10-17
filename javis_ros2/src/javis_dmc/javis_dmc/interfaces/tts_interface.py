'''JAVIS DMC의 음성 합성(TTS) 인터페이스.'''

from abc import abstractmethod
from typing import Optional

from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String

from .base_interface import BaseInterface


class TTSInterface(BaseInterface):
    '''ROS 2 서비스를 활용한 음성 합성 추상 인터페이스.'''

    @abstractmethod
    def speak(self, text: str, language: str = 'ko-KR', speed: float = 1.0, wait: bool = True) -> bool:
        '''텍스트를 음성으로 합성해 재생한다.'''

    @abstractmethod
    def stop_speaking(self) -> bool:
        '''현재 재생 중인 음성을 중단한다.'''

    @abstractmethod
    def get_estimated_duration(self, text: str, language: str = 'ko-KR', speed: float = 1.0) -> Optional[float]:
        '''주어진 문장의 추정 발화 시간을 반환한다.'''


class RosTTSInterface(TTSInterface):
    '''ROS 토픽 기반 음성 합성 인터페이스 구현.''' 

    def __init__(self, node, namespace: str = ''):
        super().__init__(node, namespace)
        self._cb_group = ReentrantCallbackGroup()
        self._speak_pub = None
        self._stop_pub = None

    def initialize(self) -> bool:
        '''TTS 명령 토픽을 초기화한다.'''
        self._speak_pub = self.node.create_publisher(
            String,
            self._create_topic_name('tts/speak'),
            10,
        )
        self._stop_pub = self.node.create_publisher(
            String,
            self._create_topic_name('tts/stop'),
            10,
        )
        self._set_initialized(True)
        return True

    def speak(self, text: str, language: str = 'ko-KR', speed: float = 1.0, wait: bool = True) -> bool:
        '''TTS 재생 요청을 발행한다.'''
        del wait
        if self._speak_pub is None:
            return False
        payload = String()
        payload.data = f'{language}|{speed:.2f}|{text}'
        self._speak_pub.publish(payload)
        return True

    def stop_speaking(self) -> bool:
        '''TTS 중지 요청을 발행한다.'''
        if self._stop_pub is None:
            return False
        payload = String()
        payload.data = 'stop'
        self._stop_pub.publish(payload)
        return True

    def get_estimated_duration(self, text: str, language: str = 'ko-KR', speed: float = 1.0) -> Optional[float]:
        '''간단한 규칙으로 발화 시간을 추정한다.'''
        del language
        if speed <= 0.0:
            return None
        base_rate = 8.0  # 초당 음절 수 추정
        length = max(len(text), 1)
        duration = (length / base_rate) / speed
        return round(duration, 2)

    def shutdown(self) -> None:
        '''생성된 자원을 정리한다.'''
        if self._speak_pub is not None:
            self.node.destroy_publisher(self._speak_pub)
            self._speak_pub = None
        if self._stop_pub is not None:
            self.node.destroy_publisher(self._stop_pub)
            self._stop_pub = None
        super().shutdown()
