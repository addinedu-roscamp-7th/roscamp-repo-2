'''JAVIS DMC의 GUI 제어 인터페이스.'''

import json
from abc import abstractmethod
from typing import Any, Callable, Dict, Optional

from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String

from .base_interface import BaseInterface


class GUIInterface(BaseInterface):
    '''도비 GUI와 통신하는 추상 인터페이스.'''

    @abstractmethod
    def subscribe_screen_event(self, callback: Callable[[Dict[str, Any]], None]) -> bool:
        '''GUI 이벤트를 구독한다.'''


class RosGUIInterface(GUIInterface):
    '''ROS 토픽 기반 GUI 인터페이스 구현.'''

    def __init__(self, node, namespace: str = ''):
        super().__init__(node, namespace)
        self._cb_group = ReentrantCallbackGroup()
        self._event_sub = None
        self._event_callback: Optional[Callable[[Dict[str, Any]], None]] = None

    def initialize(self) -> bool:
        '''GUI 토픽을 초기화한다.'''
        self._event_sub = self.node.create_subscription(
            String,
            self._create_topic_name('gui/screen_event'),
            self._on_event,
            10,
            callback_group=self._cb_group,
        )
        self._set_initialized(True)
        return True

    def subscribe_screen_event(self, callback: Callable[[Dict[str, Any]], None]) -> bool:
        '''GUI 이벤트 콜백을 설정한다.'''
        self._event_callback = callback
        return True

    def shutdown(self) -> None:
        '''생성된 자원을 정리한다.'''
        if self._event_sub is not None:
            self.node.destroy_subscription(self._event_sub)
            self._event_sub = None
        super().shutdown()

    def _on_event(self, msg: String) -> None:
        '''GUI 이벤트를 전달한다.'''
        if self._event_callback is None:
            return
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.logger.error('GUI 이벤트 페이로드 파싱에 실패했습니다.')
            return
        self._event_callback(payload)
