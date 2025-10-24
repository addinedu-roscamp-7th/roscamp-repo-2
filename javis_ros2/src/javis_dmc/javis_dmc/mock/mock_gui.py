'''JAVIS DMC GUI 인터페이스 Mock 구현 모듈.'''

import time
from typing import Any, Callable, Dict, List
from ..interfaces.gui_interface import GUIInterface
from .mock_base import MockBase



class MockGUIInterface(GUIInterface, MockBase):
    '''테스트 GUI와 연동하기 위한 Mock GUI 인터페이스.'''

    def __init__(self, node, namespace: str = '') -> None:
        GUIInterface.__init__(self, node, namespace)
        MockBase.__init__(self, node, namespace)

        self._screen_callbacks: List[Callable[[Dict[str, Any]], None]] = []
        self._current_screen = 'main'
        self._screen_data: Dict[str, Any] = {}

    def initialize(self) -> bool:
        '''Mock GUI 인터페이스를 초기화한다.'''
        self.logger.info('Mock GUI interface initialized')
        self._set_initialized(True)
        return True

    def subscribe_screen_event(self, callback: Callable[[Dict[str, Any]], None]) -> bool:
        '''Test GUI의 화면 이벤트 콜백을 등록한다.'''
        self.logger.info('Mock subscribe_screen_event')
        self._screen_callbacks.append(callback)
        return True

    def simulate_screen_event(self, event_type: str, event_data: Dict[str, Any] = None) -> None:
        '''테스트를 위해 화면 이벤트를 모의 발생시킨다.'''
        event = {
            'event_type': event_type,
            'screen': self._current_screen,
            'data': dict(event_data or {}),
            'timestamp': time.time(),
        }

        for callback in self._screen_callbacks:
            try:
                callback(event)
            except Exception as exc:  # pragma: no cover - logging only
                self.logger.error(f'Error in screen event callback: {exc}')

    def get_current_screen_info(self) -> Dict[str, Any]:
        '''현재 Mock 화면 정보를 반환한다.'''
        return {
            'screen_type': self._current_screen,
            'data': self._screen_data.copy(),
        }

    def shutdown(self) -> None:
        '''Mock GUI 인터페이스 자원을 정리한다.'''
        self.cleanup()
        super().shutdown()
