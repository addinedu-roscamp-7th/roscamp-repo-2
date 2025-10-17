'''JAVIS DMC의 LLM 연동 인터페이스.'''

from abc import abstractmethod
from typing import Any, Dict, Optional

from .base_interface import BaseInterface


class Intent:
    '''사용자 의도를 표현하는 데이터 구조.'''

    def __init__(self, action: str, parameters: Dict[str, Any], confidence: float):
        self.action = action  # 'book_voice', 'road_voice', 'drink_voice'
        self.parameters = parameters
        self.confidence = confidence


class LLMInterface(BaseInterface):
    '''LLM 통신(HTTP 기반)을 담당하는 추상 인터페이스.'''

    @abstractmethod
    def parse_book_query(self, text: str) -> Optional[Intent]:
        '''도서 관련 질의를 해석한다.'''

    @abstractmethod
    def parse_guide_query(self, text: str) -> Optional[Intent]:
        '''안내 관련 질의를 해석한다.'''

    @abstractmethod
    def parse_order_query(self, text: str) -> Optional[Intent]:
        '''주문 및 서비스 질의를 해석한다.'''

    @abstractmethod
    def set_session_id(self, session_id: str) -> bool:
        '''대화 세션 식별자를 설정한다.'''


class RosLLMInterface(LLMInterface):
    '''간단한 규칙 기반 LLM 인터페이스 구현.'''

    def __init__(self, node, namespace: str = ''):
        super().__init__(node, namespace)
        self._session_id: str = ''

    def initialize(self) -> bool:
        '''세션 관리를 초기화한다.'''
        self._session_id = ''
        self._set_initialized(True)
        return True

    def parse_book_query(self, text: str) -> Optional[Intent]:
        '''도서 관련 질의를 간단히 분석한다.'''
        if '책' not in text and '도서' not in text:
            return None
        parameters: Dict[str, Any] = {'keywords': text}
        return Intent(action='book_voice', parameters=parameters, confidence=0.6)

    def parse_guide_query(self, text: str) -> Optional[Intent]:
        '''안내 관련 질의를 간단히 분석한다.'''
        if '안내' not in text and '길' not in text:
            return None
        parameters: Dict[str, Any] = {'destination': text}
        return Intent(action='road_voice', parameters=parameters, confidence=0.5)

    def parse_order_query(self, text: str) -> Optional[Intent]:
        '''주문 관련 질의를 간단히 분석한다.'''
        if '음료' not in text and '주문' not in text:
            return None
        parameters: Dict[str, Any] = {'order': text}
        return Intent(action='drink_voice', parameters=parameters, confidence=0.5)

    def set_session_id(self, session_id: str) -> bool:
        '''대화 세션 식별자를 설정한다.'''
        self._session_id = session_id
        return True
