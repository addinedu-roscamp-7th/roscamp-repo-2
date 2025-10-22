'''LISTENING 세션 관리를 위한 데이터 구조와 유틸리티.'''

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Optional

from rclpy.duration import Duration
from rclpy.time import Time


@dataclass
class ListeningSession:
    '''Wake Word 이후 음성 세션을 관리한다.'''

    now_fn: Callable[[], Time]
    timeout_sec: float
    active: bool = False
    deadline: Optional[Time] = None
    pending_task: Optional[Dict[str, Any]] = None
    metadata: Dict[str, Any] = field(default_factory=dict)

    def start(self) -> None:
        '''LISTENING 세션을 시작한다.'''
        self.active = True
        self.pending_task = None
        self.metadata.clear()
        self._extend_deadline()

    def cancel(self) -> None:
        '''LISTENING 세션을 취소한다.'''
        self.active = False
        self.deadline = None
        self.pending_task = None
        self.metadata.clear()

    def handle_timeout(self, current_time: Optional[Time] = None) -> bool:
        '''타임아웃을 확인하고 만료 시 세션을 종료한다.'''
        if not self.active or self.deadline is None:
            return False

        now = current_time or self.now_fn()
        if now >= self.deadline:
            self.cancel()
            return True
        return False

    def update_deadline(self) -> None:
        '''세션 유지 시간을 연장한다.'''
        if not self.active:
            return
        self._extend_deadline()

    def store_task_payload(self, payload: Dict[str, Any]) -> None:
        '''Voice API 응답으로부터 작업 데이터를 보관한다.'''
        self.pending_task = payload

    def pop_task_payload(self) -> Optional[Dict[str, Any]]:
        '''저장된 작업 데이터를 반환하고 초기화한다.'''
        payload = self.pending_task
        self.pending_task = None
        return payload

    def _extend_deadline(self) -> None:
        '''현재 시각을 기준으로 타임아웃을 재설정한다.'''
        now = self.now_fn()
        self.deadline = now + Duration(seconds=float(self.timeout_sec))
