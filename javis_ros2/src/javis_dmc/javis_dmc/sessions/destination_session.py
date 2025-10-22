'''GUIDING 목적지 선택 세션을 관리한다.'''

from dataclasses import dataclass, field
from typing import Callable, Optional

from rclpy.duration import Duration
from rclpy.time import Time


@dataclass
class DestinationSession:
    '''GUI 목적지 선택 흐름을 관리하는 세션.'''

    now_fn: Callable[[], Time]
    timeout_sec: float
    active: bool = False
    deadline: Optional[Time] = None
    selected_destination: Optional[str] = None
    timeout_reason: Optional[str] = None
    metadata: dict = field(default_factory=dict)

    def begin_selection(self) -> None:
        '''목적지 선택을 시작하고 타이머를 설정한다.'''
        self.active = True
        self.selected_destination = None
        self.timeout_reason = None
        self.metadata.clear()
        self._extend_deadline()

    def resolve_selection(self, destination: str) -> None:
        '''사용자 입력을 반영하고 세션을 종료한다.'''
        if not self.active:
            return
        self.selected_destination = destination
        self.active = False
        self.deadline = None

    def abort_timeout(self) -> None:
        '''타임아웃으로 세션을 종료한다.'''
        self.active = False
        self.timeout_reason = 'destination_timeout'
        self.deadline = None

    def check_timeout(self, current_time: Optional[Time] = None) -> bool:
        '''현재 시각 기준으로 타임아웃 여부를 확인한다.'''
        if not self.active or self.deadline is None:
            return False

        now = current_time or self.now_fn()
        if now >= self.deadline:
            self.abort_timeout()
            return True
        return False

    def clear(self) -> None:
        '''세션 상태를 초기화한다.'''
        self.active = False
        self.deadline = None
        self.selected_destination = None
        self.timeout_reason = None
        self.metadata.clear()

    def _extend_deadline(self) -> None:
        '''타임아웃을 갱신한다.'''
        now = self.now_fn()
        self.deadline = now + Duration(seconds=float(self.timeout_sec))
