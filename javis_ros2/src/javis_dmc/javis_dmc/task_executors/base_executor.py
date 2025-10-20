from abc import ABC, abstractmethod
import time
from typing import Callable, Optional

from javis_dmc.states.state_enums import SubState


class BaseExecutor(ABC):
    '''작업 실행자의 공통 동작을 정의하는 추상 클래스.'''

    def __init__(self, name: str) -> None:
        # 실행자 메타 정보
        self.name = name
        self._active = False
        self._sub_state_callback: Optional[Callable[[SubState], None]] = None
        self._feedback_callback: Optional[Callable[[float], None]] = None
        self._step_delay = 0.0

    def configure(
        self,
        sub_state_callback: Optional[Callable[[SubState], None]],
        feedback_callback: Optional[Callable[[float], None]],
    ) -> None:
        '''상태 및 피드백 콜백을 설정한다.'''
        self._sub_state_callback = sub_state_callback
        self._feedback_callback = feedback_callback

    @abstractmethod
    def execute(self, goal: Optional[dict] = None) -> bool:
        '''작업 실행을 시작한다.'''

    @abstractmethod
    def cancel(self) -> None:
        '''진행 중인 작업을 취소한다.'''

    def wait_for_result(self) -> bool:
        '''실행이 완료될 때까지 대기한다 (스켈레톤).'''
        return not self._active

    def is_active(self) -> bool:
        '''실행 중인지 여부를 반환한다.'''
        return self._active

    def _begin(self) -> None:
        '''내부 상태를 실행 중으로 설정한다.'''
        self._active = True

    def _finish(self) -> None:
        '''내부 상태를 완료로 설정한다.'''
        self._active = False

    def _set_sub_state(self, sub_state: SubState) -> None:
        '''서브 상태를 외부로 알린다.'''
        if self._sub_state_callback is not None:
            self._sub_state_callback(sub_state)

    def _publish_feedback(self, progress: float) -> None:
        '''작업 진행도를 외부로 알린다.'''
        clamped = max(0.0, min(1.0, progress))
        if self._feedback_callback is not None:
            self._feedback_callback(clamped)

    def set_step_delay(self, delay: float) -> None:
        '''단계 간 대기 시간을 설정한다.'''
        self._step_delay = max(0.0, delay)

    def _delay(self) -> None:
        '''설정된 시간만큼 지연한다.'''
        if self._step_delay > 0.0:
            time.sleep(self._step_delay)
