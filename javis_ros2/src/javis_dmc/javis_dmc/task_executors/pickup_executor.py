from dataclasses import dataclass
from typing import Any, Callable, Optional

from javis_dmc.states.state_enums import SubState

from .base_executor import BaseExecutor


@dataclass
class PickupOutcome:
    '''도서 픽업 작업 결과를 표현한다.'''

    success: bool
    message: str
    total_distance: float = 0.0
    total_time: float = 0.0
    book_id: str = ''
    storage_id: int = 0


class PickupExecutor(BaseExecutor):
    '''도서 픽업 작업을 담당하는 실행자.'''

    def __init__(self) -> None:
        super().__init__('pickup_executor')
        self._runtime: Optional[Callable[[Any, Callable[[SubState], None], Callable[[float], None]], PickupOutcome]] = None

    def set_runtime(self, runtime: Callable[[Any, Callable[[SubState], None], Callable[[float], None]], PickupOutcome]) -> None:
        '''실제 도서 픽업 로직을 실행할 콜백을 등록한다.'''
        self._runtime = runtime

    def execute(self, goal: Optional[Any] = None) -> bool:
        '''도서 픽업 시나리오를 실행한다.'''
        if self.is_active():
            return False
        if self._runtime is None:
            self._set_outcome(
                PickupOutcome(
                    success=False,
                    message='도서 픽업 실행 콜백이 설정되지 않았습니다.',
                )
            )
            return False

        self._begin()
        try:
            outcome = self._runtime(goal, self._set_sub_state, self._publish_feedback)
        except Exception as exc:  # noqa: BLE001
            self._set_outcome(
                PickupOutcome(
                    success=False,
                    message=f'도서 픽업 처리 중 예외가 발생했습니다: {exc}',
                )
            )
            self._finish()
            self._set_sub_state(SubState.SUB_ERROR)
            self._publish_feedback(0.0)
            return True

        self._set_outcome(outcome)
        self._finish()
        self._set_sub_state(SubState.NONE)
        if outcome.success:
            self._publish_feedback(1.0)
        else:
            self._publish_feedback(0.0)
        return True

    def cancel(self) -> None:
        '''진행 중인 픽업 작업을 취소한다.'''
        if not self.is_active():
            return

        self._finish()
        self._set_sub_state(SubState.SUB_ERROR)
        self._publish_feedback(0.0)
        self._set_outcome(
            PickupOutcome(
                success=False,
                message='사용자 취소로 도서 픽업이 중단되었습니다.',
            )
        )
