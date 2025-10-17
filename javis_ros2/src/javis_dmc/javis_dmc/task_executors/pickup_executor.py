from typing import Optional

from javis_dmc.states.state_enums import SubState

from .base_executor import BaseExecutor


class PickupExecutor(BaseExecutor):
    '''도서 픽업 작업을 담당하는 실행자.'''

    def __init__(self) -> None:
        super().__init__('pickup_executor')
        self._sequence = [
            SubState.MOVE_TO_PICKUP,
            SubState.PICKUP_BOOK,
            SubState.MOVE_TO_STORAGE,
            SubState.STOWING_BOOK,
        ]

    def execute(self, goal: Optional[dict] = None) -> bool:
        '''도서 픽업 시나리오를 순차적으로 실행한다.'''
        if self.is_active():
            return False

        self._begin()
        total = len(self._sequence)

        for index, sub_state in enumerate(self._sequence):
            self._set_sub_state(sub_state)
            progress = float(index + 1) / float(total)
            self._publish_feedback(progress)
            self._delay()

        self._finish()
        self._set_sub_state(SubState.NONE)
        self._publish_feedback(1.0)
        return True

    def cancel(self) -> None:
        '''진행 중인 픽업 작업을 취소한다.'''
        if not self.is_active():
            return

        self._finish()
        self._set_sub_state(SubState.SUB_ERROR)
        self._publish_feedback(0.0)
