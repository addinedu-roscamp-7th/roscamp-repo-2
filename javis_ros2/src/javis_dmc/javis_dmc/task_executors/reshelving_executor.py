from typing import Optional

from javis_dmc.states.state_enums import SubState

from .base_executor import BaseExecutor


class ReshelvingExecutor(BaseExecutor):
    '''반납 도서를 정리하는 실행자.'''

    def __init__(self) -> None:
        super().__init__('reshelving_executor')
        self._sequence = [
            SubState.MOVE_TO_RETURN_DESK,
            SubState.COLLECT_RETURN_BOOKS,
            SubState.MOVE_TO_PLACE_SHELF,
            SubState.PLACE_RETURN_BOOK,
        ]

    def execute(self, goal: Optional[dict] = None) -> bool:
        '''반납 도서 정리 시나리오를 수행한다.'''
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
        '''진행 중인 정리 작업을 취소한다.'''
        if not self.is_active():
            return

        self._finish()
        self._set_sub_state(SubState.SUB_ERROR)
        self._publish_feedback(0.0)
