from dataclasses import dataclass
from typing import Optional, Set

from .state_enums import MainState, SubState


@dataclass
class StateSnapshot:
    '''상태 머신의 스냅샷을 표현하는 데이터 구조.'''

    main_state: MainState
    sub_state: SubState
    task_id: Optional[int]


class DmcStateMachine:
    '''JAVIS DMC 메인 상태 머신.'''

    _TASK_STATES: Set[MainState] = {
        MainState.PICKING_UP_BOOK,
        MainState.RESHELVING_BOOK,
        MainState.GUIDING,
        MainState.CLEANING_DESK,
        MainState.SORTING_SHELVES,
    }

    def __init__(self) -> None:
        self.main_state = MainState.INITIALIZING
        self.sub_state = SubState.NONE
        self.current_task_id: Optional[int] = None

    def set_main_state(self, state: MainState) -> None:
        '''메인 상태를 갱신한다.'''
        self.main_state = state

    def set_sub_state(self, state: SubState) -> None:
        '''서브 상태를 갱신한다.'''
        self.sub_state = state

    def clear_task(self) -> None:
        '''작업 식별자를 초기화한다.'''
        self.current_task_id = None

    def can_accept_task(self, task_state: MainState) -> bool:
        '''현재 상태에서 새로운 작업을 수락할 수 있는지 확인한다.'''
        if task_state not in self._TASK_STATES:
            return False
        return self.main_state in (MainState.IDLE, MainState.MOVING_TO_CHARGER)

    def start_task(self, task_state: MainState, task_id: Optional[int]) -> bool:
        '''작업을 시작하며 상태를 갱신한다.'''
        if not self.can_accept_task(task_state):
            return False
        self.main_state = task_state
        self.sub_state = SubState.NONE
        self.current_task_id = task_id
        return True

    def determine_post_task_state(
        self,
        success: bool,
        at_charger: bool,
        battery_warning: bool,
        battery_critical: bool,
    ) -> MainState:
        '''작업 종료 후 전이할 메인 상태를 계산한다.'''
        if battery_critical:
            return MainState.FORCE_MOVE_TO_CHARGER
        if not success:
            return MainState.IDLE
        if at_charger:
            return MainState.IDLE
        if battery_warning:
            return MainState.MOVING_TO_CHARGER
        return MainState.IDLE

    def reset_states(self) -> None:
        '''상태를 IDLE로 되돌린다.'''
        self.main_state = MainState.IDLE
        self.sub_state = SubState.NONE
        self.clear_task()

    def snapshot(self) -> StateSnapshot:
        '''현재 상태를 스냅샷으로 반환한다.'''
        return StateSnapshot(
            main_state=self.main_state,
            sub_state=self.sub_state,
            task_id=self.current_task_id,
        )
