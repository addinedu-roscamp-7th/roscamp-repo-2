from dataclasses import dataclass
from typing import Optional, Set

from .state_enums import MainState, RobotMode, SubState


@dataclass
class StateSnapshot:
    '''상태 머신의 스냅샷 데이터를 표현한다.'''

    mode: RobotMode
    main_state: MainState
    sub_state: SubState


class DmcStateMachine:
    '''JAVIS DMC 메인 상태 머신 구현.'''

    _TASK_STATES: Set[MainState] = {
        MainState.PICKING_UP_BOOK,
        MainState.RESHELVING_BOOK,
        MainState.GUIDING,
        MainState.CLEANING_DESK,
        MainState.SORTING_SHELVES,
    }

    def __init__(self) -> None:
        self.mode = RobotMode.STANDBY
        self.main_state = MainState.INITIALIZING
        self.sub_state = SubState.NONE
        self._previous_main_state: MainState = MainState.INITIALIZING

    def set_mode(self, mode: RobotMode) -> None:
        '''로봇 동작 모드를 변경한다.'''
        if self.mode == mode:
            return
        self.mode = mode

        if mode == RobotMode.AUTONOMY and self.main_state == MainState.IDLE:
            self.main_state = MainState.ROAMING
        elif mode == RobotMode.STANDBY and self.main_state == MainState.ROAMING:
            self.main_state = MainState.IDLE

    def can_switch_mode(self, mode: RobotMode) -> bool:
        '''요청된 모드로 전환 가능한지 여부를 판단한다.'''
        if self.mode == mode:
            return True
        if self.is_emergency():
            return False
        if self.is_task_active():
            return False
        if self.is_listening():
            return False
        return True

    def set_main_state(self, state: MainState) -> None:
        '''메인 상태를 갱신한다.'''
        if self.mode == RobotMode.AUTONOMY and state == MainState.IDLE:
            state = MainState.ROAMING
        elif self.mode == RobotMode.STANDBY and state == MainState.ROAMING:
            state = MainState.IDLE

        self.main_state = state

        if state not in self._TASK_STATES:
            self.sub_state = SubState.NONE

    def set_sub_state(self, state: SubState) -> None:
        '''서브 상태를 갱신한다.'''
        self.sub_state = state

    def can_accept_task(
        self,
        task_state: MainState,
        *,
        battery_warning: bool = False,
        battery_critical: bool = False,
    ) -> bool:
        '''현재 상황에서 신규 작업을 수락할 수 있는지 판단한다.'''
        if task_state not in self._TASK_STATES:
            return False
        if battery_critical or battery_warning:
            return False
        if self.main_state == MainState.EMERGENCY_STOP:
            return False
        if self.mode == RobotMode.STANDBY:
            return self.main_state == MainState.IDLE
        if self.mode == RobotMode.AUTONOMY:
            return self.main_state == MainState.ROAMING
        return False

    def start_task(self, task_state: MainState) -> bool:
        '''작업 상태로 진입한다.'''
        if not self.can_accept_task(task_state):
            return False
        self._previous_main_state = self.main_state
        self.main_state = task_state
        self.sub_state = SubState.NONE
        return True

    def enter_listening(self) -> bool:
        '''LISTENING 상태로 전환한다.'''
        if self.main_state in {
            MainState.IDLE,
            MainState.ROAMING,
            MainState.GUIDING,
        }:
            self._previous_main_state = self.main_state
            self.main_state = MainState.LISTENING
            self.sub_state = SubState.NONE
            return True
        return False

    def exit_listening(self, fallback_state: Optional[MainState] = None) -> None:
        '''LISTENING 상태를 종료하고 이전 상태로 복귀한다.'''
        if self.main_state != MainState.LISTENING:
            return

        target_state = fallback_state or self._previous_main_state

        if target_state is None:
            target_state = MainState.IDLE

        self.set_main_state(target_state)
        self.sub_state = SubState.NONE

    def enter_waiting_dest_input(self) -> bool:
        '''WAITING_DEST_INPUT 상태로 전환한다 (v4.0).
        
        GUI QueryLocationInfo 호출 시 진입.
        60초 타이머 후 타임아웃.
        '''
        if self.main_state in {
            MainState.IDLE,
            MainState.ROAMING,
        }:
            self._previous_main_state = self.main_state
            self.main_state = MainState.WAITING_DEST_INPUT
            self.sub_state = SubState.NONE
            return True
        return False

    def exit_waiting_dest_input(self, fallback_state: Optional[MainState] = None) -> None:
        '''WAITING_DEST_INPUT 상태를 종료하고 이전 상태로 복귀한다.'''
        if self.main_state != MainState.WAITING_DEST_INPUT:
            return

        target_state = fallback_state or self._previous_main_state

        if target_state is None:
            target_state = MainState.IDLE

        self.set_main_state(target_state)
        self.sub_state = SubState.NONE

    def enter_emergency_stop(self) -> None:
        '''긴급 정지 상태로 진입한다.'''
        if self.main_state != MainState.EMERGENCY_STOP:
            self._previous_main_state = self.main_state
        self.main_state = MainState.EMERGENCY_STOP
        self.sub_state = SubState.NONE

    def resume_from_emergency(self) -> None:
        '''긴급 정지 해제 후 적절한 상태로 복귀한다.'''
        if self.main_state != MainState.EMERGENCY_STOP:
            return

        if self.mode == RobotMode.AUTONOMY:
            self.main_state = MainState.ROAMING
        else:
            self.main_state = MainState.IDLE

        self.sub_state = SubState.NONE

    def is_task_active(self) -> bool:
        '''작업 상태인지 여부를 반환한다.'''
        return self.main_state in self._TASK_STATES

    def is_listening(self) -> bool:
        '''LISTENING 상태인지 여부를 반환한다.'''
        return self.main_state == MainState.LISTENING

    def is_emergency(self) -> bool:
        '''긴급 정지 상태인지 여부를 반환한다.'''
        return self.main_state == MainState.EMERGENCY_STOP

    def determine_post_task_state(
        self,
        success: bool,
        *,
        at_charger: bool,
        battery_warning: bool,
        battery_critical: bool,
    ) -> MainState:
        '''작업 종료 후 전이할 상태를 계산한다.'''
        if battery_critical:
            return MainState.FORCE_MOVE_TO_CHARGER

        if battery_warning:
            return MainState.CHARGING if at_charger else MainState.MOVING_TO_CHARGER

        if not success and self.mode == RobotMode.AUTONOMY:
            return MainState.ROAMING

        if self.mode == RobotMode.AUTONOMY:
            return MainState.ROAMING

        return MainState.IDLE

    def reset_states(self) -> None:
        '''상태를 초기 IDLE/STANDBY 조합으로 되돌린다.'''
        self.mode = RobotMode.STANDBY
        self.main_state = MainState.IDLE
        self.sub_state = SubState.NONE
        self._previous_main_state = MainState.IDLE

    def snapshot(self) -> StateSnapshot:
        '''현재 상태를 스냅샷으로 반환한다.'''
        return StateSnapshot(
            mode=self.mode,
            main_state=self.main_state,
            sub_state=self.sub_state,
        )
