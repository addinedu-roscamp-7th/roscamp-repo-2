from enum import IntEnum


class BatteryState(IntEnum):
    '''배터리 상태를 나타내는 열거형.'''

    IDLE = 0
    CHARGING = 1
    DRAINING = 2


class BatteryManager:
    '''배터리 상태와 레벨을 관리하는 클래스.'''

    def __init__(
        self,
        level: float = 100.0,
        charge_rate: float = 10.0,
        work_rate: float = -1.0,
        warning_threshold: float = 40.0,
        critical_threshold: float = 20.0,
        charge_target: float = 80.0,
    ) -> None:
        # 배터리 파라미터 초기화
        self.level = self._clamp(level)
        self.state = BatteryState.IDLE
        self.charge_rate = charge_rate
        self.work_rate = work_rate
        self.warning_threshold = warning_threshold
        self.critical_threshold = critical_threshold
        self.charge_target = charge_target
        self.test_mode_enabled = False

    def update(self, dt: float) -> float:
        '''시간 경과에 따라 배터리 레벨을 갱신한다.'''
        if dt <= 0.0:
            return self.level

        delta = 0.0

        if self.state == BatteryState.CHARGING:
            delta = (self.charge_rate / 60.0) * dt
        elif self.state == BatteryState.DRAINING:
            delta = (self.work_rate / 60.0) * dt

        self.level = self._clamp(self.level + delta)
        return self.level

    def start_charging(self) -> None:
        '''충전 상태로 전환한다.'''
        self.state = BatteryState.CHARGING

    def start_draining(self) -> None:
        '''소모 상태로 전환한다.'''
        self.state = BatteryState.DRAINING

    def set_idle(self) -> None:
        '''배터리 변화를 중지한다.'''
        self.state = BatteryState.IDLE

    def is_warning(self) -> bool:
        '''경고 구간 여부를 반환한다.'''
        return self.level <= self.warning_threshold

    def is_critical(self) -> bool:
        '''위험 구간 여부를 반환한다.'''
        return self.level <= self.critical_threshold

    def is_sufficient(self) -> bool:
        '''작업 가능 수준인지 확인한다.'''
        return self.level >= self.charge_target

    def force_set(self, level: float) -> float:
        '''레벨을 강제로 지정한다 (테스트 도구용).'''
        self.level = self._clamp(level)
        return self.level

    def enable_test_mode(self) -> None:
        '''테스트 모드를 활성화한다.'''
        self.test_mode_enabled = True

    def disable_test_mode(self) -> None:
        '''테스트 모드를 비활성화한다.'''
        self.test_mode_enabled = False

    def get_state(self) -> BatteryState:
        '''현재 배터리 상태를 반환한다.'''
        return self.state

    def set_state(self, state: BatteryState) -> None:
        '''외부에서 배터리 상태를 갱신한다.'''
        self.state = state

    def _clamp(self, value: float) -> float:
        '''배터리 레벨을 0~100 범위로 제한한다.'''
        return max(0.0, min(100.0, value))
