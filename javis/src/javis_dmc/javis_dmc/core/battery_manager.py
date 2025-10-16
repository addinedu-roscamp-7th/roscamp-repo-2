#!/usr/bin/env python3
"""Battery Manager for Dobby Robot."""

import time
from enum import Enum
from typing import Dict, Optional


class BatteryState(Enum):
    """배터리 상태."""

    IDLE = 0  # 대기 (변화 없음)
    CHARGING = 1  # 충전 중
    DRAINING = 2  # 소모 중 (작업)


class BatteryConfig:
    """배터리 설정."""

    def __init__(self, config_dict: Dict):
        """Initialize battery configuration."""
        self.initial_level = config_dict.get("initial_level", 100.0)

        rates = config_dict.get("rates", {})
        self.charging_rate = rates.get("charging", 10.0)  # %/min
        self.idle_rate = rates.get("idle", 0.0)
        self.working_rate = rates.get("working", -1.0)

        thresholds = config_dict.get("thresholds", {})
        self.critical_threshold = thresholds.get("critical", 20.0)
        self.warning_threshold = thresholds.get("warning", 40.0)
        self.charge_target = thresholds.get("charge_target", 80.0)
        self.emergency_threshold = thresholds.get("emergency", 5.0)

        test_mode = config_dict.get("test_mode", {})
        self.test_mode_enabled = test_mode.get("enabled", False)
        self.manual_control = test_mode.get("manual_control", True)


class BatteryManager:
    """배터리 관리자."""

    def __init__(self, config: BatteryConfig, logger):
        """Initialize battery manager."""
        self.config = config
        self.logger = logger

        # 현재 상태
        self.level = config.initial_level  # %
        self.state = BatteryState.IDLE
        self.is_frozen = config.test_mode_enabled

        # 콜백
        self.on_critical_callback: Optional[callable] = None
        self.on_warning_callback: Optional[callable] = None
        self.on_charge_complete_callback: Optional[callable] = None
        self.on_emergency_callback: Optional[callable] = None

        # 타이머 (마지막 업데이트 시간)
        self.last_update_time = time.time()

        self.logger.info(f"BatteryManager initialized: {self.level:.1f}%")

    def update(self) -> None:
        """배터리 레벨 업데이트 (1초마다 호출)."""
        if self.is_frozen:
            return

        # 경과 시간 계산
        current_time = time.time()
        dt_sec = current_time - self.last_update_time
        self.last_update_time = current_time

        dt_min = dt_sec / 60.0  # 분 단위로 변환

        # 상태별 변화량 계산
        if self.state == BatteryState.CHARGING:
            delta = self.config.charging_rate * dt_min
        elif self.state == BatteryState.DRAINING:
            delta = self.config.working_rate * dt_min
        else:  # IDLE
            delta = self.config.idle_rate * dt_min

        # 이전 레벨 저장
        prev_level = self.level

        # 레벨 업데이트
        self.level += delta
        self.level = max(0.0, min(100.0, self.level))

        # 변화 로그 (1% 이상 변화 시)
        if abs(self.level - prev_level) >= 1.0:
            self.logger.debug(
                f"Battery: {prev_level:.1f}% → {self.level:.1f}% "
                f"({self.state.name})"
            )

        # 임계값 체크
        self._check_thresholds(prev_level)

    def _check_thresholds(self, prev_level: float) -> None:
        """임계값 체크 및 콜백 호출."""
        # Emergency (< 5%)
        if (
            prev_level >= self.config.emergency_threshold
            and self.level < self.config.emergency_threshold
        ):
            self.logger.error(f"🚨 BATTERY EMERGENCY: {self.level:.1f}%")
            if self.on_emergency_callback:
                self.on_emergency_callback()

        # Critical (< 20%)
        if (
            prev_level >= self.config.critical_threshold
            and self.level < self.config.critical_threshold
        ):
            self.logger.warn(f"⚠️ BATTERY CRITICAL: {self.level:.1f}%")
            if self.on_critical_callback:
                self.on_critical_callback()

        # Warning (< 40%)
        elif (
            prev_level >= self.config.warning_threshold
            and self.level < self.config.warning_threshold
        ):
            self.logger.info(f"⚠️ Battery Warning: {self.level:.1f}%")
            if self.on_warning_callback:
                self.on_warning_callback()

        # Charge Complete (>= 80%)
        if (
            self.state == BatteryState.CHARGING
            and prev_level < self.config.charge_target
            and self.level >= self.config.charge_target
        ):
            self.logger.info(f"✅ Charge Complete: {self.level:.1f}%")
            if self.on_charge_complete_callback:
                self.on_charge_complete_callback()

    def start_charging(self) -> None:
        """충전 시작."""
        if self.state != BatteryState.CHARGING:
            self.state = BatteryState.CHARGING
            self.logger.info(f"🔋 Charging started: {self.level:.1f}%")

    def start_draining(self) -> None:
        """소모 시작 (작업 중)."""
        if self.state != BatteryState.DRAINING:
            self.state = BatteryState.DRAINING
            self.logger.info(f"🔻 Battery draining: {self.level:.1f}%")

    def set_idle(self) -> None:
        """대기 상태."""
        if self.state != BatteryState.IDLE:
            self.state = BatteryState.IDLE
            self.logger.info(f"⏸️ Battery idle: {self.level:.1f}%")

    def is_critical(self) -> bool:
        """긴급 충전 필요 여부."""
        return self.level < self.config.critical_threshold

    def is_warning(self) -> bool:
        """충전 권장 여부."""
        return self.level < self.config.warning_threshold

    def is_sufficient(self) -> bool:
        """작업 가능 여부."""
        return self.level >= self.config.critical_threshold

    def is_charge_complete(self) -> bool:
        """충전 목표 달성 여부."""
        return self.level >= self.config.charge_target

    def force_set(self, level: float, freeze: bool = False) -> None:
        """배터리 레벨을 강제로 설정합니다.

        Test GUI 등에서 사용됩니다.

        Args:
            level (float): 설정할 배터리 레벨 (0 ~ 100).
            freeze (bool): True로 설정 시, 배터리 레벨이 더 이상 자동으로
                변하지 않도록 고정합니다.

        """
        self.level = max(0.0, min(100.0, level))
        self.is_frozen = freeze
        self.logger.info(
            f"Battery manually set: {self.level:.1f}% " f"(frozen={self.is_frozen})"
        )

    def get_status(self) -> Dict:
        """현재 배터리 상태 반환."""
        return {
            "level": self.level,
            "state": self.state.name,
            "is_charging": self.state == BatteryState.CHARGING,
            "is_critical": self.is_critical(),
            "is_warning": self.is_warning(),
            "is_frozen": self.is_frozen,
        }
