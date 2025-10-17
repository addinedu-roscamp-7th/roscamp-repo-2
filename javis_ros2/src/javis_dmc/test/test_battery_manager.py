#!/usr/bin/env python3
"""Battery Manager Unit Test."""

import time

import pytest

from javis_dmc.core import BatteryConfig, BatteryManager, BatteryState


class MockLogger:
    """Mock Logger for testing."""

    def info(self, msg):
        """Log info."""
        print(f"[INFO] {msg}")

    def warn(self, msg):
        """Log warning."""
        print(f"[WARN] {msg}")

    def error(self, msg):
        """Log error."""
        print(f"[ERROR] {msg}")

    def debug(self, msg):
        """Log debug."""
        pass


@pytest.fixture
def battery_config():
    """기본 배터리 설정."""
    return BatteryConfig(
        {
            "initial_level": 50.0,
            "rates": {"charging": 10.0, "idle": 0.0, "working": -1.0},
            "thresholds": {
                "critical": 20.0,
                "warning": 40.0,
                "charge_target": 80.0,
                "emergency": 5.0,
            },
            "test_mode": {"enabled": False, "manual_control": True},
        }
    )


@pytest.fixture
def battery_manager(battery_config):
    """테스트용 BatteryManager fixture를 생성합니다."""
    return BatteryManager(battery_config, MockLogger())


def test_initial_state(battery_manager):
    """초기 상태 확인."""
    assert battery_manager.level == 50.0
    assert battery_manager.state == BatteryState.IDLE
    assert battery_manager.is_sufficient()
    assert not battery_manager.is_critical()


def test_charging(battery_manager):
    """충전 시뮬레이션."""
    battery_manager.level = 50.0
    battery_manager.start_charging()

    assert battery_manager.state == BatteryState.CHARGING

    # 1분 경과 시뮬레이션 (60초)
    for _ in range(60):
        time.sleep(0.01)
        battery_manager.last_update_time -= 1.0
        battery_manager.update()

    # 50% + 10% = 60%
    assert 59.0 <= battery_manager.level <= 61.0


def test_draining(battery_manager):
    """작업 중 배터리 소모."""
    battery_manager.level = 50.0
    battery_manager.start_draining()

    assert battery_manager.state == BatteryState.DRAINING

    # 1분 경과 시뮬레이션
    for _ in range(60):
        battery_manager.last_update_time -= 1.0
        battery_manager.update()

    # 50% - 1% = 49%
    assert 48.0 <= battery_manager.level <= 50.0


def test_critical_callback(battery_manager):
    """Critical 콜백 테스트."""
    callback_called = {"count": 0}

    def on_critical():
        callback_called["count"] += 1

    battery_manager.on_critical_callback = on_critical
    battery_manager.level = 21.0

    # 19%로 떨어뜨림
    battery_manager.level = 19.0
    battery_manager._check_thresholds(21.0)

    assert callback_called["count"] == 1
    assert battery_manager.is_critical()


def test_charge_complete_callback(battery_manager):
    """충전 완료 콜백 테스트."""
    callback_called = {"count": 0}

    def on_charge_complete():
        callback_called["count"] += 1

    battery_manager.on_charge_complete_callback = on_charge_complete
    battery_manager.start_charging()
    battery_manager.level = 79.0

    # 80%로 올림
    battery_manager.level = 80.0
    battery_manager._check_thresholds(79.0)

    assert callback_called["count"] == 1
    assert battery_manager.is_charge_complete()


def test_force_set(battery_manager):
    """강제 설정 (Test GUI용)."""
    battery_manager.force_set(75.0, freeze=True)

    assert battery_manager.level == 75.0
    assert battery_manager.is_frozen

    # Frozen 상태에서 update 호출해도 변화 없음
    battery_manager.start_draining()
    battery_manager.update()
    assert battery_manager.level == 75.0


def test_clamp(battery_manager):
    """레벨 범위 제한 테스트."""
    battery_manager.force_set(150.0)  # 100 초과
    assert battery_manager.level == 100.0

    battery_manager.force_set(-10.0)  # 0 미만
    assert battery_manager.level == 0.0


def test_idle_no_change(battery_manager):
    """대기 중 배터리 변화 없음."""
    battery_manager.level = 60.0
    battery_manager.set_idle()

    initial = battery_manager.level

    for _ in range(60):
        battery_manager.last_update_time -= 1.0
        battery_manager.update()

    # 변화 없음
    assert abs(battery_manager.level - initial) < 0.1


def test_get_status(battery_manager):
    """상태 딕셔너리 반환."""
    battery_manager.level = 35.0  # ← 수정: 35% < 40% (warning)
    battery_manager.start_charging()

    status = battery_manager.get_status()

    assert status["level"] == 35.0
    assert status["state"] == "CHARGING"
    assert status["is_charging"] is True
    assert status["is_critical"] is False
    assert status["is_warning"] is True  # 35% < 40%
