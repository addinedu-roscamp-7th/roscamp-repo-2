from dataclasses import dataclass


@dataclass
class Config:
    """비전/로봇 동작에 사용되는 기본 설정 값 모음."""

    # 이동/정렬 관련
    speed: int = 25
    settle_wait: float = 1.0
    center_tolerance: float = 10.0
    min_move: float = 2.0
    step: int = 20
    z_fixed: float = 250.0
    x_safe_min: int = 160
    x_safe_max: int = 200
    y_safe_min: int = -140
    y_safe_max: int = 140
    forward_x_mm: float = 40.0
    forward_y_mm: float = -6.0
    pick_z_half: float = 200.0
    pick_z_down: float = 140.0
    pick_x_offset_dobby1: float = 15.0
    pick_y_offset_dobby1: float = 0.0
    pick_z_offset_dobby1: float = 20.0
    place_x_offset_dobby1: float = -10.0
    place_y_offset_dobby1: float = 0.0
    place_z_offset_dobby1: float = 0.0
    pick_x_offset_dobby2: float = 0.0
    pick_y_offset_dobby2: float = 10.0
    pick_z_offset_dobby2: float = 20.0
    place_x_offset_dobby2: float = -20.0
    place_y_offset_dobby2: float = 30.0
    place_z_offset_dobby2: float = 0.0

    # 카메라 설정
    camera_width: int = 1280
    camera_height: int = 720
    camera_fps: int = 5
