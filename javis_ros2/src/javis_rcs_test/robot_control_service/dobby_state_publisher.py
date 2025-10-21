#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from javis_interfaces.msg import DobbyState
import itertools
import time


class DobbyStatePublisher(Node):
    def __init__(self):
        super().__init__('dobby_state_publisher', namespace='')

        # QoS 설정 (신뢰성 높이고, 최근 몇 개만 보관)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pub = self.create_publisher(DobbyState, 'dobby/state', qos)

        # 예시용 상태 시나리오 (메인/서브 상태를 시간에 따라 순환)
        # 실제 시스템에서는 관제/미션 로직에서 현재 상태를 설정해 publish 하세요.
        self.scenario = itertools.cycle([
            # 초기화 → 대기
            (DobbyState.INITIALIZING, DobbyState.NONE, False, ""),
            (DobbyState.IDLE,         DobbyState.NONE, False, ""),
            # 도서 픽업 작업
            (DobbyState.PICKING_UP_BOOK, DobbyState.MOVE_TO_PICKUP, False, ""),
            (DobbyState.PICKING_UP_BOOK, DobbyState.PICKUP_BOOK,    False, ""),
            (DobbyState.PICKING_UP_BOOK, DobbyState.MOVE_TO_STORAGE, False, ""),
            (DobbyState.PICKING_UP_BOOK, DobbyState.STOWING_BOOK,    False, ""),
            # 가이드 작업
            (DobbyState.GUIDING, DobbyState.SELECT_DEST,   False, ""),
            (DobbyState.GUIDING, DobbyState.SCAN_USER,     False, ""),
            (DobbyState.GUIDING, DobbyState.GUIDING_TO_DEST, False, ""),
            # 청소 작업
            (DobbyState.CLEANING_DESK, DobbyState.MOVE_TO_DESK, False, ""),
            (DobbyState.CLEANING_DESK, DobbyState.SCAN_DESK,    False, ""),
            (DobbyState.CLEANING_DESK, DobbyState.CLEANING_TRASH, False, ""),
            (DobbyState.CLEANING_DESK, DobbyState.MOVE_TO_BIN,  False, ""),
            # (의도적) 오류 예시
            (DobbyState.MAIN_ERROR, DobbyState.SUB_ERROR, True, "Gripper vacuum fault detected"),
            # 충전/강제복귀 예시
            (DobbyState.MOVING_TO_CHARGER, DobbyState.NONE, False, ""),
            (DobbyState.CHARGING,          DobbyState.NONE, False, ""),
            (DobbyState.FORCE_MOVE_TO_CHARGER, DobbyState.NONE, False, ""),
        ])

        self.timer = self.create_timer(0.5, self.publish_once)  # 2Hz
        self.get_logger().info('DobbyState publisher started.')

    def publish_once(self):
        msg = DobbyState()
        main_state, sub_state, is_error, err_msg = next(self.scenario)

        msg.main_state = main_state
        msg.sub_state = sub_state
        msg.is_error = is_error
        msg.error_message = err_msg

        # 로그에 사람이 읽기 쉬운 텍스트도 찍어줌
        self.get_logger().info(
            f'Publishing -> main:{self.main_state_str(main_state)} '
            f'/ sub:{self.sub_state_str(sub_state)} '
            f'/ is_error:{is_error} / msg:"{err_msg}"'
        )

        self.pub.publish(msg)

    # 사람이 읽기 쉬운 문자열 매핑 (선택 사항)
    def main_state_str(self, s: int) -> str:
        table = {
            DobbyState.INITIALIZING: "INITIALIZING",
            DobbyState.CHARGING: "CHARGING",
            DobbyState.IDLE: "IDLE",
            DobbyState.MOVING_TO_CHARGER: "MOVING_TO_CHARGER",
            DobbyState.PICKING_UP_BOOK: "PICKING_UP_BOOK",
            DobbyState.RESHELVING_BOOK: "RESHELVING_BOOK",
            DobbyState.GUIDING: "GUIDING",
            DobbyState.CLEANING_DESK: "CLEANING_DESK",
            DobbyState.SORTING_SHELVES: "SORTING_SHELVES",
            DobbyState.FORCE_MOVE_TO_CHARGER: "FORCE_MOVE_TO_CHARGER",
            DobbyState.MAIN_ERROR: "MAIN_ERROR",
        }
        return table.get(s, f"UNKNOWN({s})")

    def sub_state_str(self, s: int) -> str:
        table = {
            DobbyState.NONE: "NONE",
            # Book Pickup
            DobbyState.MOVE_TO_PICKUP: "MOVE_TO_PICKUP",
            DobbyState.PICKUP_BOOK: "PICKUP_BOOK",
            DobbyState.MOVE_TO_STORAGE: "MOVE_TO_STORAGE",
            DobbyState.STOWING_BOOK: "STOWING_BOOK",
            # Reshelving
            DobbyState.MOVE_TO_RETURN_DESK: "MOVE_TO_RETURN_DESK",
            DobbyState.COLLECT_RETURN_BOOKS: "COLLECT_RETURN_BOOKS",
            DobbyState.MOVE_TO_PLACE_SHELF: "MOVE_TO_PLACE_SHELF",
            DobbyState.PLACE_RETURN_BOOK: "PLACE_RETURN_BOOK",
            # Guiding
            DobbyState.SELECT_DEST: "SELECT_DEST",
            DobbyState.SCAN_USER: "SCAN_USER",
            DobbyState.GUIDING_TO_DEST: "GUIDING_TO_DEST",
            DobbyState.FIND_USER: "FIND_USER",
            # Cleaning
            DobbyState.MOVE_TO_DESK: "MOVE_TO_DESK",
            DobbyState.SCAN_DESK: "SCAN_DESK",
            DobbyState.CLEANING_TRASH: "CLEANING_TRASH",
            DobbyState.MOVE_TO_BIN: "MOVE_TO_BIN",
            DobbyState.TIDYING_SHELVES: "TIDYING_SHELVES",
            # Sorting
            DobbyState.MOVE_TO_SHELF: "MOVE_TO_SHELF",
            DobbyState.SCAN_BOOK: "SCAN_BOOK",
            DobbyState.SORT_BOOK: "SORT_BOOK",
            # Errors
            DobbyState.SUB_ERROR: "SUB_ERROR",
        }
        return table.get(s, f"UNKNOWN({s})")


def main():
    rclpy.init()
    node = DobbyStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('DobbyState publisher stopped.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
