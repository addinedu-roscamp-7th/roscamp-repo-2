#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from javis_interfaces.msg import DobbyState
import itertools
import time # 이 모듈은 현재 코드에서 직접 사용되지 않지만, 일반적으로 시뮬레이션에 사용될 수 있습니다.


class DobbyStatePublisher(Node):
    def __init__(self, namespace):
        super().__init__('dobby_state_publisher', namespace=namespace) # 노드 이름과 네임스페이스 설정

        # QoS 설정 (Quality of Service)
        # - ReliabilityPolicy.RELIABLE: 메시지 전송의 신뢰성을 높여 메시지 손실을 방지합니다.
        # - HistoryPolicy.KEEP_LAST: 최근 10개의 메시지만 보관합니다.
        # - Depth: 보관할 메시지의 개수 (KEEP_LAST와 함께 사용)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # DobbyState 메시지를 'dobby/state' 토픽으로 발행하는 퍼블리셔 생성
        self.pub = self.create_publisher(DobbyState, 'status/robot_state', qos)

        # 예시용 상태 시나리오 (메인/서브 상태를 시간에 따라 순환)
        # `itertools.cycle`을 사용하여 정의된 상태들을 무한히 반복합니다.
        self.scenario = itertools.cycle([
            # (main_state, sub_state, is_error, error_message) 튜플의 리스트
            (DobbyState.INITIALIZING, DobbyState.NONE, False, ""),
            (DobbyState.IDLE,         DobbyState.NONE, False, ""),
            # 도서 픽업 작업 시뮬레이션
            (DobbyState.PICKING_UP_BOOK, DobbyState.MOVE_TO_PICKUP, False, ""),
            (DobbyState.PICKING_UP_BOOK, DobbyState.PICKUP_BOOK,    False, ""),
            (DobbyState.PICKING_UP_BOOK, DobbyState.MOVE_TO_STORAGE, False, ""),
            (DobbyState.PICKING_UP_BOOK, DobbyState.STOWING_BOOK,    False, ""),
            # 가이드 작업 시뮬레이션
            (DobbyState.GUIDING, DobbyState.SELECT_DEST,   False, ""),
            (DobbyState.GUIDING, DobbyState.SCAN_USER,     False, ""),
            (DobbyState.GUIDING, DobbyState.GUIDING_TO_DEST, False, ""),
            # 청소 작업 시뮬레이션
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

        # 0.5초마다 `publish_once` 메서드를 호출하는 타이머 생성 (2Hz)
        self.timer = self.create_timer(0.5, self.publish_once)
        self.get_logger().info('DobbyState publisher started.')

    def publish_once(self):
        """타이머에 의해 호출되어 다음 상태를 발행합니다."""
        msg = DobbyState()
        # `itertools.cycle`에서 다음 상태 튜플을 가져옵니다.
        main_state, sub_state, is_error, err_msg = next(self.scenario)

        # 메시지 필드에 상태 값들을 할당합니다.
        msg.main_state = main_state
        msg.sub_state = sub_state
        msg.is_error = is_error
        msg.error_message = err_msg

        # 로그에 사람이 읽기 쉬운 텍스트로 현재 발행하는 상태를 출력합니다.
        self.get_logger().info(
            f'Publishing -> main:{self.main_state_str(main_state)} '
            f'/ sub:{self.sub_state_str(sub_state)} '
            f'/ is_error:{is_error} / msg:"{err_msg}"'
        )

        # 메시지를 발행합니다.
        self.pub.publish(msg)

    # 사람이 읽기 쉬운 문자열 매핑 함수들 (디버깅 및 로깅 편의를 위함)
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
    node = DobbyStatePublisher(namespace='dobby1')
    try:
        rclpy.spin(node) # 노드가 메시지 콜백, 타이머 콜백 등을 처리하도록 합니다.
    except KeyboardInterrupt:
        pass # Ctrl+C 등으로 종료 시 예외 처리
    finally:
        node.get_logger().info('DobbyState publisher stopped.')
        node.destroy_node() # 노드 리소스 해제
        rclpy.shutdown() # ROS2 클라이언트 라이브러리 종료


if __name__ == '__main__':
    main()
