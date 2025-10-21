#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from javis_interfaces.msg import DobbyState


class DobbyStateSubscriber(Node):
    def __init__(self):
        super().__init__('dobby_state_subscriber')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.sub = self.create_subscription(
            DobbyState,
            'dobby/state',
            self.on_state,
            qos
        )
        self.get_logger().info('DobbyState subscriber started.')

    def on_state(self, msg: DobbyState):
        self.get_logger().info(
            f'Received -> main:{self.main_state_str(msg.main_state)} '
            f'/ sub:{self.sub_state_str(msg.sub_state)} '
            f'/ is_error:{msg.is_error} / msg:"{msg.error_message}"'
        )

        # 에러 시 간단한 대응 로직 예시
        if msg.is_error or msg.main_state == DobbyState.MAIN_ERROR or msg.sub_state == DobbyState.SUB_ERROR:
            self.handle_error(msg)

    def handle_error(self, msg: DobbyState):
        # 실제 시스템에서는 관제/알림/리커버리 로직 호출
        self.get_logger().warn(
            f'[ERROR] main:{self.main_state_str(msg.main_state)} '
            f'/ sub:{self.sub_state_str(msg.sub_state)} -> {msg.error_message}'
        )

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
            DobbyState.MOVE_TO_PICKUP: "MOVE_TO_PICKUP",
            DobbyState.PICKUP_BOOK: "PICKUP_BOOK",
            DobbyState.MOVE_TO_STORAGE: "MOVE_TO_STORAGE",
            DobbyState.STOWING_BOOK: "STOWING_BOOK",
            DobbyState.MOVE_TO_RETURN_DESK: "MOVE_TO_RETURN_DESK",
            DobbyState.COLLECT_RETURN_BOOKS: "COLLECT_RETURN_BOOKS",
            DobbyState.MOVE_TO_PLACE_SHELF: "MOVE_TO_PLACE_SHELF",
            DobbyState.PLACE_RETURN_BOOK: "PLACE_RETURN_BOOK",
            DobbyState.SELECT_DEST: "SELECT_DEST",
            DobbyState.SCAN_USER: "SCAN_USER",
            DobbyState.GUIDING_TO_DEST: "GUIDING_TO_DEST",
            DobbyState.FIND_USER: "FIND_USER",
            DobbyState.MOVE_TO_DESK: "MOVE_TO_DESK",
            DobbyState.SCAN_DESK: "SCAN_DESK",
            DobbyState.CLEANING_TRASH: "CLEANING_TRASH",
            DobbyState.MOVE_TO_BIN: "MOVE_TO_BIN",
            DobbyState.TIDYING_SHELVES: "TIDYING_SHELVES",
            DobbyState.MOVE_TO_SHELF: "MOVE_TO_SHELF",
            DobbyState.SCAN_BOOK: "SCAN_BOOK",
            DobbyState.SORT_BOOK: "SORT_BOOK",
            DobbyState.SUB_ERROR: "SUB_ERROR",
        }
        return table.get(s, f"UNKNOWN({s})")


def main():
    rclpy.init()
    node = DobbyStateSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('DobbyState subscriber stopped.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
