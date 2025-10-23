#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

from javis_interfaces.msg import DobbyState, BatteryStatus
from javis_interfaces.action import PickupBook, CleanSeat

class MockRcsNode(Node):
    """
    OrchestratorNode 테스트를 위한 가상 로봇(DMC) 노드.
    실제 dmc_node의 핵심 기능(상태 발행, 액션 서버)을 모방합니다.
    """
    def __init__(self):
        super().__init__('mock_rcs_node')

        # 파라미터로 로봇 네임스페이스를 받음 (예: 'dobby1')
        self.declare_parameter('robot_namespace', 'dobby1')
        self.namespace = self.get_parameter('robot_namespace').get_parameter_value().string_value

        self.current_state = DobbyState.IDLE
        self.battery_level = 100.0

        # --- 상태 발행 퍼블리셔 및 타이머 ---
        self.state_publisher = self.create_publisher(DobbyState, f'/{self.namespace}/status/robot_state', 10)
        self.battery_publisher = self.create_publisher(BatteryStatus, f'/{self.namespace}/status/battery_status', 10)
        self.status_timer = self.create_timer(1.0, self.publish_status)

        # --- 액션 서버 ---
        self.pickup_action_server = ActionServer(
            self,
            PickupBook,
            f'/{self.namespace}/main/pickup_book',
            execute_callback=self.pickup_execute_callback,
            goal_callback=lambda goal_request: GoalResponse.ACCEPT,
            cancel_callback=lambda cancel_request: CancelResponse.ACCEPT
        )

        self.clean_action_server = ActionServer(
            self,
            CleanSeat,
            f'/{self.namespace}/main/clean_seat',
            execute_callback=self.clean_execute_callback,
            goal_callback=lambda goal_request: GoalResponse.ACCEPT,
            cancel_callback=lambda cancel_request: CancelResponse.ACCEPT
        )

        self.get_logger().info(f"Mock RCS Node '{self.namespace}' is ready.")

    def publish_status(self):
        """주기적으로 로봇의 상태와 배터리 정보를 발행합니다."""
        state_msg = DobbyState()
        state_msg.main_state = self.current_state
        self.state_publisher.publish(state_msg)

        battery_msg = BatteryStatus()
        battery_msg.charge_percentage = self.battery_level
        self.battery_publisher.publish(battery_msg)

    async def pickup_execute_callback(self, goal_handle):
        """'pickup_book' 액션 요청을 처리합니다."""
        book_name = goal_handle.request.book_name
        self.get_logger().info(f"'{self.namespace}' starting pickup_book for '{book_name}'...")

        # 상태를 작업 중으로 변경
        self.current_state = DobbyState.PICKING_UP_BOOK
        self.publish_status()

        # 5초간 작업하는 척 시뮬레이션
        for i in range(5):
            feedback_msg = PickupBook.Feedback()
            feedback_msg.status = f"Picking up... {i+1}/5"
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()
        result = PickupBook.Result()
        result.success = True
        result.message = f"Successfully picked up {book_name}"
        
        # 상태를 다시 IDLE로 변경
        self.current_state = DobbyState.IDLE
        self.get_logger().info(f"'{self.namespace}' finished pickup_book.")
        return result

    async def clean_execute_callback(self, goal_handle):
        """'clean_seat' 액션 요청을 처리합니다."""
        request = goal_handle.request
        seat_id = request.seat_id
        self.get_logger().info(
            f"'{self.namespace}' starting clean_seat for seat {seat_id}..."
            f"\n\t- Return Desk Loc: ({request.return_desk_location.x:.2f}, {request.return_desk_location.y:.2f})"
            f"\n\t- Bin Loc: ({request.bin_location.x:.2f}, {request.bin_location.y:.2f})"
        )
        self.current_state = DobbyState.CLEANING_DESK
        self.publish_status()

        start_time = time.time()

        # 5초간 작업 시뮬레이션하며 피드백 전송
        for i in range(5):
            progress = (i + 1) * 20
            feedback_msg = CleanSeat.Feedback()
            feedback_msg.progress_percent = float(progress)
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f"Cleaning seat {seat_id}... {progress}%")
            time.sleep(1)

        goal_handle.succeed()

        result = CleanSeat.Result()
        result.success = True
        result.message = f"Seat {seat_id} cleaned successfully."
        result.seat_id = seat_id
        result.trash_collected_count = 3  # Mock data
        result.trash_types = ['paper_cup', 'plastic_bottle', 'napkin']  # Mock data
        result.total_distance_m = 15.5  # Mock data
        result.total_time_sec = float(time.time() - start_time)

        self.current_state = DobbyState.IDLE
        self.get_logger().info(f"'{self.namespace}' finished clean_seat.")
        return result

def main(args=None):
    rclpy.init(args=args)
    mock_rcs_node = MockRcsNode()
    rclpy.spin(mock_rcs_node)
    mock_rcs_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()