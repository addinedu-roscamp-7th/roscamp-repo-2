#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from collections import deque
import threading

from javis_interfaces.srv import MyJson
from javis_interfaces.msg import DobbyState, BatteryStatus
from geometry_msgs.msg import Pose, Pose2D, Point, Quaternion # Pose, Pose2D, Point, Quaternion 추가
from javis_interfaces.action import PickupBook, CleanSeat
from .clean_seat import CleanSeat as CleanSeatNode # CleanSeat 노드 클래스를 직접 임포트
from .pickup_book import PickupBook as PickupBookNode # PickupBook 노드 클래스를 직접 임포트

class OrchestratorNode(Node):
    """
    RCS(Robot Control Service)의 핵심 노드.
    여러 로봇의 상태를 모니터링하고, 작업을 할당하는 오케스트레이터 역할을 수행합니다.
    """
    def __init__(self):
        super().__init__('robot_control_service')

        # --- 파라미터 선언 및 초기화 ---
        self.declare_parameter('robot_namespaces', ['dobby1', 'dobby2'])
        self.robot_namespaces = self.get_parameter('robot_namespaces').get_parameter_value().string_array_value
        
        self.robot_states = {}
        self.task_queue = deque()
        self.action_clients = {} # 로봇별 액션 클라이언트를 저장

        # --- ROS2 인터페이스 초기화 ---
        # 외부로부터 작업 요청을 받는 서비스 서버
        self.task_service = self.create_service(MyJson, 'robot_task', self.task_service_callback)

        # 각 로봇의 상태와 배터리를 구독
        for ns in self.robot_namespaces:
            self.robot_states[ns] = {'state': None, 'battery': None}
            self.create_subscription(
                DobbyState,
                f'/{ns}/status/robot_state',
                lambda msg, namespace=ns: self.robot_state_callback(msg, namespace),
                10
            )
            self.create_subscription(
                BatteryStatus,
                f'/{ns}/status/battery_status',
                lambda msg, namespace=ns: self.battery_status_callback(msg, namespace),
                10
            )
            self.get_logger().info(f"Subscribing to topics for robot '{ns}'")

            # 로봇별 액션 클라이언트 생성 및 저장
            self.action_clients[ns] = {
                'pickup_book': ActionClient(self, PickupBook, f'/{ns}/main/pickup_book'),
                'clean_seat': ActionClient(self, CleanSeat, f'/{ns}/main/clean_seat')
            }

        # 주기적으로 작업 큐를 확인하고 할당을 시도하는 타이머
        self.assignment_timer = self.create_timer(1.0, self.process_task_queue)

        self.get_logger().info(f"OrchestratorNode is ready, managing robots: {self.robot_namespaces}")
        self.get_logger().info("Service [/robot_task] is ready.")

    def task_service_callback(self, request, response):
        """외부로부터 작업 요청을 받아 큐에 추가합니다."""
        try:
            task_data = json.loads(request.payload)
            self.get_logger().info(f"New task received: {task_data.get('task_name', 'Unknown')}")
            
            # TODO: 작업 데이터 유효성 검증
            
            self.task_queue.append(task_data)
            
            response.ok = True
            response.message = json.dumps({'status': 'Task queued successfully'})
        except json.JSONDecodeError as e:
            response.ok = False
            response.message = f"Invalid JSON format: {e}"
            self.get_logger().error(f"Failed to decode task request: {request.payload}")
        
        return response

    def robot_state_callback(self, msg, namespace):
        """로봇의 메인 상태를 업데이트합니다."""
        if self.robot_states.get(namespace):
            self.robot_states[namespace]['state'] = msg.main_state
            # self.get_logger().info(f"State update for {namespace}: {msg.main_state}") # 디버깅용

    def battery_status_callback(self, msg, namespace):
        """로봇의 배터리 상태를 업데이트합니다."""
        if self.robot_states.get(namespace):
            self.robot_states[namespace]['battery'] = msg.charge_percentage
            # self.get_logger().info(f"Battery update for {namespace}: {msg.percentage}%") # 디버깅용

    def process_task_queue(self):
        """작업 큐를 확인하고 가능한 경우 로봇에게 작업을 할당합니다."""
        if not self.task_queue:
            return

        # 가장 기본적인 할당 로직: IDLE 상태인 첫 번째 로봇을 찾는다.
        # 향후 로봇의 위치, 배터리 잔량 등을 고려하여 고도화할 수 있습니다.
        available_robot = None
        for ns, status in self.robot_states.items():
            # DobbyState.msg의 IDLE 상태 값은 2입니다. (DevelopmentPlan.md 참조)
            if status['state'] == DobbyState.IDLE:
                available_robot = ns
                break
        
        if available_robot:
            task_to_assign = self.task_queue.popleft()
            self.get_logger().info(f"Assigning task '{task_to_assign.get('task_name')}' to robot '{available_robot}'")
            self.execute_task(available_robot, task_to_assign)
        else:
            self.get_logger().info("No available robots at the moment. Task remains in queue.")

    def execute_task(self, robot_namespace, task_data):
        """선택된 로봇에게 실제 작업을 지시합니다."""
        task_name = task_data.get('task_name')
        self.get_logger().info(f"Executing '{task_name}' on '{robot_namespace}'...")

        if task_name == 'pickup_book':
            book_id = task_data.get('book_id')
            self.get_logger().info(f"Sending 'pickup_book' goal for book_id {book_id} to robot '{robot_namespace}'")

            thread = threading.Thread(target=self._run_pickup_book_task, args=(robot_namespace, book_id, task_data))
            thread.start()
        elif task_name == 'clean_seat':
            # [수정됨] Executor를 사용하여 'clean_seat' 노드의 로직을 현재 프로세스에서 실행
            seat_id = task_data.get('seat_id', 0)
            self.get_logger().info(f"Executing 'clean_seat' task for seat {seat_id} on robot '{robot_namespace}' using an executor.")
            
            # 별도의 스레드에서 작업을 실행하여 메인 스레드(Orchestrator)의 스핀을 막지 않도록 합니다.
            thread = threading.Thread(target=self._run_clean_seat_task, args=(robot_namespace, seat_id, task_data))
            thread.start()

    def _run_clean_seat_task(self, robot_namespace, seat_id, task_data):
        """별도 스레드에서 CleanSeat 노드를 생성하고 실행합니다."""
        temp_executor = SingleThreadedExecutor()
        clean_seat_node = None
        try:
            # 임시 CleanSeat 노드 생성
            clean_seat_node = CleanSeatNode(namespace=f'{robot_namespace}/main')
            temp_executor.add_node(clean_seat_node)

            # task_data에서 seat_id를 제외한 나머지 인자를 kwargs로 전달합니다.
            kwargs = {k: v for k, v in task_data.items() if k != 'seat_id'}
            # 작업 실행 및 완료 대기
            task_future = clean_seat_node.run_task(seat_id, **kwargs)
            temp_executor.spin_until_future_complete(task_future)
            
            result = task_future.result()
            self.get_logger().info(f"Clean seat task for '{robot_namespace}' completed. Success: {result['success']}, Msg: '{result['message']}'")

        except Exception as e:
            self.get_logger().error(f"An error occurred during clean_seat task for '{robot_namespace}': {e}")
        finally:
            if clean_seat_node:
                clean_seat_node.destroy_node()
            temp_executor.shutdown()

    def _run_pickup_book_task(self, robot_namespace, book_id, task_data):
        temp_executor = SingleThreadedExecutor()
        pickup_book_node = None
        try:
            pickup_book_node = PickupBookNode(namespace=f'{robot_namespace}/main')
            temp_executor.add_node(pickup_book_node)

            kwargs = {k: v for k, v in task_data.items() if k != 'book_id'}

            task_future = pickup_book_node.run_task(book_id, **kwargs)
            temp_executor.spin_until_future_complete(task_future)

            result = task_future.result()
            self.get_logger().info(f"Pickup book task for '{robot_namespace}' completed. Success : {result['success']}, Msg: '{result['message']}'")

        except Exception as e:
            self.get_logger().error(f"An error occurred during pickup_book task for '{robot_namespace}': {e}")
        finally:
            if pickup_book_node:
                pickup_book_node.destroy_node()
            temp_executor.shutdown()

    def goal_response_callback(self, robot_namespace, future):
        """골 전송 요청에 대한 서버의 수락/거부 응답을 처리합니다."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"Goal for '{robot_namespace}' was rejected by the server.")
            return

        self.get_logger().info(f"Goal for '{robot_namespace}' accepted. Waiting for result...")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(
            lambda future: self.get_result_callback(robot_namespace, future)
        )

    def get_result_callback(self, robot_namespace, future):
        """액션 실행의 최종 결과를 처리합니다."""
        result = future.result().result
        self.get_logger().info(
            f"Result for '{robot_namespace}': success={result.success}, message='{result.message}'"
        )
        # TODO: 작업 완료 후 로봇 상태를 다시 IDLE로 변경하거나 후속 조치 수행

    def feedback_callback(self, robot_namespace, feedback_msg):
        """액션 실행 중 서버로부터 오는 피드백을 처리합니다."""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"Feedback2 from '{robot_namespace}': {feedback}"
        )

def main(args=None):
    rclpy.init(args=args)
    orchestrator_node = OrchestratorNode()
    rclpy.spin(orchestrator_node)
    orchestrator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()