#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from collections import deque
from datetime import datetime
import threading

from javis_interfaces.srv import MyJson
from javis_interfaces.msg import DobbyState, BatteryStatus, Scheduling
from geometry_msgs.msg import Pose, Pose2D, Point, Quaternion # Pose, Pose2D, Point, Quaternion 추가
from javis_interfaces.action import PickupBook, CleanSeat, GuidePerson, ReshelvingBook, PerformTask
from .clean_seat import CleanSeat as CleanSeatNode # CleanSeat 노드 클래스를 직접 임포트
from .pickup_book import PickupBook as PickupBookNode # PickupBook 노드 클래스를 직접 임포트
from .guide_person import GuidePerson as GuidePersonNode
from .reshelving_book import ReshelvingBook as ReshelvingBookNode
from .kreacher_perform import KreacherPerform as KreacherNode, get_kreacher_state
from rclpy.qos import QoSProfile, DurabilityPolicy

class OrchestratorNode(Node):
    no = 0
    """
    RCS(Robot Control Service)의 핵심 노드.
    여러 로봇의 상태를 모니터링하고, 작업을 할당하는 오케스트레이터 역할을 수행합니다.
    """
    def __init__(self):
        super().__init__('robot_control_service')

        self.declare_parameter('robot_namespaces', ['dobby1', 'dobby2', 'kreacher'])
        self.robot_namespaces = self.get_parameter('robot_namespaces').get_parameter_value().string_array_value
        
        self.robot_states = {}
        self.task_queue = deque()
        self.action_clients = {}

        # Scheduling 메시지를 발행할 퍼블리셔를 생성합니다.
        

        self.task_service = self.create_service(MyJson, 'robot_task', self.task_service_callback)
        
        for ns in self.robot_namespaces:
            self.robot_states[ns] = {'state': None, 'battery': None}
            # kreacher는 상태 토픽을 발행하지 않으므로 구독에서 제외
            qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            if ns != 'kreacher':
                self.create_subscription(
                    DobbyState,
                    f'/{ns}/status/robot_state',
                    lambda msg, namespace=ns: self.robot_state_callback(msg, namespace),
                    qos_profile
                )
                self.create_subscription(
                    BatteryStatus,
                    f'/{ns}/status/battery_status',
                    lambda msg, namespace=ns: self.battery_status_callback(msg, namespace),
                    10
                )
                
            self.task_scheduling_pub = self.create_publisher(Scheduling, f'/{ns}/status/task_scheduling', 10)
            self.get_logger().info(f"Subscribing to topics for robot '{ns}'")

            self.action_clients[ns] = {
                'pickup_book': ActionClient(self, PickupBook, f'/{ns}/main/pickup_book'),
                'clean_seat': ActionClient(self, CleanSeat, f'/{ns}/main/clean_seat'),
                'guide_person': ActionClient(self, GuidePerson, f'/{ns}/main/guide_person'),
                'reshelving_book': ActionClient(self, ReshelvingBook, f'/{ns}/main/reshelving_book'),
                'kreacher': ActionClient(self, PerformTask, f'/kreacher/action/perform_task'),
            } 

        self.assignment_timer = self.create_timer(1.0, self.process_task_queue)

        self.get_logger().info(f"OrchestratorNode is ready, managing robots: {self.robot_namespaces}")
        self.get_logger().info("Service [/robot_task] is ready.")

    def task_service_callback(self, request, response):
        try:
            task_data = json.loads(request.payload)
            self.get_logger().info(f"New task received: {task_data.get('task_name', 'Unknown')}")

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

    def battery_status_callback(self, msg, namespace):
        """로봇의 배터리 상태를 업데이트합니다."""
        if self.robot_states.get(namespace):
            self.robot_states[namespace]['battery'] = msg.charge_percentage

    def process_task_queue(self):
        """작업 큐를 확인하고 가능한 경우 로봇에게 작업을 할당합니다."""
        if not self.task_queue:
            return
       
        task_to_assign = self.task_queue[0]
        task_name = task_to_assign.get('task_name')
        available_robot = None

        # 작업 종류에 따라 적합한 로봇을 찾습니다.
        if task_name == 'kreacher':
            # 'kreacher' 작업은 kreacher의 상태를 직접 확인합니다.
            if get_kreacher_state():
                available_robot = 'kreacher'
        else:
            # 그 외 작업(dobby 작업)은 유휴 상태의 dobby 로봇을 찾습니다.
            for ns, status in self.robot_states.items():
                if ns != 'kreacher' and status['state'] == DobbyState.IDLE:
                    available_robot = ns
                    break
        
        if available_robot:
            task_to_assign = self.task_queue.popleft()
            self.get_logger().info(f"Assigning task '{task_to_assign.get('task_name')}' to robot '{available_robot}'")

            # Scheduling 메시지를 생성하고 발행합니다.
            scheduling_msg = Scheduling()
            self.no += 1
            # task_data에서 값을 가져오거나, 없으면 기본값을 사용합니다.
            scheduling_msg.no = self.no 
            scheduling_msg.robot_name = available_robot
            scheduling_msg.task_id = task_to_assign.get('task_name', '')
            scheduling_msg.priority = task_to_assign.get('priority', 0)
            scheduling_msg.status = 1  # 1: 작업 할당됨 (Assigned) 상태로 가정
            scheduling_msg.message=""
            self.date_str = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            scheduling_msg.task_create_time = self.date_str

            self.task_scheduling_pub.publish(scheduling_msg)
            self.get_logger().info(f"Published scheduling message for task_id: {scheduling_msg.task_id} to robot: {scheduling_msg.robot_name}, no : {scheduling_msg.no}, priority : {scheduling_msg.priority}")


            self.execute_task(available_robot, task_to_assign)
        else:
            self.get_logger().debug("No available robots at the moment. Task remains in queue.")
    

    def execute_task(self, robot_namespace, task_data):
        """선택된 로봇에게 실제 작업을 지시합니다."""
        task_name = task_data.get('task_name')
        self.get_logger().info(f"Executing '{task_name}' on '{robot_namespace}'...")

        if task_name == 'pickup_book':
            book_id = task_data.get('book_id')
            self.get_logger().info(f"Sending 'pickup_book' goal for book_id {book_id} to robot '{robot_namespace}'")

            scheduling_msg = Scheduling()
            scheduling_msg.no = self.no 
            scheduling_msg.robot_name = robot_namespace
            scheduling_msg.task_id = task_name
            scheduling_msg.priority = 0
            scheduling_msg.status = 4
            scheduling_msg.message=""
            self.date_str = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            scheduling_msg.task_create_time = self.date_str
            self.task_scheduling_pub.publish(scheduling_msg)
            self.get_logger().info(f'task_data: {task_data}')
            shelf_approach_location = task_data.get('shelf_approach_location')

            thread = threading.Thread(target=self._run_pickup_book_task, args=(robot_namespace, shelf_approach_location,book_id, task_data))
            thread.start()
        elif task_name == 'clean_seat':
            
            seat_id = task_data.get('seat_id', 0)
            self.get_logger().info(f"Executing 'clean_seat' task for seat {seat_id} on robot '{robot_namespace}' using an executor.")

            scheduling_msg = Scheduling()
            scheduling_msg.no = self.no 
            scheduling_msg.robot_name = robot_namespace
            scheduling_msg.task_id = task_name
            scheduling_msg.priority = 0
            scheduling_msg.status = 4
            scheduling_msg.message=""
            self.date_str = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            scheduling_msg.task_create_time = self.date_str
            self.task_scheduling_pub.publish(scheduling_msg)
            
            thread = threading.Thread(target=self._run_clean_seat_task, args=(robot_namespace, seat_id, task_data))
            thread.start()
        elif task_name == 'guide_person':
            dest_location = task_data.get('dest_location')
            self.get_logger().info(f"Sending 'guide_person' goal for dest_location {dest_location} to robot '{robot_namespace}'")

            scheduling_msg = Scheduling()
            scheduling_msg.no = self.no 
            scheduling_msg.robot_name = robot_namespace
            scheduling_msg.task_id = task_name
            scheduling_msg.priority = 0
            scheduling_msg.status = 4
            scheduling_msg.message=""
            self.date_str = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            scheduling_msg.task_create_time = self.date_str
            self.task_scheduling_pub.publish(scheduling_msg)

            thread = threading.Thread(target=self._run_guide_person_task, args=(robot_namespace, dest_location, task_data))
            thread.start()
        elif task_name == 'reshelving_book':
            return_desk_id = task_data.get('return_desk_id')
            self.get_logger().info(f"Sending 'reshelving_book' goal for return_desk_id {return_desk_id} to robot '{robot_namespace}'")

            scheduling_msg = Scheduling()
            scheduling_msg.no = self.no 
            scheduling_msg.robot_name = robot_namespace
            scheduling_msg.task_id = task_name
            scheduling_msg.priority = 0
            scheduling_msg.status = 4
            scheduling_msg.message=""
            self.date_str = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            scheduling_msg.task_create_time = self.date_str
            self.task_scheduling_pub.publish(scheduling_msg)

            thread = threading.Thread(target=self._run_reshelving_book_task, args=(robot_namespace, return_desk_id, task_data))

            
            thread.start()
        elif task_name == 'kreacher':
            order_id = task_data.get('order_id')
            menu_id = task_data.get('menu_id')
            self.get_logger().info(f"Sending 'kreacher' goal for order_id {order_id} to robot '{robot_namespace}'")

            scheduling_msg = Scheduling()
            scheduling_msg.no = self.no 
            scheduling_msg.robot_name = robot_namespace
            scheduling_msg.task_id = task_name
            scheduling_msg.priority = 0
            scheduling_msg.status = 4
            scheduling_msg.message=""
            self.date_str = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            scheduling_msg.task_create_time = self.date_str
            self.task_scheduling_pub.publish(scheduling_msg)

            thread = threading.Thread(target=self._run_kreacher_task, args=(robot_namespace, order_id, menu_id, task_data))
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
            if result['success']:
                scheduling_msg = Scheduling()
                scheduling_msg.no = self.no 
                scheduling_msg.robot_name = robot_namespace 
                scheduling_msg.task_id = task_data.get('task_name', '') # result에서 가져오는 대신 task_data에서 가져옵니다.
                scheduling_msg.priority = 0
                scheduling_msg.status = 2  # 2: 작업 완료 (Completed)
                scheduling_msg.task_create_time = self.date_str
                scheduling_msg.message=result['message']
                self.task_scheduling_pub.publish(scheduling_msg)
            else:
                scheduling_msg = Scheduling()
                scheduling_msg.no = self.no 
                scheduling_msg.robot_name = robot_namespace 
                scheduling_msg.task_id = task_data.get('task_name', '') # result에서 가져오는 대신 task_data에서 가져옵니다.
                scheduling_msg.priority = 0
                scheduling_msg.status = 3  # 2: 작업 완료 (Completed)
                scheduling_msg.task_create_time = self.date_str
                scheduling_msg.message=result['message']
                self.task_scheduling_pub.publish(scheduling_msg)


        except Exception as e:
            self.get_logger().error(f"An error occurred during pickup_book task for '{robot_namespace}': {e}")
        finally:
            if pickup_book_node:
                pickup_book_node.destroy_node()
            temp_executor.shutdown()

    def _run_guide_person_task(self, robot_namespace, dest_location, task_data):
        temp_executor = SingleThreadedExecutor()
        guide_person_node = None
        try:
            guide_person_node = GuidePersonNode(namespace=f'{robot_namespace}/main')
            temp_executor.add_node(guide_person_node)

            kwargs = {k: v for k, v in task_data.items() if k != 'dest_location'}

            task_future = guide_person_node.run_task(dest_location, **kwargs)
            temp_executor.spin_until_future_complete(task_future)

            result = task_future.result()
            self.get_logger().info(f"Guide_person task for '{robot_namespace}' completed. Success : {result['success']}, Msg: '{result['message']}'")

            if result['success']:
                scheduling_msg = Scheduling()
                scheduling_msg.no = self.no 
                scheduling_msg.robot_name = robot_namespace 
                scheduling_msg.task_id = task_data.get('task_name', '') # result에서 가져오는 대신 task_data에서 가져옵니다.
                scheduling_msg.priority = 0
                scheduling_msg.status = 2  # 2: 작업 완료 (Completed)
                scheduling_msg.task_create_time = self.date_str
                scheduling_msg.message=result['message']
                self.task_scheduling_pub.publish(scheduling_msg)
            else:
                scheduling_msg = Scheduling()
                scheduling_msg.no = self.no 
                scheduling_msg.robot_name = robot_namespace 
                scheduling_msg.task_id = task_data.get('task_name', '') # result에서 가져오는 대신 task_data에서 가져옵니다.
                scheduling_msg.priority = 0
                scheduling_msg.status = 3  # 2: 작업 완료 (Completed)
                scheduling_msg.task_create_time = self.date_str
                scheduling_msg.message=result['message']
                self.task_scheduling_pub.publish(scheduling_msg)

        except Exception as e:
            self.get_logger().error(f"An error occurred during guide_person task for '{robot_namespace}': {e}")
        finally:
            if guide_person_node:
                guide_person_node.destroy_node()
            temp_executor.shutdown()

    def _run_reshelving_book_task(self, robot_namespace, return_desk_id, task_data):
        temp_executor = SingleThreadedExecutor()
        reshelving_book_node = None
        try:
            reshelving_book_node = ReshelvingBookNode(namespace=f'{robot_namespace}/main')
            temp_executor.add_node(reshelving_book_node)

            kwargs = {k: v for k, v in task_data.items() if k != 'dest_location'}

            task_future = reshelving_book_node.run_task(return_desk_id, **kwargs)
            temp_executor.spin_until_future_complete(task_future)

            result = task_future.result()
            self.get_logger().info(f"Reshelving book task for '{robot_namespace}' completed. Success : {result['success']}, Msg: '{result['message']}'")

            if result['success']:
                scheduling_msg = Scheduling()
                scheduling_msg.no = self.no 
                scheduling_msg.robot_name = robot_namespace 
                scheduling_msg.task_id = task_data.get('task_name', '') # result에서 가져오는 대신 task_data에서 가져옵니다.
                scheduling_msg.priority = 0
                scheduling_msg.status = 2  # 2: 작업 완료 (Completed)
                scheduling_msg.task_create_time = self.date_str
                scheduling_msg.message=result['message']
                self.task_scheduling_pub.publish(scheduling_msg)
            else:
                scheduling_msg = Scheduling()
                scheduling_msg.no = self.no 
                scheduling_msg.robot_name = robot_namespace 
                scheduling_msg.task_id = task_data.get('task_name', '') # result에서 가져오는 대신 task_data에서 가져옵니다.
                scheduling_msg.priority = 0
                scheduling_msg.status = 3  # 2: 작업 완료 (Completed)
                scheduling_msg.task_create_time = self.date_str
                scheduling_msg.message=result['message']
                self.task_scheduling_pub.publish(scheduling_msg)

        except Exception as e:
            self.get_logger().error(f"An error occurred during reshelving_book task for '{robot_namespace}': {e}")
        finally:
            if reshelving_book_node:
                reshelving_book_node.destroy_node()
            temp_executor.shutdown()
    def _run_kreacher_task(self, robot_namespace, order_id, menu_id, task_data):
        temp_executor = SingleThreadedExecutor()
        kreacher_node = None
        try:
            kreacher_node = KreacherNode(namespace=f'{robot_namespace}/action')
            temp_executor.add_node(kreacher_node)

            kwargs = {k: v for k, v in task_data.items() if k not in ['order_id', 'menu_id']}

            task_future = kreacher_node.run_task(order_id, menu_id, **kwargs)
            temp_executor.spin_until_future_complete(task_future)

            result = task_future.result()
            self.get_logger().info(f"Kreacher task for '{robot_namespace}' completed. Success : {result['success']}, Msg: '{result['message']}'")

            if result['success']:
                scheduling_msg = Scheduling()
                scheduling_msg.no = self.no 
                scheduling_msg.robot_name = robot_namespace 
                scheduling_msg.task_id = task_data.get('task_name', '') # result에서 가져오는 대신 task_data에서 가져옵니다.
                scheduling_msg.priority = 0
                scheduling_msg.status = 2  # 2: 작업 완료 (Completed)
                scheduling_msg.task_create_time = self.date_str
                scheduling_msg.message=result['message']
                self.task_scheduling_pub.publish(scheduling_msg)
            else:
                scheduling_msg = Scheduling()
                scheduling_msg.no = self.no 
                scheduling_msg.robot_name = robot_namespace 
                scheduling_msg.task_id = task_data.get('task_name', '') # result에서 가져오는 대신 task_data에서 가져옵니다.
                scheduling_msg.priority = 0
                scheduling_msg.status = 3  # 2: 작업 완료 (Completed)
                scheduling_msg.task_create_time = self.date_str
                scheduling_msg.message=result['message']
                self.task_scheduling_pub.publish(scheduling_msg)

        except Exception as e:
            self.get_logger().error(f"An error occurred during krecaher task for '{robot_namespace}': {e}")
        finally:
            if kreacher_node:
                kreacher_node.destroy_node()
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