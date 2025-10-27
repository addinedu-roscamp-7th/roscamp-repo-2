#!/usr/bin/env python3
"""
Test GUI Node
ROS 2 노드로 백그라운드에서 동작하며 PyQt6 GUI와 통신
"""

import sys
import threading
from queue import Queue

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter

from PyQt6.QtCore import QObject, pyqtSignal

from javis_dmc_test_msgs.msg import MockStatus
from rcl_interfaces.msg import Log
from javis_interfaces.srv import QueryLocationInfo, RequestGuidance, CreateUserGuide
from geometry_msgs.msg import Pose2D


class GuiNode(Node, QObject):
    """
    GUI를 위한 ROS 2 노드
    - Node를 먼저 상속 (super()가 Node.__init__()을 먼저 호출)
    - QObject를 상속하여 PyQt Signal/Slot 사용 가능
    """
    
    # PyQt Signals (백그라운드 스레드에서 GUI 스레드로 안전하게 데이터 전달)
    status_signal = pyqtSignal(str, str)  # (node_name, mode)
    log_signal = pyqtSignal(str, int, str, str)  # (name, level, msg, timestamp)
    query_location_signal = pyqtSignal(bool, str, float, float, float, str)  # (found, location_id, x, y, theta, message)
    request_guidance_signal = pyqtSignal(bool, str, str)  # (success, task_id, message)
    create_task_signal = pyqtSignal(bool, str, str)  # (success, task_id, message)
    
    def __init__(self):
        # 다중 상속: 두 부모 클래스를 모두 명시적으로 초기화
        Node.__init__(self, 'test_gui')
        QObject.__init__(self)
        
        self.get_logger().info('Test GUI Node started')
        
        # MockStatus 구독
        self.status_subscription = self.create_subscription(
            MockStatus,
            'dobby1/mock_system/status',
            self.status_callback,
            10
        )
        
        # /rosout 구독 (로그 메시지)
        self.log_subscription = self.create_subscription(
            Log,
            '/rosout',
            self.log_callback,
            50
        )
        
        # Service Clients 생성
        self.query_location_client = self.create_client(
            QueryLocationInfo,
            'dobby1/admin/query_location_info'
        )
        self.request_guidance_client = self.create_client(
            RequestGuidance,
            'dobby1/admin/request_guidance'
        )
        self.create_user_task_client = self.create_client(
            CreateUserGuide,
            '/rcs/create_user_task'
        )
        
        self.get_logger().info('Subscriptions and Service Clients created')
    
    def status_callback(self, msg: MockStatus):
        """
        MockStatus 토픽 콜백
        백그라운드 스레드에서 실행되므로 시그널을 통해 GUI로 전달
        """
        self.status_signal.emit(msg.node_name, msg.mode)
    
    def log_callback(self, msg: Log):
        """
        /rosout 토픽 콜백
        로그 메시지를 GUI로 전달
        """
        # 타임스탬프 변환
        timestamp = f"{msg.stamp.sec}.{msg.stamp.nanosec:09d}"
        self.log_signal.emit(msg.name, msg.level, msg.msg, timestamp)
    
    def set_mock_mode(self, node_name: str, mode: str):
        """
        특정 Mock 노드의 mode 파라미터 변경
        
        Args:
            node_name: Mock 노드 이름 (예: "mock_ddc_navigate_to_pose")
            mode: 변경할 모드 ("active", "error", "on", "off")
        """
        if not self.context.ok():
            self.get_logger().error('ROS 컨텍스트가 종료되어 Mock 모드를 변경할 수 없습니다.')
            return False

        try:
            from rclpy.parameter import Parameter
            from rcl_interfaces.srv import SetParameters
            
            # Parameter Service Client 생성
            client = self.create_client(SetParameters, f'/{node_name}/set_parameters')
            
            if not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error(f'Service not available: /{node_name}/set_parameters')
                return False
            
            # 파라미터 요청 생성
            request = SetParameters.Request()
            param = Parameter('mode', value=mode).to_parameter_msg()
            request.parameters = [param]
            
            # 비동기 호출
            future = client.call_async(request)
            
            # 콜백 등록
            def done_callback(future):
                try:
                    response = future.result()
                    if response.results and response.results[0].successful:
                        self.get_logger().info(f'Successfully set {node_name} mode to {mode}')
                    else:
                        self.get_logger().error(f'Failed to set {node_name} mode')
                except Exception as e:
                    self.get_logger().error(f'Exception in set_mock_mode: {e}')
            
            future.add_done_callback(done_callback)
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to set mode for {node_name}: {e}')
            return False
    
    def call_query_location_info(self, location_name: str):
        """
        QueryLocationInfo 서비스 호출
        
        Args:
            location_name: 조회할 위치 이름 (빈 문자열이면 전체 목록)
        """
        if not self.context.ok():
            self.get_logger().error('ROS 컨텍스트가 종료되어 위치 조회를 수행할 수 없습니다.')
            self.query_location_signal.emit(False, '', 0.0, 0.0, 0.0, 'ROS 컨텍스트가 종료되었습니다. GUI를 재시작하세요.')
            return
        if not self.query_location_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('QueryLocationInfo service not available')
            self.query_location_signal.emit(False, '', 0.0, 0.0, 0.0, 'Service not available')
            return
        
        request = QueryLocationInfo.Request()
        request.location_name = location_name
        
        future = self.query_location_client.call_async(request)
        
        def done_callback(future):
            try:
                response = future.result()
                if response.found:
                    self.get_logger().info(f'Location found: {response.location_id}')
                    self.query_location_signal.emit(
                        response.found,
                        response.location_id,
                        response.pose.x,
                        response.pose.y,
                        response.pose.theta,
                        response.message
                    )
                else:
                    self.get_logger().warn(f'Location not found: {location_name}')
                    self.query_location_signal.emit(False, '', 0.0, 0.0, 0.0, response.message)
            except Exception as e:
                self.get_logger().error(f'Exception in query_location_info callback: {e}')
                self.query_location_signal.emit(False, '', 0.0, 0.0, 0.0, str(e))
        
        future.add_done_callback(done_callback)
    
    def call_request_guidance(self, destination_name: str, dest_x: float, dest_y: float, dest_theta: float, source: str):
        """
        RequestGuidance 서비스 호출
        
        Args:
            destination_name: 목적지 이름
            dest_x, dest_y, dest_theta: 목적지 좌표
            source: 요청 출처 ("gui" or "voice")
        """
        if not self.context.ok():
            self.get_logger().error('ROS 컨텍스트가 종료되어 길안내 요청을 처리할 수 없습니다.')
            self.request_guidance_signal.emit(False, '', 'ROS 컨텍스트가 종료되었습니다. GUI를 재시작하세요.')
            return
        if not self.request_guidance_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('RequestGuidance service not available')
            self.request_guidance_signal.emit(False, '', 'Service not available')
            return
        
        request = RequestGuidance.Request()
        request.destination_name = destination_name
        request.dest_pose.x = dest_x
        request.dest_pose.y = dest_y
        request.dest_pose.theta = dest_theta
        request.request_source = source
        request.user_context = ''
        
        future = self.request_guidance_client.call_async(request)
        
        def done_callback(future):
            try:
                response = future.result()
                if response.success:
                    self.get_logger().info(f'Guidance request success: {response.task_id}')
                    self.request_guidance_signal.emit(response.success, response.task_id, response.message)
                else:
                    self.get_logger().warn(f'Guidance request failed: {response.message}')
                    self.request_guidance_signal.emit(False, '', response.message)
            except Exception as e:
                self.get_logger().error(f'Exception in request_guidance callback: {e}')
                self.request_guidance_signal.emit(False, '', str(e))
        
        future.add_done_callback(done_callback)
    
    def call_create_user_task(self, task_type: str, destination_name: str, dest_x: float, dest_y: float, dest_theta: float):
        """
        CreateUserTask 서비스 호출 (RCS Mock)
        
        Args:
            task_type: 작업 타입 (guide_person, pickup_book, etc.)
            destination_name: 목적지 이름
            dest_x, dest_y, dest_theta: 목적지 좌표
        """
        if not self.context.ok():
            self.get_logger().error('ROS 컨텍스트가 종료되어 작업 생성을 처리할 수 없습니다.')
            self.create_task_signal.emit(False, '', 'ROS 컨텍스트가 종료되었습니다. GUI를 재시작하세요.')
            return
        if not self.create_user_task_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('CreateUserTask service not available')
            self.create_task_signal.emit(False, '', 'Service not available')
            return
        
        request = CreateUserGuide.Request()
        request.task_type = task_type
        request.destination_name = destination_name
        request.dest_location = Pose2D()
        request.dest_location.x = dest_x
        request.dest_location.y = dest_y
        request.dest_location.theta = dest_theta
        request.user_initiated = True
        
        future = self.create_user_task_client.call_async(request)
        
        def done_callback(future):
            try:
                response = future.result()
                if response.success:
                    self.get_logger().info(f'Task creation success: {response.task_id} (type: {task_type})')
                    self.create_task_signal.emit(response.success, response.task_id, response.message)
                else:
                    self.get_logger().warn(f'Task creation failed: {response.message}')
                    self.create_task_signal.emit(False, '', response.message)
            except Exception as e:
                self.get_logger().error(f'Exception in create_user_task callback: {e}')
                self.create_task_signal.emit(False, '', str(e))
        
        future.add_done_callback(done_callback)


def spin_node(node, executor):
    """백그라운드 스레드에서 ROS 2 노드 실행"""
    try:
        executor.spin()
    except Exception as e:
        node.get_logger().error(f'Exception in spin_node: {e}')


def main(args=None):
    """
    메인 함수
    - ROS 2 노드를 백그라운드 스레드에서 실행
    - PyQt6 GUI를 메인 스레드에서 실행
    """
    # ROS 2 초기화
    rclpy.init(args=args)
    
    # GUI Node 생성
    gui_node = GuiNode()
    
    # MultiThreadedExecutor 생성
    executor = MultiThreadedExecutor()
    executor.add_node(gui_node)
    
    # ROS 2 노드를 백그라운드 스레드에서 실행
    spin_thread = threading.Thread(target=spin_node, args=(gui_node, executor), daemon=True)
    spin_thread.start()
    
    # PyQt6 애플리케이션 생성 및 메인 윈도우 실행
    from PyQt6.QtWidgets import QApplication
    from .main_window import MainWindow
    
    app = QApplication(sys.argv)
    window = MainWindow(gui_node)
    window.show()
    
    # GUI 메인 루프 실행 (메인 스레드)
    exit_code = app.exec()
    
    # 종료 처리
    gui_node.get_logger().info('Shutting down GUI...')
    executor.shutdown()
    gui_node.destroy_node()
    rclpy.shutdown()
    
    spin_thread.join(timeout=2.0)
    
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
