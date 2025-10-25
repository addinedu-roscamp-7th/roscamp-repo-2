'''Mock Bridge Node - DMC의 서비스/액션 호출을 가로채서 Mock 응답 반환.'''

import time
from typing import Any, Dict

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger

from javis_interfaces.action import MoveToTarget, PickBook, PlaceBook
from javis_interfaces.srv import DriveControlCommand, ChangeArmPose, ChangeTrackingMode
from javis_dmc_test_msgs.srv import SetMockMethod


class MockBridgeNode(Node):
    '''개별 서비스/액션별 Mock 응답을 제어하는 브릿지 노드.'''

    def __init__(self):
        super().__init__('mock_bridge_node')

        self.declare_parameter('robot_namespace', 'dobby1')
        self.namespace = self.get_parameter('robot_namespace').value

        # Mock 응답 설정 (메서드별)
        self.mock_config: Dict[str, Dict[str, Any]] = {
            'drive': {
                'move_to_target': {'enabled': False, 'success': True, 'delay': 0.0},
                'stop': {'enabled': False, 'success': True, 'delay': 0.0},
                'resume': {'enabled': False, 'success': True, 'delay': 0.0},
                'start_patrol': {'enabled': False, 'success': True, 'delay': 1.0},
            },
            'arm': {
                'pick_book': {'enabled': False, 'success': True, 'delay': 3.0},
                'place_book': {'enabled': False, 'success': True, 'delay': 2.0},
                'change_pose': {'enabled': False, 'success': True, 'delay': 0.5},
            },
            'ai': {
                'detect_book': {'enabled': False, 'success': True, 'delay': 0.5},
                'change_tracking_mode': {'enabled': False, 'success': True, 'delay': 0.0},
            },
        }

        # 제어 서비스 (TEST_GUI가 호출)
        self.create_service(
            SetMockMethod,
            '/test/mock_bridge/set_method',
            self._on_set_mock_method,
        )

        # Mock Action Servers
        self._create_mock_actions()

        # Mock Service Servers
        self._create_mock_services()

        self.get_logger().info(f'Mock Bridge 노드 초기화 완료 (namespace: {self.namespace})')

    def _create_mock_actions(self):
        '''DMC가 호출하는 액션을 Mock으로 대체.'''

        # drive/move_to_target 액션
        self.move_to_target_server = ActionServer(
            self,
            MoveToTarget,
            f'/{self.namespace}/drive/move_to_target',
            execute_callback=self._execute_move_to_target,
        )

        # arm/pick_book 액션
        self.pick_book_server = ActionServer(
            self,
            PickBook,
            f'/{self.namespace}/arm/pick_book',
            execute_callback=self._execute_pick_book,
        )

        # arm/place_book 액션
        self.place_book_server = ActionServer(
            self,
            PlaceBook,
            f'/{self.namespace}/arm/place_book',
            execute_callback=self._execute_place_book,
        )

        self.get_logger().info('Mock Action Servers 생성 완료')

    def _create_mock_services(self):
        '''DMC가 호출하는 서비스를 Mock으로 대체.'''

        # drive/control_command 서비스 (stop, resume 등)
        self.drive_control_service = self.create_service(
            DriveControlCommand,
            f'/{self.namespace}/drive/control_command',
            self._handle_drive_control,
        )

        # arm/change_pose 서비스
        self.arm_change_pose_service = self.create_service(
            ChangeArmPose,
            f'/{self.namespace}/arm/change_pose',
            self._handle_arm_change_pose,
        )

        # ai/change_tracking_mode 서비스
        self.ai_tracking_service = self.create_service(
            ChangeTrackingMode,
            f'/{self.namespace}/ai/change_tracking_mode',
            self._handle_ai_tracking,
        )

        self.get_logger().info('Mock Service Servers 생성 완료')

    def _execute_move_to_target(self, goal_handle):
        '''move_to_target 액션의 Mock 실행.'''
        config = self.mock_config['drive']['move_to_target']

        if not config['enabled']:
            self.get_logger().warn('move_to_target Mock 비활성화 - 실제 서비스로 전달 필요')
            result = MoveToTarget.Result()
            result.success = False
            result.message = 'Mock disabled but no real service available'
            goal_handle.abort()
            return result

        goal = goal_handle.request
        self.get_logger().info(f'[Mock] move_to_target: {goal.location_name}')

        # 지연 시간 시뮬레이션
        delay = config.get('delay', 0.0)
        if delay > 0:
            for _ in range(int(delay * 10)):
                time.sleep(0.1)
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result = MoveToTarget.Result()
                    result.success = False
                    result.message = 'Canceled'
                    return result

        # 결과 반환
        result = MoveToTarget.Result()
        result.success = config['success']
        result.message = f'Mock response (success={config["success"]})'

        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return result

    def _execute_pick_book(self, goal_handle):
        '''pick_book 액션의 Mock 실행.'''
        config = self.mock_config['arm']['pick_book']

        if not config['enabled']:
            result = PickBook.Result()
            result.success = False
            result.message = 'Mock disabled'
            goal_handle.abort()
            return result

        goal = goal_handle.request
        self.get_logger().info(f'[Mock] pick_book: {goal.book_id}')

        # 지연 시간 시뮬레이션
        time.sleep(config.get('delay', 0.0))

        # 결과 반환
        result = PickBook.Result()
        result.success = config['success']
        result.book_id = goal.book_id
        result.message = f'Mock response (success={config["success"]})'

        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return result

    def _execute_place_book(self, goal_handle):
        '''place_book 액션의 Mock 실행.'''
        config = self.mock_config['arm']['place_book']

        if not config['enabled']:
            result = PlaceBook.Result()
            result.success = False
            result.message = 'Mock disabled'
            goal_handle.abort()
            return result

        goal = goal_handle.request
        self.get_logger().info(f'[Mock] place_book: {goal.book_id}')

        time.sleep(config.get('delay', 0.0))

        result = PlaceBook.Result()
        result.success = config['success']
        result.book_id = goal.book_id
        result.message = f'Mock response (success={config["success"]})'

        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return result

    def _handle_drive_control(self, request, response):
        '''drive control 서비스의 Mock 처리.'''
        command = request.command.lower()  # 'stop', 'resume' 등

        config = self.mock_config['drive'].get(command, {'enabled': False})

        if not config['enabled']:
            response.success = False
            response.message = f'Mock for {command} is disabled'
            return response

        self.get_logger().info(f'[Mock] drive/{command}')

        response.success = config['success']
        response.message = f'Mock response for {command}'
        return response

    def _handle_arm_change_pose(self, request, response):
        '''arm change_pose 서비스의 Mock 처리.'''
        config = self.mock_config['arm']['change_pose']

        if not config['enabled']:
            response.success = False
            response.message = 'Mock disabled'
            return response

        self.get_logger().info(f'[Mock] arm/change_pose: {request.pose_type}')

        time.sleep(config.get('delay', 0.0))

        response.success = config['success']
        response.message = f'Pose changed to {request.pose_type} (Mock)'
        return response

    def _handle_ai_tracking(self, request, response):
        '''ai change_tracking_mode 서비스의 Mock 처리.'''
        config = self.mock_config['ai']['change_tracking_mode']

        if not config['enabled']:
            response.success = False
            response.message = 'Mock disabled'
            return response

        self.get_logger().info(f'[Mock] ai/change_tracking_mode: {request.mode}')

        response.success = config['success']
        response.message = f'Tracking mode changed to {request.mode} (Mock)'
        return response

    def _on_set_mock_method(self, request, response):
        '''TEST_GUI로부터 Mock 설정 변경.'''
        interface = request.interface.lower()
        method = request.method.lower()
        enabled = request.enabled
        success = request.success
        delay = request.delay

        if interface not in self.mock_config:
            response.success = False
            response.message = f'Unknown interface: {interface}'
            return response

        if method not in self.mock_config[interface]:
            response.success = False
            response.message = f'Unknown method: {interface}.{method}'
            return response

        # Mock 설정 업데이트
        self.mock_config[interface][method] = {
            'enabled': enabled,
            'success': success,
            'delay': delay,
        }

        response.success = True
        response.message = f'Mock {interface}.{method} updated: enabled={enabled}, success={success}, delay={delay}s'
        self.get_logger().info(response.message)

        return response


def main(args=None):
    '''Mock Bridge 노드 실행 엔트리 포인트.'''
    rclpy.init(args=args)
    node = MockBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('사용자 중단')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
