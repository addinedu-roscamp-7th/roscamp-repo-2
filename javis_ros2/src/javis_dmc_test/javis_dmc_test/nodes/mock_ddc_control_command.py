#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Mock DDC ControlCommand Service Server

주행 제어 명령(control_command)을 시뮬레이션하는 Mock 서버입니다.
STOP, RESUME 등의 주행 제어 명령을 처리합니다.

Author: JAVIS Team
Date: 2025-10-27
"""

import rclpy

from javis_dmc_test.nodes.mock_server_base import MockServerBase
from javis_interfaces.srv import DriveControlCommand


class MockControlCommandService(MockServerBase):
    '''Mock 주행 제어 명령 Service Server
    
    주행 제어 명령(STOP, RESUME 등)을 처리합니다.
    
    Request:
    - command: 명령 문자열 ('STOP', 'RESUME')
    
    Response:
    - success: 성공 여부
    - message: 결과 메시지
    '''
    
    def __init__(self):
        super().__init__('mock_ddc_control_command')
        
        self._service = self.create_service(
            DriveControlCommand,
            'dobby1/drive/control_command',
            self._handle_control_command
        )
        
        self.get_logger().info('Mock DriveControlCommand Service ready')
    
    def _handle_control_command(self, request, response):
        '''DriveControlCommand 서비스 핸들러'''
        self.get_logger().info(f'DriveControlCommand 수신: {request.command}')
        
        if self.is_error():
            response.success = False
            response.current_state = f'Mock error 모드: {request.command} 실패'
            self.get_logger().warn(f'ControlCommand 실패: {request.command}')
        else:
            response.success = True
            response.current_state = f'{request.command} 명령 처리 완료'
            self.get_logger().info(f'ControlCommand 성공: {request.command}')
        
        return response


def main(args=None):
    '''메인 함수'''
    rclpy.init(args=args)
    
    node = MockControlCommandService()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
