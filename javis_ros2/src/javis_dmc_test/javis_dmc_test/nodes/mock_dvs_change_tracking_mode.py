#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Mock DVS ChangeTrackingMode Service Server

추적 모드 전환(change_tracking_mode)을 시뮬레이션하는 Mock 서버입니다.
피안내자 등록(registration) 및 추적(tracking) 모드를 전환합니다.

Author: JAVIS Team
Date: 2025-10-27
"""

import rclpy

from javis_dmc_test.nodes.mock_server_base import MockServerBase
from javis_interfaces.srv import ChangeTrackingMode


class MockChangeTrackingModeService(MockServerBase):
    '''Mock 추적 모드 전환 Service Server
    
    피안내자 등록 및 추적 모드를 전환하는 서비스를 시뮬레이션합니다.
    
    Request:
    - mode_name: 모드 이름 ('registration', 'tracking', 'idle')
    
    Response:
    - success: 성공 여부
    - message: 결과 메시지
    '''
    
    def __init__(self):
        super().__init__('mock_dvs_change_tracking_mode')
        
        self._service = self.create_service(
            ChangeTrackingMode,
            'dobby1/ai/change_tracking_mode',
            self._handle_change_mode
        )
        
        self._current_tracking_mode = 'idle'
        
        self.get_logger().info('Mock ChangeTrackingMode Service ready')
    
    def _handle_change_mode(self, request, response):
        '''ChangeTrackingMode 서비스 핸들러'''
        mode = request.mode_name
        
        self.get_logger().info(
            f'ChangeTrackingMode 요청: {self._current_tracking_mode} → {mode}'
        )
        
        if self.is_error():
            response.success = False
            response.message = f'Mock error 모드: {mode} 전환 실패'
            self.get_logger().warn(f'ChangeTrackingMode 실패: {mode}')
        else:
            self._current_tracking_mode = mode
            response.success = True
            response.message = f'{mode} 모드로 전환 완료'
            self.get_logger().info(f'ChangeTrackingMode 성공: {mode}')
        
        return response


def main(args=None):
    '''메인 함수'''
    rclpy.init(args=args)
    
    node = MockChangeTrackingModeService()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
