#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Mock GUI RequestGuidance Service Server

DMC가 GUI 인터페이스를 통해 길안내를 요청하는 서비스를 시뮬레이션합니다.
실제로는 DMC가 이 서비스를 제공하지만, Mock 테스트를 위해 구현합니다.

Author: JAVIS Team
Date: 2025-10-27
"""

import rclpy
from datetime import datetime

from javis_dmc_test.nodes.mock_server_base import MockServerBase
from javis_interfaces.srv import RequestGuidance


class MockRequestGuidanceService(MockServerBase):
    '''Mock 길안내 요청 Service Server
    
    GUI → DMC 길안내 요청을 시뮬레이션합니다.
    
    Request:
    - destination_name: 목적지 이름
    - dest_pose: 목적지 좌표
    - request_source: 요청 출처 ("gui" or "voice")
    
    Response:
    - success: 성공 여부
    - task_id: 생성된 작업 ID
    - message: 결과 메시지
    '''
    
    def __init__(self):
        super().__init__('mock_gui_request_guidance')
        
        # Service Server 생성
        self._service = self.create_service(
            RequestGuidance,
            'dobby1/admin/request_guidance',
            self._handle_request
        )
        
        self._guidance_count = 0
        
        self.get_logger().info('Mock RequestGuidance Service ready')
    
    def _handle_request(self, request, response):
        '''RequestGuidance 서비스 핸들러'''
        dest_name = request.destination_name
        source = request.request_source
        pose = request.dest_pose
        
        self.get_logger().info(
            f'RequestGuidance 요청: '
            f'목적지={dest_name}, '
            f'좌표=({pose.x:.1f}, {pose.y:.1f}), '
            f'출처={source}'
        )
        
        # error 모드 체크
        if self.is_error():
            response.success = False
            response.task_id = ''
            response.message = 'Mock error 모드: 길안내 요청 실패'
            self.get_logger().warn('RequestGuidance 실패 (error 모드)')
            return response
        
        # 배터리 체크 시뮬레이션 (Mock에서는 항상 통과)
        battery_level = 80.0  # Mock 배터리
        if battery_level < 40.0:
            response.success = False
            response.task_id = ''
            response.message = f'배터리 부족 (현재: {battery_level:.0f}%, 필요: 40%)'
            self.get_logger().warn(f'RequestGuidance 실패: 배터리 부족 ({battery_level}%)')
            return response
        
        # task_id 생성
        self._guidance_count += 1
        task_id = f'guidance_{datetime.now().strftime("%Y%m%d_%H%M%S")}_{self._guidance_count}'
        
        # 성공 응답
        response.success = True
        response.task_id = task_id
        response.message = f'{dest_name}로 길안내를 시작합니다'
        
        self.get_logger().info(
            f'RequestGuidance 성공: '
            f'task_id={task_id}, '
            f'출처={source}'
        )
        
        # v4.0 플로우 로깅
        if source == 'gui':
            self.get_logger().info(
                f'[v4.0 GUI 플로우] '
                f'WAITING_DEST_INPUT → GUIDING (60초 타이머 취소)'
            )
        elif source == 'voice':
            self.get_logger().info(
                f'[v4.0 VRC 플로우] '
                f'LISTENING → GUIDING (20초 타이머 취소)'
            )
        
        return response


def main(args=None):
    '''메인 함수'''
    rclpy.init(args=args)
    
    node = MockRequestGuidanceService()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
