#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Mock VRC SetListeningMode Service Server

DMC가 VRC(음성 인식 컨트롤러)의 리스닝 모드를 제어하는 서비스를 시뮬레이션합니다.
Wake Word 감지 후 LISTENING 상태 진입/해제를 제어합니다.

Author: JAVIS Team
Date: 2025-10-27
"""

import rclpy
from std_srvs.srv import SetBool

from javis_dmc_test.nodes.mock_server_base import MockServerBase


class MockSetListeningModeService(MockServerBase):
    '''Mock VRC SetListeningMode Service Server
    
    DMC → VRC 리스닝 모드 제어를 시뮬레이션합니다.
    
    Request:
    - data: True (LISTENING 시작), False (LISTENING 종료)
    
    Response:
    - success: 성공 여부
    - message: 결과 메시지
    '''
    
    def __init__(self):
        super().__init__('mock_vrc_set_listening_mode')
        
        # Service Server 생성
        self._service = self.create_service(
            SetBool,
            'voice_recognition_controller/set_listening_mode',
            self._handle_set_listening_mode
        )
        
        self._is_listening = False
        
        self.get_logger().info('Mock SetListeningMode Service ready')
    
    def _handle_set_listening_mode(self, request, response):
        '''SetListeningMode 서비스 핸들러'''
        enable = request.data
        
        self.get_logger().info(
            f'SetListeningMode 요청: '
            f'{"LISTENING 시작" if enable else "LISTENING 종료"}'
        )
        
        # error 모드 체크
        if self.is_error():
            response.success = False
            response.message = 'Mock error 모드: 리스닝 모드 전환 실패'
            self.get_logger().warn('SetListeningMode 실패 (error 모드)')
            return response
        
        # 리스닝 모드 변경
        old_state = self._is_listening
        self._is_listening = enable
        
        if enable:
            # LISTENING 시작
            response.success = True
            response.message = '음성 리스닝 모드 활성화 (20초 대기)'
            
            self.get_logger().info(
                '[v4.0 VRC 플로우] '
                'Wake Word 감지 → LISTENING 모드 활성화 → 20초 타이머 시작'
            )
        else:
            # LISTENING 종료
            response.success = True
            response.message = '음성 리스닝 모드 비활성화'
            
            self.get_logger().info(
                '[v4.0 VRC 플로우] '
                'LISTENING 종료 → 오디오 스트림 중단'
            )
        
        # 상태 변화 로깅
        if old_state != self._is_listening:
            self.get_logger().info(
                f'VRC 상태 전환: '
                f'{"OFF → ON" if enable else "ON → OFF"}'
            )
        
        return response


def main(args=None):
    '''메인 함수'''
    rclpy.init(args=args)
    
    node = MockSetListeningModeService()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
