#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Mock Server Base Class

모든 Mock 서버의 공통 기능을 제공하는 베이스 클래스입니다.
- mode 파라미터 관리 (active, error, on, off)
- 파라미터 변경 콜백
- MockStatus 토픽 발행

Author: JAVIS Team
Date: 2025-10-27
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor
from javis_dmc_test_msgs.msg import MockStatus


class MockServerBase(Node):
    '''모든 Mock 서버의 베이스 클래스
    
    주요 기능:
    - mode 파라미터 관리 (active, error, on, off)
    - 파라미터 변경 시 MockStatus 토픽 발행
    - 자식 클래스에서 on_mode_changed() 오버라이드 가능
    '''
    
    def __init__(self, node_name: str, default_mode: str = 'active'):
        '''초기화
        
        Args:
            node_name: 노드 이름
            default_mode: 기본 모드 (active, error, on, off)
        '''
        super().__init__(node_name)
        
        # mode 파라미터 선언
        self.declare_parameter(
            'mode',
            default_mode,
            descriptor=ParameterDescriptor(
                description='Mock mode: active, error, on, off'
            )
        )
        
        # 현재 mode 값 저장
        self._mode = self.get_parameter('mode').get_parameter_value().string_value
        
        # 파라미터 변경 콜백 등록
        self.add_on_set_parameters_callback(self._parameter_callback)
        
        # MockStatus 퍼블리셔 생성
        self._status_publisher = self.create_publisher(
            MockStatus,
            'dobby1/mock_system/status',
            10
        )
        
        # 주기적 상태 발행 타이머 (1초마다)
        self._status_timer = self.create_timer(
            5.0,  # 1초 주기
            self._publish_status
        )
        
        self.get_logger().info(
            f'Mock Server [{self.get_name()}] started. Initial mode: {self._mode}'
        )
        
        # 초기 상태 발행
        self._publish_status()
    
    @property
    def mode(self) -> str:
        '''현재 mode 값 반환'''
        return self._mode
    
    def _parameter_callback(self, params: list) -> SetParametersResult:
        '''파라미터 변경 콜백
        
        mode 파라미터가 변경되면:
        1. 내부 _mode 변수 업데이트
        2. MockStatus 토픽 발행
        3. on_mode_changed() 호출 (자식 클래스 오버라이드 가능)
        '''
        for param in params:
            if param.name == 'mode':
                old_mode = self._mode
                self._mode = param.value
                
                self.get_logger().info(
                    f'Mode changed: {old_mode} → {self._mode}'
                )
                
                # 상태 발행
                self._publish_status()
                
                # 자식 클래스 콜백 호출
                self.on_mode_changed(old_mode, self._mode)
                
                return SetParametersResult(successful=True)
        
        return SetParametersResult(successful=True)
    
    def _publish_status(self):
        '''현재 노드 이름과 모드를 MockStatus 토픽으로 발행'''
        msg = MockStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.node_name = self.get_name()
        msg.mode = self._mode
        
        self._status_publisher.publish(msg)
    
    def on_mode_changed(self, old_mode: str, new_mode: str):
        '''모드 변경 시 호출되는 콜백 (자식 클래스 오버라이드 가능)
        
        Args:
            old_mode: 이전 모드
            new_mode: 새 모드
        '''
        pass
    
    def is_active(self) -> bool:
        '''현재 모드가 active인지 확인'''
        return self._mode == 'active'
    
    def is_error(self) -> bool:
        '''현재 모드가 error인지 확인'''
        return self._mode == 'error'
    
    def is_on(self) -> bool:
        '''현재 모드가 on인지 확인 (Topic Publisher용)'''
        return self._mode == 'on'
    
    def is_off(self) -> bool:
        '''현재 모드가 off인지 확인 (Topic Publisher용)'''
        return self._mode == 'off'


def main():
    '''테스트용 메인 함수'''
    rclpy.init()
    
    # 테스트: MockServerBase 직접 생성
    node = MockServerBase('test_mock_server')
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
