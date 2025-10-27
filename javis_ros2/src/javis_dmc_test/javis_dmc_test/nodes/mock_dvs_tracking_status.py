#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Mock DVS TrackingStatus Publisher

추적 상태(tracking_status)를 주기적으로 발행하는 Mock 서버입니다.
피안내자 추적 상태를 시뮬레이션합니다.

Author: JAVIS Team
Date: 2025-10-27
"""

import rclpy
from rclpy.parameter import Parameter

from javis_dmc_test.nodes.mock_server_base import MockServerBase
from javis_interfaces.msg import TrackingStatus


class MockTrackingStatusPublisher(MockServerBase):
    '''Mock 추적 상태 Publisher
    
    피안내자 추적 상태를 주기적으로 발행합니다.
    
    mode='on': 1초마다 TrackingStatus 발행
    mode='off': 발행 중지
    '''
    
    def __init__(self):
        # 부모 클래스 초기화 (기본 mode='active')
        super().__init__('mock_dvs_tracking_status')
        
        # TrackingStatus Publisher 생성
        self._publisher = self.create_publisher(
            TrackingStatus,
            'dobby1/ai/tracking/status',
            10
        )
        
        # 타이머 생성 (처음에는 중지)
        self._timer = self.create_timer(1.0, self._timer_callback)
        self._timer.cancel()
        
        # mode 파라미터를 'off'로 변경 (이때 on_mode_changed 호출됨)
        self.set_parameters([Parameter('mode', Parameter.Type.STRING, 'off')])
        
        self.get_logger().info('TrackingStatus Publisher 대기 (off 모드)')
    
    def on_mode_changed(self, old_mode: str, new_mode: str):
        '''모드 변경 시 호출되는 콜백 (부모 클래스 오버라이드)'''
        if new_mode == 'on':
            self._timer.reset()
            self.get_logger().info('TrackingStatus Publisher 시작')
        else:
            self._timer.cancel()
            self.get_logger().info('TrackingStatus Publisher 중지')
    
    def _timer_callback(self):
        '''타이머 콜백 - TrackingStatus 발행'''
        msg = TrackingStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.person_detected = True
        msg.tracking_id = 'mock_user_123'
        msg.confidence = 0.95
        
        self._publisher.publish(msg)
        
        # 5초마다 로깅 (너무 많은 로그 방지)
        if self.get_clock().now().nanoseconds % 5000000000 < 1000000000:
            self.get_logger().info(
                f'TrackingStatus 발행: person_detected={msg.person_detected}, '
                f'tracking_id={msg.tracking_id}'
            )


def main(args=None):
    '''메인 함수'''
    rclpy.init(args=args)
    
    node = MockTrackingStatusPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
