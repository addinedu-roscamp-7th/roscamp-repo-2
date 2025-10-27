#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Mock DDC GuideNavigation Action Server

사람 추종 주행(guide_navigation)을 시뮬레이션하는 Mock 서버입니다.
길안내 시 피안내자를 따라가는 주행을 시뮬레이션합니다.

Author: JAVIS Team
Date: 2025-10-27
"""

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
import time

from javis_dmc_test.nodes.mock_server_base import MockServerBase
from javis_interfaces.action import GuideNavigation


class MockGuideNavigationAction(MockServerBase):
    '''Mock 사람 추종 주행 Action Server
    
    길안내 시 피안내자를 따라가는 주행을 시뮬레이션합니다.
    
    Feedback:
    - distance_remaining: 목적지까지 남은 거리 (10.0 → 0.0)
    - person_detected: 피안내자 감지 여부
    
    Result:
    - success: 성공 여부 (mode에 따라 결정)
    - message: 결과 메시지
    '''
    
    def __init__(self):
        super().__init__('mock_ddc_guide_navigation')
        
        self._action_server = ActionServer(
            self,
            GuideNavigation,
            'dobby1/drive/guide_navigation',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback
        )
        
        self.get_logger().info('Mock GuideNavigation Action ready')
    
    def _goal_callback(self, goal_request):
        '''Goal 수신 시 호출'''
        self.get_logger().info(
            f'GuideNavigation Goal 수신: '
            f'dest=({goal_request.dest_location.x:.1f}, '
            f'{goal_request.dest_location.y:.1f})'
        )
        return GoalResponse.ACCEPT
    
    def _cancel_callback(self, goal_handle):
        '''Cancel 요청 시 호출'''
        self.get_logger().info('GuideNavigation Cancel 요청')
        return CancelResponse.ACCEPT
    
    def _execute_callback(self, goal_handle):
        '''Action 실행 콜백'''
        self.get_logger().info('GuideNavigation 시작')
        
        # Feedback 메시지 생성
        feedback = GuideNavigation.Feedback()
        
        # 초기 거리 10.0m에서 시작
        distances = [10.0, 7.5, 5.0, 2.5, 0.0]
        
        for distance in distances:
            # 취소 요청 확인
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = GuideNavigation.Result()
                result.success = False
                result.message = '사용자에 의해 취소됨'
                self.get_logger().info('GuideNavigation 취소됨')
                return result
            
            # Feedback 업데이트
            feedback.distance_remaining = distance
            feedback.person_detected = True
            
            goal_handle.publish_feedback(feedback)
            
            self.get_logger().info(
                f'GuideNavigation 진행 중: 남은 거리 {distance:.1f}m'
            )
            
            time.sleep(1.0)  # 1초 간격
        
        # Result 생성
        result = GuideNavigation.Result()
        
        if self.is_active():
            goal_handle.succeed()
            result.success = True
            result.message = '목적지 도착 완료'
            result.total_distance_m = 10.0
            result.total_time_sec = 5.0
            self.get_logger().info('GuideNavigation 성공')
        else:
            goal_handle.abort()
            result.success = False
            result.message = 'Mock error 모드로 인한 실패'
            result.total_distance_m = 5.0
            result.total_time_sec = 2.5
            self.get_logger().warn('GuideNavigation 실패 (error 모드)')
        
        return result


def main(args=None):
    '''메인 함수'''
    rclpy.init(args=args)
    
    node = MockGuideNavigationAction()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
