#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Mock DDC NavigateToPose Action Server

NAV2의 NavigateToPose 액션을 시뮬레이션하는 Mock 서버입니다.
지정된 PoseStamped 목표까지 이동하는 상황을 모사합니다.

Author: JAVIS Team
Date: 2025-10-27
"""

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
import time

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from javis_dmc_test.nodes.mock_server_base import MockServerBase


class MockNavigateToPoseAction(MockServerBase):
    '''Mock NAV2 NavigateToPose Action Server
    
    지정된 PoseStamped 목표까지 이동하는 동작을 시뮬레이션한다.
    
    Feedback:
    - current_pose: 현재 추정 위치
    - distance_remaining: 남은 거리
    - navigation_time: 경과 시간
    - estimated_time_remaining: 예상 남은 시간
    
    Result:
    - error_code: 0이면 성공, 이외에는 실패
    - error_msg : 실패 사유
    '''
    
    def __init__(self):
        super().__init__('mock_ddc_navigate_to_pose')
        
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'dobby1/drive/navigate_to_pose',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback
        )
        
        self.get_logger().info('Mock NavigateToPose Action ready')
    
    def _goal_callback(self, goal_request: NavigateToPose.Goal):
        '''Goal 수신 시 호출'''
        target = goal_request.pose.pose.position
        self.get_logger().info(
            f'NavigateToPose Goal 수신: '
            f'({target.x:.1f}, {target.y:.1f})'
        )
        return GoalResponse.ACCEPT
    
    def _cancel_callback(self, goal_handle):
        '''Cancel 요청 시 호출'''
        self.get_logger().info('NavigateToPose Cancel 요청')
        return CancelResponse.ACCEPT
    
    def _execute_callback(self, goal_handle):
        '''Action 실행 콜백'''
        self.get_logger().info('NavigateToPose 시작')
        
        feedback = NavigateToPose.Feedback()
        feedback.current_pose = PoseStamped()
        total_steps = 5
        for step in range(total_steps):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = NavigateToPose.Result()
                result.error_code = 1
                result.error_msg = '취소됨'
                self.get_logger().info('NavigateToPose 취소됨')
                return result
            
            progress_ratio = step / (total_steps - 1)
            remaining = max(0.0, 10.0 * (1.0 - progress_ratio))
            now = self.get_clock().now()
            
            feedback.current_pose = goal_handle.request.pose
            feedback.current_pose.header.stamp = now.to_msg()
            feedback.navigation_time = Duration(sec=int(progress_ratio * 20))
            feedback.estimated_time_remaining = Duration(sec=int(20 - progress_ratio * 20))
            feedback.number_of_recoveries = 0
            feedback.distance_remaining = remaining
            
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(
                f'NavigateToPose 진행: {progress_ratio * 100:.0f}% '
                f'(남은 거리: {remaining:.2f}m)'
            )
            time.sleep(1.0)
        
        result = NavigateToPose.Result()
        
        if self.is_active():
            goal_handle.succeed()
            result.error_code = NavigateToPose.Result.NONE
            result.error_msg = ''
            self.get_logger().info('NavigateToPose 성공')
        else:
            goal_handle.abort()
            result.error_code = 1
            result.error_msg = 'Mock error 모드'
            self.get_logger().warn('NavigateToPose 실패 (error 모드)')
        
        return result


def main(args=None):
    '''메인 함수'''
    rclpy.init(args=args)
    
    node = MockNavigateToPoseAction()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
