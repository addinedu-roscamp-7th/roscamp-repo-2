#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Mock DDC MoveToTarget Action Server

목표 위치 이동(move_to_target)을 시뮬레이션하는 Mock 서버입니다.
지정된 목표 위치로 이동하는 것을 시뮬레이션합니다.

Author: JAVIS Team
Date: 2025-10-27
"""

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
import time

from javis_dmc_test.nodes.mock_server_base import MockServerBase
from javis_interfaces.action import MoveToTarget


class MockMoveToTargetAction(MockServerBase):
    '''Mock 목표 위치 이동 Action Server
    
    지정된 목표 위치로 이동하는 것을 시뮬레이션합니다.
    
    Feedback:
    - progress_percent: 진행률 (0 → 100)
    
    Result:
    - success: 성공 여부
    - message: 결과 메시지
    '''
    
    def __init__(self):
        super().__init__('mock_ddc_move_to_target')
        
        self._action_server = ActionServer(
            self,
            MoveToTarget,
            'dobby1/drive/move_to_target',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback
        )
        
        self.get_logger().info('Mock MoveToTarget Action ready')
    
    def _goal_callback(self, goal_request):
        '''Goal 수신 시 호출'''
        self.get_logger().info(
            f'MoveToTarget Goal 수신: '
            f'target=({goal_request.target_location.x:.1f}, '
            f'{goal_request.target_location.y:.1f})'
        )
        return GoalResponse.ACCEPT
    
    def _cancel_callback(self, goal_handle):
        '''Cancel 요청 시 호출'''
        self.get_logger().info('MoveToTarget Cancel 요청')
        return CancelResponse.ACCEPT
    
    def _execute_callback(self, goal_handle):
        '''Action 실행 콜백'''
        self.get_logger().info('MoveToTarget 시작')
        
        feedback = MoveToTarget.Feedback()
        
        # 진행률: 0% → 20% → 50% → 80% → 100%
        progress_steps = [0, 20, 50, 80, 100]
        
        for progress in progress_steps:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = MoveToTarget.Result()
                result.success = False
                result.message = '취소됨'
                self.get_logger().info('MoveToTarget 취소됨')
                return result
            
            feedback.progress_percent = progress
            goal_handle.publish_feedback(feedback)
            
            self.get_logger().info(f'MoveToTarget 진행: {progress}%')
            
            time.sleep(1.0)
        
        result = MoveToTarget.Result()
        
        if self.is_active():
            goal_handle.succeed()
            result.success = True
            result.message = '목표 위치 도착'
            self.get_logger().info('MoveToTarget 성공')
        else:
            goal_handle.abort()
            result.success = False
            result.message = 'Mock error 모드'
            self.get_logger().warn('MoveToTarget 실패 (error 모드)')
        
        return result


def main(args=None):
    '''메인 함수'''
    rclpy.init(args=args)
    
    node = MockMoveToTargetAction()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
