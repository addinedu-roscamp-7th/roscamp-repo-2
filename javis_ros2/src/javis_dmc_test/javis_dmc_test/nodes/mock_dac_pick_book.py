#!/usr/bin/env python3
"""
Mock DAC PickBook Action Server
책 픽업 액션 Mock
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
import time

from javis_interfaces.action import PickBook
from javis_dmc_test.nodes.mock_server_base import MockServerBase


class MockPickBookServer(MockServerBase):
    """책 픽업 액션 Mock 서버"""
    
    def __init__(self):
        super().__init__('mock_dac_pick_book')
        
        # Action Server 생성
        self._action_server = ActionServer(
            self,
            PickBook,
            'dobby1/arm/pick_book',
            self.execute_callback
        )
        
        self.get_logger().info('Mock DAC PickBook Action Server ready')
    
    def execute_callback(self, goal_handle: ServerGoalHandle):
        """
        책 픽업 액션 실행 핸들러
        
        Goal:
            string book_id  # 픽업할 책 ID
            geometry_msgs/Pose pick_pose  # 픽업 위치
        
        Result:
            bool success
            string message
        """
        goal = goal_handle.request
        self.get_logger().info(f'PickBook Action called - book_id: {goal.book_id}, mode: {self.mode}')
        
        # Feedback 발행 (진행률 시뮬레이션)
        feedback_msg = PickBook.Feedback()
        
        for i in range(1, 4):
            if not goal_handle.is_active:
                self.get_logger().warn('PickBook Action canceled')
                cancel_result = PickBook.Result()
                cancel_result.book_id = goal.book_id
                cancel_result.success = False
                cancel_result.message = 'Canceled'
                return cancel_result
            
            feedback_msg.current_action = f'책 픽업 중... {i}/3'
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.5)
        
        # Result 반환
        result = PickBook.Result()
        result.book_id = goal.book_id
        
        if self.is_error():
            # Error 모드: 픽업 실패
            goal_handle.abort()
            result.success = False
            result.message = f'Mock error: 책 픽업 실패 (book_id: {goal.book_id})'
            self.get_logger().warn(f'PickBook failed (error mode) - book_id: {goal.book_id}')
        else:
            # Active 모드: 픽업 성공
            goal_handle.succeed()
            result.success = True
            result.message = f'Mock: 책 픽업 성공 (book_id: {goal.book_id})'
            self.get_logger().info(f'PickBook succeeded - book_id: {goal.book_id}')
        
        return result


def main(args=None):
    rclpy.init(args=args)
    node = MockPickBookServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
