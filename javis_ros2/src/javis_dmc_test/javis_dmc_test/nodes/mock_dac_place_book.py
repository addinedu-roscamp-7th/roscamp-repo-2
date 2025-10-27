#!/usr/bin/env python3
"""
Mock DAC PlaceBook Action Server
책 배치 액션 Mock
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
import time

from javis_interfaces.action import PlaceBook
from javis_dmc_test.nodes.mock_server_base import MockServerBase


class MockPlaceBookServer(MockServerBase):
    """책 배치 액션 Mock 서버"""
    
    def __init__(self):
        super().__init__('mock_dac_place_book')
        
        # Action Server 생성
        self._action_server = ActionServer(
            self,
            PlaceBook,
            'dobby1/arm/place_book',
            self.execute_callback
        )
        
        self.get_logger().info('Mock DAC PlaceBook Action Server ready')
    
    def execute_callback(self, goal_handle: ServerGoalHandle):
        """
        책 배치 액션 실행 핸들러
        
        Goal:
            string book_id  # 배치할 책 ID
            geometry_msgs/Pose place_pose  # 배치 위치
        
        Result:
            bool success
            string message
        """
        goal = goal_handle.request
        self.get_logger().info(f'PlaceBook Action called - book_id: {goal.book_id}, mode: {self.mode}')
        
        # Feedback 발행 (진행률 시뮬레이션)
        feedback_msg = PlaceBook.Feedback()
        
        for i in range(1, 4):
            if not goal_handle.is_active:
                self.get_logger().warn('PlaceBook Action canceled')
                return PlaceBook.Result(success=False, message='Canceled')
            
            feedback_msg.status = f'책 배치 중... {i}/3'
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.5)
        
        # Result 반환
        result = PlaceBook.Result()
        
        if self.is_error():
            # Error 모드: 배치 실패
            goal_handle.abort()
            result.success = False
            result.message = f'Mock error: 책 배치 실패 (book_id: {goal.book_id})'
            self.get_logger().warn(f'PlaceBook failed (error mode) - book_id: {goal.book_id}')
        else:
            # Active 모드: 배치 성공
            goal_handle.succeed()
            result.success = True
            result.message = f'Mock: 책 배치 성공 (book_id: {goal.book_id})'
            self.get_logger().info(f'PlaceBook succeeded - book_id: {goal.book_id}')
        
        return result


def main(args=None):
    rclpy.init(args=args)
    node = MockPlaceBookServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
