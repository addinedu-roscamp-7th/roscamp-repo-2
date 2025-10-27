#!/usr/bin/env python3
"""
Mock DAC CollectReturnedBooks Action Server
반납도서 수거 액션 Mock
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
import time

from javis_interfaces.action import CollectReturnedBooks
from javis_dmc_test.nodes.mock_server_base import MockServerBase


class MockCollectReturnedBooksServer(MockServerBase):
    """반납도서 수거 액션 Mock 서버"""
    
    def __init__(self):
        super().__init__('mock_dac_collect_returned_books')
        
        # Action Server 생성
        self._action_server = ActionServer(
            self,
            CollectReturnedBooks,
            'dobby1/arm/collect_returned_books',
            self.execute_callback
        )
        
        self.get_logger().info('Mock DAC CollectReturnedBooks Action Server ready')
    
    def execute_callback(self, goal_handle: ServerGoalHandle):
        """
        반납도서 수거 액션 실행 핸들러
        
        Goal:
            geometry_msgs/Pose collection_pose  # 수거 위치
        
        Result:
            bool success
            int32 books_collected  # 수거한 책 개수
            string message
        """
        goal = goal_handle.request
        self.get_logger().info(f'CollectReturnedBooks Action called - mode: {self.mode}')
        
        # Feedback 발행 (진행률 시뮬레이션)
        feedback_msg = CollectReturnedBooks.Feedback()
        
        for i in range(1, 5):
            if not goal_handle.is_active:
                self.get_logger().warn('CollectReturnedBooks Action canceled')
                return CollectReturnedBooks.Result(success=False, books_collected=0, message='Canceled')
            
            feedback_msg.current_book = i
            feedback_msg.status = f'반납도서 수거 중... {i}/4권'
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.5)
        
        # Result 반환
        result = CollectReturnedBooks.Result()
        
        if self.is_error():
            # Error 모드: 수거 실패
            goal_handle.abort()
            result.success = False
            result.books_collected = 0
            result.message = 'Mock error: 반납도서 수거 실패'
            self.get_logger().warn('CollectReturnedBooks failed (error mode)')
        else:
            # Active 모드: 수거 성공
            goal_handle.succeed()
            result.success = True
            result.books_collected = 4  # Mock: 4권 수거
            result.message = 'Mock: 반납도서 4권 수거 완료'
            self.get_logger().info(f'CollectReturnedBooks succeeded - collected: {result.books_collected} books')
        
        return result


def main(args=None):
    rclpy.init(args=args)
    node = MockCollectReturnedBooksServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
