#!/usr/bin/env python3
"""
Mock DAC SortBook Action Server
도서 분류 액션 Mock
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
import time

from javis_interfaces.action import SortBook
from javis_dmc_test.nodes.mock_server_base import MockServerBase


class MockSortBookServer(MockServerBase):
    """도서 분류 액션 Mock 서버"""
    
    def __init__(self):
        super().__init__('mock_dac_sort_book')
        
        # Action Server 생성
        self._action_server = ActionServer(
            self,
            SortBook,
            'dobby1/arm/sort_book',
            self.execute_callback
        )
        
        self.get_logger().info('Mock DAC SortBook Action Server ready')
    
    def execute_callback(self, goal_handle: ServerGoalHandle):
        """
        도서 분류 액션 실행 핸들러
        
        Goal:
            string[] book_ids  # 분류할 책 ID 리스트
        
        Result:
            bool success
            int32 books_sorted  # 분류 완료한 책 개수
            string message
        """
        goal = goal_handle.request
        total_books = len(goal.book_ids)
        self.get_logger().info(f'SortBook Action called - books: {total_books}, mode: {self.mode}')
        
        # Feedback 발행 (진행률 시뮬레이션)
        feedback_msg = SortBook.Feedback()
        
        for i in range(1, total_books + 1):
            if not goal_handle.is_active:
                self.get_logger().warn('SortBook Action canceled')
                return SortBook.Result(success=False, books_sorted=0, message='Canceled')
            
            feedback_msg.current_book_index = i - 1
            feedback_msg.status = f'도서 분류 중... {i}/{total_books}권'
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.3)
        
        # Result 반환
        result = SortBook.Result()
        
        if self.is_error():
            # Error 모드: 분류 실패
            goal_handle.abort()
            result.success = False
            result.books_sorted = 0
            result.message = 'Mock error: 도서 분류 실패'
            self.get_logger().warn('SortBook failed (error mode)')
        else:
            # Active 모드: 분류 성공
            goal_handle.succeed()
            result.success = True
            result.books_sorted = total_books
            result.message = f'Mock: 도서 {total_books}권 분류 완료'
            self.get_logger().info(f'SortBook succeeded - sorted: {result.books_sorted} books')
        
        return result


def main(args=None):
    rclpy.init(args=args)
    node = MockSortBookServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
