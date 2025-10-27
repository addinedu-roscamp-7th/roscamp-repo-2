#!/usr/bin/env python3
"""
Mock DAC CleanDesk Action Server
책상 청소 액션 Mock
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
import time

from javis_interfaces.action import CleanDesk
from javis_dmc_test.nodes.mock_server_base import MockServerBase


class MockCleanDeskServer(MockServerBase):
    """책상 청소 액션 Mock 서버"""
    
    def __init__(self):
        super().__init__('mock_dac_clean_desk')
        
        # Action Server 생성
        self._action_server = ActionServer(
            self,
            CleanDesk,
            'dobby1/arm/clean_desk',
            self.execute_callback
        )
        
        self.get_logger().info('Mock DAC CleanDesk Action Server ready')
    
    def execute_callback(self, goal_handle: ServerGoalHandle):
        """
        책상 청소 액션 실행 핸들러
        
        Goal:
            geometry_msgs/Pose desk_pose  # 청소할 책상 위치
        
        Result:
            bool success
            string message
        """
        goal = goal_handle.request
        self.get_logger().info(f'CleanDesk Action called - mode: {self.mode}')
        
        # Feedback 발행 (진행률 시뮬레이션)
        feedback_msg = CleanDesk.Feedback()
        
        steps = ['먼지 제거', '표면 닦기', '정리 완료']
        for i, step in enumerate(steps, 1):
            if not goal_handle.is_active:
                self.get_logger().warn('CleanDesk Action canceled')
                return CleanDesk.Result(success=False, message='Canceled')
            
            feedback_msg.status = f'{step} 중... ({i}/{len(steps)})'
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.5)
        
        # Result 반환
        result = CleanDesk.Result()
        
        if self.is_error():
            # Error 모드: 청소 실패
            goal_handle.abort()
            result.success = False
            result.message = 'Mock error: 책상 청소 실패'
            self.get_logger().warn('CleanDesk failed (error mode)')
        else:
            # Active 모드: 청소 성공
            goal_handle.succeed()
            result.success = True
            result.message = 'Mock: 책상 청소 완료'
            self.get_logger().info('CleanDesk succeeded')
        
        return result


def main(args=None):
    rclpy.init(args=args)
    node = MockCleanDeskServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
