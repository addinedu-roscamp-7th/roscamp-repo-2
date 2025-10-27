#!/usr/bin/env python3
"""
Mock DAC DisposeTrash Action Server
쓰레기 처리 액션 Mock
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
import time

from javis_interfaces.action import DisposeTrash
from javis_dmc_test.nodes.mock_server_base import MockServerBase


class MockDisposeTrashServer(MockServerBase):
    """쓰레기 처리 액션 Mock 서버"""
    
    def __init__(self):
        super().__init__('mock_dac_dispose_trash')
        
        # Action Server 생성
        self._action_server = ActionServer(
            self,
            DisposeTrash,
            'dobby1/arm/dispose_trash',
            self.execute_callback
        )
        
        self.get_logger().info('Mock DAC DisposeTrash Action Server ready')
    
    def execute_callback(self, goal_handle: ServerGoalHandle):
        """
        쓰레기 처리 액션 실행 핸들러
        
        Goal:
            geometry_msgs/Pose disposal_pose  # 쓰레기통 위치
        
        Result:
            bool success
            string message
        """
        goal = goal_handle.request
        self.get_logger().info(f'DisposeTrash Action called - mode: {self.mode}')
        
        # Feedback 발행 (진행률 시뮬레이션)
        feedback_msg = DisposeTrash.Feedback()
        
        steps = ['쓰레기통 접근', '쓰레기 투기', '처리 완료']
        for i, step in enumerate(steps, 1):
            if not goal_handle.is_active:
                self.get_logger().warn('DisposeTrash Action canceled')
                return DisposeTrash.Result(success=False, message='Canceled')
            
            feedback_msg.status = f'{step} 중... ({i}/{len(steps)})'
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.5)
        
        # Result 반환
        result = DisposeTrash.Result()
        
        if self.is_error():
            # Error 모드: 처리 실패
            goal_handle.abort()
            result.success = False
            result.message = 'Mock error: 쓰레기 처리 실패'
            self.get_logger().warn('DisposeTrash failed (error mode)')
        else:
            # Active 모드: 처리 성공
            goal_handle.succeed()
            result.success = True
            result.message = 'Mock: 쓰레기 처리 완료'
            self.get_logger().info('DisposeTrash succeeded')
        
        return result


def main(args=None):
    rclpy.init(args=args)
    node = MockDisposeTrashServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
