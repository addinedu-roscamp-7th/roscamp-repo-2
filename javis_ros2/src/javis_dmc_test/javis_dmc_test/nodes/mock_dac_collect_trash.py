#!/usr/bin/env python3
"""
Mock DAC CollectTrash Action Server
쓰레기 수거 액션 Mock
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
import time

from javis_interfaces.action import CollectTrash
from javis_dmc_test.nodes.mock_server_base import MockServerBase


class MockCollectTrashServer(MockServerBase):
    """쓰레기 수거 액션 Mock 서버"""
    
    def __init__(self):
        super().__init__('mock_dac_collect_trash')
        
        # Action Server 생성
        self._action_server = ActionServer(
            self,
            CollectTrash,
            'dobby1/arm/collect_trash',
            self.execute_callback
        )
        
        self.get_logger().info('Mock DAC CollectTrash Action Server ready')
    
    def execute_callback(self, goal_handle: ServerGoalHandle):
        """
        쓰레기 수거 액션 실행 핸들러
        
        Goal:
            geometry_msgs/Point trash_position  # 쓰레기 위치
        
        Result:
            bool success
            string message
        """
        goal = goal_handle.request
        self.get_logger().info(f'CollectTrash Action called - pos: ({goal.trash_position.x}, {goal.trash_position.y}), mode: {self.mode}')
        
        # Feedback 발행 (진행률 시뮬레이션)
        feedback_msg = CollectTrash.Feedback()
        
        steps = ['쓰레기 접근', '쓰레기 파지', '수거 완료']
        for i, step in enumerate(steps, 1):
            if not goal_handle.is_active:
                self.get_logger().warn('CollectTrash Action canceled')
                return CollectTrash.Result(success=False, message='Canceled')
            
            feedback_msg.status = f'{step} 중... ({i}/{len(steps)})'
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.5)
        
        # Result 반환
        result = CollectTrash.Result()
        
        if self.is_error():
            # Error 모드: 수거 실패
            goal_handle.abort()
            result.success = False
            result.message = 'Mock error: 쓰레기 수거 실패'
            self.get_logger().warn('CollectTrash failed (error mode)')
        else:
            # Active 모드: 수거 성공
            goal_handle.succeed()
            result.success = True
            result.message = 'Mock: 쓰레기 수거 완료'
            self.get_logger().info('CollectTrash succeeded')
        
        return result


def main(args=None):
    rclpy.init(args=args)
    node = MockCollectTrashServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
