#!/usr/bin/env python3
"""
Mock DVS DetectTrash Service Server
쓰레기 감지 서비스 Mock
"""

import rclpy
from rclpy.node import Node

from javis_interfaces.srv import DetectTrash
from javis_dmc_test.nodes.mock_server_base import MockServerBase


class MockDetectTrashServer(MockServerBase):
    """쓰레기 감지 서비스 Mock 서버"""
    
    def __init__(self):
        super().__init__('mock_dvs_detect_trash')
        
        # Service Server 생성
        self.service = self.create_service(
            DetectTrash,
            'dobby1/ai/detect_trash',
            self.handle_detect_trash
        )
        
        self.get_logger().info('Mock DVS DetectTrash Service ready')
    
    def handle_detect_trash(self, request, response):
        """
        쓰레기 감지 서비스 핸들러
        
        Request:
            bool trigger  # 감지 트리거
        
        Response:
            bool success
            bool trash_detected
            string trash_type
            geometry_msgs/Point trash_position
            string message
        """
        self.get_logger().info(f'DetectTrash called - mode: {self.mode}')
        
        if self.is_error():
            # Error 모드: 감지 실패
            response.success = False
            response.trash_detected = False
            response.trash_type = ''
            response.trash_position.x = 0.0
            response.trash_position.y = 0.0
            response.trash_position.z = 0.0
            response.message = 'Mock error: 쓰레기 감지 실패'
            self.get_logger().warn('DetectTrash failed (error mode)')
        else:
            # Active 모드: 쓰레기 감지 성공
            response.success = True
            response.trash_detected = True
            response.trash_type = 'paper'  # Mock 데이터: 종이 쓰레기
            response.trash_position.x = 5.5
            response.trash_position.y = 3.2
            response.trash_position.z = 0.1
            response.message = 'Mock: 쓰레기 감지 완료 (종이)'
            self.get_logger().info(f'DetectTrash success - type: {response.trash_type}, pos: ({response.trash_position.x}, {response.trash_position.y})')
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MockDetectTrashServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
