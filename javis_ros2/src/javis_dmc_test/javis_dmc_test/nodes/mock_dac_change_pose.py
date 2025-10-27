#!/usr/bin/env python3
"""
Mock DAC ChangeArmPose Service Server
로봇팔 자세 변경 서비스 Mock
"""

import rclpy
import time

from javis_interfaces.srv import ChangeArmPose
from javis_dmc_test.nodes.mock_server_base import MockServerBase


class MockChangeArmPoseServer(MockServerBase):
    """로봇팔 자세 변경 서비스 Mock 서버"""
    
    def __init__(self):
        super().__init__('mock_dac_change_pose')
        
        # Service Server 생성
        self._service = self.create_service(
            ChangeArmPose,
            'dobby1/arm/change_pose',
            self._handle_change_pose
        )
        
        self.get_logger().info('Mock DAC ChangeArmPose Service ready')
    
    def _handle_change_pose(self, request, response):
        """
        로봇팔 자세 변경 서비스 핸들러
        
        Request:
            uint8 pose_type  # 0: 관측자세, 1: 초기자세, 2: 사용자 지정
            geometry_msgs/Pose target_pose
        
        Response:
            bool success
            string message
        """
        pose_types = {
            0: '관측자세',
            1: '초기자세',
            2: '사용자 지정 자세'
        }
        pose_name = pose_types.get(request.pose_type, '알 수 없는 자세')
        
        self.get_logger().info(
            f'ChangeArmPose 요청 수신: pose_type={request.pose_type} ({pose_name}), mode={self.mode}'
        )
        
        # Mock 처리 시간 (0.5초)
        time.sleep(0.5)
        
        if self.is_error():
            # Error 모드: 자세 변경 실패
            response.success = False
            response.message = f'Mock error: {pose_name} 변경 실패'
            self.get_logger().warn(f'ChangeArmPose 실패 (error 모드) - {pose_name}')
        else:
            # Active 모드: 자세 변경 성공
            response.success = True
            response.message = f'Mock: {pose_name}로 변경 완료'
            self.get_logger().info(f'ChangeArmPose 성공 - {pose_name}')
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MockChangeArmPoseServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
