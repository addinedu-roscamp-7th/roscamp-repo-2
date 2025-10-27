#!/usr/bin/env python3
"""
GUI 길안내 테스트 노드
v4.0 플로우에 따라 QueryLocationInfo와 RequestGuidance 서비스를 테스트합니다.

테스트 시나리오:
1. QueryLocationInfo("") 호출 - 목록 요청 (WAITING_DEST_INPUT 진입)
2. QueryLocationInfo("화장실") 호출 - 위치 조회
3. RequestGuidance 호출 - 길안내 시작 (WAITING_DEST_INPUT → GUIDING)
"""

import rclpy
from rclpy.node import Node
from javis_interfaces.srv import QueryLocationInfo, RequestGuidance
from geometry_msgs.msg import Pose2D
import sys


class TestGuidanceNode(Node):
    def __init__(self):
        super().__init__('test_gui_guidance_node')
        
        # 네임스페이스 파라미터
        self.declare_parameter('robot_namespace', 'dobby1')
        self.namespace = self.get_parameter('robot_namespace').get_parameter_value().string_value
        
        # 서비스 클라이언트 생성
        query_service = f'/{self.namespace}/admin/query_location_info'
        request_service = f'/{self.namespace}/admin/request_guidance'
        
        self.query_client = self.create_client(QueryLocationInfo, query_service)
        self.request_client = self.create_client(RequestGuidance, request_service)
        
        self.get_logger().info('=== GUI 길안내 테스트 노드 시작 ===')
        self.get_logger().info(f'네임스페이스: {self.namespace}')
        self.get_logger().info(f'Query 서비스: {query_service}')
        self.get_logger().info(f'Request 서비스: {request_service}')
        
    def wait_for_services(self, timeout_sec=5.0):
        """서비스 연결 대기"""
        self.get_logger().info('DMC 서비스 대기 중...')
        
        if not self.query_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error('QueryLocationInfo 서비스를 찾을 수 없습니다')
            return False
            
        if not self.request_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error('RequestGuidance 서비스를 찾을 수 없습니다')
            return False
            
        self.get_logger().info('✓ 모든 서비스 연결 완료')
        return True
        
    def test_query_location_list(self):
        """테스트 1: 목적지 목록 요청 (빈 문자열)"""
        self.get_logger().info('\n[테스트 1] 목적지 목록 요청')
        self.get_logger().info('호출: QueryLocationInfo(location_name="")')
        
        request = QueryLocationInfo.Request()
        request.location_name = ""
        
        future = self.query_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'응답: found={response.found}')
            if response.found:
                self.get_logger().info(f'  - location_name: {response.location_name}')
                self.get_logger().info(f'  - location_id: {response.location_id}')
                self.get_logger().info(f'  - pose: x={response.pose.x:.1f}, y={response.pose.y:.1f}, theta={response.pose.theta:.2f}')
                self.get_logger().info(f'  - description: {response.description}')
                self.get_logger().info(f'  - aliases: {response.aliases}')
            self.get_logger().info(f'  - message: {response.message}')
            self.get_logger().info('✓ DMC는 WAITING_DEST_INPUT 상태로 전환됨 (60초 타이머 시작)')
            return True
        else:
            self.get_logger().error('서비스 호출 실패')
            return False
            
    def test_query_specific_location(self, location_name):
        """테스트 2: 특정 위치 조회"""
        self.get_logger().info(f'\n[테스트 2] 특정 위치 조회: "{location_name}"')
        self.get_logger().info(f'호출: QueryLocationInfo(location_name="{location_name}")')
        
        request = QueryLocationInfo.Request()
        request.location_name = location_name
        
        future = self.query_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'응답: found={response.found}')
            if response.found:
                self.get_logger().info(f'  - location_name: {response.location_name}')
                self.get_logger().info(f'  - location_id: {response.location_id}')
                self.get_logger().info(f'  - pose: x={response.pose.x:.1f}, y={response.pose.y:.1f}, theta={response.pose.theta:.2f}')
                self.get_logger().info(f'  - description: {response.description}')
                self.get_logger().info(f'  - aliases: {response.aliases}')
                return response.pose
            else:
                self.get_logger().warn(f'  - message: {response.message}')
                return None
        else:
            self.get_logger().error('서비스 호출 실패')
            return None
            
    def test_request_guidance(self, destination_name, dest_pose):
        """테스트 3: 길안내 요청"""
        self.get_logger().info(f'\n[테스트 3] 길안내 요청: "{destination_name}"')
        self.get_logger().info(f'호출: RequestGuidance(destination_name="{destination_name}", dest_pose={{x:{dest_pose.x:.1f}, y:{dest_pose.y:.1f}}}, request_source="gui")')
        
        request = RequestGuidance.Request()
        request.destination_name = destination_name
        request.dest_pose = dest_pose
        request.request_source = "gui"
        request.user_context = "test_gui_guidance_node"
        
        future = self.request_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'응답: success={response.success}')
            self.get_logger().info(f'  - message: {response.message}')
            if response.success:
                self.get_logger().info(f'  - task_id: {response.task_id}')
                self.get_logger().info('✓ DMC는 WAITING_DEST_INPUT → GUIDING 전환됨 (타이머 취소)')
            return response.success
        else:
            self.get_logger().error('서비스 호출 실패')
            return False
            
    def run_full_scenario(self, destination="화장실"):
        """v4.0 전체 시나리오 실행"""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('GUI 길안내 v4.0 전체 시나리오 테스트 시작')
        self.get_logger().info('='*60)
        
        # Step 1: 목록 요청 (WAITING_DEST_INPUT 진입)
        if not self.test_query_location_list():
            return False
            
        self.get_logger().info('\n>>> 3초 대기 (GUI가 목록을 표시하는 시간)')
        import time
        time.sleep(3)
        
        # Step 2: 특정 위치 조회
        pose = self.test_query_specific_location(destination)
        if pose is None:
            return False
            
        self.get_logger().info('\n>>> 2초 대기 (사용자가 버튼을 선택하는 시간)')
        time.sleep(2)
        
        # Step 3: 길안내 요청
        if not self.test_request_guidance(destination, pose):
            return False
            
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('✓ 전체 시나리오 테스트 완료')
        self.get_logger().info('='*60)
        return True
        
    def run_quick_test(self, destination="화장실"):
        """빠른 테스트: 위치 조회 → 즉시 길안내 요청"""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info(f'빠른 테스트: "{destination}" 길안내')
        self.get_logger().info('='*60)
        
        # QueryLocationInfo("")로 WAITING_DEST_INPUT 진입
        self.get_logger().info('[1단계] 목적지 입력 의사 표현 (빈 문자열 조회)')
        request = QueryLocationInfo.Request()
        request.location_name = ""
        future = self.query_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is None:
            self.get_logger().error('목록 요청 실패')
            return False
            
        import time
        time.sleep(1)
        
        # 특정 위치 조회
        self.get_logger().info(f'[2단계] 목적지 조회: "{destination}"')
        pose = self.test_query_specific_location(destination)
        if pose is None:
            return False
            
        time.sleep(1)
        
        # 즉시 길안내 요청
        self.get_logger().info('[3단계] 길안내 요청')
        return self.test_request_guidance(destination, pose)


def main(args=None):
    rclpy.init(args=args)
    node = TestGuidanceNode()
    
    try:
        # 서비스 연결 대기
        if not node.wait_for_services():
            node.get_logger().error('DMC 노드가 실행 중인지 확인하세요')
            return
            
        # 명령줄 인자 처리
        if len(sys.argv) > 1:
            command = sys.argv[1]
            
            if command == "list":
                # 목록만 조회
                node.test_query_location_list()
                
            elif command == "query":
                # 특정 위치만 조회
                location = sys.argv[2] if len(sys.argv) > 2 else "화장실"
                node.test_query_specific_location(location)
                
            elif command == "quick":
                # 빠른 테스트
                location = sys.argv[2] if len(sys.argv) > 2 else "화장실"
                node.run_quick_test(location)
                
            elif command == "full":
                # 전체 시나리오
                location = sys.argv[2] if len(sys.argv) > 2 else "화장실"
                node.run_full_scenario(location)
                
            else:
                node.get_logger().error(f'알 수 없는 명령: {command}')
                node.get_logger().info('사용법: ros2 run javis_dmc_test test_gui_guidance [list|query|quick|full] [목적지]')
        else:
            # 기본: 전체 시나리오
            node.run_full_scenario()
            
    except KeyboardInterrupt:
        node.get_logger().info('사용자에 의해 중단됨')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
