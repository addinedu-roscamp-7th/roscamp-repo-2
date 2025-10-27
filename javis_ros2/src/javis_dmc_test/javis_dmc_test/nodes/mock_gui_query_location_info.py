#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Mock GUI QueryLocationInfo Service Server

DMC가 GUI 인터페이스를 통해 위치 정보를 조회하는 서비스를 시뮬레이션합니다.
실제로는 DMC가 이 서비스를 제공하지만, Mock 테스트를 위해 구현합니다.

Author: JAVIS Team
Date: 2025-10-27
"""

import rclpy
from geometry_msgs.msg import Pose2D

from javis_dmc_test.nodes.mock_server_base import MockServerBase
from javis_interfaces.srv import QueryLocationInfo


class MockQueryLocationInfoService(MockServerBase):
    '''Mock 위치 정보 조회 Service Server
    
    GUI → DMC 위치 정보 조회를 시뮬레이션합니다.
    
    Request:
    - location_name: 조회할 위치 이름 (빈 문자열 = 전체 목록)
    
    Response:
    - found: 위치 발견 여부
    - pose: 위치 좌표
    - description: 위치 설명
    - aliases: 별칭 목록
    '''
    
    def __init__(self):
        super().__init__('mock_gui_query_location_info')
        
        # Mock 위치 데이터 (library_locations.yaml 시뮬레이션)
        self._locations = {
            '화장실': {
                'pose': Pose2D(x=10.5, y=-5.0, theta=1.57),
                'description': '1층 화장실 (남/여)',
                'aliases': ['화장실', 'toilet', 'restroom', '남자 화장실', '여자 화장실']
            },
            '카페': {
                'pose': Pose2D(x=15.0, y=8.0, theta=3.14),
                'description': '1층 카페테리아',
                'aliases': ['카페', 'cafe', '커피', '카페테리아']
            },
            '출입구': {
                'pose': Pose2D(x=0.0, y=0.0, theta=0.0),
                'description': '도서관 정문 출입구',
                'aliases': ['출입구', 'entrance', '정문', '입구']
            },
            '안내데스크': {
                'pose': Pose2D(x=2.0, y=1.0, theta=0.0),
                'description': '1층 안내 데스크',
                'aliases': ['안내데스크', '안내', 'information', 'desk', '데스크']
            },
            '열람실': {
                'pose': Pose2D(x=20.0, y=5.0, theta=0.0),
                'description': '2층 열람실',
                'aliases': ['열람실', 'reading room', '독서실']
            }
        }
        
        # Service Server 생성
        self._service = self.create_service(
            QueryLocationInfo,
            'dobby1/admin/query_location_info',
            self._handle_query
        )
        
        self.get_logger().info('Mock QueryLocationInfo Service ready')
        self.get_logger().info(f'등록된 위치: {list(self._locations.keys())}')
    
    def _handle_query(self, request, response):
        '''QueryLocationInfo 서비스 핸들러'''
        location_name = request.location_name
        
        self.get_logger().info(f'QueryLocationInfo 요청: "{location_name}"')
        
        # error 모드 체크
        if self.is_error():
            response.found = False
            response.message = 'Mock error 모드: 위치 조회 실패'
            self.get_logger().warn('QueryLocationInfo 실패 (error 모드)')
            return response
        
        # 빈 문자열 = 전체 목록 요청 (v4.0: GUI 터치 시)
        if not location_name or location_name == '':
            self.get_logger().info('전체 위치 목록 반환 (GUI 터치 감지)')
            # 첫 번째 위치만 반환 (실제로는 모든 위치를 배열로 반환해야 하지만 단순화)
            first_location = list(self._locations.keys())[0]
            loc_data = self._locations[first_location]
            
            response.found = True
            response.pose = loc_data['pose']
            response.description = f'등록된 위치: {", ".join(self._locations.keys())}'
            response.aliases = list(self._locations.keys())
            response.message = f'{len(self._locations)}개 위치 정보 반환'
            return response
        
        # 정확한 이름으로 검색
        if location_name in self._locations:
            loc_data = self._locations[location_name]
            response.found = True
            response.pose = loc_data['pose']
            response.description = loc_data['description']
            response.aliases = loc_data['aliases']
            response.message = f'{location_name} 위치 정보 반환'
            
            self.get_logger().info(
                f'위치 발견: {location_name} = '
                f'({loc_data["pose"].x:.1f}, {loc_data["pose"].y:.1f})'
            )
            return response
        
        # 별칭으로 검색
        for name, data in self._locations.items():
            if location_name.lower() in [alias.lower() for alias in data['aliases']]:
                response.found = True
                response.pose = data['pose']
                response.description = data['description']
                response.aliases = data['aliases']
                response.message = f'{name}(별칭: {location_name}) 위치 정보 반환'
                
                self.get_logger().info(
                    f'별칭으로 위치 발견: {location_name} → {name}'
                )
                return response
        
        # 찾지 못함
        response.found = False
        response.message = f'"{location_name}" 위치를 찾을 수 없습니다'
        self.get_logger().warn(f'위치 미발견: {location_name}')
        
        return response


def main(args=None):
    '''메인 함수'''
    rclpy.init(args=args)
    
    node = MockQueryLocationInfoService()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
