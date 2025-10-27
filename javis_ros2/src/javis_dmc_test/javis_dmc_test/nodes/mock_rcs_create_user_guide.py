#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Mock RCS Servers

RCS의 CreateUserGuide Service를 시뮬레이션하는 Mock 서버입니다.
DMC로부터 사용자 주도 작업 생성 요청을 받아 각 작업 타입에 맞는 Action Goal을 전송합니다.

v4.0 핵심 기능:
- CreateUserGuide Service Server (/rcs/create_user_task)
- 5개 작업 타입 지원:
  * guide_person: GuidePerson Action
  * pickup_book: PickupBook Action
  * reshelving_book: ReshelvingBook Action
  * clean_seat: CleanSeat Action
  * rearrange_book: RearrangeBook Action
- user_initiated 플래그 처리

Author: JAVIS Team
Date: 2025-10-27
"""

import rclpy
from rclpy.action import ActionClient
from datetime import datetime

from javis_dmc_test.nodes.mock_server_base import MockServerBase
from javis_interfaces.srv import CreateUserGuide
from javis_interfaces.action import (
    GuidePerson,
    PickupBook,
    ReshelvingBook,
    CleanSeat,
    RearrangeBook,
)
from geometry_msgs.msg import Pose, Pose2D


class MockCreateUserTaskServer(MockServerBase):
    '''Mock RCS CreateUserGuide Service Server
    
    DMC → RCS 사용자 작업 생성 요청을 처리하고
    각 작업 타입에 맞는 Action Goal을 DMC에 전송합니다.
    
    지원 작업 타입:
    - guide_person: 길안내
    - pickup_book: 도서 픽업
    - reshelving_book: 반납 도서 정리
    - clean_seat: 좌석 청소
    - rearrange_book: 서가 정리
    
    플로우:
    1. DMC로부터 CreateUserGuide 서비스 호출 수신
    2. task_type에 따라 적절한 Action Client 선택
    3. task_id 생성
    4. Action Goal 생성 및 전송
    5. Response 반환
    '''
    
    def __init__(self):
        super().__init__('mock_rcs_create_user_task')
        
        # CreateUserGuide Service Server 생성
        self._service = self.create_service(
            CreateUserGuide,
            '/rcs/create_user_task',
            self._handle_create_user_task
        )
        
        # 각 작업 타입별 Action Client 생성
        self._action_clients = {
            'guide_person': ActionClient(
                self,
                GuidePerson,
                'dobby1/main/guide_person'
            ),
            'pickup_book': ActionClient(
                self,
                PickupBook,
                'dobby1/main/pickup_book'
            ),
            'reshelving_book': ActionClient(
                self,
                ReshelvingBook,
                'dobby1/main/reshelving_book'
            ),
            'clean_seat': ActionClient(
                self,
                CleanSeat,
                'dobby1/main/clean_seat'
            ),
            'rearrange_book': ActionClient(
                self,
                RearrangeBook,
                'dobby1/main/rearrange_book'
            ),
        }
        
        self.get_logger().info('Mock RCS CreateUserGuide Service ready (5개 작업 타입 지원)')
        self.get_logger().info('  - guide_person, pickup_book, reshelving_book, clean_seat, rearrange_book')
    
    def _handle_create_user_task(self, request, response):
        '''CreateUserGuide 서비스 핸들러
        
        Args:
            request: CreateUserGuide.Request
                - task_type: 작업 타입 (guide_person, pickup_book, etc.)
                - dest_location: 목적지 좌표
                - destination_name: 목적지 이름
                - user_initiated: 사용자 주도 작업 플래그
            
            response: CreateUserGuide.Response
                - success: 성공 여부
                - task_id: 생성된 작업 ID
                - message: 상태 메시지
        '''
        task_type = request.task_type.lower()
        
        self.get_logger().info(
            f'CreateUserGuide 요청 수신: '
            f'task_type={task_type}, '
            f'destination={request.destination_name}, '
            f'user_initiated={request.user_initiated}'
        )
        
        # mode가 error인 경우 실패 응답
        if self.is_error():
            response.success = False
            response.task_id = ''
            response.message = 'Mock RCS가 error 모드입니다'
            self.get_logger().warn('CreateUserGuide 실패: error 모드')
            return response
        
        # 지원하지 않는 작업 타입 체크
        if task_type not in self._action_clients:
            response.success = False
            response.task_id = ''
            response.message = f'지원하지 않는 작업 타입: {task_type}'
            self.get_logger().warn(f'CreateUserGuide 실패: 알 수 없는 task_type={task_type}')
            return response
        
        # task_id 생성
        task_prefix = task_type.replace('_', '')
        task_id = f'{task_prefix}_{datetime.now().strftime("%Y%m%d_%H%M%S")}'
        
        # 작업 타입별 Goal 생성 및 전송
        action_client = self._action_clients[task_type]
        goal = self._create_goal(task_type, request)
        
        if goal is None:
            response.success = False
            response.task_id = ''
            response.message = f'{task_type} Goal 생성 실패'
            self.get_logger().error(f'Goal 생성 실패: task_type={task_type}')
            return response
        
        self.get_logger().info(f'{task_type} Action Goal 전송 준비')
        
        # Action 서버 대기
        if not action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn(f'{task_type} Action 서버를 찾을 수 없습니다')
            response.success = False
            response.task_id = ''
            response.message = f'{task_type} Action 서버 연결 실패'
            return response
        
        # Action Goal 전송
        send_goal_future = action_client.send_goal_async(goal)
        
        # 비동기 전송이므로 콜백 등록
        send_goal_future.add_done_callback(
            lambda future: self._goal_response_callback(future, task_id, task_type)
        )
        
        # Response 즉시 반환
        response.success = True
        response.task_id = task_id
        response.message = f'{task_type} 작업 생성 완료 (destination: {request.destination_name})'
        
        self.get_logger().info(f'CreateUserGuide 성공: task_id={task_id}, type={task_type}')
        
        return response
    
    def _create_goal(self, task_type: str, request):
        '''작업 타입에 맞는 Action Goal 생성
        
        Args:
            task_type: 작업 타입
            request: CreateUserGuide.Request
        
        Returns:
            Action Goal 객체 또는 None
        '''
        if task_type == 'guide_person':
            goal = GuidePerson.Goal()
            
            # Pose2D 복사본 생성
            dest_loc = Pose2D()
            dest_loc.x = request.dest_location.x
            dest_loc.y = request.dest_location.y
            dest_loc.theta = request.dest_location.theta
            goal.dest_location = dest_loc
            
            goal.destination_name = request.destination_name
            goal.user_initiated = request.user_initiated
            return goal
        
        elif task_type == 'pickup_book':
            goal = PickupBook.Goal()
            # Mock 데이터 설정
            goal.book_id = 'BOOK_12345'
            
            # Pose2D 복사본 생성
            shelf_loc = Pose2D()
            shelf_loc.x = request.dest_location.x
            shelf_loc.y = request.dest_location.y
            shelf_loc.theta = request.dest_location.theta
            goal.shelf_approach_location = shelf_loc
            
            storage_loc = Pose2D()
            storage_loc.x = request.dest_location.x
            storage_loc.y = request.dest_location.y
            storage_loc.theta = request.dest_location.theta
            goal.storage_approach_location = storage_loc
            
            # Mock Pose 데이터
            goal.book_pick_pose = Pose()
            goal.storage_slot_pose = Pose()
            goal.storage_id = 1
            
            self.get_logger().info(f'PickupBook Goal 생성: book_id={goal.book_id}')
            return goal
        
        elif task_type == 'reshelving_book':
            goal = ReshelvingBook.Goal()
            # Pose2D 복사본 생성
            return_loc = Pose2D()
            return_loc.x = request.dest_location.x
            return_loc.y = request.dest_location.y
            return_loc.theta = request.dest_location.theta
            goal.return_desk_location = return_loc
            self.get_logger().info('ReshelvingBook Goal 생성')
            return goal
        
        elif task_type == 'clean_seat':
            goal = CleanSeat.Goal()
            # Pose2D 복사본 생성
            desk_loc = Pose2D()
            desk_loc.x = request.dest_location.x
            desk_loc.y = request.dest_location.y
            desk_loc.theta = request.dest_location.theta
            goal.desk_location = desk_loc
            goal.desk_id = 'DESK_01'
            self.get_logger().info(f'CleanSeat Goal 생성: desk_id={goal.desk_id}')
            return goal
        
        elif task_type == 'rearrange_book':
            goal = RearrangeBook.Goal()
            # Pose2D 복사본 생성
            shelf_loc = Pose2D()
            shelf_loc.x = request.dest_location.x
            shelf_loc.y = request.dest_location.y
            shelf_loc.theta = request.dest_location.theta
            goal.shelf_location = shelf_loc
            goal.shelf_id = 'SHELF_A1'
            self.get_logger().info(f'RearrangeBook Goal 생성: shelf_id={goal.shelf_id}')
            return goal
        
        else:
            self.get_logger().error(f'알 수 없는 task_type: {task_type}')
            return None
    
    def _goal_response_callback(self, future, task_id: str, task_type: str):
        '''Action Goal 응답 콜백'''
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().warn(
                f'{task_type} Action Goal 거부됨: task_id={task_id}'
            )
            return
        
        self.get_logger().info(
            f'{task_type} Action Goal 수락됨: task_id={task_id}'
        )
        
        # Result 콜백 등록
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future: self._result_callback(future, task_id, task_type)
        )
    
    def _result_callback(self, future, task_id: str, task_type: str):
        '''Action Result 콜백'''
        result = future.result().result
        
        # 작업 타입별 결과 로깅
        if task_type == 'guide_person':
            self.get_logger().info(
                f'GuidePerson 완료: '
                f'task_id={task_id}, '
                f'success={result.success}, '
                f'distance={result.total_distance_m:.1f}m, '
                f'time={result.total_time_sec:.1f}s, '
                f'message={result.message}'
            )
        elif task_type == 'pickup_book':
            self.get_logger().info(
                f'PickupBook 완료: '
                f'task_id={task_id}, '
                f'success={result.success}, '
                f'book_id={result.book_id}, '
                f'message={result.message}'
            )
        elif task_type == 'reshelving_book':
            self.get_logger().info(
                f'ReshelvingBook 완료: '
                f'task_id={task_id}, '
                f'success={result.success}, '
                f'books_processed={result.books_processed}, '
                f'message={result.message}'
            )
        elif task_type == 'clean_seat':
            self.get_logger().info(
                f'CleanSeat 완료: '
                f'task_id={task_id}, '
                f'success={result.success}, '
                f'trash_count={result.trash_collected_count}, '
                f'message={result.message}'
            )
        elif task_type == 'rearrange_book':
            self.get_logger().info(
                f'RearrangeBook 완료: '
                f'task_id={task_id}, '
                f'success={result.success}, '
                f'message={result.message}'
            )
        else:
            self.get_logger().info(
                f'{task_type} 완료: task_id={task_id}, success={result.success}'
            )


def main(args=None):
    '''메인 함수'''
    rclpy.init(args=args)
    
    node = MockCreateUserTaskServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
