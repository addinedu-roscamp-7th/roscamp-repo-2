📘 Dobby Main Controller 완전 구현 가이드
3주 프로젝트용 - 사소한 것도 놓치지 않는 완전판

📋 목차

프로젝트 전체 구조
1단계: 프로젝트 생성
2단계: 메시지 정의
3단계: Interface 구현
4단계: State Machine 구현
5단계: Task Coordinator 구현
6단계: Main Controller 구현
7단계: Launch 파일
8단계: 테스트
트러블슈팅


1. 프로젝트 전체 구조
dobby_main_controller/
├── package.xml                              # ROS2 패키지 정보
├── setup.py                                 # Python 패키지 설정
├── setup.cfg                                # 설정
│
├── dobby_main_controller/
│   ├── __init__.py
│   │
│   ├── dobby_main_controller_node.py       # 메인 노드 (진입점)
│   │
│   ├── core/
│   │   ├── __init__.py
│   │   ├── state_machine.py                # 상태 관리
│   │   └── task_coordinator.py             # 작업 조율
│   │
│   ├── interfaces/
│   │   ├── __init__.py
│   │   ├── base_interface.py               # 공통 부모 클래스
│   │   ├── arm_interface.py                # DAC 통신
│   │   ├── drive_interface.py              # DDC 통신
│   │   └── ai_interface.py                 # AIS 통신
│   │
│   └── mock/
│       ├── __init__.py
│       └── mock_interfaces.py              # 테스트용 Mock
│
├── config/
│   └── params.yaml                         # 설정 파라미터
│
├── launch/
│   ├── dobby_main.launch.py                # 실제 모드
│   └── dobby_test.launch.py                # 테스트 모드
│
└── test/
    ├── test_state_machine.py
    └── test_interfaces.py

2. 1단계: 프로젝트 생성
2.1 패키지 생성
bash# 작업 공간으로 이동
cd ~/ros2_ws/src

# 패키지 생성
ros2 pkg create dobby_main_controller \
  --build-type ament_python \
  --dependencies rclpy std_msgs geometry_msgs \
  --node-name dobby_main_controller_node

cd dobby_main_controller
2.2 디렉토리 구조 생성
bash# 필요한 디렉토리 생성
mkdir -p dobby_main_controller/core
mkdir -p dobby_main_controller/interfaces
mkdir -p dobby_main_controller/mock
mkdir -p config
mkdir -p launch
mkdir -p test

# __init__.py 파일 생성
touch dobby_main_controller/__init__.py
touch dobby_main_controller/core/__init__.py
touch dobby_main_controller/interfaces/__init__.py
touch dobby_main_controller/mock/__init__.py
2.3 package.xml 수정
xml<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>dobby_main_controller</name>
  <version>1.0.0</version>
  <description>Dobby Main Controller for library automation</description>
  <maintainer email="your_email@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <!-- 빌드 도구 -->
  <buildtool_depend>ament_python</buildtool_depend>

  <!-- 실행 의존성 -->
  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>action_msgs</exec_depend>
  
  <!-- 커스텀 메시지 패키지 (나중에 생성) -->
  <exec_depend>dobby_interfaces</exec_depend>

  <!-- 테스트 -->
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
2.4 setup.py 수정
pythonfrom setuptools import setup, find_packages
from glob import glob
import os

package_name = 'dobby_main_controller'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch 파일
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
        # Config 파일
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Dobby Main Controller',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dobby_main_controller = dobby_main_controller.dobby_main_controller_node:main',
        ],
    },
)
2.5 setup.cfg
ini[develop]
script_dir=$base/lib/dobby_main_controller

[install]
install_scripts=$base/lib/dobby_main_controller

3. 2단계: 메시지 정의
3.1 별도 패키지 생성 (dobby_interfaces)
bashcd ~/ros2_ws/src
ros2 pkg create dobby_interfaces --build-type ament_cmake

cd dobby_interfaces
mkdir -p msg srv action
```

### 3.2 필요한 메시지 파일들

#### msg/DobbyState.msg
```
# Main State
uint8 INITIALIZING = 0
uint8 CHARGING = 1
uint8 IDLE = 2
uint8 MOVING_TO_CHARGER = 3
uint8 PICKING_UP_BOOK = 4
uint8 RESHELVING_BOOK = 5
uint8 GUIDING = 6
uint8 CLEANING_DESK = 7
uint8 ERROR = 99

uint8 main_state

# Sub State
uint8 NONE = 100
uint8 MOVE_TO_PICKUP = 101
uint8 PICKING_BOOK = 102
uint8 MOVE_TO_STORAGE = 103
uint8 STOWING_BOOK = 104
uint8 MOVE_TO_RETURN_DESK = 105
uint8 COLLECT_RETURN_BOOKS = 106
uint8 MOVE_TO_PLACE_SHELF = 107
uint8 PLACE_RETURN_BOOK = 108
uint8 SELECT_DEST = 109
uint8 SCAN_USER = 110
uint8 GUIDING_TO_DEST = 111

uint8 sub_state

# Error
bool is_error
string error_message
```

#### action/PickupBook.action
```
# Goal
int32 task_id
string book_id
geometry_msgs/Pose shelf_approach_pose
geometry_msgs/Pose book_pose
int32 storage_id
geometry_msgs/Pose storage_approach_pose
geometry_msgs/Pose storage_slot_pose
---
# Result
int32 task_id
bool success
string message
float64 total_distance_m
float64 total_time_sec
---
# Feedback
int32 progress_percent
string current_state
```

#### action/ReshelvingBook.action
```
# Goal
int32 task_id
geometry_msgs/Pose return_desk_pose
---
# Result
int32 task_id
bool success
string message
int32 books_processed
string[] failed_book_ids
float64 total_distance_m
float64 total_time_sec
---
# Feedback
int32 progress_percent
string current_state
```

#### action/GuidePerson.action
```
# Goal
int32 task_id
geometry_msgs/Pose destination_pose
---
# Result
int32 task_id
bool success
string error_code
float64 total_distance_m
float64 total_time_sec
---
# Feedback
float64 distance_remaining_m
bool person_detected
3.3 CMakeLists.txt (dobby_interfaces)
cmakecmake_minimum_required(VERSION 3.8)
project(dobby_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DobbyState.msg"
  "action/PickupBook.action"
  "action/ReshelvingBook.action"
  "action/GuidePerson.action"
  DEPENDENCIES std_msgs geometry_msgs action_msgs
)

ament_package()
3.4 빌드 및 확인
bashcd ~/ros2_ws
colcon build --packages-select dobby_interfaces
source install/setup.bash

# 확인
ros2 interface list | grep dobby

4. 3단계: Interface 구현
4.1 base_interface.py (공통 부모 클래스)
파일 위치: dobby_main_controller/interfaces/base_interface.py
python"""
Base Interface for all communication interfaces
"""
from abc import ABC, abstractmethod
from typing import Optional
import rclpy
from rclpy.node import Node


class BaseInterface(ABC):
    """
    모든 인터페이스의 공통 부모 클래스
    
    역할:
    - 노드 참조 저장
    - 로봇 ID 관리
    - 공통 로깅 기능
    """
    
    def __init__(self, node: Node):
        """
        Args:
            node: ROS2 노드 인스턴스
        """
        self.node = node
        self.logger = node.get_logger()
        
        # 노드에서 robot_id 파라미터 가져오기
        try:
            self.robot_id = node.get_parameter('robot_id').value
        except:
            self.robot_id = 'dobby1'  # 기본값
            self.logger.warning(f"robot_id not found, using default: {self.robot_id}")
    
    def _get_topic_name(self, topic: str) -> str:
        """
        네임스페이스를 포함한 토픽 이름 생성
        
        Args:
            topic: 기본 토픽 이름 (예: 'arm/pick_book')
            
        Returns:
            완전한 토픽 이름 (예: 'dobby1/arm/pick_book')
        """
        # 이미 robot_id가 포함되어 있으면 그대로 반환
        if topic.startswith(self.robot_id):
            return topic
        # 아니면 앞에 추가
        return f'{self.robot_id}/{topic}'
    
    @abstractmethod
    async def initialize(self):
        """
        인터페이스 초기화 (Action Client 등 생성)
        각 서브클래스에서 구현 필요
        """
        pass
    
    def log_info(self, message: str):
        """편의 함수: INFO 로그"""
        self.logger.info(f"[{self.__class__.__name__}] {message}")
    
    def log_warn(self, message: str):
        """편의 함수: WARN 로그"""
        self.logger.warning(f"[{self.__class__.__name__}] {message}")
    
    def log_error(self, message: str):
        """편의 함수: ERROR 로그"""
        self.logger.error(f"[{self.__class__.__name__}] {message}")
4.2 arm_interface.py (DAC 통신)
파일 위치: dobby_main_controller/interfaces/arm_interface.py
python"""
Arm Interface - DMC와 DAC(Dobby Arm Controller) 간 통신
"""
from typing import Optional
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose

# 커스텀 액션 임포트 (실제로는 dobby_interfaces에서)
# from dobby_interfaces.action import PickBook, PlaceBook
# 임시로 주석 처리 - 메시지 생성 후 주석 해제

from .base_interface import BaseInterface


class ArmInterface(BaseInterface):
    """
    DAC(팔 컨트롤러)와 통신하기 위한 인터페이스
    
    기능:
    - pick_book: 책 집기
    - place_book: 책 놓기  
    - change_pose: 팔 자세 변경
    """
    
    def __init__(self, node):
        super().__init__(node)
        
        # Action Clients (초기화는 async initialize()에서)
        self.pick_client = None
        self.place_client = None
        
        # Service Client
        self.change_pose_client = None
    
    async def initialize(self):
        """
        Action Client 및 Service Client 초기화
        
        주의: 이 메서드는 반드시 async로 호출되어야 함
        """
        self.log_info("Initializing ArmInterface...")
        
        # TODO: 실제 액션 타입으로 교체
        # from dobby_interfaces.action import PickBook, PlaceBook
        
        # Action Client 생성
        # self.pick_client = ActionClient(
        #     self.node,
        #     PickBook,
        #     self._get_topic_name('arm/pick_book')
        # )
        # self.place_client = ActionClient(
        #     self.node,
        #     PlaceBook,
        #     self._get_topic_name('arm/place_book')
        # )
        
        # 서버 대기 (타임아웃 5초)
        # await self._wait_for_servers()
        
        self.log_info("ArmInterface initialized")
    
    async def _wait_for_servers(self, timeout_sec: float = 5.0):
        """Action Server가 준비될 때까지 대기"""
        self.log_info("Waiting for arm action servers...")
        
        # pick_ready = self.pick_client.wait_for_server(timeout_sec=timeout_sec)
        # place_ready = self.place_client.wait_for_server(timeout_sec=timeout_sec)
        
        # if not pick_ready:
        #     self.log_warn("Pick action server not available")
        # if not place_ready:
        #     self.log_warn("Place action server not available")
    
    async def pick_book(
        self, 
        book_id: str, 
        book_pose: Pose, 
        carrier_slot_id: int
    ) -> dict:
        """
        책을 집는 명령 전송
        
        Args:
            book_id: 책 ID
            book_pose: 책의 3D 위치
            carrier_slot_id: 로봇 내부 보관함 슬롯 번호
            
        Returns:
            dict: {'success': bool, 'message': str, 'book_id': str}
        """
        self.log_info(f"Requesting to pick book: {book_id}")
        
        # TODO: 실제 액션 호출
        # goal = PickBook.Goal()
        # goal.book_id = book_id
        # goal.book_pose = book_pose
        # goal.carrier_slot_id = carrier_slot_id
        
        # try:
        #     # 비동기 전송
        #     goal_handle = await self.pick_client.send_goal_async(goal)
        #     
        #     if not goal_handle.accepted:
        #         return {'success': False, 'message': 'Goal rejected'}
        #     
        #     # 결과 대기
        #     result = await goal_handle.get_result_async()
        #     
        #     return {
        #         'success': result.result.success,
        #         'message': result.result.message,
        #         'book_id': result.result.book_id
        #     }
        # except Exception as e:
        #     self.log_error(f"Pick book failed: {e}")
        #     return {'success': False, 'message': str(e)}
        
        # 임시 반환 (메시지 생성 전)
        return {'success': True, 'message': 'Mock success', 'book_id': book_id}
    
    async def place_book(
        self, 
        book_id: str,
        carrier_slot_id: int,
        storage_pose: Pose,
        storage_id: int
    ) -> dict:
        """
        책을 놓는 명령 전송
        
        Args:
            book_id: 책 ID
            carrier_slot_id: 로봇 내부 보관함 슬롯 번호
            storage_pose: 목표 저장 위치
            storage_id: 저장함 ID
            
        Returns:
            dict: {'success': bool, 'message': str}
        """
        self.log_info(f"Requesting to place book: {book_id}")
        
        # TODO: 실제 구현 (pick_book과 유사)
        
        return {'success': True, 'message': 'Mock success'}
    
    async def change_pose(self, pose_type: str) -> dict:
        """
        팔 자세 변경 (관측 자세, 초기 자세 등)
        
        Args:
            pose_type: 'observation', 'initial', 'custom'
            
        Returns:
            dict: {'success': bool, 'message': str}
        """
        self.log_info(f"Requesting pose change: {pose_type}")
        
        # TODO: Service 호출 구현
        
        return {'success': True, 'message': 'Mock success'}
4.3 drive_interface.py (DDC 통신)
파일 위치: dobby_main_controller/interfaces/drive_interface.py
python"""
Drive Interface - DMC와 DDC(Dobby Drive Controller) 간 통신
"""
from typing import Optional
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose

from .base_interface import BaseInterface


class DriveInterface(BaseInterface):
    """
    DDC(주행 컨트롤러)와 통신하기 위한 인터페이스
    
    기능:
    - move_to_target: 목적지 이동
    - guide_navigation: 사용자 안내 주행
    - stop: 긴급 정지
    """
    
    def __init__(self, node):
        super().__init__(node)
        
        # Action Clients
        self.move_client = None
        self.guide_client = None
        
        # Service Client
        self.control_client = None
    
    async def initialize(self):
        """Action Client 초기화"""
        self.log_info("Initializing DriveInterface...")
        
        # TODO: 실제 액션 클라이언트 생성
        # from dobby_interfaces.action import MoveToTarget, GuideNavigation
        
        # self.move_client = ActionClient(
        #     self.node,
        #     MoveToTarget,
        #     self._get_topic_name('drive/move_to_target')
        # )
        
        self.log_info("DriveInterface initialized")
    
    async def move_to_target(
        self, 
        target_pose: Pose, 
        location_name: str = ""
    ) -> dict:
        """
        특정 위치로 이동
        
        Args:
            target_pose: 목표 위치 (x, y, orientation)
            location_name: 위치 이름 (예: "책장_A")
            
        Returns:
            dict: {'success': bool, 'final_pose': Pose, 'message': str}
        """
        self.log_info(f"Moving to: {location_name} ({target_pose.position.x}, {target_pose.position.y})")
        
        # TODO: 실제 액션 호출
        
        return {'success': True, 'final_pose': target_pose, 'message': 'Arrived'}
    
    async def guide_navigation(
        self, 
        destination_pose: Pose
    ) -> dict:
        """
        사용자 안내 주행 (사람을 따라가며 목적지로 이동)
        
        Args:
            destination_pose: 최종 목적지
            
        Returns:
            dict: {'success': bool, 'message': str}
        """
        self.log_info("Starting guide navigation")
        
        # TODO: 실제 구현
        
        return {'success': True, 'message': 'Guidance completed'}
    
    async def stop(self) -> dict:
        """
        긴급 정지
        
        Returns:
            dict: {'success': bool}
        """
        self.log_info("Emergency stop requested")
        
        # TODO: Service 호출
        
        return {'success': True}
4.4 ai_interface.py (AIS 통신)
파일 위치: dobby_main_controller/interfaces/ai_interface.py
python"""
AI Interface - DMC와 AIS(AI Image Service) 간 통신
"""
from typing import Optional
from geometry_msgs.msg import Pose

from .base_interface import BaseInterface


class AIInterface(BaseInterface):
    """
    AIS(AI 서비스)와 통신하기 위한 인터페이스
    
    기능:
    - detect_book: 책 감지
    - check_storage_box: 보관함 확인
    - detect_trash: 쓰레기 감지
    - 추적 상태 구독
    """
    
    def __init__(self, node):
        super().__init__(node)
        
        # Service Clients
        self.detect_book_client = None
        self.check_storage_client = None
        self.detect_trash_client = None
        
        # Topic Subscriber
        self.tracking_subscriber = None
        self.latest_tracking = None
    
    async def initialize(self):
        """Service Client 및 Subscriber 초기화"""
        self.log_info("Initializing AIInterface...")
        
        # TODO: Service 클라이언트 생성
        # from dobby_interfaces.srv import DetectBook, CheckStorageBox
        
        # self.detect_book_client = self.node.create_client(
        #     DetectBook,
        #     self._get_topic_name('ai/detect_book')
        # )
        
        # Topic 구독
        # from dobby_interfaces.msg import TrackingStatus
        # self.tracking_subscriber = self.node.create_subscription(
        #     TrackingStatus,
        #     self._get_topic_name('ai/tracking/status'),
        #     self._tracking_callback,
        #     10
        # )
        
        self.log_info("AIInterface initialized")
    
    def _tracking_callback(self, msg):
        """AI가 보내는 추적 상태를 계속 수신"""
        self.latest_tracking = msg
        
        if not msg.person_detected:
            self.log_warn("Person lost in tracking!")
    
    async def detect_book(self, book_id: str) -> dict:
        """
        AI에게 책 위치 감지 요청
        
        Args:
            book_id: 감지할 책 ID
            
        Returns:
            dict: {
                'detected': bool,
                'book_id': str,
                'book_pose': Pose,
                'confidence': float
            }
        """
        self.log_info(f"Requesting book detection: {book_id}")
        
        # TODO: Service 호출
        
        return {
            'detected': True,
            'book_id': book_id,
            'book_pose': Pose(),
            'confidence': 0.95
        }
    
    async def check_storage_box(self, storage_id: int) -> dict:
        """
        보관함 상태 확인
        
        Args:
            storage_id: 보관함 ID
            
        Returns:
            dict: {
                'is_valid': bool,
                'storage_id': int,
                'is_empty': bool,
                'storage_pose': Pose
            }
        """
        self.log_info(f"Checking storage box: {storage_id}")
        
        # TODO: Service 호출
        
        return {
            'is_valid': True,
            'storage_id': storage_id,
            'is_empty': True,
            'storage_pose': Pose()
        }
    
    async def detect_trash(self, seat_location: Pose) -> dict:
        """
        좌석의 쓰레기 감지
        
        Args:
            seat_location: 좌석 위치
            
        Returns:
            dict: {
                'trash_found': bool,
                'trash_count': int,
                'trash_types': list,
                'trash_poses': list
            }
        """
        self.log_info("Detecting trash")
        
        # TODO: Service 호출
        
        return {
            'trash_found': False,
            'trash_count': 0,
            'trash_types': [],
            'trash_poses': []
        }
    
    def get_tracking_status(self) -> Optional[dict]:
        """
        최신 추적 상태 조회
        
        Returns:
            dict 또는 None
        """
        if self.latest_tracking is None:
            return None
        
        return {
            'person_detected': self.latest_tracking.person_detected,
            'tracking_id': self.latest_tracking.tracking_id,
            'distance': self.latest_tracking.distance_to_person,
            'is_lost': self.latest_tracking.is_lost
        }

5. 4단계: Mock Interfaces 구현
5.1 mock_interfaces.py
파일 위치: dobby_main_controller/mock/mock_interfaces.py
python"""
Mock Interfaces for Testing - 실제 하드웨어 없이 테스트용
"""
import asyncio
from typing import Optional
from geometry_msgs.msg import Pose

from ..interfaces.base_interface import BaseInterface


class MockArmInterface(BaseInterface):
    """
    테스트용 가짜 팔 인터페이스
    즉시 성공 반환 또는 설정된 시나리오대로 동작
    """
    
    def __init__(self, node):
        super().__init__(node)
        self.success_rate = 1.0  # 성공 확률 (0.0 ~ 1.0)
        self.delay_sec = 2.0     # 시뮬레이션 지연 시간
    
    async def initialize(self):
        self.log_info("MockArmInterface initialized (Test Mode)")
    
    async def pick_book(self, book_id: str, book_pose: Pose, carrier_slot_id: int) -> dict:
        """가짜 책 집기 - 지연 후 성공 반환"""
        self.log_info(f"[MOCK] Picking book: {book_id}")
        
        # 실제 작업 시뮬레이션 (2초 대기)
        await asyncio.sleep(self.delay_sec)
        
        import random
        success = random.random() < self.success_rate
        
        if success:
            self.log_info(f"[MOCK] Book picked successfully: {book_id}")
            return {
                'success': True,
                'message': 'Mock pick succeeded',
                'book_id': book_id
            }
        else:
            self.log_warn(f"[MOCK] Failed to pick book: {book_id}")
            return {
                'success': False,
                'message': 'Mock pick failed',
                'book_id': book_id
            }
    
    async def place_book(
        self, 
        book_id: str, 
        carrier_slot_id: int, 
        storage_pose: Pose, 
        storage_id: int
    ) -> dict:
        """가짜 책 놓기"""
        self.log_info(f"[MOCK] Placing book: {book_id}")
        await asyncio.sleep(self.delay_sec)
        
        return {'success': True, 'message': 'Mock place succeeded'}
    
    async def change_pose(self, pose_type: str) -> dict:
        """가짜 자세 변경"""
        self.log_info(f"[MOCK] Changing pose to: {pose_type}")
        await asyncio.sleep(0.5)
        
        return {'success': True, 'message': 'Mock pose change succeeded'}


class MockDriveInterface(BaseInterface):
    """테스트용 가짜 주행 인터페이스"""
    
    def __init__(self, node):
        super().__init__(node)
        self.success_rate = 1.0
        self.delay_sec = 3.0  # 이동은 조금 더 오래 걸림
    
    async def initialize(self):
        self.log_info("MockDriveInterface initialized (Test Mode)")
    
    async def move_to_target(self, target_pose: Pose, location_name: str = "") -> dict:
        """가짜 이동"""
        self.log_info(f"[MOCK] Moving to: {location_name}")
        await asyncio.sleep(self.delay_sec)
        
        return {
            'success': True,
            'final_pose': target_pose,
            'message': f'Mock arrived at {location_name}'
        }
    
    async def guide_navigation(self, destination_pose: Pose) -> dict:
        """가짜 안내 주행"""
        self.log_info("[MOCK] Guide navigation started")
        await asyncio.sleep(self.delay_sec)
        
        return {'success': True, 'message': 'Mock guidance completed'}
    
    async def stop(self) -> dict:
        """가짜 정지"""
        self.log_info("[MOCK] Emergency stop")
        return {'success': True}


class MockAIInterface(BaseInterface):
    """테스트용 가짜 AI 인터페이스"""
    
    def __init__(self, node):
        super().__init__(node)
        self.success_rate = 1.0
        self.latest_tracking = None
    
    async def initialize(self):
        self.log_info("MockAIInterface initialized (Test Mode)")
    
    async def detect_book(self, book_id: str) -> dict:
        """가짜 책 감지"""
        self.log_info(f"[MOCK] Detecting book: {book_id}")
        await asyncio.sleep(1.0)
        
        # 가짜 좌표 생성
        mock_pose = Pose()
        mock_pose.position.x = 5.0
        mock_pose.position.y = 2.0
        mock_pose.position.z = 1.5
        
        return {
            'detected': True,
            'book_id': book_id,
            'book_pose': mock_pose,
            'confidence': 0.95
        }
    
    async def check_storage_box(self, storage_id: int) -> dict:
        """가짜 보관함 확인"""
        self.log_info(f"[MOCK] Checking storage: {storage_id}")
        await asyncio.sleep(0.5)
        
        return {
            'is_valid': True,
            'storage_id': storage_id,
            'is_empty': True,
            'storage_pose': Pose()
        }
    
    async def detect_trash(self, seat_location: Pose) -> dict:
        """가짜 쓰레기 감지"""
        self.log_info("[MOCK] Detecting trash")
        await asyncio.sleep(1.0)
        
        return {
            'trash_found': False,
            'trash_count': 0,
            'trash_types': [],
            'trash_poses': []
        }
    
    def get_tracking_status(self) -> Optional[dict]:
        """가짜 추적 상태"""
        return {
            'person_detected': True,
            'tracking_id': 'mock_person_1',
            'distance': 2.5,
            'is_lost': False
        }

6. 5단계: State Machine 구현
6.1 transitions 라이브러리 설치
bashpip3 install transitions
6.2 state_machine.py
파일 위치: dobby_main_controller/core/state_machine.py
python"""
State Machine Manager - Dobby의 상태 관리
"""
from transitions import Machine
from typing import Callable, Optional
import rclpy


class DobbyStateMachine:
    """
    Dobby의 상태를 관리하는 State Machine
    
    Main States:
    - INITIALIZING: 초기화 중
    - CHARGING: 충전 중
    - IDLE: 대기
    - PICKING_UP_BOOK: 도서 픽업 중
    - RESHELVING_BOOK: 반납 도서 정리 중
    - GUIDING: 길 안내 중
    - MOVING_TO_CHARGER: 충전기로 이동 중
    - FORCE_MOVE_TO_CHARGER: 강제 충전 이동
    - ERROR: 에러 상태
    """
    
    # 상태 정의
    states = [
        'INITIALIZING',
        'CHARGING',
        'IDLE',
        {
            'name': 'PICKING_UP_BOOK',
            'children': [
                'MOVE_TO_PICKUP',
                'PICKING_BOOK', 
                'MOVE_TO_STORAGE',
                'STOWING_BOOK'
            ]
        },
        {
            'name': 'RESHELVING_BOOK',
            'children': [
                'MOVE_TO_RETURN_DESK',
                'COLLECT_RETURN_BOOKS',
                'MOVE_TO_PLACE_SHELF',
                'PLACE_RETURN_BOOK'
            ]
        },
        {
            'name': 'GUIDING',
            'children': [
                'SELECT_DEST',
                'SCAN_USER',
                'GUIDING_TO_DEST',
                'FIND_USER'
            ]
        },
        'MOVING_TO_CHARGER',
        'FORCE_MOVE_TO_CHARGER',
        'ERROR'
    ]
    
    def __init__(self, logger=None):
        """
        Args:
            logger: ROS2 logger (선택사항)
        """
        self.logger = logger
        
        # Machine 초기화
        self.machine = Machine(
            model=self,
            states=DobbyStateMachine.states,
            initial='INITIALIZING',
            auto_transitions=False,  # 자동 전이 비활성화
            ignore_invalid_triggers=True  # 잘못된 전이 무시
        )
        
        # 전이(Transition) 정의
        self._define_transitions()
        
        # 현재 서브 상태 추적
        self.current_sub_state = 'NONE'
    
    def _define_transitions(self):
        """
        상태 전이 규칙 정의
        
        format: {'trigger': '메서드명', 'source': '출발상태', 'dest': '도착상태'}
        """
        transitions = [
            # 초기화 -> 충전
            {'trigger': 'complete_init', 'source': 'INITIALIZING', 'dest': 'CHARGING'},
            
            # 충전 -> 대기
            {'trigger': 'charge_complete', 'source': 'CHARGING', 'dest': 'IDLE'},
            
            # 대기 -> 각 작업
            {'trigger': 'start_pickup', 'source': 'IDLE', 'dest': 'PICKING_UP_BOOK_MOVE_TO_PICKUP'},
            {'trigger': 'start_reshelving', 'source': 'IDLE', 'dest': 'RESHELVING_BOOK_MOVE_TO_RETURN_DESK'},
            {'trigger': 'start_guiding', 'source': 'IDLE', 'dest': 'GUIDING_SELECT_DEST'},
            
            # 픽업 작업 내부 전이
            {
                'trigger': 'arrived_at_pickup',
                'source': 'PICKING_UP_BOOK_MOVE_TO_PICKUP',
                'dest': 'PICKING_UP_BOOK_PICKING_BOOK'
            },
            {
                'trigger': 'book_picked',
                'source': 'PICKING_UP_BOOK_PICKING_BOOK',
                'dest': 'PICKING_UP_BOOK_MOVE_TO_STORAGE'
            },
            {
                'trigger': 'arrived_at_storage',
                'source': 'PICKING_UP_BOOK_MOVE_TO_STORAGE',
                'dest': 'PICKING_UP_BOOK_STOWING_BOOK'
            },
            {
                'trigger': 'stowing_complete',
                'source': 'PICKING_UP_BOOK_STOWING_BOOK',
                'dest': 'IDLE'
            },
            
            # 반납 작업 내부 전이
            {
                'trigger': 'arrived_at_return_desk',
                'source': 'RESHELVING_BOOK_MOVE_TO_RETURN_DESK',
                'dest': 'RESHELVING_BOOK_COLLECT_RETURN_BOOKS'
            },
            {
                'trigger': 'collection_complete',
                'source': 'RESHELVING_BOOK_COLLECT_RETURN_BOOKS',
                'dest': 'RESHELVING_BOOK_MOVE_TO_PLACE_SHELF'
            },
            {
                'trigger': 'arrived_at_shelf',
                'source': 'RESHELVING_BOOK_MOVE_TO_PLACE_SHELF',
                'dest': 'RESHELVING_BOOK_PLACE_RETURN_BOOK'
            },
            {
                'trigger': 'placement_complete',
                'source': 'RESHELVING_BOOK_PLACE_RETURN_BOOK',
                'dest': 'IDLE'
            },
            
            # 길 안내 내부 전이
            {
                'trigger': 'destination_selected',
                'source': 'GUIDING_SELECT_DEST',
                'dest': 'GUIDING_SCAN_USER'
            },
            {
                'trigger': 'user_recognized',
                'source': 'GUIDING_SCAN_USER',
                'dest': 'GUIDING_GUIDING_TO_DEST'
            },
            {
                'trigger': 'user_lost',
                'source': 'GUIDING_GUIDING_TO_DEST',
                'dest': 'GUIDING_FIND_USER'
            },
            {
                'trigger': 'user_found',
                'source': 'GUIDING_FIND_USER',
                'dest': 'GUIDING_GUIDING_TO_DEST'
            },
            {
                'trigger': 'guidance_complete',
                'source': 'GUIDING_GUIDING_TO_DEST',
                'dest': 'IDLE'
            },
            
            # 배터리 관련
            {
                'trigger': 'low_battery',
                'source': '*',  # 모든 상태에서
                'dest': 'FORCE_MOVE_TO_CHARGER'
            },
            {
                'trigger': 'arrived_at_charger',
                'source': ['MOVING_TO_CHARGER', 'FORCE_MOVE_TO_CHARGER'],
                'dest': 'CHARGING'
            },
            
            # 에러
            {'trigger': 'error_occurred', 'source': '*', 'dest': 'ERROR'},
            {'trigger': 'error_resolved', 'source': 'ERROR', 'dest': 'IDLE'}
        ]
        
        for transition in transitions:
            self.machine.add_transition(**transition)
    
    def get_current_state(self) -> str:
        """
        현재 상태 조회
        
        Returns:
            현재 상태 문자열 (예: 'PICKING_UP_BOOK_MOVE_TO_PICKUP')
        """
        return self.state
    
    def get_main_state(self) -> str:
        """
        메인 상태만 조회 (서브 상태 제외)
        
        Returns:
            메인 상태 (예: 'PICKING_UP_BOOK')
        """
        state = self.state
        
        # '_'로 분리해서 첫 부분 추출
        if '_' in state and state.count('_') >= 3:
            # 예: PICKING_UP_BOOK_MOVE_TO_PICKUP -> PICKING_UP_BOOK
            parts = state.split('_')
            
            # PICKING_UP_BOOK, RESHELVING_BOOK 등 처리
            if 'PICKING' in state:
                return 'PICKING_UP_BOOK'
            elif 'RESHELVING' in state:
                return 'RESHELVING_BOOK'
            elif 'GUIDING' in state:
                return 'GUIDING'
        
        return state
    
    def get_sub_state(self) -> str:
        """
        서브 상태만 조회
        
        Returns:
            서브 상태 (예: 'MOVE_TO_PICKUP') 또는 'NONE'
        """
        state = self.state
        
        if '_' not in state:
            return 'NONE'
        
        # 메인 상태를 제외한 나머지 반환
        main = self.get_main_state()
        if state.startswith(main):
            sub = state[len(main)+1:]  # +1은 '_' 제거
            return sub if sub else 'NONE'
        
        return 'NONE'
    
    def log_transition(self, message: str):
        """상태 전이 로그"""
        if self.logger:
            self.logger.info(f"[StateMachine] {message}")
        else:
            print(f"[StateMachine] {message}")
    
    # Callbacks (상태 진입 시 자동 호출)
    def on_enter_IDLE(self):
        self.log_transition("Entered IDLE state - Ready for new task")
    
    def on_enter_PICKING_UP_BOOK_MOVE_TO_PICKUP(self):
        self.log_transition("Starting pickup task - Moving to pickup location")
    
    def on_enter_PICKING_UP_BOOK_PICKING_BOOK(self):
        self.log_transition("Picking book with arm")
    
    def on_enter_ERROR(self):
        self.log_transition("ERROR state entered!")
    
    def on_enter_FORCE_MOVE_TO_CHARGER(self):
        self.log_transition("CRITICAL: Low battery - Force moving to charger")

7. 6단계: Task Coordinator 구현
7.1 task_coordinator.py
파일 위치: dobby_main_controller/core/task_coordinator.py
python"""
Task Coordinator - 작업 실행 조율
"""
import asyncio
from typing import Dict, Any, Optional
from geometry_msgs.msg import Pose

from .state_machine import DobbyStateMachine


class TaskCoordinator:
    """
    로봇의 작업을 조율하고 실행하는 클래스
    
    역할:
    - RCS로부터 받은 작업 목표 해석
    - State Machine과 Interface를 연결
    - 작업 단계별 실행 및 피드백
    """
    
    def __init__(self, node, state_machine: DobbyStateMachine):
        """
        Args:
            node: ROS2 노드
            state_machine: 상태 관리 객체
        """
        self.node = node
        self.logger = node.get_logger()
        self.state_machine = state_machine
        
        # Interfaces (나중에 외부에서 설정)
        self.arm = None
        self.drive = None
        self.ai = None
        
        # 배터리 관리
        self.battery_level = 100.0
        
        # 현재 작업 정보
        self.current_task = None
        self.task_result = {}
    
    def set_interfaces(self, arm, drive, ai):
        """
        인터페이스 설정
        
        Args:
            arm: ArmInterface
            drive: DriveInterface
            ai: AIInterface
        """
        self.arm = arm
        self.drive = drive
        self.ai = ai
        
        self.logger.info("Interfaces connected to TaskCoordinator")
    
    async def execute_pickup_task(self, goal) -> Dict[str, Any]:
        """
        도서 픽업 작업 실행
        
        Args:
            goal: PickupBook.Goal (task_id, book_id, poses 등)
            
        Returns:
            dict: 작업 결과
        """
        self.logger.info(f"Starting pickup task: {goal.book_id}")
        self.current_task = {
            'type': 'pickup',
            'task_id': goal.task_id,
            'book_id': goal.book_id
        }
        
        # 배터리 체크
        if not self._check_battery_level():
            return self._create_error_result("Low battery")
        
        try:
            # 1. 상태: MOVE_TO_PICKUP
            self.state_machine.start_pickup()
            self.logger.info("[1/4] Moving to pickup location")
            
            result = await self.drive.move_to_target(
                target_pose=goal.shelf_approach_pose,
                location_name=f"Shelf for {goal.book_id}"
            )
            
            if not result['success']:
                return self._create_error_result("Failed to move to pickup location")
            
            # 2. 상태: PICKING_BOOK
            self.state_machine.arrived_at_pickup()
            self.logger.info("[2/4] Picking book")
            
            # AI에게 책 위치 확인
            book_info = await self.ai.detect_book(goal.book_id)
            if not book_info['detected']:
                return self._create_error_result("Book not detected")
            
            # 팔로 책 집기
            pick_result = await self.arm.pick_book(
                book_id=goal.book_id,
                book_pose=book_info['book_pose'],
                carrier_slot_id=1  # 첫 번째 슬롯
            )
            
            if not pick_result['success']:
                return self._create_error_result("Failed to pick book")
            
            # 3. 상태: MOVE_TO_STORAGE
            self.state_machine.book_picked()
            self.logger.info("[3/4] Moving to storage location")
            
            result = await self.drive.move_to_target(
                target_pose=goal.storage_approach_pose,
                location_name=f"Storage {goal.storage_id}"
            )
            
            if not result['success']:
                return self._create_error_result("Failed to move to storage")
            
            # 4. 상태: STOWING_BOOK
            self.state_machine.arrived_at_storage()
            self.logger.info("[4/4] Stowing book in storage")
            
            # 보관함 확인
            storage_info = await self.ai.check_storage_box(goal.storage_id)
            if not storage_info['is_valid']:
                return self._create_error_result("Invalid storage box")
            
            # 책 보관
            place_result = await self.arm.place_book(
                book_id=goal.book_id,
                carrier_slot_id=1,
                storage_pose=goal.storage_slot_pose,
                storage_id=goal.storage_id
            )
            
            if not place_result['success']:
                return self._create_error_result("Failed to place book")
            
            # 완료!
            self.state_machine.stowing_complete()
            self.logger.info("Pickup task completed successfully!")
            
            return {
                'task_id': goal.task_id,
                'success': True,
                'message': 'Pickup completed',
                'total_distance_m': 0.0,  # TODO: 실제 계산
                'total_time_sec': 0.0
            }
            
        except Exception as e:
            self.logger.error(f"Pickup task failed: {e}")
            self.state_machine.error_occurred()
            return self._create_error_result(str(e))
    
    async def execute_reshelving_task(self, goal) -> Dict[str, Any]:
        """
        반납 도서 정리 작업 실행
        
        Args:
            goal: ReshelvingBook.Goal
            
        Returns:
            dict: 작업 결과
        """
        self.logger.info("Starting reshelving task")
        self.current_task = {
            'type': 'reshelving',
            'task_id': goal.task_id
        }
        
        # 배터리 체크
        if not self._check_battery_level():
            return self._create_error_result("Low battery")
        
        try:
            # 1. 반납대로 이동
            self.state_machine.start_reshelving()
            self.logger.info("[1/4] Moving to return desk")
            
            result = await self.drive.move_to_target(
                target_pose=goal.return_desk_pose,
                location_name="Return Desk"
            )
            
            if not result['success']:
                return self._create_error_result("Failed to move to return desk")
            
            # 2. 반납 도서 수거
            self.state_machine.arrived_at_return_desk()
            self.logger.info("[2/4] Collecting returned books")
            
            # TODO: 실제 구현 (여러 책 수거)
            await asyncio.sleep(3)
            
            # 3. 책장으로 이동
            self.state_machine.collection_complete()
            self.logger.info("[3/4] Moving to bookshelf")
            
            # TODO: 책장 위치 계산
            shelf_pose = Pose()
            result = await self.drive.move_to_target(
                target_pose=shelf_pose,
                location_name="Bookshelf A"
            )
            
            # 4. 책 배치
            self.state_machine.arrived_at_shelf()
            self.logger.info("[4/4] Placing books on shelf")
            
            # TODO: 실제 배치 로직
            await asyncio.sleep(3)
            
            # 완료
            self.state_machine.placement_complete()
            self.logger.info("Reshelving task completed!")
            
            return {
                'task_id': goal.task_id,
                'success': True,
                'message': 'Reshelving completed',
                'books_processed': 0,
                'failed_book_ids': [],
                'total_distance_m': 0.0,
                'total_time_sec': 0.0
            }
            
        except Exception as e:
            self.logger.error(f"Reshelving task failed: {e}")
            self.state_machine.error_occurred()
            return self._create_error_result(str(e))
    
    async def execute_guide_task(self, goal) -> Dict[str, Any]:
        """
        길 안내 작업 실행
        
        Args:
            goal: GuidePerson.Goal
            
        Returns:
            dict: 작업 결과
        """
        self.logger.info("Starting guide task")
        self.current_task = {
            'type': 'guiding',
            'task_id': goal.task_id
        }
        
        if not self._check_battery_level():
            return self._create_error_result("Low battery")
        
        try:
            # 1. 목적지 선택 (이미 goal에 포함)
            self.state_machine.start_guiding()
            self.logger.info("[1/3] Destination set")
            
            # 2. 사용자 인식
            self.state_machine.destination_selected()
            self.logger.info("[2/3] Scanning for user")
            
            # TODO: 사용자 인식 로직
            await asyncio.sleep(2)
            
            # 3. 안내 시작
            self.state_machine.user_recognized()
            self.logger.info("[3/3] Guiding to destination")
            
            result = await self.drive.guide_navigation(goal.destination_pose)
            
            if not result['success']:
                return self._create_error_result("Guide navigation failed")
            
            # 완료
            self.state_machine.guidance_complete()
            self.logger.info("Guide task completed!")
            
            return {
                'task_id': goal.task_id,
                'success': True,
                'error_code': '',
                'total_distance_m': 0.0,
                'total_time_sec': 0.0
            }
            
        except Exception as e:
            self.logger.error(f"Guide task failed: {e}")
            self.state_machine.error_occurred()
            return self._create_error_result(str(e))
    
    def _check_battery_level(self) -> bool:
        """
        배터리 레벨 확인
        
        Returns:
            bool: 작업 가능 여부
        """
        # TODO: 실제 배터리 토픽 구독
        
        if self.battery_level < 20.0:
            self.logger.warning(f"Battery too low: {self.battery_level}%")
            self.state_machine.low_battery()
            return False
        
        return True
    
    def _create_error_result(self, message: str) -> Dict[str, Any]:
        """에러 결과 생성"""
        return {
            'task_id': self.current_task.get('task_id', 0),
            'success': False,
            'message': message
        }
    
    def update_battery(self, level: float):
        """
        배터리 레벨 업데이트 (외부에서 호출)
        
        Args:
            level: 배터리 레벨 (0~100)
        """
        self.battery_level = level
        
        # 자동 충전 체크
        if level < 20.0:
            current_state = self.state_machine.get_main_state()
            if current_state not in ['CHARGING', 'FORCE_MOVE_TO_CHARGER']:
                self.logger.warning("Battery critical! Moving to charger")
                self.state_machine.low_battery()

8. 7단계: Main Controller Node 구현
8.1 dobby_main_controller_node.py
파일 위치: dobby_main_controller/dobby_main_controller_node.py
python"""
Dobby Main Controller Node - 메인 진입점
"""
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer
import asyncio

# 커스텀 모듈
from .core.state_machine import DobbyStateMachine
from .core.task_coordinator import TaskCoordinator

# Interfaces
from .interfaces.arm_interface import ArmInterface
from .interfaces.drive_interface import DriveInterface
from .interfaces.ai_interface import AIInterface

# Mock Interfaces
from .mock.mock_interfaces import (
    MockArmInterface,
    MockDriveInterface,
    MockAIInterface
)

# 메시지 (TODO: 실제 메시지로 교체)
# from dobby_interfaces.action import PickupBook, ReshelvingBook, GuidePerson
# from dobby_interfaces.msg import DobbyState


class DobbyMainController(Node):
    """
    Dobby Main Controller
    
    역할:
    - 전체 시스템 통합
    - RCS와의 통신 (Action Server)
    - 상태 발행 (Topic Publisher)
    """
    
    def __init__(self):
        super().__init__('dobby_main_controller')
        
        self.get_logger().info("=== Dobby Main Controller Starting ===")
        
        # 1. 파라미터 선언
        self._declare_parameters()
        
        # 2. State Machine 초기화
        self.state_machine = DobbyStateMachine(logger=self.get_logger())
        
        # 3. Interfaces 초기화
        self._init_interfaces()
        
        # 4. Task Coordinator 초기화
        self.task_coordinator = TaskCoordinator(self, self.state_machine)
        self.task_coordinator.set_interfaces(
            self.arm_interface,
            self.drive_interface,
            self.ai_interface
        )
        
        # 5. RCS 통신 (Action Servers, Publishers)
        self._init_rcs_communication()
        
        # 6. 타이머 (상태 발행)
        self.state_timer = self.create_timer(0.1, self._publish_state)  # 10Hz
        
        self.get_logger().info("=== Dobby Main Controller Ready ===")
    
    def _declare_parameters(self):
        """파라미터 선언 및 읽기"""
        # 로봇 ID
        self.declare_parameter('robot_id', 'dobby1')
        self.robot_id = self.get_parameter('robot_id').value
        
        # 테스트 모드 (Mock 사용 여부)
        self.declare_parameter('test_mode', False)
        self.test_mode = self.get_parameter('test_mode').value
        
        self.get_logger().info(f"Robot ID: {self.robot_id}")
        self.get_logger().info(f"Test Mode: {self.test_mode}")
    
    def _init_interfaces(self):
        """Interface 초기화"""
        if self.test_mode:
            self.get_logger().info("Using MOCK Interfaces (Test Mode)")
            self.arm_interface = MockArmInterface(self)
            self.drive_interface = MockDriveInterface(self)
            self.ai_interface = MockAIInterface(self)
        else:
            self.get_logger().info("Using REAL Interfaces")
            self.arm_interface = ArmInterface(self)
            self.drive_interface = DriveInterface(self)
            self.ai_interface = AIInterface(self)
        
        # 비동기 초기화 (TODO: 나중에 실행)
        # asyncio.run(self.arm_interface.initialize())
        # asyncio.run(self.drive_interface.initialize())
        # asyncio.run(self.ai_interface.initialize())
    
    def _init_rcs_communication(self):
        """RCS와의 통신 설정"""
        self.get_logger().info("Initializing RCS communication")
        
        # TODO: Action Servers 생성
        # self.pickup_server = ActionServer(
        #     self,
        #     PickupBook,
        #     f'{self.robot_id}/action/pickup_book',
        #     self._handle_pickup_request
        # )
        
        # TODO: State Publisher 생성
        # self.state_publisher = self.create_publisher(
        #     DobbyState,
        #     f'{self.robot_id}/status/robot_state',
        #     10
        # )
        
        self.get_logger().info("RCS communication initialized")
    
    async def _handle_pickup_request(self, goal_handle):
        """
        RCS로부터 픽업 요청 수신
        
        Args:
            goal_handle: Action goal handle
            
        Returns:
            Result
        """
        self.get_logger().info("Received pickup request from RCS")
        goal = goal_handle.request
        
        # Task Coordinator에게 실행 요청
        result = await self.task_coordinator.execute_pickup_task(goal)
        
        # 성공/실패 처리
        if result['success']:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        
        # TODO: 실제 Result 메시지로 변환
        # return PickupBook.Result(**result)
        return result
    
    def _publish_state(self):
        """현재 상태를 RCS에 발행 (10Hz)"""
        # TODO: 실제 메시지로 발행
        # msg = DobbyState()
        # msg.main_state = self._get_main_state_code()
        # msg.sub_state = self._get_sub_state_code()
        # msg.is_error = (self.state_machine.get_main_state() == 'ERROR')
        # self.state_publisher.publish(msg)
        pass
    
    def _get_main_state_code(self) -> int:
        """상태 문자열을 코드로 변환"""
        state_map = {
            'INITIALIZING': 0,
            'CHARGING': 1,
            'IDLE': 2,
            'MOVING_TO_CHARGER': 3,
            'PICKING_UP_BOOK': 4,
            'RESHELVING_BOOK': 5,
            'GUIDING': 6,
            'ERROR': 99
        }
        
        main_state = self.state_machine.get_main_state()
        return state_map.get(main_state, 99)


def main(args=None):
    """메인 진입점"""
    rclpy.init(args=args)
    
    try:
        # 노드 생성
        controller = DobbyMainController()
        
        # Executor 설정 (비동기 처리를 위해 MultiThreaded 사용)
        executor = MultiThreadedExecutor()
        executor.add_node(controller)
        
        # 실행
        try:
            executor.spin()
        except KeyboardInterrupt:
            controller.get_logger().info("Keyboard interrupt, shutting down")
        finally:
            executor.shutdown()
            controller.destroy_node()
            
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

9. 8단계: Config 및 Launch 파일
9.1 config/params.yaml
파일 위치: config/params.yaml
yaml# Dobby Main Controller Parameters

# 로봇 설정
robot_id: 'dobby1'
test_mode: false

# 좌표 설정 (예시)
locations:
  charge_station:
    x: 0.0
    y: 0.0
    z: 0.0
  
  return_desk:
    x: 5.0
    y: 2.0
    z: 0.0
  
  bookshelf_a:
    x: 10.0
    y: 3.0
    z: 0.0

# 배터리 임계값
battery:
  critical_level: 20.0  # 강제 충전
  warning_level: 40.0   # 경고
  full_level: 80.0      # 작업 재개 가능

# 타임아웃 설정 (초)
timeouts:
  action_timeout: 60.0
  service_timeout: 5.0
  user_lost_timeout: 30.0

# Mock 설정 (테스트 모드)
mock:
  success_rate: 1.0     # 성공 확률 (0.0~1.0)
  arm_delay: 2.0        # 팔 동작 시뮬레이션 지연
  drive_delay: 3.0      # 주행 시뮬레이션 지연
  ai_delay: 1.0         # AI 처리 시뮬레이션 지연
9.2 launch/dobby_main.launch.py (실제 모드)
파일 위치: launch/dobby_main.launch.py
python"""
Dobby Main Controller Launch (Real Mode)
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 패키지 경로
    pkg_dir = get_package_share_directory('dobby_main_controller')
    config_file = os.path.join(pkg_dir, 'config', 'params.yaml')
    
    # Launch Arguments
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='dobby1',
        description='Robot ID (dobby1, dobby2, ...)'
    )
    
    # Node
    dobby_node = Node(
        package='dobby_main_controller',
        executable='dobby_main_controller',
        name='dobby_main_controller',
        namespace=LaunchConfiguration('robot_id'),
        parameters=[
            config_file,
            {
                'robot_id': LaunchConfiguration('robot_id'),
                'test_mode': False  # 실제 모드
            }
        ],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        robot_id_arg,
        dobby_node
    ])
9.3 launch/dobby_test.launch.py (테스트 모드)
파일 위치: launch/dobby_test.launch.py
python"""
Dobby Main Controller Launch (Test Mode with Mock)
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 패키지 경로
    pkg_dir = get_package_share_directory('dobby_main_controller')
    config_file = os.path.join(pkg_dir, 'config', 'params.yaml')
    
    # Launch Arguments
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='dobby1',
        description='Robot ID'
    )
    
    # Node (TEST MODE)
    dobby_node = Node(
        package='dobby_main_controller',
        executable='dobby_main_controller',
        name='dobby_main_controller',
        namespace=LaunchConfiguration('robot_id'),
        parameters=[
            config_file,
            {
                'robot_id': LaunchConfiguration('robot_id'),
                'test_mode': True  # Mock 사용
            }
        ],
        output='screen',
        emulate_tty=True,
        prefix=['xterm -e']  # 별도 터미널에서 실행 (선택사항)
    )
    
    return LaunchDescription([
        robot_id_arg,
        dobby_node
    ])

10. 9단계: 빌드 및 테스트
10.1 빌드
bash# 작업 공간으로 이동
cd ~/ros2_ws

# dobby_interfaces 먼저 빌드
colcon build --packages-select dobby_interfaces

# dobby_main_controller 빌드
colcon build --packages-select dobby_main_controller

# 소스
source install/setup.bash
10.2 실행 (테스트 모드)
bash# 테스트 모드 실행
ros2 launch dobby_main_controller dobby_test.launch.py

# 또는 dobby2로 실행
ros2 launch dobby_main_controller dobby_test.launch.py robot_id:=dobby2
10.3 상태 확인
bash# 다른 터미널에서

# 노드 확인
ros2 node list

# 토픽 확인
ros2 topic list

# 액션 확인
ros2 action list

# 상태 구독 (TODO: 메시지 생성 후)
# ros2 topic echo /dobby1/status/robot_state
10.4 수동 액션 호출 테스트
bash# TODO: 메시지 생성 후
# ros2 action send_goal /dobby1/action/pickup_book \
#   dobby_interfaces/action/PickupBook \
#   "{task_id: 1, book_id: 'book123', ...}"
```

---

## 11. 10단계: 트러블슈팅

### 문제 1: ModuleNotFoundError

**증상**:
```
ModuleNotFoundError: No module named 'dobby_main_controller.core'
해결:
bash# 빌드 후 반드시 source
cd ~/ros2_ws
source install/setup.bash

# 또는 ~/.bashrc에 추가
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### 문제 2: Action/Service Not Found

**증상**:
```
No module named 'dobby_interfaces.action'
해결:

dobby_interfaces가 빌드되었는지 확인

bashros2 interface list | grep dobby

없으면 빌드

bashcolcon build --packages-select dobby_interfaces
source install/setup.bash
```

### 문제 3: __init__.py 누락

**증상**:
```
ImportError: cannot import name 'ArmInterface'
해결:
모든 디렉토리에 __init__.py 확인
bash# 생성
touch dobby_main_controller/__init__.py
touch dobby_main_controller/core/__init__.py
touch dobby_main_controller/interfaces/__init__.py
touch dobby_main_controller/mock/__init__.py
문제 4: Executor Hanging
증상:
노드가 응답 없음
해결:
MultiThreadedExecutor 사용 확인 (dobby_main_controller_node.py)
문제 5: Namespace 충돌
증상:
dobby1과 dobby2가 같은 토픽 사용
해결:
Launch 파일에서 namespace 설정 확인
pythonnamespace=LaunchConfiguration('robot_id')

12. 체크리스트
Week 1 완료 기준

 프로젝트 구조 생성
 dobby_interfaces 패키지 생성 및 메시지 정의
 BaseInterface 구현
 ArmInterface, DriveInterface, AIInterface 구현
 MockArmInterface, MockDriveInterface, MockAIInterface 구현
 State Machine 기본 상태 (IDLE, CHARGING만) 동작
 Launch 파일로 실행 가능
 로그 출력 확인

Week 2 완료 기준

 도서 픽업 전체 플로우 구현 (State Machine + Task Coordinator)
 Mock으로 픽업 시나리오 테스트 성공
 반납 도서 정리 구현
 배터리 관리 로직 추가

Week 3 완료 기준

 길 안내 구현
 실제 DAC/DDC와 연동 테스트 (가능한 경우)
 에러 처리 추가
 문서화 완료
 데모 시나리오 준비


13. 참고 명령어 모음
bash# ===== 빌드 =====
colcon build --packages-select dobby_interfaces
colcon build --packages-select dobby_main_controller
colcon build --symlink-install  # 개발 시 편리

# ===== 실행 =====
ros2 launch dobby_main_controller dobby_test.launch.py
ros2 launch dobby_main_controller dobby_main.launch.py robot_id:=dobby2

# ===== 디버깅 =====
ros2 node list
ros2 node info /dobby1/dobby_main_controller
ros2 topic list
ros2 topic echo /dobby1/status/robot_state
ros2 action list
ros2 service list

# ===== 로그 =====
ros2 run dobby_main_controller dobby_main_controller --ros-args --log-level DEBUG

# ===== 정리 =====
rm -rf build/ install/ log/

14. 다음 단계
이 문서로 구현 후, 다음 단계로 진행:

실제 메시지 연동: dobby_interfaces의 Action/Service 실제 연결
DAC/DDC 연동: Mock 대신 실제 컨트롤러와 통신
시나리오 테스트: 전체 플로우 End-to-End 테스트
성능 최적화: 응답 시간, 메모리 사용량 최적화
에러 처리 강화: 재시도 로직, 복구 메커니즘
GUI 개발: Test Control Panel (선택사항)