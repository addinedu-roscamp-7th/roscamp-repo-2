# TEST GUI 리팩토링 설계서 v2.0

**작성일**: 2025-03-15
**목적**: Mock 제어 분리 및 실시간 개별 서비스/액션 제어 UI 구현

---

## 1. 핵심 설계 원칙

### 1.1 Mock 제어 주체 변경

**현재 (문제)**:
```
DMC Node ──초기화시──> Mock Interface 생성
   ↓
재시작 필수 (Mock/Real 전환)
```

**개선 후**:
```
DMC Node ───────────> Real Interface Only
                            ↓
                    (ROS2 Service/Action)
                            ↓
TEST_GUI ──실시간 제어──> Mock Bridge (독립 노드)
                            ↓
                    (응답 가로채기 & Override)
```

### 1.2 설계 목표

1. ✅ **DMC와 Mock 완전 분리**: DMC는 Mock 코드를 import하지 않음
2. ✅ **실시간 제어**: 노드 재시작 없이 개별 서비스/액션의 Mock 응답 변경
3. ✅ **세밀한 제어**: 인터페이스 단위가 아니라 **메서드별** Mock 설정
4. ✅ **작업 전송 UI**: 수동 상태 설정 대신 **Mock RCS**로 실제 작업 Goal 전송

---

## 2. 아키텍처 설계

### 2.1 컴포넌트 구조

```
┌─────────────────┐
│   RCS (실제)    │
│  또는 Mock RCS  │ ← TEST_GUI에서 작업 Goal 전송
└────────┬────────┘
         │ Action Goal
         ↓
┌─────────────────────────────────────────────────────────────┐
│                      DMC Node                               │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐     │
│  │ State Machine│  │  Executors   │  │  Interfaces  │     │
│  └──────────────┘  └──────────────┘  └──────┬───────┘     │
│                                              │             │
│                                    ROS2 Service/Action     │
└──────────────────────────────────────────────┼─────────────┘
                                               │
         ┌─────────────────────────────────────┼─────────────────────┐
         │                                     ↓                     │
         │                          실제 하위 컨트롤러               │
         │                    (DDC, DAC, DVS, GUI, VRC)             │
         │                                     ↑                     │
         │                                     │                     │
         │                        (Mock이 활성화되면 가로채기)        │
         │                                     │                     │
         ↓                                     ↓                     │
┌────────────────────────┐         ┌──────────────────────────────┐│
│     TEST_GUI Node      │◄────────│    Mock Bridge Node          ││
│                        │  제어   │  (개별 서비스/액션별 Mock)    ││
│  ┌──────────────────┐ │         │                              ││
│  │ Mock 제어 패널   │ │         │  ┌─────────────────────────┐ ││
│  │ - Drive (10개)   │ │         │  │ Drive Mock Services     │ ││
│  │ - Arm (6개)      │ │         │  │ - move_to_target        │ ││
│  │ - AI (7개)       │ │         │  │ - stop/resume           │ ││
│  │ - GUI (1개)      │ │         │  │ - start_patrol          │ ││
│  │ - VRC (4개)      │ │         │  │ ...                     │ ││
│  └──────────────────┘ │         │  └─────────────────────────┘ ││
│                        │         │                              ││
│  ┌──────────────────┐ │         │  ┌─────────────────────────┐ ││
│  │ 작업 전송 버튼   │ │         │  │ Arm Mock Actions        │ ││
│  │ - 도서 픽업      │ │         │  │ - pick_book             │ ││
│  │ - 길 안내        │ │         │  │ - place_book            │ ││
│  │ - 반납 정리      │ │         │  │ ...                     │ ││
│  │ - 좌석 청소      │ │         │  └─────────────────────────┘ ││
│  │ - 서가 정리      │ │         │                              ││
│  └──────────────────┘ │         │  ┌─────────────────────────┐ ││
└────────────────────────┘         │  │ AI Mock Services        │ ││
                                   │  │ - detect_book           │ ││
                                   │  │ - change_tracking_mode  │ ││
                                   │  │ ...                     │ ││
                                   │  └─────────────────────────┘ ││
                                   └──────────────────────────────┘│
                                                                   │
                                   ← Mock이 비활성화되면 통과 ──────┘
```

### 2.2 Mock Bridge 동작 원리

**시나리오: DMC가 `move_to_target` 액션 호출**

```python
# DMC Node (변경 없음)
future = self.drive.move_to_target(pose, 'pickup_shelf')

# RosDriveInterface (변경 없음)
goal = MoveToTarget.Goal()
goal.target_pose = pose
goal.location_name = 'pickup_shelf'
future = self.action_client.send_goal_async(goal)
```

**Mock Bridge가 비활성화된 경우**:
```
DMC → /dobby1/drive/move_to_target → [실제 DDC]
```

**Mock Bridge가 활성화된 경우**:
```
DMC → /dobby1/drive/move_to_target
       ↓
Mock Bridge가 가로채기
   ↓
설정된 Mock 응답 반환 (success=True/False, delay=2.0s 등)
   ↓
DMC는 실제 액션 서버로부터 응답받은 것처럼 처리
```

---

## 3. Mock Bridge 노드 구현

### 3.1 패키지 구조

```
javis_dmc_test/
├── javis_dmc_test/
│   ├── __init__.py
│   ├── mock_bridge_node.py        # Mock Bridge 메인 노드
│   ├── mock_rcs_node.py            # Mock RCS (작업 전송용)
│   └── interfaces/
│       ├── mock_drive_bridge.py    # Drive 메서드별 Mock
│       ├── mock_arm_bridge.py      # Arm 메서드별 Mock
│       ├── mock_ai_bridge.py       # AI 메서드별 Mock
│       └── mock_gui_bridge.py      # GUI/VRC Mock
├── launch/
│   └── test_gui_full.launch.py     # TEST_GUI + Mock Bridge + Mock RCS
├── config/
│   └── mock_responses.yaml         # 기본 Mock 응답 설정
├── package.xml
└── setup.py
```

### 3.2 Mock Bridge Node 구현

```python
# javis_dmc_test/mock_bridge_node.py
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from javis_interfaces.action import MoveToTarget, PickBook, PlaceBook
from javis_interfaces.srv import DriveControlCommand, ChangeArmPose, ChangeTrackingMode
from javis_dmc_test_msgs.srv import SetMockMethod

class MockBridgeNode(Node):
    '''개별 서비스/액션별 Mock 응답을 제어하는 브릿지 노드'''

    def __init__(self):
        super().__init__('mock_bridge_node')

        self.declare_parameter('robot_namespace', 'dobby1')
        self.namespace = self.get_parameter('robot_namespace').value

        # Mock 응답 설정 (메서드별)
        self.mock_config = {
            'drive': {
                'move_to_target': {'enabled': False, 'success': True, 'delay': 0.0},
                'stop': {'enabled': False, 'success': True},
                'resume': {'enabled': False, 'success': True},
                'start_patrol': {'enabled': False, 'success': True, 'delay': 1.0},
            },
            'arm': {
                'pick_book': {'enabled': False, 'success': True, 'delay': 3.0},
                'place_book': {'enabled': False, 'success': True, 'delay': 2.0},
                'change_pose': {'enabled': False, 'success': True, 'delay': 0.5},
            },
            'ai': {
                'detect_book': {'enabled': False, 'success': True, 'delay': 0.5},
                'change_tracking_mode': {'enabled': False, 'success': True},
            },
        }

        # 제어 서비스 (TEST_GUI가 호출)
        self.create_service(
            SetMockMethod,
            '/test/mock_bridge/set_method',
            self._on_set_mock_method
        )

        # Mock Action Servers (DMC가 호출하는 액션을 가로챔)
        self._create_mock_actions()

        # Mock Service Servers (DMC가 호출하는 서비스를 가로챔)
        self._create_mock_services()

        self.get_logger().info('Mock Bridge 노드 초기화 완료')

    def _create_mock_actions(self):
        '''DMC가 호출하는 액션을 Mock으로 대체'''

        # drive/move_to_target 액션
        self.move_to_target_server = ActionServer(
            self,
            MoveToTarget,
            f'/{self.namespace}/drive/move_to_target',
            execute_callback=self._execute_move_to_target,
        )

        # arm/pick_book 액션
        self.pick_book_server = ActionServer(
            self,
            PickBook,
            f'/{self.namespace}/arm/pick_book',
            execute_callback=self._execute_pick_book,
        )

        # arm/place_book 액션
        self.place_book_server = ActionServer(
            self,
            PlaceBook,
            f'/{self.namespace}/arm/place_book',
            execute_callback=self._execute_place_book,
        )

        # ... 나머지 액션들

    def _create_mock_services(self):
        '''DMC가 호출하는 서비스를 Mock으로 대체'''

        # drive/control_command 서비스 (stop, resume 등)
        self.drive_control_service = self.create_service(
            DriveControlCommand,
            f'/{self.namespace}/drive/control_command',
            self._handle_drive_control,
        )

        # arm/change_pose 서비스
        self.arm_change_pose_service = self.create_service(
            ChangeArmPose,
            f'/{self.namespace}/arm/change_pose',
            self._handle_arm_change_pose,
        )

        # ai/change_tracking_mode 서비스
        self.ai_tracking_service = self.create_service(
            ChangeTrackingMode,
            f'/{self.namespace}/ai/change_tracking_mode',
            self._handle_ai_tracking,
        )

    def _execute_move_to_target(self, goal_handle):
        '''move_to_target 액션의 Mock 실행'''
        config = self.mock_config['drive']['move_to_target']

        if not config['enabled']:
            # Mock이 비활성화되면 실제 서비스로 전달 (pass-through)
            self.get_logger().warn('move_to_target Mock 비활성화 - 실제 서비스가 없으면 실패')
            result = MoveToTarget.Result()
            result.success = False
            result.message = 'Mock disabled but no real service available'
            goal_handle.abort()
            return result

        # Mock 실행
        goal = goal_handle.request
        self.get_logger().info(f'[Mock] move_to_target: {goal.location_name}')

        # 지연 시간 시뮬레이션
        import time
        delay = config.get('delay', 0.0)
        if delay > 0:
            for i in range(int(delay * 10)):
                time.sleep(0.1)
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result = MoveToTarget.Result()
                    result.success = False
                    result.message = 'Canceled'
                    return result

        # 결과 반환
        result = MoveToTarget.Result()
        result.success = config['success']
        result.message = f'Mock response (success={config["success"]})'

        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return result

    def _execute_pick_book(self, goal_handle):
        '''pick_book 액션의 Mock 실행'''
        config = self.mock_config['arm']['pick_book']

        if not config['enabled']:
            result = PickBook.Result()
            result.success = False
            result.message = 'Mock disabled'
            goal_handle.abort()
            return result

        goal = goal_handle.request
        self.get_logger().info(f'[Mock] pick_book: {goal.book_id}')

        # 지연 시간 시뮬레이션
        import time
        time.sleep(config.get('delay', 0.0))

        # 결과 반환
        result = PickBook.Result()
        result.success = config['success']
        result.book_id = goal.book_id
        result.message = f'Mock response (success={config["success"]})'

        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return result

    def _handle_drive_control(self, request, response):
        '''drive control 서비스의 Mock 처리'''
        command = request.command.lower()  # 'stop', 'resume' 등

        config = self.mock_config['drive'].get(command, {'enabled': False})

        if not config['enabled']:
            response.success = False
            response.message = f'Mock for {command} is disabled'
            return response

        self.get_logger().info(f'[Mock] drive/{command}')

        response.success = config['success']
        response.message = f'Mock response for {command}'
        return response

    def _handle_arm_change_pose(self, request, response):
        '''arm change_pose 서비스의 Mock 처리'''
        config = self.mock_config['arm']['change_pose']

        if not config['enabled']:
            response.success = False
            response.message = 'Mock disabled'
            return response

        self.get_logger().info(f'[Mock] arm/change_pose: {request.pose_type}')

        import time
        time.sleep(config.get('delay', 0.0))

        response.success = config['success']
        response.message = f'Pose changed to {request.pose_type} (Mock)'
        return response

    def _on_set_mock_method(self, request, response):
        '''TEST_GUI로부터 Mock 설정 변경'''
        interface = request.interface.lower()  # 'drive', 'arm', 'ai'
        method = request.method.lower()
        enabled = request.enabled
        success = request.success
        delay = request.delay

        if interface not in self.mock_config:
            response.success = False
            response.message = f'Unknown interface: {interface}'
            return response

        if method not in self.mock_config[interface]:
            response.success = False
            response.message = f'Unknown method: {interface}.{method}'
            return response

        # Mock 설정 업데이트
        self.mock_config[interface][method] = {
            'enabled': enabled,
            'success': success,
            'delay': delay,
        }

        response.success = True
        response.message = f'Mock {interface}.{method} updated: enabled={enabled}, success={success}, delay={delay}s'
        self.get_logger().info(response.message)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = MockBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

### 3.3 Mock RCS Node 구현

```python
# javis_dmc_test/mock_rcs_node.py
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from javis_interfaces.action import PickupBook, GuidePerson, ReshelvingBook
from geometry_msgs.msg import Pose

class MockRCSNode(Node):
    '''TEST_GUI에서 작업 Goal을 DMC에 전송하는 Mock RCS'''

    def __init__(self):
        super().__init__('mock_rcs_node')

        self.declare_parameter('robot_namespace', 'dobby1')
        self.namespace = self.get_parameter('robot_namespace').value

        # Action Clients (DMC의 Action Server로 Goal 전송)
        self.pickup_client = ActionClient(
            self,
            PickupBook,
            f'/{self.namespace}/main/pickup_book'
        )

        self.guiding_client = ActionClient(
            self,
            GuidePerson,
            f'/{self.namespace}/main/guide_person'
        )

        self.reshelving_client = ActionClient(
            self,
            ReshelvingBook,
            f'/{self.namespace}/main/reshelving_book'
        )

        self.get_logger().info('Mock RCS 노드 초기화 완료')

    def send_pickup_task(self, book_id: str = 'TEST_BOOK_001', storage_id: int = 1):
        '''도서 픽업 작업 전송'''
        if not self.pickup_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('PickupBook 액션 서버가 준비되지 않았습니다')
            return None

        goal = PickupBook.Goal()
        goal.book_id = book_id
        goal.storage_id = storage_id

        # 위치 더미 데이터
        goal.shelf_approach_location.x = 1.0
        goal.shelf_approach_location.y = 2.0
        goal.shelf_approach_location.theta = 0.0

        goal.book_pick_pose.position.x = 1.0
        goal.book_pick_pose.position.y = 2.0
        goal.book_pick_pose.position.z = 0.5

        goal.storage_approach_location.x = 0.0
        goal.storage_approach_location.y = 0.0
        goal.storage_approach_location.theta = 0.0

        goal.storage_slot_pose.position.x = 0.0
        goal.storage_slot_pose.position.y = 0.0
        goal.storage_slot_pose.position.z = 0.3

        self.get_logger().info(f'도서 픽업 작업 전송: {book_id}')
        future = self.pickup_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)
        return future

    def send_guiding_task(self, dest_x: float = 5.0, dest_y: float = 3.0):
        '''길 안내 작업 전송'''
        if not self.guiding_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('GuidePerson 액션 서버가 준비되지 않았습니다')
            return None

        goal = GuidePerson.Goal()
        goal.dest_location.x = dest_x
        goal.dest_location.y = dest_y
        goal.dest_location.theta = 0.0
        goal.destination_name = '열람실'

        self.get_logger().info(f'길 안내 작업 전송: ({dest_x}, {dest_y})')
        future = self.guiding_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)
        return future

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('작업 Goal이 거부되었습니다')
            return

        self.get_logger().info('작업 Goal이 수락되었습니다')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'작업 결과: success={result.success}, message={result.message}')
```

---

## 4. TEST_GUI 개선 UI 설계

### 4.1 새로운 레이아웃

```
┌────────────────────────────────────────────────────────────────────┐
│ JAVIS DMC Test GUI v2.0                                            │
├────────────────────────────────────────────────────────────────────┤
│                                                                    │
│  [로봇 상태] ─────────────────────────────────────────────────     │
│   모드: STANDBY    메인: IDLE    서브: NONE    배터리: 85.3%      │
│   LISTENING: 비활성    순찰: 중지    피드백: -                     │
│                                                                    │
├────────────────────────────────────────────────────────────────────┤
│  [작업 전송] ──────────────────────────────────────────────────   │
│   [도서 픽업]  [길 안내]  [반납 정리]  [좌석 청소]  [서가 정리]   │
│                                                                    │
├────────────────────────────────────────────────────────────────────┤
│  [Mock 제어 - 실시간 개별 설정] ───────────────────────────────   │
│                                                                    │
│  ┌── Drive Interface (10개) ──────────────────────────────────┐  │
│  │ Method             | Enabled | Success | Delay | Actions    │  │
│  ├────────────────────┼─────────┼─────────┼───────┼───────────┤  │
│  │ move_to_target     │ [✓]     │ [✓]     │ 0.0s  │ [테스트]  │  │
│  │ stop               │ [ ]     │ [✓]     │ 0.0s  │ [테스트]  │  │
│  │ resume             │ [ ]     │ [✓]     │ 0.0s  │ [테스트]  │  │
│  │ start_patrol       │ [ ]     │ [✓]     │ 1.0s  │ [테스트]  │  │
│  │ enable_follow_mode │ [ ]     │ [✓]     │ 0.0s  │ [테스트]  │  │
│  └─────────────────────────────────────────────────────────────┘  │
│                                                                    │
│  ┌── Arm Interface (6개) ───────────────────────────────────── │  │
│  │ Method             | Enabled | Success | Delay | Actions    │  │
│  ├────────────────────┼─────────┼─────────┼───────┼───────────┤  │
│  │ pick_book          │ [✓]     │ [✓]     │ 3.0s  │ [테스트]  │  │
│  │ place_book         │ [✓]     │ [✓]     │ 2.0s  │ [테스트]  │  │
│  │ change_pose        │ [ ]     │ [✓]     │ 0.5s  │ [테스트]  │  │
│  │ collect_books      │ [ ]     │ [✓]     │ 5.0s  │ [테스트]  │  │
│  └─────────────────────────────────────────────────────────────┘  │
│                                                                    │
│  ┌── AI Interface (7개) ────────────────────────────────────── │  │
│  │ Method             | Enabled | Success | Delay | Actions    │  │
│  ├────────────────────┼─────────┼─────────┼───────┼───────────┤  │
│  │ detect_book        │ [ ]     │ [✓]     │ 0.5s  │ [테스트]  │  │
│  │ change_tracking    │ [✓]     │ [✓]     │ 0.0s  │ [테스트]  │  │
│  │ check_storage_box  │ [ ]     │ [✓]     │ 0.3s  │ [테스트]  │  │
│  └─────────────────────────────────────────────────────────────┘  │
│                                                                    │
│  [일괄 설정]  [모두 활성화]  [모두 비활성화]  [모두 성공]  [모두 실패] │
│                                                                    │
├────────────────────────────────────────────────────────────────────┤
│  [이벤트 로그] ────────────────────────────────────────────────   │
│   [15:30:45] [Mock] drive.move_to_target enabled=True            │
│   [15:30:47] [RCS] 도서 픽업 작업 전송: TEST_BOOK_001            │
│   [15:30:50] [Mock] arm.pick_book 실행 (delay=3.0s)              │
│   [15:30:53] [Result] 작업 완료: success=True                    │
└────────────────────────────────────────────────────────────────────┘
```

### 4.2 UI 구현 코드 (Tkinter)

```python
# test_gui/panels/mock_control_panel.py
import tkinter as tk
from tkinter import ttk

class MockControlPanel(tk.Frame):
    '''Mock 제어 패널 - 개별 메서드별 설정'''

    INTERFACE_METHODS = {
        'Drive': [
            'move_to_target', 'stop', 'resume', 'start_patrol',
            'enable_follow_mode', 'disable_follow_mode', 'rotate_in_place',
            'cancel_patrol'
        ],
        'Arm': [
            'pick_book', 'place_book', 'change_pose',
            'collect_books', 'collect_trash', 'dispose_trash'
        ],
        'AI': [
            'detect_book', 'change_tracking_mode', 'check_storage_box',
            'verify_book_position', 'identify_bookshelf', 'detect_trash'
        ],
    }

    def __init__(self, parent, ros_node):
        super().__init__(parent, bg='#242b36')
        self.node = ros_node

        # 메서드별 위젯 저장
        self.method_widgets = {}  # {('drive', 'move_to_target'): {'enabled': tk.BooleanVar(), ...}}

        self._build_ui()

    def _build_ui(self):
        '''UI 구성'''
        title = tk.Label(
            self,
            text='Mock 제어 - 개별 메서드 설정',
            bg='#242b36',
            fg='#8ab4f8',
            font=('맑은 고딕', 12, 'bold')
        )
        title.pack(pady=10)

        # 각 인터페이스별 패널
        for interface_name, methods in self.INTERFACE_METHODS.items():
            self._build_interface_panel(interface_name, methods)

        # 일괄 제어 버튼
        self._build_bulk_controls()

    def _build_interface_panel(self, interface_name: str, methods: list):
        '''인터페이스별 패널 생성'''
        frame = ttk.LabelFrame(
            self,
            text=f'{interface_name} Interface ({len(methods)}개)',
            style='Dark.TLabelframe'
        )
        frame.pack(fill='x', padx=10, pady=5)

        # 헤더
        headers = ['Method', 'Enabled', 'Success', 'Delay(s)', 'Actions']
        for col, header in enumerate(headers):
            tk.Label(
                frame,
                text=header,
                bg='#2f3644',
                fg='#f5f7fa',
                font=('맑은 고딕', 9, 'bold'),
                width=15 if col == 0 else 10
            ).grid(row=0, column=col, sticky='ew', padx=2, pady=2)

        # 각 메서드별 행
        interface_lower = interface_name.lower()
        for idx, method in enumerate(methods, start=1):
            self._build_method_row(frame, idx, interface_lower, method)

    def _build_method_row(self, parent, row: int, interface: str, method: str):
        '''메서드별 제어 행 생성'''
        # 변수 생성
        enabled_var = tk.BooleanVar(value=False)
        success_var = tk.BooleanVar(value=True)
        delay_var = tk.DoubleVar(value=0.0)

        self.method_widgets[(interface, method)] = {
            'enabled': enabled_var,
            'success': success_var,
            'delay': delay_var,
        }

        # Method 이름
        tk.Label(
            parent,
            text=method,
            bg='#242b36',
            fg='#f5f7fa',
            font=('맑은 고딕', 9),
            anchor='w'
        ).grid(row=row, column=0, sticky='w', padx=5, pady=2)

        # Enabled 체크박스
        ttk.Checkbutton(
            parent,
            variable=enabled_var,
            command=lambda: self._on_method_change(interface, method)
        ).grid(row=row, column=1, padx=5, pady=2)

        # Success 체크박스
        ttk.Checkbutton(
            parent,
            variable=success_var,
            command=lambda: self._on_method_change(interface, method)
        ).grid(row=row, column=2, padx=5, pady=2)

        # Delay 입력
        delay_entry = ttk.Entry(parent, textvariable=delay_var, width=8)
        delay_entry.grid(row=row, column=3, padx=5, pady=2)
        delay_entry.bind('<Return>', lambda e: self._on_method_change(interface, method))

        # 테스트 버튼
        ttk.Button(
            parent,
            text='테스트',
            command=lambda: self._test_method(interface, method),
            style='Dark.TButton'
        ).grid(row=row, column=4, padx=5, pady=2)

    def _on_method_change(self, interface: str, method: str):
        '''메서드 설정 변경 시 Mock Bridge에 전송'''
        widgets = self.method_widgets[(interface, method)]

        enabled = widgets['enabled'].get()
        success = widgets['success'].get()
        delay = widgets['delay'].get()

        # Mock Bridge에 설정 전송
        self.node.set_mock_method(interface, method, enabled, success, delay)

    def _test_method(self, interface: str, method: str):
        '''개별 메서드 테스트 실행'''
        self.node.get_logger().info(f'[테스트] {interface}.{method} 호출')

        # TODO: 실제 메서드 호출 로직 구현
        # 예: move_to_target 테스트 → 더미 Pose로 호출
        if interface == 'drive' and method == 'move_to_target':
            # Mock RCS를 통해 테스트 호출
            pass

    def _build_bulk_controls(self):
        '''일괄 제어 버튼'''
        frame = tk.Frame(self, bg='#242b36')
        frame.pack(fill='x', padx=10, pady=10)

        buttons = [
            ('모두 활성화', lambda: self._bulk_set_enabled(True)),
            ('모두 비활성화', lambda: self._bulk_set_enabled(False)),
            ('모두 성공', lambda: self._bulk_set_success(True)),
            ('모두 실패', lambda: self._bulk_set_success(False)),
        ]

        for text, command in buttons:
            ttk.Button(
                frame,
                text=text,
                command=command,
                style='Dark.TButton'
            ).pack(side='left', padx=5)

    def _bulk_set_enabled(self, enabled: bool):
        '''모든 메서드의 Enabled 일괄 설정'''
        for (interface, method), widgets in self.method_widgets.items():
            widgets['enabled'].set(enabled)
            self._on_method_change(interface, method)

    def _bulk_set_success(self, success: bool):
        '''모든 메서드의 Success 일괄 설정'''
        for (interface, method), widgets in self.method_widgets.items():
            widgets['success'].set(success)
            self._on_method_change(interface, method)
```

---

## 5. 작업 전송 UI (Mock RCS)

### 5.1 작업 전송 패널

```python
# test_gui/panels/task_sender_panel.py
class TaskSenderPanel(tk.Frame):
    '''작업 전송 패널 - Mock RCS 역할'''

    def __init__(self, parent, mock_rcs_node):
        super().__init__(parent, bg='#242b36')
        self.rcs = mock_rcs_node

        self._build_ui()

    def _build_ui(self):
        title = tk.Label(
            self,
            text='작업 전송 (Mock RCS)',
            bg='#242b36',
            fg='#8ab4f8',
            font=('맑은 고딕', 11, 'bold')
        )
        title.pack(pady=5)

        # 작업 버튼들
        tasks = [
            ('도서 픽업', self._send_pickup),
            ('길 안내', self._send_guiding),
            ('반납 정리', self._send_reshelving),
            ('좌석 청소', self._send_cleaning),
            ('서가 정리', self._send_sorting),
        ]

        for text, command in tasks:
            ttk.Button(
                self,
                text=text,
                command=command,
                style='Dark.TButton',
                width=15
            ).pack(side='left', padx=5, pady=5)

    def _send_pickup(self):
        '''도서 픽업 작업 전송'''
        self.rcs.send_pickup_task('TEST_BOOK_001', storage_id=1)

    def _send_guiding(self):
        '''길 안내 작업 전송'''
        self.rcs.send_guiding_task(dest_x=5.0, dest_y=3.0)

    # ... 나머지 작업들
```

---

## 6. Launch 파일

```python
# launch/test_gui_full.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    robot_namespace = LaunchConfiguration('robot_namespace')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_namespace',
            default_value='dobby1',
            description='로봇 네임스페이스'
        ),

        # DMC 노드 (Mock 코드 제거됨)
        Node(
            package='javis_dmc',
            executable='dmc_node',
            namespace=robot_namespace,
            parameters=[
                {'robot_namespace': robot_namespace},
                {'use_mock_interfaces': False},  # ← 항상 False
            ],
        ),

        # Mock Bridge 노드
        Node(
            package='javis_dmc_test',
            executable='mock_bridge_node',
            parameters=[
                {'robot_namespace': robot_namespace},
            ],
        ),

        # Mock RCS 노드
        Node(
            package='javis_dmc_test',
            executable='mock_rcs_node',
            parameters=[
                {'robot_namespace': robot_namespace},
            ],
        ),

        # TEST GUI 노드
        Node(
            package='javis_dmc',
            executable='test_gui_node',
            parameters=[
                {'robot_namespace': robot_namespace},
            ],
        ),
    ])
```

---

## 7. 마이그레이션 계획

### 7.1 Phase 1: Mock Bridge 구현 (3일)
- [ ] `javis_dmc_test` 패키지 생성
- [ ] `MockBridgeNode` 기본 구조 구현
- [ ] Drive 인터페이스 Mock 구현 (3개: move_to_target, stop, resume)
- [ ] 제어 서비스 (`SetMockMethod`) 구현
- [ ] 기본 테스트

### 7.2 Phase 2: TEST_GUI 개선 (2일)
- [ ] Mock 제어 패널 UI 구현
- [ ] Mock RCS 노드 구현
- [ ] 작업 전송 패널 구현
- [ ] 기존 "수동 상태 설정" 제거

### 7.3 Phase 3: 전체 인터페이스 Mock 구현 (3일)
- [ ] Arm 인터페이스 Mock 6개
- [ ] AI 인터페이스 Mock 7개
- [ ] GUI/VRC Mock
- [ ] 통합 테스트

### 7.4 Phase 4: DMC 노드 Mock 코드 제거 (1일)
- [ ] `dmc_node.py`에서 Mock import 제거
- [ ] `use_mock_interfaces` 파라미터 제거
- [ ] 테스트 및 검증

---

## 8. 핵심 이점

### 8.1 이전 vs 개선 후

| 항목 | 이전 (DMC 내장 Mock) | 개선 후 (Mock Bridge) |
|------|---------------------|----------------------|
| **DMC-Mock 결합도** | 강함 (import 필요) | 없음 (완전 분리) |
| **Mock 전환** | 노드 재시작 필수 | 실시간 전환 |
| **제어 단위** | 인터페이스 (4개) | 메서드 (28개) |
| **디버깅** | 어려움 (코드 수정) | 쉬움 (GUI 클릭) |
| **테스트 시나리오** | 불가능 | 가능 (메서드별 설정) |

### 8.2 사용 시나리오 예시

**시나리오: 도서 픽업 작업 디버깅**

1. TEST_GUI에서 Mock 설정:
   - `drive.move_to_target`: Enabled=✓, Success=✓, Delay=0.5s
   - `arm.pick_book`: Enabled=✓, Success=❌, Delay=2.0s (← 실패 시뮬레이션)
   - `arm.change_pose`: Enabled=✓, Success=✓, Delay=0.3s

2. "도서 픽업" 버튼 클릭 → Mock RCS가 Goal 전송

3. DMC 실행:
   - `move_to_target` → Mock Bridge가 0.5초 후 성공 반환
   - `pick_book` → Mock Bridge가 2초 후 **실패** 반환
   - DMC Executor가 실패 처리 로직 실행

4. 로그 확인:
   ```
   [15:45:12] [DMC] 도서 픽업 작업 시작
   [15:45:12] [Mock] drive.move_to_target 실행 (delay=0.5s)
   [15:45:13] [DMC] 서브 상태: PICKUP_BOOK
   [15:45:13] [Mock] arm.pick_book 실행 (delay=2.0s)
   [15:45:15] [Mock] arm.pick_book 실패 반환
   [15:45:15] [DMC] 도서 집기 실패 - 작업 중단
   ```

5. `arm.pick_book`만 Success=✓로 변경 후 재테스트 (재시작 불필요!)

---

## 9. 다음 단계

1. **`javis_dmc_test` 패키지 생성** 및 Mock Bridge 프로토타입
2. **TEST_GUI에 Mock 제어 패널 추가**
3. **Mock RCS로 작업 전송 테스트**
4. **전체 통합 테스트 후 DMC Mock 코드 제거**

**작성자**: Claude Code
**검토 필요**: 팀 리뷰 및 기술 스택 최종 확정
