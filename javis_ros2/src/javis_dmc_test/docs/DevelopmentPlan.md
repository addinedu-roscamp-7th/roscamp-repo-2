# Dobby '브레인(DMC)' 통합 테스트용 Mock 노드 설계 문서

> **Version:** 7.0 (Implementation Details Added)
> **Status:** Final Design
> **Date:** 2025-10-26

---

## 📋 목차

1.  [목적](#1-목적)
2.  [핵심 아키텍처](#2-핵심-아키텍처)
    -   [2.1 mode 파라미터 값 정의](#21-mode-파라미터-값-정의)
3.  [패키지 구조 및 메시지 정의](#3-패키지-구조-및-메시지-정의)
    -   [3.1 패키지 폴더 구조](#31-패키지-폴더-구조-2개-패키지)
    -   [3.2 메시지 정의 (MockStatus.msg)](#32-메시지-정의-mockstatusmsg)
4.  [Mock 노드 명세](#4-mock-노드-명세-확장)
    -   [4.1 Mock Servers](#41-mock-servers-dmc의-의존성-노드)
    -   [4.2 Mock Clients](#42-mock-clients-dmc를-호출하는-노드)
5.  [Test GUI (Control Panel) 명세](#5-test-gui-control-panel-명세-v6)
6.  [핵심 로직 구현](#6-핵심-로직-구현)
    -   [6.1 `gui_node.py`](#61-guinodepy-gui의-ros-2-노드)
    -   [6.2 `mock_*.py`](#62-mock_py-mock-서버-노드)
7.  [현재 구현 상태](#7-현재-구현-상태)
    -   [7.1 Status GUI 구현](#71-status-gui-구현)
    -   [7.2 구현 예정 항목](#72-구현-예정-항목)

---

## 1. 목적

본 문서는 `dobby_main_controller(DMC)` 노드의 독립 기능 및 로직을 검증하기 위한 **Mock 노드 시스템 설계**를 정의합니다.
복잡한 외부 의존 노드(DDC, DAC, DVS 등)가 미구현 또는 불안정한 상황에서도:

-   상태 머신
-   작업 실행 로직
-   엣지 케이스 (실패 처리 등)

을 안정적으로 테스트할 수 있도록 합니다.

---

## 2. 핵심 아키텍처

-   **1 Mock = 1 Node**: 테스트 대상이 되는 각 인터페이스는 개별 Python 노드(`rclpy.node.Node`)로 구현합니다.
-   **mode 파라미터 제어**: 'Mock 서버' 노드들은 `mode` (`active`/`error`/`on`/`off`) 파라미터를 선언합니다.
-   **GUI (Push & Trigger)**: GUI 노드는 'Mock 서버'의 `mode`를 파라미터로 제어(Push)하고, 'Mock 클라이언트'의 동작을 실행(Trigger)합니다.
-   **Mock 상태 발행 (Publish)**: 'Mock 서버' 노드는 `mode` 변경 시 `dobby1/mock_system/status` 토픽으로 자신의 상태를 발행합니다.
-   **GUI 상태 수신 (Subscribe)**: GUI 노드는 `dobby1/mock_system/status` 토픽을 구독하여 Mock 서버들의 상태를 UI에 실시간으로 반영합니다.

### 2.1 mode 파라미터 값 정의

| mode 값 | 설명 |
| :--- | :--- |
| `active` | 정상적인 응답/데이터 반환 (기본값) |
| `error` | 실패 응답 반환 (`result.success=False`) |
| `off` | 토픽 발행 중지 (토픽 Publisher 전용) |
| `on` | 토픽 발행 시작 (토픽 Publisher 전용) |

---

## 3. 패키지 구조 및 메시지 정의

### 3.1 패키지 폴더 구조 (2개 패키지)

#### 패키지 1: `javis_dmc_test_msgs`

```ascii
javis_dmc_test_msgs/        # 📦 메시지 전용
├── package.xml
├── CMakeLists.txt
└── msg/
    └── MockStatus.msg      # Mock 서버 상태 발행 메시지
```

#### 패키지 2: `javis_dmc_test`

```ascii
javis_dmc_test/             # 🧠 전체 테스트 시스템 로직
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   ├── javis_dmc_test
│   ├── main_window.ui
│   ├── dmc_test_goals.yaml       # 👈 [3.1.3] Action Goal 정의
│   └── test_locations.yaml       # 👈 길안내 테스트용 위치 정보
├── launch/
│   └── dmc_mock.launch.py        # 👈 [3.1.2] 실행 런치 파일
└── javis_dmc_test/
    ├── nodes/                    # 👈 [3.1.1] Mock 노드 직접 구현 (1 Mock = 1 File)
    │   ├── __init__.py
    │   ├── mock_server_base.py   # 👈 MockServerBase (Node 상속)
    │   │
    │   ├── # RCS Mock (1개)
    │   ├── mock_rcs_create_user_guide.py
    │   │
    │   ├── # DDC Mock (3개)
    │   ├── mock_ddc_navigate_to_pose.py
    │   ├── mock_ddc_guide_navigation.py
    │   ├── mock_ddc_control_command.py
    │   │
    │   ├── # DVS Mock (3개)
    │   ├── mock_dvs_change_tracking_mode.py
    │   ├── mock_dvs_detect_trash.py
    │   ├── mock_dvs_tracking_status.py
    │   │
    │   ├── # DAC Mock (8개)
    │   ├── mock_dac_pick_book.py
    │   ├── mock_dac_place_book.py
    │   ├── mock_dac_collect_returned_books.py
    │   ├── mock_dac_sort_book.py
    │   ├── mock_dac_clean_desk.py
    │   ├── mock_dac_collect_trash.py
    │   ├── mock_dac_dispose_trash.py
    │   ├── mock_dac_change_pose.py
    │   │
    │   ├── # GUI Mock (2개) - DMC의 GUI 인터페이스 Mock
    │   ├── mock_gui_query_location_info.py
    │   ├── mock_gui_request_guidance.py
    │   │
    │   └── # VRC Mock (2개) - DMC의 VRC 인터페이스 Mock
    │       ├── mock_vrc_set_listening_mode.py
    │       └── mock_vrc_stt_result.py
    │
    ├── gui/                      # 👈 [설계] PyQt6 기반 Mock 제어 GUI (구현 예정)
    │   ├── gui_node.py
    │   ├── main_window.py
    │   └── widgets/
    │       └── widget_guidance.py
    │
    └── status_gui/               # 👈 [7.1] Tkinter 기반 DMC 상태 모니터링 GUI (구현 완료)
        ├── __init__.py
        ├── status_gui_node.py    # ROS 2 노드 (백그라운드 스레드)
        └── status_gui_widget.py  # Tkinter UI (메인 스레드)
```

**설계 원칙: 1 Mock = 1 Node = 1 File**
- 각 Mock 노드는 `MockServerBase(Node)`를 상속하여 독립 ROS2 노드로 동작
- 각 파일에 `main()` 함수 포함하여 `setup.py`에 executable로 등록
- 별도의 `mocks/` 디렉토리 불필요 (nodes/에 직접 구현)

**총 Mock 노드 개수: 19개**
- Phase 1 (필수): 11개 (RCS 1 + DDC 3 + DVS 2 + GUI 2 + VRC 2)
- Phase 2 (선택): 8개 (DAC 8 + DVS 1)

```

#### 3.1.1 setup.py (Entry Points)
각 Mock 노드를 개별 executable로 등록합니다.

```Python
# setup.py
from setuptools import setup
import os
from glob import glob

package_name = 'javis_dmc_test'

setup(
    # ... (other setup args) ...
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name, 'resource'), glob('resource/*.*')), # .ui, .yaml 설치
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')), # 런치파일 설치
    ],
    entry_points={
        'console_scripts': [
            # Test GUI
            'start_gui = javis_dmc_test.gui.gui_node:main',
            
            # Phase 1: 필수 Mock 노드 (11개)
            'mock_rcs_create_user_guide = javis_dmc_test.nodes.mock_rcs_create_user_guide:main',
            'mock_ddc_navigate_to_pose = javis_dmc_test.nodes.mock_ddc_navigate_to_pose:main',
            'mock_ddc_guide_navigation = javis_dmc_test.nodes.mock_ddc_guide_navigation:main',
            'mock_ddc_control_command = javis_dmc_test.nodes.mock_ddc_control_command:main',
            'mock_dvs_change_tracking_mode = javis_dmc_test.nodes.mock_dvs_change_tracking_mode:main',
            'mock_dvs_tracking_status = javis_dmc_test.nodes.mock_dvs_tracking_status:main',
            'mock_gui_query_location_info = javis_dmc_test.nodes.mock_gui_query_location_info:main',
            'mock_gui_request_guidance = javis_dmc_test.nodes.mock_gui_request_guidance:main',
            'mock_vrc_set_listening_mode = javis_dmc_test.nodes.mock_vrc_set_listening_mode:main',
            'mock_vrc_stt_result = javis_dmc_test.nodes.mock_vrc_stt_result:main',
            
            # Phase 2: 추가 Mock 노드 (9개)
            'mock_dvs_detect_trash = javis_dmc_test.nodes.mock_dvs_detect_trash:main',
            'mock_dac_pick_book = javis_dmc_test.nodes.mock_dac_pick_book:main',
            'mock_dac_place_book = javis_dmc_test.nodes.mock_dac_place_book:main',
            'mock_dac_collect_returned_books = javis_dmc_test.nodes.mock_dac_collect_returned_books:main',
            'mock_dac_sort_book = javis_dmc_test.nodes.mock_dac_sort_book:main',
            'mock_dac_clean_desk = javis_dmc_test.nodes.mock_dac_clean_desk:main',
            'mock_dac_collect_trash = javis_dmc_test.nodes.mock_dac_collect_trash:main',
            'mock_dac_dispose_trash = javis_dmc_test.nodes.mock_dac_dispose_trash:main',
            'mock_dac_change_pose = javis_dmc_test.nodes.mock_dac_change_pose:main',
            
            # Status GUI (기존)
            'status_gui_node = javis_dmc_test.status_gui.status_gui_node:main',
        ],
    },
)
```

**원칙**: 각 Mock 노드는 독립적인 ROS2 노드로 실행되며, 개별적으로 파라미터 제어가 가능합니다.

#### 3.1.2 dmc_mock.launch.py (Example)
Phase 1 (필수 11개 Mock 노드)만 실행하는 런치 파일입니다.

```Python
# launch/dmc_mock.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. 테스트 대상인 'DMC' 노드 (실제 DMC 패키지에서 가져옴)
        Node(
            package='javis_dmc',  # 실제 DMC 패키지
            executable='dmc_node',
            name='dobby_main_controller',
            namespace='dobby1',
            # (필요시) remappings=[...],
        ),

        # 2. Test GUI 노드 실행
        Node(
            package='javis_dmc_test',
            executable='start_gui',
            name='dmc_mock_gui_node',
            output='screen'
        ),

        # 3. Phase 1: 필수 Mock 노드들 (11개) - 각각 독립 실행
        
        # RCS Mock (1개)
        Node(
            package='javis_dmc_test',
            executable='mock_rcs_create_user_guide',
            name='mock_rcs_create_user_guide',
            output='screen',
        ),
        
        # DDC Mock (3개)
        Node(
            package='javis_dmc_test',
            executable='mock_ddc_navigate_to_pose',
            name='mock_ddc_navigate_to_pose',
            output='screen',
        ),
        Node(
            package='javis_dmc_test',
            executable='mock_ddc_guide_navigation',
            name='mock_ddc_guide_navigation',
            output='screen',
        ),
        Node(
            package='javis_dmc_test',
            executable='mock_ddc_control_command',
            name='mock_ddc_control_command',
            output='screen',
        ),
        
        # DVS Mock (2개)
        Node(
            package='javis_dmc_test',
            executable='mock_dvs_change_tracking_mode',
            name='mock_dvs_change_tracking_mode',
            output='screen',
        ),
        Node(
            package='javis_dmc_test',
            executable='mock_dvs_tracking_status',
            name='mock_dvs_tracking_status',
            output='screen',
        ),
        
        # GUI Mock (2개)
        Node(
            package='javis_dmc_test',
            executable='mock_gui_query_location_info',
            name='mock_gui_query_location_info',
            output='screen',
        ),
        Node(
            package='javis_dmc_test',
            executable='mock_gui_request_guidance',
            name='mock_gui_request_guidance',
            output='screen',
        ),
        
        # VRC Mock (2개)
        Node(
            package='javis_dmc_test',
            executable='mock_vrc_set_listening_mode',
            name='mock_vrc_set_listening_mode',
            output='screen',
        ),
        Node(
            package='javis_dmc_test',
            executable='mock_vrc_stt_result',
            name='mock_vrc_stt_result',
            output='screen',
        ),
    ])
```

**중요**: 각 Mock 노드가 독립적으로 실행되어 개별 제어 및 모니터링이 가능합니다.
- GUI에서 `ros2 param set /mock_ddc_navigate_to_pose mode error` 명령으로 특정 Mock만 실패 모드로 전환 가능
- Phase 2 노드들은 필요 시 추가

#### 3.1.3 dmc_test_goals.yaml (Example Format)
📌 파일 경로: resource/dmc_test_goals.yaml

```YAML

# RCS Mock 클라이언트가 DMC에 전송할 Action Goal 정의
# 키 이름은 GUI 버튼의 ID 또는 Action 이름과 일치시키는 것이 좋습니다.

pickup_book:
  goal:
    book_id: "BK-12345"
    storage_id: 1
    # ... (dobby1/main/pickup_book Action의 Goal 필드들) ...
    # shelf_approach_location: { x: 1.0, y: 2.0, theta: 0.0 }
    # book_pick_pose: { ... }

guide_person:
  goal:
    dest_location: 
      x: 10.5
      y: -5.0
      theta: 1.57  # '화장실' 좌표 (library_locations.yaml과 동일)

guide_person_cafe:
  goal:
    dest_location:
      x: 15.0
      y: 8.0
      theta: 3.14  # '카페' 좌표

guide_person_entrance:
  goal:
    dest_location:
      x: 0.0
      y: 0.0
      theta: 0.0  # '출입구' 좌표

clean_seat:
  goal:
    seat_id: 32
    # ... (dobby1/main/clean_seat Action의 Goal 필드들) ...

reshelving_book:
  goal:
    return_desk_id: 1
    # ... (dobby1/main/reshelving_book Action의 Goal 필드들) ...
```

> **참고:** `guide_person_*` 항목들은 `library_locations.yaml`의 좌표와 일치시켜 일관성을 유지합니다.

### 3.2 메시지 정의 (MockStatus.msg)
📌 파일 경로: javis_dmc_test_msgs/msg/MockStatus.msg

```코드 스니펫

# Mock 노드의 현재 동작 모드 발행 메시지
std_msgs/Header header
string node_name   # 예: "mock_dvs_detect_trash_service"
string mode        # 예: "active", "error", "on", "off"
```

### 4. Mock 노드 명세 (확장)
Mock 노드는 두 종류로 나뉩니다:

#### 4.1 Mock Servers (DMC의 의존성 노드)
각 Mock Server는 mode 파라미터를 가지며, 상태를 dobby1/mock_system/status로 발행합니다.

**설계 원칙: 1 Mock = 1 Node**
- 각 Mock은 독립 ROS2 노드로 실행
- 개별 파라미터 제어: `ros2 param set /mock_node_name mode [active|error|on|off]`

| 카테고리 | 노드 이름 | 타입 | 인터페이스 | 역할 | Phase |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **RCS** | `mock_rcs_create_user_guide` | Service | `/rcs/create_user_task` | Service Server (v4.0) | 1 |
| **Drive** | `mock_ddc_navigate_to_pose` | Action | `dobby1/drive/navigate_to_pose` | Action Server | 1 |
| **Drive** | `mock_ddc_guide_navigation` | Action | `dobby1/drive/guide_navigation` | Action Server | 1 |
| **Drive** | `mock_ddc_control_command` | Service | `dobby1/drive/control_command` | Service Server | 1 |
| **AI** | `mock_dvs_change_tracking_mode` | Service | `dobby1/ai/change_tracking_mode` | Service Server | 1 |
| **AI** | `mock_dvs_tracking_status` | Topic | `dobby1/ai/tracking/status` | Topic Publisher | 1 |
| **AI** | `mock_dvs_detect_trash` | Service | `dobby1/ai/detect_trash` | Service Server | 2 |
| **Arm** | `mock_dac_pick_book` | Action | `dobby1/arm/pick_book` | Action Server | 2 |
| **Arm** | `mock_dac_place_book` | Action | `dobby1/arm/place_book` | Action Server | 2 |
| **Arm** | `mock_dac_collect_returned_books` | Action | `dobby1/arm/collect_returned_books` | Action Server | 2 |
| **Arm** | `mock_dac_sort_book` | Action | `dobby1/arm/sort_book` | Action Server | 2 |
| **Arm** | `mock_dac_clean_desk` | Action | `dobby1/arm/clean_desk` | Action Server | 2 |
| **Arm** | `mock_dac_collect_trash` | Action | `dobby1/arm/collect_trash` | Action Server | 2 |
| **Arm** | `mock_dac_dispose_trash` | Action | `dobby1/arm/dispose_trash` | Action Server | 2 |
| **Arm** | `mock_dac_change_pose` | Service | `dobby1/arm/change_pose` | Service Server | 2 |
| **GUI** | `mock_gui_query_location_info` | Service | `dobby1/admin/query_location_info` | Service Server (v4.0) | 1 |
| **GUI** | `mock_gui_request_guidance` | Service | `dobby1/admin/request_guidance` | Service Server (v4.0) | 1 |
| **VRC** | `mock_vrc_set_listening_mode` | Service | `voice_recognition_controller/set_listening_mode` | Service Server | 1 |
| **VRC** | `mock_vrc_stt_result` | Topic | `dobby1/voice_recognition_controller/stt_result` | Topic Publisher | 1 |

**총 19개 Mock 노드**
- **Phase 1 (필수)**: 11개 - v4.0 길안내 플로우 검증에 필수
- **Phase 2 (선택)**: 8개 - 추가 작업 시나리오 테스트용


#### 4.2 Mock Clients (DMC를 호출하는 노드)
모든 Mock Clients는 GUI 내에서 제어되며 별도 파라미터는 없습니다.


| 담당 | Mock 노드 (GUI 내 기능) | 프로토콜 | 인터페이스 (DMC의) | 역할 |
| :--- | :--- | :--- | :--- | :--- |
| **RCS** | `mock_rcs_task_client` | Action | `dobby1/main/pickup_book` | Action Client |
| **RCS** | `mock_rcs_task_client` | Action | `dobby1/main/guide_person` | Action Client |
| **RCS** | `mock_rcs_task_client` | Action | `dobby1/main/clean_seat` | Action Client |
| **RCS** | `mock_rcs_task_client` | Action | `...` (기타 모든 RCS->DMC 작업) | Action Client |
| **RCS** | `mock_rcs_create_user_task` | Service | `/rcs/create_user_task` | Service Server (v4.0 추가) |
| **VRC** | `mock_vrc_guidance_client` | Service | `dobby1/admin/request_guidance` | Service Client (v4.0 업데이트) |
| **GUI** | `mock_gui_guidance_client` | Service | `dobby1/admin/query_location_info` | Service Client |
| **GUI** | `mock_gui_guidance_client` | Service | `dobby1/admin/request_guidance` | Service Client |

> **v4.0 변경사항:**
> - **RCS Mock 추가**: `CreateUserTask` 서비스 서버 구현 (DMC → RCS 호출을 받아서 GuidePerson Action 전송)
> - **VRC Mock 업데이트**: `submit_voice_task` 제거, `request_guidance` 직접 호출로 변경
> - VRC는 LLM Service에서 좌표 획득 후 RequestGuidance 호출 (QueryLocationInfo 사용 안 함)

---

## 4.3 길안내 관련 인터페이스 상세 (신규 추가)

### 4.3.1 위치 정보 조회 서비스 (`QueryLocationInfo.srv`)

**목적:** GUI에서 지도 터치 시 또는 음성으로 요청된 목적지의 좌표 및 메타정보를 조회합니다.

**인터페이스:**
- **타입:** Service
- **이름:** `dobby1/admin/query_location_info`
- **메시지:** `javis_interfaces/srv/admin/QueryLocationInfo`

**요청 (Request):**
```
string location_name  # 조회할 위치 이름 (예: "화장실", "안내데스크")
```

**응답 (Response):**
```
bool found                    # 위치 정보 존재 여부
string location_name          # 확인된 위치 이름
string location_id            # 고유 식별자 (예: "restroom_1f")
geometry_msgs/Pose2D pose     # 2D 위치 및 방향
string description            # 위치 설명
string[] aliases              # 별칭 목록
string message                # 응답 메시지
```

**사용 시나리오:**
1. GUI에서 사용자가 지도의 "화장실" 버튼 터치
2. GUI → DMC: `query_location_info(location_name="화장실")`
3. DMC는 `library_locations.yaml`에서 위치 조회
4. DMC → GUI: `{found: true, pose: {x: 10.5, y: -5.0, theta: 1.57}, ...}`
5. GUI는 지도에 목적지 마커 표시

### 4.3.2 길안내 요청 서비스 (`RequestGuidance.srv`)

**목적:** GUI 터치와 VRC 음성 요청 모두 동일한 서비스를 사용하여 길안내 작업을 RCS에 요청합니다.

**인터페이스:**
- **타입:** Service
- **이름:** `dobby1/admin/request_guidance`
- **메시지:** `javis_interfaces/srv/admin/RequestGuidance`

**요청 (Request):**
```
string destination_name       # 목적지 이름 (예: "화장실")
geometry_msgs/Pose2D dest_pose  # 목적지 2D 좌표
string request_source         # 요청 출처 ("gui" 또는 "voice")
string user_context           # 사용자 컨텍스트 (선택사항)
```

**응답 (Response):**
```
bool success                  # 요청 성공 여부
string message                # 응답 메시지
string task_id                # 생성된 작업 ID (RCS에서 할당)
```

**통합 플로우 (v4.0):**

**[GUI 터치 시나리오]**
```
1. GUI: 초기 화면 표시 ("길안내 시 터치해주세요" 문구)
2. 사용자가 화면 터치
3. GUI → DMC: QueryLocationInfo("") - 목적지 입력 의사 표현 + 목록 요청
4. DMC: State IDLE/ROAMING → WAITING_DEST_INPUT (60초 타이머 시작)
5. DMC → GUI: 목적지 목록 반환 (화장실, 카페, 출입구 등)
6. GUI: 지도 화면 표시 + 목적지 버튼들 업데이트
7. 사용자가 "화장실" 버튼 터치
8. GUI → DMC: RequestGuidance("화장실", pose, "gui")
9. DMC: 배터리 체크 (≥40%)
10. DMC: 60초 타이머 취소
11. DMC → RCS: CreateUserTask(user_initiated=True)
12. RCS: user_initiated=True 확인 → 상태 체크 무시
13. RCS → DMC: GuidePerson Action Goal (user_initiated=True)
14. DMC: State WAITING_DEST_INPUT → GUIDING
15. DMC → GUI: {success: true, task_id="task_123"}
```

**[VRC 음성 시나리오]**
```
1. VRC: "도비야, 화장실 가고 싶어" (사용자 음성)
2. DMC: State IDLE/ROAMING → LISTENING (20초 타이머)
3. VRC → LLM Service: "화장실 가고 싶어" (STT + Intent 분석)
4. LLM Service → VRC: {intent: "navigation", target: "화장실", pose: {x: 10.5, y: -5.0}}
5. VRC → DMC: RequestGuidance("화장실", pose, "voice")
6. DMC: 배터리 체크 (≥40%)
7. DMC: 20초 타이머 취소
8. DMC → RCS: CreateUserTask(user_initiated=True)
9. RCS: user_initiated=True 확인 → 상태 체크 무시
10. RCS → DMC: GuidePerson Action Goal (user_initiated=True)
11. DMC: State LISTENING → GUIDING
12. DMC → VRC: {success: true, task_id="task_123"}
```

> **v4.0 핵심 변경사항**:
> - **QueryLocationInfo**: GUI만 사용 (목적지 입력 의사 표현 = WAITING_DEST_INPUT 진입)
> - **VRC**: LLM Service에서 좌표 획득 (QueryLocationInfo 사용 안 함)
> - **DMC → RCS**: CreateUserTask Service 호출 (사용자 주도 작업)
> - **user_initiated 플래그**: RCS가 상태 체크 무시하고 즉시 Action 호출

### 4.3.3 도서관 위치 정보 파일 (`library_locations.yaml`)

**파일 위치:** `javis_dmc/config/library_locations.yaml`

**목적:** 도서관 내 주요 시설(화장실, 카페, 출입구 등)의 좌표와 메타정보를 저장합니다.

**구조:**
```yaml
locations:
  - name: "화장실"
    id: "restroom_1f"
    aliases: ["화장실", "남자 화장실", "여자 화장실", "restroom"]
    pose:
      x: 10.5
      y: -5.0
      theta: 1.57
    description: "1층 화장실"
  
  - name: "안내데스크"
    id: "info_desk"
    aliases: ["안내데스크", "안내", "데스크", "information"]
    pose:
      x: 2.0
      y: 1.0
      theta: 0.0
    description: "1층 안내 데스크"

settings:
  default_approach_distance: 0.5  # 도착 판정 거리 (미터)
  max_search_distance: 50.0       # 최대 검색 반경 (미터)
```

**DMC 동작:**
- DMC는 시작 시 `library_locations.yaml` 로드
- `query_location_info` 서비스 요청 시 location_name 또는 aliases로 검색
- 대소문자 무시, 부분 매칭 지원 (예: "화장실" == "남자 화장실")

### 4.3.4 RCS 길안내 작업 할당 플로우

**RCS의 역할:**
1. DMC로부터 `request_guidance` 서비스 호출 수신
2. 작업 검증 (목적지 유효성, 로봇 상태 확인)
3. 작업 큐에 `guide_person` 작업 추가
4. `dobby1/main/guide_person` Action Goal을 DMC에 전송
5. DMC는 Action 수락 후 GUIDING 상태로 전환

**작업 검증 조건:**
- 로봇이 IDLE 또는 ROAMING 상태
- 배터리 레벨 >= 40%
- 목적지 좌표가 유효 범위 내
- 진행 중인 다른 작업 없음

**실패 처리:**
- 검증 실패 시 RCS는 `request_guidance` 응답에서 `success: false` 반환
- DMC는 사용자에게 실패 메시지 전달 (GUI 또는 TTS)

---

### 5. Test GUI (Control Panel) 명세 (v6)
노드명: dmc_mock_gui_node 구현: PyQt6 + rclpy.node.Node

#### 5.1 전체 레이아웃
좌측: 제어 패널 (QTabWidget)

우측: 로그 뷰 (QTextEdit, ReadOnly)

(옵션) /rosout 토픽 구독 가능

#### 5.2 탭 구성

[탭 1: RCS (Task Triggers)]
버튼 클릭 시 dmc_test_goals.yaml에서 Goal을 불러와 Action 호출


```
[ Button: Send 'pickup_book' Task ]
[ Button: Send 'guide_person' Task ]
[ Button: Send 'clean_seat' Task ]
[ Button: Send 'reshelving_book' Task ]
```

**길안내 작업 테스트:**
- `guide_person` 버튼 클릭 시 `dmc_test_goals.yaml`에서 목적지 좌표 로드
- 예시 Goal: `dest_location: {x: 10.5, y: -5.0, theta: 1.57}` (화장실)

[탭 2: VRC (Voice Triggers)]
```
[ Section: Wake Word 및 LISTENING 모드 ]
  [ Button: Trigger Wake Word ] → DMC 상태를 LISTENING으로 전환
  [ Label: LISTENING Status ] (자동 업데이트)

[ Section: 음성 길안내 요청 (v4.0) ]
  [ Input: 목적지 이름 ] "화장실" / "카페" / "출입구"
  [ Input: 좌표 (LLM 제공) ] x=10.5, y=-5.0, theta=1.57
  [ Button: Request Guidance (Voice) ] → request_guidance 서비스 호출
  [ Output: 요청 결과 표시 ]
    - Success: true
    - Task ID: task_123
    - Message: "화장실로 안내를 시작합니다"

[ Section: 빠른 음성 테스트 ]
  [ Button: "화장실 가고 싶어" ] → LLM 좌표 + RequestGuidance
  [ Button: "카페로 안내해줘" ] → LLM 좌표 + RequestGuidance
  [ Button: "출입구 어디야?" ] → LLM 좌표 + RequestGuidance
```

> **v4.0 변경사항**:
> - `submit_voice_task` 제거
> - VRC는 LLM Service에서 좌표를 직접 받아서 `request_guidance` 호출
> - GUI와 동일한 서비스 사용, `request_source="voice"`로 구분

**음성 길안내 테스트:**
- VRC Mock이 LLM 역할 시뮬레이션 (목적지 → 좌표 매핑)
- "화장실" 입력 → `library_locations.yaml`에서 좌표 로드
- `request_guidance(dest_name="화장실", dest_pose={...}, source="voice")` 호출

[탭 3: GUI Guidance (길안내 요청)]
목적: Dobby GUI의 터치 기반 길안내 요청을 시뮬레이션합니다.

```
[ Section: 위치 정보 조회 ]
  [ Input: location_name ] "화장실"
  [ Button: Query Location ] → query_location_info 서비스 호출
  [ Output: 조회 결과 표시 ]
    - Found: true
    - Pose: x=10.5, y=-5.0, theta=1.57
    - Description: "1층 화장실"

[ Section: 길안내 요청 ]
  [ Dropdown: 목적지 선택 ] "화장실" / "카페" / "출입구" / "안내데스크"
  [ Button: Request Guidance (GUI) ] → request_guidance 서비스 호출
  [ Output: 요청 결과 표시 ]
    - Success: true
    - Task ID: task_123
    - Message: "길안내 작업이 생성되었습니다"

[ Section: 빠른 테스트 ]
  [ Button: 화장실 안내 ] 
  [ Button: 카페 안내 ]
  [ Button: 출입구 안내 ]
  → library_locations.yaml의 좌표를 사용하여 즉시 request_guidance 호출
```

[탭 4: Drive (DDC Mocks)]
목적: DDC Mock 서버들의 mode를 제어 UI (각 Mock 노드별로 섹션 반복):
```
dobby1/drive/navigate_to_pose:
[ Label: STATUS: ACTIVE ] (MockStatus 토픽으로 자동 업데이트됨)
[ Button: Set Active ] [ Button: Set Error ]

dobby1/drive/guide_navigation:
[ Label: STATUS: ACTIVE ]
[ Button: Set Active ] [ Button: Set Error ]
→ 길안내 시 사람 추종 주행 Mock 제어
(다른 DDC Mock 인터페이스에 대해서도 반복)
```

[탭 5: Arm (DAC Mocks)]
목적: DAC Mock 서버들의 mode를 제어합니다. UI (각 Mock 노드별로 섹션 반복):
```
dobby1/arm/pick_book:
[ Label: STATUS: ERROR ]
[ Button: Set Active ] [ Button: Set Error ]
(다른 DAC Mock 인터페이스에 대해서도 반복)
```

[탭 6: AI (DVS Mocks)]
목적: DVS Mock 서버/퍼블리셔의 mode를 제어합니다. UI (각 Mock 노드별로 섹션 반복):
```
dobby1/ai/detect_trash:
[ Label: STATUS: ACTIVE ]
[ Button: Set Active ] [ Button: Set Error ]

dobby1/ai/change_tracking_mode:
[ Label: STATUS: ACTIVE ]
[ Button: Set Active ] [ Button: Set Error ]
→ 길안내 시 피안내자 등록/추적 Mock 제어

dobby1/ai/tracking/status: (Topic Publisher)
[ Label: STATUS: OFF ]
[ Button: Set ON ] [ Button: Set OFF ]
→ 피안내자 추적 상태 발행 Mock 제어
```

> **참고:** 탭 3 (GUI Guidance)은 길안내 관련 신규 서비스 테스트를 위해 추가되었습니다.

### 6. 핵심 로직 구현
#### 6.1 gui_node.py (GUI의 ROS 2 노드)
이 노드는 전체 테스트 시스템의 '지휘 본부' 역할을 합니다.

스레딩: rclpy.spin()은 threading.Thread를 통해 백그라운드 스레드에서 실행됩니다.

**클라이언트 보유 (Push & Trigger):**

- ParameterServiceClient 딕셔너리 (모든 Mock 서버 mode 설정용)

- ActionClient 딕셔너리 (DMC 작업 호출용 - Mock RCS 기능)

- ServiceClient 딕셔너리 (DMC 서비스 호출용 - Mock VRC 기능)
  - `query_location_info`: 위치 정보 조회 (신규)
  - `request_guidance`: 길안내 요청 (신규)
  - `set_listening_mode`: 음성 모드 제어
  - `submit_voice_task`: 음성 작업 제출

**YAML Goal 로더:**

- gui_node (또는 widget_rcs)는 시작 시 ament_index_python을 사용해 resource/dmc_test_goals.yaml 파일의 경로를 찾아 내용을 로드합니다.

- RCS 탭의 버튼 클릭 시, 이 로드된 데이터에서 해당 작업의 Goal 메시지를 구성하여 ActionClient로 전송합니다.

** 구독자 보유 (Subscribe):**

- MockStatus 구독자: dobby1/mock_system/status 토픽을 구독합니다. (메시지 타입: javis_dmc_test_msgs.msg.MockStatus)

- /rosout 구독자 (선택 사항): rcl_logging_rosout/Log 메시지를 구독하여 우측 로그 뷰에 모든 시스템 로그를 표시합니다.

**PyQt Signal/Slot (핵심):**

- ROS 콜백 (예: status_callback)은 백그라운드 스레드에서 실행됩니다.

- 해결: gui_node는 PyQt6.QtCore.QObject를 상속하고 QtCore.pyqtSignal을 정의합니다.

- status_callback이 메시지를 받으면, self.my_signal.emit(node_name, mode)와 같이 시그널을 **방출(emit)**합니다.

- main_window.py (GUI 스레드)에 이 시그널에 연결된 슬롯(Slot) 함수가 있어, 안전하게 QLabel의 텍스트를 업데이트합니다.

```
# gui_node.py 콜백 예시
def status_callback(self, msg):
    # 백그라운드 스레드에서 실행됨
    # GUI를 직접 건드리지 않고 시그널만 방출
    self.status_signal.emit(msg.node_name, msg.mode)

# main_window.py 슬롯 예시
@QtCore.pyqtSlot(str, str)
def update_status_label(self, node_name, mode):
    # GUI 스레드에서 안전하게 실행됨
    label_to_update = self.find_label_by_name(node_name)
    if label_to_update:
        label_to_update.setText(f"STATUS: {mode.upper()}")
```

#### 6.2 mock_*.py (Mock 서버 노드)
(DDC, DAC, DVS Mock 서버 노드의 구현은 모두 이 패턴을 따릅니다.)

6.2.1 MockServerBase (Common Logic)
📌 구현 위치: mocks/mock_server_base.py

```Python

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from javis_dmc_test_msgs.msg import MockStatus # 분리된 msg 임포트

class MockServerBase(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        
        # 1. 'mode' 파라미터 선언 (기본값 'active')
        self.declare_parameter('mode', 'active', 
            rclpy.node.ParameterDescriptor(description='Mock mode: active, error, off, on'))
        
        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        
        # 2. 파라미터 변경 시 호출될 콜백 등록
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # 3. 상태 발행을 위한 공통 퍼블리셔
        self.status_publisher = self.create_publisher(
            MockStatus, 
            'dobby1/mock_system/status', 
            10
        )
        
        self.get_logger().info(f'Mock Server [{self.get_name()}] started. Initial mode: {self.mode}')
        self.publish_status() # 시작 시 첫 상태 발행

    def parameter_callback(self, params):
        """파라미터가 변경되면 self.mode를 업데이트하고 상태를 발행합니다."""
        for param in params:
            if param.name == 'mode':
                self.mode = param.value
                self.get_logger().info(f'Mode changed to: {self.mode}')
                self.publish_status() # 변경 즉시 상태 발행
                return SetParametersResult(successful=True)
        return SetParametersResult(successful=True)

    def publish_status(self):
        """현재 노드 이름과 모드를 'dobby1/mock_system/status' 토픽으로 발행합니다."""
        msg = MockStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.node_name = self.get_name() # "mock_dac_pick_book" 등
        msg.mode = self.mode
        self.status_publisher.publish(msg)
```
#### 6.2.2 Action/Service Server Logic (Example)
MockServerBase를 상속받아 execute_callback 또는 service_callback만 구현합니다.

```Python

# mocks/mock_dac_servers.py 예시

from .mock_server_base import MockServerBase
# (PickBook Action 임포트)

class MockPickBookAction(MockServerBase):
    def __init__(self, node_name='mock_dac_pick_book'):
        super().__init__(node_name)
        # Action 서버 생성
        self._action_server = ActionServer(
            self,
            PickBook,
            'dobby1/arm/pick_book',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'PickBook Action called in mode: {self.mode}')
        
        # (필요시 time.sleep(1) 등으로 지연시간 시뮬레이션)
        
        if self.mode == "active":
            goal_handle.succeed()
            result = PickBook.Result()
            result.success = True
            return result
        else: # 'error' 또는 그 외
            goal_handle.abort()
            result = PickBook.Result()
            result.success = False
            return result

```

#### 6.2.3 Topic Publisher Logic (Example)
on/off 모드를 사용하여 rclpy.Timer를 제어합니다.

📌 구현 위치: mocks/mock_dvs_servers.py

```Python

# mocks/mock_dvs_servers.py 예시

from .mock_server_base import MockServerBase
from javis_interfaces.msg import TrackingStatus # (예시) 실제 추적 메시지

class MockDvsTrackingStatus(MockServerBase):
    def __init__(self, node_name='mock_dvs_tracking_status'):
        super().__init__(node_name)
        
        # 1. 파라미터 모드를 'off'로 재정의
        self.undeclare_parameter('mode') # 부모의 'active' 선언 제거
        self.declare_parameter('mode', 'off') # 'off'를 기본값으로 선언
        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        
        # 2. 토픽 퍼블리셔 생성
        self.publisher_ = self.create_publisher(TrackingStatus, 'dobby1/ai/tracking/status', 10)
        
        # 3. 1초마다 발행할 타이머 생성 (처음엔 시작하지 않음)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.timer.cancel() # 일단 정지
        
        if self.mode == 'on':
            self.timer.reset() # 'on' 상태로 시작하면 타이머 시작

        self.publish_status() # 부모의 상태 발행 함수 호출

    def parameter_callback(self, params):
        # 부모 콜백 호출 (mode 변수 업데이트 및 상태 발행)
        result = super().parameter_callback(params) 
        
        # 4. 'on'/'off' 모드에 따라 타이머 제어
        if self.mode == 'on':
            self.get_logger().info('Mode set to ON. Starting publisher timer.')
            self.timer.reset() # 타이머 시작 (또는 재시작)
        else: # 'off' 또는 'error' 등 다른 모든 상태
            self.get_logger().info('Mode set to OFF. Stopping publisher timer.')
            self.timer.cancel() # 타이머 정지
            
        return result

    def timer_callback(self):
        # 5. 타이머가 실행 중일 때(즉, mode='on'일 때) Mock 메시지 발행
        msg = TrackingStatus()
        msg.person_detected = True
        msg.tracking_id = "mock_user_123"
        # ... (기타 메시지 필드 채우기) ...
        self.publisher_.publish(msg)

```

---

## 7. 현재 구현 상태

본 섹션에서는 위의 설계를 기반으로 **실제로 구현된 항목**과 **구현 예정 항목**을 구분하여 설명합니다.

### 7.1 Status GUI 구현

**구현 완료**: DMC 상태 모니터링을 위한 Status GUI가 `javis_dmc_test/status_gui/` 디렉토리에 구현되어 있습니다.

#### 7.1.1 패키지 구조

```
javis_dmc_test/status_gui/
├── __init__.py
├── status_gui_node.py      # ROS 2 노드 (백그라운드 스레드)
└── status_gui_widget.py    # Tkinter UI 애플리케이션 (메인 스레드)
```

#### 7.1.2 실행 방법

```bash
# Entry point: setup.py에 등록됨
ros2 run javis_dmc_test status_gui_node
```

#### 7.1.3 아키텍처

**이중 스레딩 구조**로 Tkinter와 ROS 2를 안전하게 통합합니다:

**ROS 스레드** (`status_gui_node.py`):
- `MultiThreadedExecutor`로 `rclpy.spin()` 백그라운드 실행
- DMC 상태 구독: `/{namespace}/status/robot_state` (DobbyState)
- 배터리 상태 구독: `/{namespace}/status/battery_status` (BatteryStatus)
- 로그 구독: `/rosout` (Log)
- 모드 피드백 구독: `/{namespace}/admin/mode_feedback` (String)
- 상태 전환 구독: `/{namespace}/debug/state_transitions` (String)
- 서비스 클라이언트:
  - `/{namespace}/admin/set_robot_mode` (SetRobotMode)
  - `/{namespace}/admin/emergency_stop` (Trigger)
  - `/{namespace}/admin/resume_navigation` (Trigger)
  - `/{namespace}/set_listening_mode` (SetBool)
  - `/{namespace}/debug/describe_state_machine` (Trigger)
- 스레드 안전 `Queue`를 통해 GUI 스레드에 이벤트 전달

**GUI 스레드** (`status_gui_widget.py`):
- Tkinter 메인 루프 실행 (`tk.mainloop()`)
- 주요 컴포넌트:
  - `StateGraphCanvas`: 상태 다이어그램 시각화
  - `StatusGuiApp`: 전체 애플리케이션 관리
- UI 구성:
  - 상단: 현재 상태 표시 (Main State, Sub State, Mode)
  - 좌측: 상태 그래프 시각화
  - 우측: 로그 디스플레이 (색상 코딩: INFO/WARN/ERROR)
  - 하단: 제어 버튼 (모드 전환, 비상 정지, 리스닝 모드)

**통신 패턴**:
```
ROS 콜백 (백그라운드) → event_queue.put() → GUI 폴링 (after_idle) → UI 업데이트
GUI 버튼 클릭 → ROS 서비스 호출 (블로킹) → 응답
```

#### 7.1.4 주요 특징

1. **상태 시각화**: DMC의 상태 머신을 실시간으로 시각화
2. **배터리 모니터링**: 배터리 레벨 및 충전 상태 표시
3. **로그 통합**: 시스템 전체 로그를 색상별로 구분하여 표시
4. **원격 제어**: 모드 전환, 비상 정지, 리스닝 모드 활성화 등
5. **네임스페이스 지원**: `robot_namespace` 파라미터로 여러 로봇 지원

#### 7.1.5 Status GUI와 설계된 Test GUI의 차이

| 항목 | Status GUI (현재 구현) | Test GUI (설계 문서) |
| :--- | :--- | :--- |
| **목적** | DMC 상태 **모니터링 및 제어** | Mock 시스템 **제어 및 테스트 시나리오 실행** |
| **UI 프레임워크** | Tkinter | PyQt6 |
| **주요 기능** | - 상태 시각화<br>- 모드 전환<br>- 비상 정지<br>- 로그 표시 | - Mock 서버 mode 제어<br>- RCS 작업 전송<br>- VRC 이벤트 트리거<br>- Goal YAML 로딩 |
| **구독 토픽** | `robot_state`, `battery_status`, `/rosout` | `dobby1/mock_system/status` |
| **제어 대상** | DMC 노드 | Mock 서버 노드 + DMC 노드 |
| **구현 상태** | ✅ 완료 | ⏳ 설계 완료, 구현 예정 |

### 7.2 구현 예정 항목

다음 항목들은 설계가 완료되었으나 아직 구현되지 않았습니다:

#### 7.2.1 Mock 서버 노드 (`mocks/`)
- `mock_server_base.py`: 모든 Mock 서버의 베이스 클래스
- `mock_ddc_servers.py`: Drive 인터페이스 Mock
- `mock_dac_servers.py`: Arm 인터페이스 Mock
- `mock_dvs_servers.py`: AI/Vision 인터페이스 Mock

#### 7.2.2 Mock 시스템 제어 GUI (`gui/`)
- `gui_node.py`: PyQt6 기반 ROS 2 노드
- `main_window.py`: PyQt6 메인 윈도우
- `widgets/`: 커스텀 위젯들
  - `widget_guidance.py`: 길안내 요청 탭 위젯 (신규)
    - 위치 정보 조회 UI
    - 길안내 요청 UI
    - 빠른 테스트 버튼

#### 7.2.3 실행 스크립트 (`nodes/`)
- `start_gui.py`: Test GUI 실행
- `start_mock_ddc.py`: DDC Mock 서버 그룹 실행
- `start_mock_dac.py`: DAC Mock 서버 그룹 실행
- `start_mock_dvs.py`: DVS Mock 서버 그룹 실행
- `start_mock_rcs.py`: RCS Mock 클라이언트 실행 (길안내 작업 생성)

#### 7.2.4 리소스 파일
- `resource/dmc_test_goals.yaml`: RCS 작업 Goal 정의
  - `guide_person`: 화장실 길안내
  - `guide_person_cafe`: 카페 길안내
  - `guide_person_entrance`: 출입구 길안내
- `resource/main_window.ui`: PyQt6 UI 파일 (선택사항)
- `resource/test_locations.yaml`: 테스트용 위치 정보 (library_locations.yaml 복사본)

#### 7.2.5 런치 파일
- `launch/dmc_mock.launch.py`: 전체 Mock 시스템 통합 실행

#### 7.2.6 메시지 정의
- `javis_dmc_test_msgs/msg/MockStatus.msg`: Mock 상태 메시지

### 7.3 구현 우선순위

다음 순서로 구현하는 것을 권장합니다:

1. **메시지 정의** (`MockStatus.msg`) - 다른 모든 구현의 기반
2. **Mock 서버 베이스** (`mock_server_base.py`) - 공통 로직
3. **길안내 서비스 통합** (우선순위 상향) - 신규 기능 검증
   - `library_locations.yaml` 로드 로직
   - `query_location_info` 서비스 구현
   - `request_guidance` 서비스 구현
4. **Mock 서버 구현** (`mock_ddc_servers.py`, `mock_dac_servers.py`, `mock_dvs_servers.py`)
   - 특히 `guide_navigation` Mock (사람 추종 주행)
   - `change_tracking_mode` Mock (피안내자 등록/추적)
5. **실행 스크립트** (`nodes/start_mock_*.py`)
6. **리소스 파일** (`dmc_test_goals.yaml` - 길안내 Goal 포함)
7. **Test GUI - 길안내 탭** (`widget_guidance.py`)
8. **Test GUI - 전체** (`gui_node.py`, `main_window.py`)
9. **런치 파일** (`dmc_mock.launch.py`)

---

## 8. 길안내 테스트 시나리오 (신규 추가)

### 8.1 GUI 터치 기반 길안내 테스트

**목적:** Dobby GUI에서 터치로 목적지를 선택하는 시나리오를 검증합니다.

**테스트 절차:**
1. Test GUI 탭 3 (GUI Guidance) 열기
2. "화장실" 선택 후 "Query Location" 클릭
3. 결과 확인: `found: true`, `pose: {x:10.5, y:-5.0, theta:1.57}`
4. "Request Guidance (GUI)" 버튼 클릭
5. DMC 상태 전환 확인: `IDLE/ROAMING → GUIDING`
6. Status GUI에서 상태 모니터링

**예상 결과:**
- `query_location_info` 응답: 화장실 좌표 반환
- `request_guidance` 응답: `success: true`, `task_id: "task_xxx"`
- DMC Action 수락: `guide_person` Goal 수신
- DDC Mock: `guide_navigation` Action 호출

### 8.2 음성 기반 길안내 테스트

**목적:** VRC 음성 요청으로 길안내를 시작하는 시나리오를 검증합니다.

**테스트 절차:**
1. Test GUI 탭 2 (VRC) 열기
2. "Trigger Wake Word" 클릭 → DMC `LISTENING` 상태 전환
3. 목적지 입력: "카페"
4. "Trigger Voice Task" 클릭
5. DMC 상태 전환 확인: `LISTENING → GUIDING`
6. Status GUI에서 상태 모니터링

**예상 결과:**
- `submit_voice_task` 호출 성공
- DMC 내부적으로 `query_location_info("카페")` 실행
- `request_guidance` 서비스 호출 (source: "voice")
- RCS Action Goal 전송: `guide_person` with cafe coordinates

### 8.3 실패 시나리오 테스트

**목적:** 예외 상황 처리를 검증합니다.

**테스트 케이스:**
1. **존재하지 않는 위치 조회**
   - Input: "존재하지않는곳"
   - Expected: `found: false`, `message: "위치를 찾을 수 없습니다"`

2. **배터리 부족 시 작업 거부**
   - DMC 배터리를 30%로 설정 (Mock)
   - 길안내 요청
   - Expected: `success: false`, `message: "배터리 부족"`

3. **진행 중인 작업이 있을 때**
   - 다른 작업 실행 중
   - 길안내 요청
   - Expected: `success: false`, `message: "진행 중인 작업이 있습니다"`

### 8.4 통합 테스트 시나리오

**목적:** 전체 GUI 길안내 플로우를 검증합니다.

**시나리오 (v4.0):**
1. Mock 서버 모두 `active` 모드로 설정
2. DMC 상태: IDLE (배터리 >= 40%)
3. **Mock RCS**: CreateUserTask Service Server 시작 (대기 중)
4. GUI에서 QueryLocationInfo("") 호출 (목적지 입력 의사 표현)
5. DMC: State IDLE → **WAITING_DEST_INPUT** (60초 타이머 시작)
6. DMC → GUI: 목적지 목록 반환 [화장실, 카페, 출입구, ...]
7. GUI에서 RequestGuidance("화장실", pose, "gui") 호출
8. DMC: 배터리 체크 (≥40%)
9. DMC → **Mock RCS**: CreateUserTask Service 호출 (user_initiated=True)
10. **Mock RCS**: user_initiated=True 확인 → GuidePerson Action Goal 전송
11. DMC: 60초 타이머 취소
12. DMC: Action Goal 수락 → State **WAITING_DEST_INPUT → GUIDING**
13. DMC → GUI: {success: true, task_id="task_123"}
14. DMC → DDC `guide_navigation` 호출 (Mock: active)
15. DMC → DVS `change_tracking_mode(REGISTRATION)` 호출 (Mock: active)
16. 피안내자 등록 완료 (Mock 응답)
17. DVS `change_tracking_mode(TRACKING)` 전환
18. 목적지 도착 (Mock 응답)
19. 작업 완료: `GUIDING → IDLE`

**검증 포인트 (v4.0):**
- QueryLocationInfo 호출 시 **WAITING_DEST_INPUT** 상태 전환 확인
- 60초 타이머 시작 확인
- RequestGuidance 호출 시 **DMC → Mock RCS CreateUserTask** 호출 확인
- Mock RCS가 **user_initiated=True** 플래그 확인하고 즉시 Action 호출
- 60초 타이머 취소 확인
- 배터리 >= 40% 체크 (RequestGuidance 시)
- 상태 전환 순서 정확 (IDLE → **WAITING_DEST_INPUT** → **GUIDING** → IDLE)
- Mock RCS CreateUserTask Service 로깅 확인
- 로그 메시지 정확성

---
