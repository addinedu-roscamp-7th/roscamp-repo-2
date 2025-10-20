JAVIS DMC (Dobby Main Controller) 상세 설계서 v5.1
📋 목차

시스템 컨텍스트
전체 아키텍처 다이어그램
패키지 구조
클래스 다이어그램
State Machine 설계
배터리 관리
설계 요약


1. 시스템 컨텍스트
1.1 설계 확정 사항
항목결정 사항DDC 위치 정보DDC → RCS 직접 + DMC 구독 (병렬)TF 프레임공통 map + 개별 odom/base_link (dobby1/, dobby2/)메시지 패키지javis_interfaces (기존) + javis_dmc_test_msgs (테스트용)테스트 도구Test GUI만 사용작업 수락 판단RCS가 판단, DMC는 최종 확인만DDC Dockingmove_to_target에 파라미터 추가
1.2 DMC 역할
┌─────────────────────────────────────────────────────────────┐
│                Robot Control Service (RCS)                   │
│  - 작업 큐 관리                                               │
│  - 작업 수락 판단 (DMC 상태 + 배터리 확인)                      │
│  - 위치 정보 수신 (DDC Topic 직접 구독)                        │
└──────────────────┬──────────────────────────────────────────┘
                   │ ROS2 Action
         ┌─────────┴─────────┐
         │                   │
    ┌────▼────┐         ┌────▼────┐
    │  DMC1   │         │  DMC2   │
    │(dobby1) │         │(dobby2) │
    │         │         │         │
    │ 역할:    │         │ 역할:    │
    │ - 상태   │         │ - 상태   │
    │   관리   │         │   관리   │
    │ - 작업   │         │ - 작업   │
    │   실행   │         │   실행   │
    │ - 배터리 │         │ - 배터리 │
    │   감시   │         │   감시   │
    └────┬────┘         └────┬────┘
         │                   │
    ┌────┼────┬────┬────┬────┼────┐
    │    │    │    │    │    │    │
  DDC  DAC  AIS  GUI  DDC  DAC  ...
   │                   │
   │ Topic (병렬)       │
   ├──────────────────>│ RCS (위치 모니터링)
   └──────────────────>│ DMC (위치 확인)
```

---

## 2. 전체 아키텍처 다이어그램

### 2.1 Namespace 구조
```
dobby1/
├── main/
│   ├── pickup_book
│   ├── reshelving_book
│   ├── guide_person
│   ├── clean_seat
│   └── sorting_shelves
├── drive/
│   ├── move_to_target (Action)
│   ├── guide_navigation (Action)
│   └── control_command (Service)
├── arm/
│   ├── pick_book (Action)
│   ├── place_book (Action)
│   └── ...
├── ai/
│   ├── detect_book (Service)
│   ├── register_person (Service)
│   └── tracking/status (Topic)
├── gui/
│   ├── update_screen (Service)
│   └── screen_event (Topic)
├── status/
│   ├── robot_state (Topic)
│   ├── battery_status (Topic)
│   └── current_pose (Topic)
└── test/  # Test GUI용
    ├── set_battery (Service)
    └── set_mock_response (Service)

dobby2/
└── (동일 구조)
```

### 2.2 TF 프레임 구조
```
map (공통)
 ├─ dobby1/odom
 │   └─ dobby1/base_link
 │       ├─ dobby1/camera_link
 │       └─ dobby1/gripper_link
 └─ dobby2/odom
     └─ dobby2/base_link
         ├─ dobby2/camera_link
         └─ dobby2/gripper_link
```

---

## 3. 패키지 구조
```
javis_dmc/
├── javis_dmc/                      # Python 패키지
│   ├── __init__.py
│   ├── dmc_node.py                 # Main Node
│   ├── battery_manager.py          # 배터리 관리
│   │
│   ├── states/                     # 상태 정의 및 SMACH State 클래스
│   │   ├── __init__.py
│   │   ├── main_states.py          # Main State 클래스
│   │   └── state_enums.py          # State Enum (javis_interfaces 사용)
│   │
│   ├── task_executors/             # 작업 실행 로직
│   │   ├── __init__.py
│   │   ├── base_executor.py
│   │   ├── pickup_executor.py
│   │   ├── reshelving_executor.py
│   │   ├── guiding_executor.py
│   │   ├── cleaning_executor.py
│   │   └── sorting_executor.py
│   │
│   ├── interfaces/                 # 하위 컨트롤러 인터페이스
│   │   ├── __init__.py
│   │   ├── base_interface.py
│   │   ├── drive_interface.py
│   │   ├── arm_interface.py
│   │   ├── ai_interface.py
│   │   └── gui_interface.py
│   │
│   ├── mock/                       # Mock 구현
│   │   ├── __init__.py
│   │   ├── mock_drive.py
│   │   ├── mock_arm.py
│   │   ├── mock_ai.py
│   │   ├── mock_gui.py
│   │   ├── mock_llm.py
│   │   ├── mock_stt.py
│   │   └── mock_tts.py
│   │
│   └── utils/
│       ├── __init__.py
│       ├── logger.py
│       └── ros_utils.py
│
├── test_gui/                       # Test GUI
│   ├── __init__.py
│   ├── test_gui_node.py
│   └── test_gui_widget.py
│
├── config/
│   ├── dmc_params.yaml
│   ├── battery_config.yaml
│   ├── action_timeouts.yaml
│   └── charger_location.yaml
│
├── launch/
│   ├── dmc_single.launch.py
│   ├── dmc_multi.launch.py
│   └── dmc_test.launch.py
│
├── test/
│   ├── test_battery_manager.py
│   ├── test_state_machine.py
│   └── test_executors.py
│
├── package.xml
├── setup.py
├── setup.cfg
└── README.md

javis_dmc_test_msgs/               # 테스트용 메시지 패키지
├── srv/
│   ├── SetBattery.srv
│   └── SetMockResponse.srv
├── package.xml
└── CMakeLists.txt
```

---

## 4. 클래스 다이어그램

### 4.1 최상위 구조
```
┌─────────────────────────────────────────────────────────┐
│        DobbyMainController (rclpy.Node)                  │
├─────────────────────────────────────────────────────────┤
│ - namespace: str                                        │
│ - main_sm: smach.StateMachine                           │
│ - current_executor: Optional[BaseExecutor]              │
│ - current_pose: Pose2D                                  │
│                                                         │
│ - battery: BatteryManager                               │
│ - drive: DriveInterface                                 │
│ - arm: ArmInterface                                     │
│ - ai: AIInterface                                       │
│ - gui: GUIInterface                                     │
└─────────────┬───────────────┬───────────────┬───────────┘
              │               │               │
              │ has           │ has           │ has
              ▼               ▼               ▼
    ┌──────────────┐  ┌─────────────┐  ┌──────────────┐
    │ Battery      │  │ SMACH       │  │ Base         │
    │ Manager      │  │ State       │  │ Executor     │
    │              │  │ Machine     │  │              │
    └──────────────┘  └─────────────┘  └──────────────┘
```

### 4.2 DobbyMainController 멤버
```
DobbyMainController
├── [상태]
│   ├── namespace: str
│   ├── main_sm: StateMachine
│   ├── current_executor: Optional[BaseExecutor]
│   └── current_pose: Pose2D
│
├── [컴포넌트]
│   ├── battery: BatteryManager
│   ├── drive: DriveInterface
│   ├── arm: ArmInterface
│   ├── ai: AIInterface
│   └── gui: GUIInterface
│
├── [Action Servers]
│   ├── pickup_server
│   ├── reshelving_server
│   ├── guide_server
│   ├── clean_server
│   └── sorting_server
│
├── [Publishers]
│   ├── state_pub: Publisher<DobbyState>
│   └── battery_pub: Publisher<BatteryStatus>
│
├── [Subscribers]
│   └── current_pose_sub: Subscription<Pose2D>
│
└── [Timers]
    ├── battery_timer (1Hz)
    ├── state_timer (10Hz)
    └── sm_timer (100Hz)
```

### 4.3 인터페이스 계층
```
BaseInterface (ABC)
├── initialize() → bool
├── shutdown()
└── _create_topic_name(topic) → str
    │
    ├── DriveInterface
    │   ├── move_to_target(...)
    │   ├── guide_navigation(...)
    │   ├── control_command(...)
    │   ├── cancel_all_actions()
    │   └── is_action_active()
    │
    ├── ArmInterface
    │   ├── pick_book(...)
    │   ├── place_book(...)
    │   ├── cancel_all_actions()
    │   └── is_action_active()
    │
    ├── AIInterface
    │   ├── detect_book(...)
    │   ├── register_person(...)
    │   └── change_tracking_mode(...)
    │
    └── GUIInterface
        ├── update_screen(...)
        └── wait_for_event(...)
```

### 4.4 Executor 계층
```
BaseExecutor (ABC)
├── execute()
├── wait_for_result()
├── cancel()
├── _set_sub_state(sub_state)
└── _publish_feedback(progress)
    │
    ├── PickupExecutor
    ├── ReshelvingExecutor
    ├── GuidingExecutor
    ├── CleaningExecutor
    └── SortingExecutor

5. State Machine 설계
5.1 Main State 정의 (DobbyState.msg 기준)
State값설명배터리 변화작업 수락INITIALIZING0시스템 초기화없음❌CHARGING1충전 중 (battery < 40%)+10%/min❌IDLE2대기 (충전소, battery >= 40%)+10%/min (충전소), 없음 (다른 위치)✅MOVING_TO_CHARGER3충전소로 이동-1%/min✅ (battery >= 40%)PICKING_UP_BOOK4도서 픽업 실행-1%/min❌RESHELVING_BOOK5반납 정리 실행-1%/min❌GUIDING6길안내 실행-1%/min❌CLEANING_DESK7좌석 정리 실행-1%/min❌SORTING_SHELVES8서가 정리 실행-1%/min❌FORCE_MOVE_TO_CHARGER9긴급 충전 복귀-1%/min❌MAIN_ERROR99에러 상태없음❌
5.2 Sub State 정의 (DobbyState.msg 기준)
General

NONE = 100

Book Pickup Task (101~104)

MOVE_TO_PICKUP = 101 (책장으로 이동)
PICKUP_BOOK = 102 (도서 픽업 → 운반함)
MOVE_TO_STORAGE = 103 (보관함으로 이동)
STOWING_BOOK = 104 (보관함에 배치)

Reshelve Book Task (105~108)

MOVE_TO_RETURN_DESK = 105 (반납대로 이동)
COLLECT_RETURN_BOOKS = 106 (반납 도서 수거)
MOVE_TO_PLACE_SHELF = 107 (책장으로 이동)
PLACE_RETURN_BOOK = 108 (책장에 배치)

Guiding Task (109~112)

SELECT_DEST = 109 (목적지 선택)
SCAN_USER = 110 (사용자 스캔)
GUIDING_TO_DEST = 111 (목적지 안내)
FIND_USER = 112 (사용자 재탐색)

Cleaning Desk Task (113~117)

MOVE_TO_DESK = 113 (좌석으로 이동)
SCAN_DESK = 114 (좌석 스캔)
CLEANING_TRASH = 115 (쓰레기 수거)
MOVE_TO_BIN = 116 (쓰레기통으로 이동)
TIDYING_SHELVES = 117 (정리)

Sorting Shelf Task (118~120)

MOVE_TO_SHELF = 118 (서가로 이동)
SCAN_BOOK = 119 (도서 스캔)
SORT_BOOK = 120 (도서 정리)

Error

SUB_ERROR = 199

5.3 Main State Diagram
mermaidstateDiagram-v2
    [*] --> INITIALIZING
    
    INITIALIZING --> CHARGING: init_complete
    
    CHARGING --> IDLE: battery >= 40%
    
    note right of CHARGING
        충전 중
        battery < 40%
        작업 수락 불가
        +10%/min
    end note
    
    IDLE --> PICKING_UP_BOOK: task_assigned(PICKUP)
    IDLE --> RESHELVING_BOOK: task_assigned(RESHELVING)
    IDLE --> GUIDING: task_assigned(GUIDING)
    IDLE --> CLEANING_DESK: task_assigned(CLEANING)
    IDLE --> SORTING_SHELVES: task_assigned(SORTING)
    IDLE --> FORCE_MOVE_TO_CHARGER: battery < 20%
    
    note right of IDLE
        대기 (충전소)
        battery >= 40%
        작업 수락 가능
        충전소면 +10%/min
    end note
    
    PICKING_UP_BOOK --> IDLE: complete & at_charger
    PICKING_UP_BOOK --> MOVING_TO_CHARGER: complete & not_at_charger
    PICKING_UP_BOOK --> IDLE: failed
    PICKING_UP_BOOK --> FORCE_MOVE_TO_CHARGER: battery < 20%
    
    RESHELVING_BOOK --> IDLE: complete & at_charger
    RESHELVING_BOOK --> MOVING_TO_CHARGER: complete & not_at_charger
    RESHELVING_BOOK --> IDLE: failed
    RESHELVING_BOOK --> FORCE_MOVE_TO_CHARGER: battery < 20%
    
    GUIDING --> IDLE: complete & at_charger
    GUIDING --> MOVING_TO_CHARGER: complete & not_at_charger
    GUIDING --> IDLE: failed
    GUIDING --> FORCE_MOVE_TO_CHARGER: battery < 20%
    
    CLEANING_DESK --> IDLE: complete & at_charger
    CLEANING_DESK --> MOVING_TO_CHARGER: complete & not_at_charger
    CLEANING_DESK --> IDLE: failed
    CLEANING_DESK --> FORCE_MOVE_TO_CHARGER: battery < 20%
    
    SORTING_SHELVES --> IDLE: complete & at_charger
    SORTING_SHELVES --> MOVING_TO_CHARGER: complete & not_at_charger
    SORTING_SHELVES --> IDLE: failed
    SORTING_SHELVES --> FORCE_MOVE_TO_CHARGER: battery < 20%
    
    MOVING_TO_CHARGER --> IDLE: arrived & battery >= 40%
    MOVING_TO_CHARGER --> CHARGING: arrived & battery < 40%
    MOVING_TO_CHARGER --> PICKING_UP_BOOK: task_assigned & battery >= 40%
    MOVING_TO_CHARGER --> RESHELVING_BOOK: task_assigned & battery >= 40%
    MOVING_TO_CHARGER --> GUIDING: task_assigned & battery >= 40%
    MOVING_TO_CHARGER --> CLEANING_DESK: task_assigned & battery >= 40%
    MOVING_TO_CHARGER --> SORTING_SHELVES: task_assigned & battery >= 40%
    MOVING_TO_CHARGER --> FORCE_MOVE_TO_CHARGER: battery < 20%
    
    note right of MOVING_TO_CHARGER
        충전소로 이동
        battery >= 40%면 작업 수락 가능
        작업 수락 시 이동 취소
        -1%/min
    end note
    
    FORCE_MOVE_TO_CHARGER --> IDLE: arrived & battery >= 40%
    FORCE_MOVE_TO_CHARGER --> CHARGING: arrived & battery < 40%
    
    note right of FORCE_MOVE_TO_CHARGER
        긴급 충전 복귀
        - 모든 Action 취소
        - Executor 중단
        - RCS 실패 보고
    end note
    
    IDLE --> MAIN_ERROR: critical_error
    PICKING_UP_BOOK --> MAIN_ERROR: critical_error
    RESHELVING_BOOK --> MAIN_ERROR: critical_error
    GUIDING --> MAIN_ERROR: critical_error
    CLEANING_DESK --> MAIN_ERROR: critical_error
    SORTING_SHELVES --> MAIN_ERROR: critical_error
    MOVING_TO_CHARGER --> MAIN_ERROR: critical_error
    FORCE_MOVE_TO_CHARGER --> MAIN_ERROR: critical_error
    
    MAIN_ERROR --> IDLE: error_resolved
5.4 Sub State Diagram - Pickup Book
mermaidstateDiagram-v2
    [*] --> MOVE_TO_PICKUP
    
    MOVE_TO_PICKUP --> PICKUP_BOOK: arrived
    PICKUP_BOOK --> MOVE_TO_STORAGE: picked
    MOVE_TO_STORAGE --> STOWING_BOOK: arrived
    STOWING_BOOK --> [*]: stored
5.5 Sub State Diagram - Reshelving Book
mermaidstateDiagram-v2
    [*] --> MOVE_TO_RETURN_DESK
    
    MOVE_TO_RETURN_DESK --> COLLECT_RETURN_BOOKS: arrived
    COLLECT_RETURN_BOOKS --> MOVE_TO_PLACE_SHELF: collected
    MOVE_TO_PLACE_SHELF --> PLACE_RETURN_BOOK: arrived
    PLACE_RETURN_BOOK --> MOVE_TO_PLACE_SHELF: more_books
    PLACE_RETURN_BOOK --> [*]: all_done
5.6 Sub State Diagram - Guiding
mermaidstateDiagram-v2
    [*] --> SELECT_DEST
    
    SELECT_DEST --> SCAN_USER: dest_selected
    SCAN_USER --> GUIDING_TO_DEST: user_registered
    GUIDING_TO_DEST --> [*]: arrived
    GUIDING_TO_DEST --> FIND_USER: user_lost
    FIND_USER --> GUIDING_TO_DEST: user_found
    FIND_USER --> [*]: timeout
5.7 Sub State Diagram - Cleaning Desk
mermaidstateDiagram-v2
    [*] --> MOVE_TO_DESK
    
    MOVE_TO_DESK --> SCAN_DESK: arrived
    SCAN_DESK --> CLEANING_TRASH: trash_found
    SCAN_DESK --> [*]: clean
    CLEANING_TRASH --> MOVE_TO_BIN: collected
    MOVE_TO_BIN --> TIDYING_SHELVES: arrived
    TIDYING_SHELVES --> [*]: done
5.8 Sub State Diagram - Sorting Shelves
mermaidstateDiagram-v2
    [*] --> MOVE_TO_SHELF
    
    MOVE_TO_SHELF --> SCAN_BOOK: arrived
    SCAN_BOOK --> SORT_BOOK: misplaced_found
    SCAN_BOOK --> MOVE_TO_SHELF: next_shelf
    SCAN_BOOK --> [*]: all_done
    SORT_BOOK --> SCAN_BOOK: sorted
```

---

## 6. 배터리 관리

### 6.1 배터리 상태
```
BatteryState (Enum)
├── IDLE = 0        # 변화 없음
├── CHARGING = 1    # +10%/min
└── DRAINING = 2    # -1%/min
```

### 6.2 배터리 레벨별 동작

| 레벨 | 상태 | DMC 동작 | 작업 수락 |
|------|------|----------|----------|
| 100% - 80% | 정상 (충전 완료) | CHARGING → IDLE | ✅ |
| 79% - 40% | 정상 | 작업 수행 | ✅ |
| 39% - 20% | 경고 | 작업 완료 후 충전소 이동 | ✅ |
| 19% - 0% | 위험 | 강제 충전 복귀 | ❌ |

### 6.3 BatteryManager 구조
```
BatteryManager
├── [멤버]
│   ├── level: float (0.0 ~ 100.0)
│   ├── state: BatteryState
│   ├── charge_rate: float (10.0)
│   ├── work_rate: float (-1.0)
│   ├── critical_threshold: float (20.0)
│   ├── warning_threshold: float (40.0)
│   ├── charge_target: float (80.0)
│   └── test_mode_enabled: bool
│
└── [메서드]
    ├── update(dt)
    ├── start_charging()
    ├── start_draining()
    ├── set_idle()
    ├── is_critical() → bool
    ├── is_warning() → bool
    ├── is_sufficient() → bool
    ├── force_set(level)
    ├── enable_test_mode()
    └── disable_test_mode()

7. 설계 요약
7.1 핵심 특징
항목내용Main State10개 (DobbyState.msg 정의 기준)Sub State21개 (DobbyState.msg 정의 기준)작업 타입5개 (Pickup, Reshelving, Guiding, Cleaning, Sorting)인터페이스4개 (Drive, Arm, AI, GUI)Mock 지원Interface 기반 교체 가능Test GUI배터리 제어 + Mock 설정 + 상태 모니터링
7.2 파일 구성
파일역할dmc_node.pyMain Node, State Machine, Action Serversbattery_manager.py배터리 관리main_states.pySMACH State 클래스 구현base_executor.pyExecutor 공통 기능pickup_executor.py 등작업별 실행 로직*_interface.py하위 컨트롤러 통신mock_*.py테스트용 Mocktest_gui_*.pyTest GUI
