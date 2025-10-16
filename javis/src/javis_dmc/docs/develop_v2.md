###  DMC (Dobby Main Controller) 상세 설계서 v2.0

v2.02025-10-16-
• TaskManager 제거 (RCS가 관리)<br>
• 음성 인식 기능 추가 (LISTENING 상태)<br>
• 도비 내부 운반함(carrier) 반영<br>
• 도서 픽업/반납 시나리오 정정<br>
• SMACH 기반 설계 확정


1. 시스템 컨텍스트
1.1 DMC의 위치와 역할
                    ┌─────────────────────────────────────┐
                    │  Robot Control Service (RCS)        │
                    │  ================================    │
                    │  [역할]                              │
                    │  - 작업 큐 관리                      │
                    │  - 여러 로봇에게 작업 분배            │
                    │  - 작업 우선순위 결정                 │
                    │  - 작업 이력 관리                    │
                    │  - 스케줄링 (반납 정리: 1시간마다)    │
                    └──────────────┬──────────────────────┘
                                   │ ROS2 Action/Topic
                    ┌──────────────▼──────────────────────┐
                    │  DMC (Dobby Main Controller)        │
                    │  ===========================        │
                    │  [역할]                              │
                    │  - 단일 작업 실행 (한 번에 1개)      │
                    │  - 상태 관리 (SMACH)                 │
                    │  - 하위 컨트롤러 조율                 │
                    │  - 배터리 모니터링                    │
                    │  - 음성 인식 처리                     │
                    │                                      │
                    │  [책임]                              │
                    │  - 작업 수락/거부 판단                │
                    │  - 작업 성공/실패 보고                │
                    │  - 긴급 상황 처리                     │
                    │  - 현재 작업 상태 발행                │
                    └──────┬────────┬──────────┬──────────┘
                           │        │          │
            ┌──────────────┤        │          └─────────────┐
            │ ROS2         │        │ ROS2                   │ ROS2
    ┌───────▼────────┐ ┌──▼────────▼──┐ ┌─────────────┐ ┌───▼──────────┐
    │ DDC            │ │ DAC           │ │ AIS         │ │ Dobby GUI    │
    │ (Drive)        │ │ (Arm)         │ │ (Vision)    │ │              │
    │ - Nav2 주행    │ │ - MoveIt 제어 │ │ - 객체 인식 │ │ - 사용자 UI  │
    │ - 추종 모드    │ │ - 그리퍼 제어 │ │ - 사람 추적 │ │ - 터치 입력  │
    │ - 충돌 회피    │ │ - 내부 운반함 │ │ - 장애물    │ │ - 상태 표시  │
    └────────────────┘ └───────────────┘ └─────────────┘ └──────────────┘

    ┌─────────────────────────────────────────────────────────────────┐
    │  External Services (HTTP/WebSocket)                             │
    │  - STT Service (Speech-to-Text)                                 │
    │  - TTS Service (Text-to-Speech)                                 │
    │  - LLM Service (Intent Parsing)                                 │
    │    • POST /LLM/books   (도서 검색)                              │
    │    • POST /LLM/guide   (길안내)                                 │
    │    • POST /LLM/order   (음료 주문 - Kreacher용)                │
    └─────────────────────────────────────────────────────────────────┘
```

### 1.2 DMC가 관리하는 것 vs 관리하지 않는 것

| 구분 | DMC 관리 ✅ | 다른 컴포넌트 관리 ❌ |
|------|------------|---------------------|
| **상태** | Main State (IDLE, EXECUTING_TASK, CHARGING, LISTENING 등) | Sub State는 SMACH Sub-SM에서 관리 |
| **작업 큐** | 현재 작업 1개만 (`current_task`) | RCS가 전체 큐 관리 |
| **작업 흐름** | 작업 시작/종료 판단, State 전환 | 각 하위 컨트롤러가 세부 실행 |
| **타임아웃** | 전체 작업 타임아웃 (SMACH Timeout) | Action 레벨 타임아웃은 각 컨트롤러 |
| **배터리** | 전체 로봇 배터리 상태 관리 및 긴급 처리 | - |
| **음성 인식** | Wake word 감지, LLM 호출, 의도→작업 변환 | STT/TTS는 외부 서비스 |
| **경로 계획** | - | DDC가 Nav2로 처리 |
| **암 제어** | - | DAC가 MoveIt으로 처리 |
| **도서 정보** | - | Application Service (DB) |
| **보관함 잠금** | - | Authentication Controller (Serial) |

### 1.3 도비 로봇 하드웨어 구성 (DMC 관점)
```
┌─────────────────────────────────────────────────────────┐
│                    Dobby Robot                          │
│                                                         │
│  ┌──────────────────────────────────────────────────┐  │
│  │  Dobby GUI (Touch Display)                       │  │
│  │  - 목적지 선택 화면                               │  │
│  │  - 사용자 인식 안내 화면                           │  │
│  │  - 안내 진행 화면                                 │  │
│  └──────────────────────────────────────────────────┘  │
│                                                         │
│  ┌──────────────────────────────────────────────────┐  │
│  │  6축 로봇 팔 (Manipulator)                        │  │
│  │  - MoveIt 기반 제어 (DAC)                         │  │
│  │  - 그리퍼 (Gripper)                               │  │
│  │  - 카메라 (Eye-in-Hand)                           │  │
│  └──────────────────────────────────────────────────┘  │
│                          ↕                              │
│  ┌──────────────────────────────────────────────────┐  │
│  │  내부 운반함 (Internal Carrier)                   │  │
│  │  - 여러 슬롯 (carrier_slot_id)                    │  │
│  │  - 도서 일시 보관 (픽업 중, 반납 중)               │  │
│  │  - 슬롯별 상태 관리 (empty/occupied)              │  │
│  └──────────────────────────────────────────────────┘  │
│                                                         │
│  ┌──────────────────────────────────────────────────┐  │
│  │  자율 주행 플랫폼 (AMR)                            │  │
│  │  - Nav2 기반 (DDC)                                │  │
│  │  - LiDAR, Depth Camera                            │  │
│  │  - 배터리 (가상, Test GUI로 제어)                  │  │
│  └──────────────────────────────────────────────────┘  │
│                                                         │
│  ┌──────────────────────────────────────────────────┐  │
│  │  스피커 & 마이크                                   │  │
│  │  - TTS 출력                                       │  │
│  │  - STT 입력                                       │  │
│  └──────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────┘
```

---

## 2. 컴포넌트 아키텍처

### 2.1 DMC 내부 구조 (TaskManager 제거)
```
┌────────────────────────────────────────────────────────────────┐
│              DobbyMainControllerNode (rclpy.Node)              │
│                                                                │
│  [멤버 변수]                                                    │
│  - namespace: str (dobby1, dobby2, ...)                       │
│  - current_task: Optional[Task]  ← 현재 작업 1개만 저장        │
│  - is_emergency_stop: bool                                    │
│                                                                │
├────────────────────────────────────────────────────────────────┤
│                                                                │
│  ┌──────────────────────────────────────────────────────┐     │
│  │ StateMachineManager                                  │     │
│  │                                                      │     │
│  │  - main_sm: MainStateMachine (SMACH)                │     │
│  │    • IDLE, CHARGING, EXECUTING_TASK, LISTENING...   │     │
│  │                                                      │     │
│  │  - task_sms: Dict[TaskType, SMACH.StateMachine]     │     │
│  │    • PICKUP_BOOK_SM                                 │     │
│  │    • RESHELVING_BOOK_SM                             │     │
│  │    • GUIDING_SM                                     │     │
│  │    • CLEANING_DESK_SM                               │     │
│  │    • SORTING_SHELVES_SM                             │     │
│  │                                                      │     │
│  │  [메서드]                                             │     │
│  │  + initialize_state_machines()                      │     │
│  │  + execute_main_sm()                                │     │
│  │  + get_current_state() → (main, sub)               │     │
│  │  + trigger(event_name)                              │     │
│  │  + preempt_current_task()                           │     │
│  └──────────────────────────────────────────────────────┘     │
│                          ↕                                     │
│  ┌──────────────────────────────────────────────────────┐     │
│  │ InterfaceManager                                     │     │
│  │                                                      │     │
│  │  - drive: DriveInterface (DDC)                       │     │
│  │  - arm: ArmInterface (DAC)                           │     │
│  │  - ai: AIInterface (AIS)                             │     │
│  │  - gui: GUIInterface (Dobby GUI)                     │     │
│  │  - llm: LLMInterface (HTTP Client)                   │     │
│  │  - tts: TTSInterface (ROS2 Service)                  │     │
│  │  - stt: STTInterface (ROS2 Topic Subscriber)         │     │
│  │                                                      │     │
│  │  [메서드]                                             │     │
│  │  + initialize_all() → bool                          │     │
│  │  + shutdown_all()                                   │     │
│  │  + get_interface(name: str) → BaseInterface         │     │
│  └──────────────────────────────────────────────────────┘     │
│                          ↕                                     │
│  ┌──────────────────────────────────────────────────────┐     │
│  │ BatteryManager                                       │     │
│  │                                                      │     │
│  │  - level: float (0.0 ~ 100.0)                       │     │
│  │  - state: BatteryState (IDLE/CHARGING/DRAINING)     │     │
│  │  - config: BatteryConfig                            │     │
│  │    • charge_rate: 10.0 (%/min)                      │     │
│  │    • work_rate: -1.0 (%/min)                        │     │
│  │    • critical_threshold: 20.0 (%)                   │     │
│  │    • charge_target: 80.0 (%)                        │     │
│  │                                                      │     │
│  │  [메서드]                                             │     │
│  │  + update(dt: float)                                │     │
│  │  + start_charging()                                 │     │
│  │  + start_draining()                                 │     │
│  │  + set_idle()                                       │     │
│  │  + is_critical() → bool                             │     │
│  │  + is_sufficient() → bool                           │     │
│  │  + force_set(level: float)  # Test GUI용            │     │
│  └──────────────────────────────────────────────────────┘     │
│                          ↕                                     │
│  ┌──────────────────────────────────────────────────────┐     │
│  │ StatePublisher                                       │     │
│  │                                                      │     │
│  │  - robot_state_pub: Publisher<DobbyState>            │     │
│  │  - battery_status_pub: Publisher<BatteryStatus>      │     │
│  │  - current_pose_sub: Subscription<Pose>              │     │
│  │                                                      │     │
│  │  [메서드]                                             │     │
│  │  + publish_state(main, sub, error)                  │     │
│  │  + publish_battery(level, is_charging)              │     │
│  │  + get_current_pose() → Pose                        │     │
│  └──────────────────────────────────────────────────────┘     │
│                          ↕                                     │
│  ┌──────────────────────────────────────────────────────┐     │
│  │ VoiceCommandHandler (NEW)                            │     │
│  │                                                      │     │
│  │  - wake_word_filter: WakeWordFilter                 │     │
│  │  - session_manager: VoiceSessionManager             │     │
│  │                                                      │     │
│  │  [메서드]                                             │     │
│  │  + on_stt_result(text: str)                         │     │
│  │  + is_wake_word(text: str) → bool                   │     │
│  │  + parse_intent(text: str) → Intent                 │     │
│  │  + start_listening_session()                        │     │
│  │  + end_listening_session()                          │     │
│  └──────────────────────────────────────────────────────┘     │
└────────────────────────────────────────────────────────────────┘
```

### 2.2 클래스 다이어그램
```
┌──────────────────────────────────────────┐
│   DobbyMainControllerNode                │
│   (rclpy.Node)                           │
├──────────────────────────────────────────┤
│ - namespace: str                         │
│ - current_task: Optional[Task]           │
│ - is_emergency_stop: bool                │
│                                          │
│ - sm_manager: StateMachineManager        │
│ - interface_manager: InterfaceManager    │
│ - battery_manager: BatteryManager        │
│ - state_publisher: StatePublisher        │
│ - voice_handler: VoiceCommandHandler     │
├──────────────────────────────────────────┤
│ + __init__(namespace: str)               │
│ + initialize() → bool                    │
│ + shutdown()                             │
│ + spin()                                 │
│                                          │
│ # RCS 작업 할당                           │
│ + handle_task_assignment(request)        │
│ + can_accept_task() → bool               │
│ + start_task_execution(task)             │
│                                          │
│ # 배터리 이벤트                           │
│ + on_battery_update(level: float)        │
│ + on_battery_critical()                  │
│                                          │
│ # 음성 이벤트                             │
│ + on_wake_word_detected()                │
│ + on_voice_command(intent: Intent)       │
└────────────┬─────────────────────────────┘
             │
             │ composition
             ├───────────────────────────────────────┐
             │                                       │
┌────────────▼──────────────┐        ┌──────────────▼──────────┐
│ StateMachineManager       │        │ InterfaceManager        │
├───────────────────────────┤        ├─────────────────────────┤
│ - main_sm: SMACH.SM       │        │ - drive: DriveIF        │
│ - task_sms: Dict          │        │ - arm: ArmIF            │
│                           │        │ - ai: AIIF              │
├───────────────────────────┤        │ - gui: GUIIF            │
│ + initialize_sms()        │        │ - llm: LLMIF            │
│ + execute_main_sm()       │        │ - tts: TTSIF            │
│ + get_current_state()     │        │ - stt: STTIF            │
│ + trigger(event)          │        ├─────────────────────────┤
│ + preempt()               │        │ + initialize_all()      │
└───────────────────────────┘        │ + shutdown_all()        │
                                     │ + get_interface(name)   │
┌───────────────────────────┐        └─────────────────────────┘
│ BatteryManager            │
├───────────────────────────┤        ┌─────────────────────────┐
│ - level: float            │        │ StatePublisher          │
│ - state: BatteryState     │        ├─────────────────────────┤
│ - config: BatteryConfig   │        │ - robot_state_pub       │
├───────────────────────────┤        │ - battery_status_pub    │
│ + update(dt)              │        │ - current_pose_sub      │
│ + start_charging()        │        ├─────────────────────────┤
│ + start_draining()        │        │ + publish_state()       │
│ + is_critical() → bool    │        │ + publish_battery()     │
│ + force_set(level)        │        │ + get_current_pose()    │
└───────────────────────────┘        └─────────────────────────┘

┌───────────────────────────┐
│ VoiceCommandHandler       │
├───────────────────────────┤
│ - wake_word_filter        │
│ - session_manager         │
├───────────────────────────┤
│ + on_stt_result(text)     │
│ + is_wake_word() → bool   │
│ + parse_intent() → Intent │
│ + start_listening()       │
│ + end_listening()         │
└───────────────────────────┘
```

### 2.3 인터페이스 계층 구조
```
┌─────────────────────────────────────────────────────┐
│              BaseInterface (ABC)                    │
│  ─────────────────────────────────────────────      │
│  - node: Node                                       │
│  - namespace: str                                   │
│  - logger: Logger                                   │
│  ─────────────────────────────────────────────      │
│  + initialize() → bool                              │
│  + shutdown()                                       │
│  + _create_topic_name(topic: str) → str             │
└───────────┬─────────────────────────────────────────┘
            │
            ├─── DriveInterface
            │    • move_to_target(pose) → Future
            │    • enable_follow_mode(tracking_topic)
            │    • disable_follow_mode()
            │    • stop()
            │    • get_current_pose() → Pose
            │
            ├─── ArmInterface
            │    • pick_book(book_id, pose, slot) → Future
            │    • place_book(book_id, slot, storage) → Future
            │    • collect_books(book_list) → Future
            │    • collect_trash(trash_pose) → Future
            │    • dispose_trash(bin_pose) → Future
            │    • change_pose(pose_type) → bool
            │
            ├─── AIInterface
            │    • detect_book(book_id) → BookPose
            │    • check_storage_box(box_id) → BoxStatus
            │    • verify_book_position(book_id) → bool
            │    • identify_bookshelf() → ShelfInfo
            │    • detect_trash(seat_pose) → List[Trash]
            │    • register_person() → tracking_id
            │    • change_tracking_mode(mode)
            │    • subscribe_tracking_status(callback)
            │
            ├─── GUIInterface
            │    • update_screen(screen_type, data)
            │    • subscribe_screen_event(callback)
            │
            ├─── LLMInterface (NEW)
            │    • parse_book_query(text) → Intent
            │    • parse_guide_query(text) → Intent
            │    • parse_order_query(text) → Intent
            │    • set_session_id(session_id)
            │
            ├─── TTSInterface (NEW)
            │    • speak(text, wait=True)
            │    • stop_speaking()
            │
            └─── STTInterface (NEW)
                 • subscribe_stt_result(callback)
                 • get_latest_result() → str
```

---

## 3. SMACH State Machine 설계

### 3.1 Main State Machine

#### 3.1.1 상태 다이어그램
```
stateDiagram-v2
    [*] --> INITIALIZING: 전원 ON
    
    INITIALIZING --> CHARGING: init_complete
    
    CHARGING --> IDLE: battery >= 40%
    
    IDLE --> LISTENING: wake_word_detected
    IDLE --> EXECUTING_TASK: task_assigned (battery >= 20%)
    IDLE --> FORCE_MOVE_TO_CHARGER: battery < 20%
    
    LISTENING --> IDLE: timeout_20s
    LISTENING --> IDLE: invalid_command
    LISTENING --> EXECUTING_TASK: valid_task_command
    
    EXECUTING_TASK --> IDLE: task_complete (battery >= 40%)
    EXECUTING_TASK --> MOVING_TO_CHARGER: task_complete (battery < 40%)
    EXECUTING_TASK --> IDLE: task_failed
    EXECUTING_TASK --> FORCE_MOVE_TO_CHARGER: battery < 20%
    
    MOVING_TO_CHARGER --> CHARGING: arrived_at_charger
    MOVING_TO_CHARGER --> FORCE_MOVE_TO_CHARGER: battery < 20%
    
    FORCE_MOVE_TO_CHARGER --> CHARGING: arrived_at_charger
    
    IDLE --> ERROR: critical_error
    EXECUTING_TASK --> ERROR: critical_error
    LISTENING --> ERROR: critical_error
    
    ERROR --> IDLE: error_resolved
    
    note right of LISTENING
        음성 명령 대기
        - "도비야" 감지 시 진입
        - TTS: "네, 무엇을 도와드릴까요?"
        - 20초 타임아웃
        - LLM으로 의도 파싱
    end note
    
    note right of EXECUTING_TASK
        작업 실행 중
        - Task SM 실행
        - 배터리 소모 (-1%/min)
        - 진행률 발행
    end note
    
    note right of FORCE_MOVE_TO_CHARGER
        긴급 충전 복귀
        - 모든 작업 중단
        - RCS에 실패 보고
        - 충전소 강제 이동
    end note
```

#### 3.1.2 Main State 정의표

| State | 코드값 | 진입 조건 | 퇴장 조건 | on_enter 동작 | on_exit 동작 |
|-------|--------|----------|----------|--------------|-------------|
| **INITIALIZING** | 0 | 시스템 부팅 | 2초 경과 | 하드웨어 체크, 파라미터 로드, 인터페이스 초기화 | 초기화 완료 로그 |
| **CHARGING** | 1 | 충전소 도착 | 배터리 >= 40% | 배터리 충전 시작 (+10%/min), GUI 업데이트 | 충전 완료 알림 |
| **IDLE** | 2 | 충전/작업 완료 | 작업 할당 또는 음성 호출 | 대기 상태 발행, current_task=None | - |
| **LISTENING** | 10 | "도비야" 감지 | 타임아웃 또는 명령 인식 | TTS "네, 무엇을 도와드릴까요?", 타이머 20초 시작 | 세션 종료, 타이머 취소 |
| **EXECUTING_TASK** | 3 | 작업 할당/음성 명령 | 작업 완료/실패 | Task SM 시작, 배터리 소모 시작, RCS에 시작 알림 | Task SM 정리 |
| **MOVING_TO_CHARGER** | 9 | 작업 후 배터리 < 40% | 충전소 도착 | DDC에 충전소 이동 명령 | - |
| **FORCE_MOVE_TO_CHARGER** | 4 | 배터리 < 20% (긴급) | 충전소 도착 | 모든 Action Cancel, RCS에 실패 보고, 긴급 이동 | - |
| **ERROR** | 99 | 치명적 에러 | 에러 해결 | 모든 동작 정지, 관리자 알림, 에러 상태 발행 | - |

#### 3.1.3 Transition 조건표

| Trigger | Source | Destination | Condition | Before Callback | After Callback |
|---------|--------|-------------|-----------|-----------------|----------------|
| `init_complete` | INITIALIZING | CHARGING | - | - | `log_init_done` |
| `charge_complete` | CHARGING | IDLE | `battery >= 40` | - | `publish_ready_state` |
| `wake_word_detected` | IDLE | LISTENING | - | `start_voice_session` | - |
| `task_assigned` | IDLE | EXECUTING_TASK | `battery >= 20 AND no_current_task` | `prepare_task` | `start_task_sm` |
| `timeout_20s` | LISTENING | IDLE | - | `speak_timeout_msg` | `end_voice_session` |
| `invalid_command` | LISTENING | IDLE | - | `speak_not_understood` | `end_voice_session` |
| `valid_task_command` | LISTENING | EXECUTING_TASK | `battery >= 20` | `prepare_voice_task` | `start_task_sm` |
| `task_complete` | EXECUTING_TASK | IDLE | `battery >= 40` | - | `report_success_to_rcs` |
| `task_complete` | EXECUTING_TASK | MOVING_TO_CHARGER | `battery < 40` | - | `report_success_and_move` |
| `task_failed` | EXECUTING_TASK | IDLE | - | `cleanup_task` | `report_failure_to_rcs` |
| `battery_critical` | IDLE, EXECUTING_TASK, MOVING_TO_CHARGER, LISTENING | FORCE_MOVE_TO_CHARGER | `battery < 20` | `emergency_abort_all` | `emergency_move_to_charger` |
| `arrived_at_charger` | MOVING_TO_CHARGER, FORCE_MOVE_TO_CHARGER | CHARGING | - | - | - |
| `critical_error` | * (any) | ERROR | - | `stop_all_actions` | `notify_admin` |
| `error_resolved` | ERROR | IDLE | - | - | - |

### 3.2 Task State Machines (Sub-State Machines)

#### 3.2.1 도서 픽업 (PickupBookSM)

**시나리오 정정 (문서 기반):**
```
1. 책장으로 이동
2. 도서 감지 (AIS)
3. 도서 픽업 → 내부 운반함(carrier_slot)에 적재 (DAC)
4. 픽업 보관함(Storage Box) 위치로 이동
5. 내부 운반함에서 픽업 → 픽업 보관함에 배치 (DAC)
6. 픽업 보관함 정위치 확인 (AIS)
```
```
stateDiagram-v2
    [*] --> MOVE_TO_PICKUP: 작업 시작
    
    MOVE_TO_PICKUP --> DETECT_BOOK: arrived_at_shelf
    MOVE_TO_PICKUP --> [*]: move_failed
    
    DETECT_BOOK --> PICKUP_TO_CARRIER: book_detected
    DETECT_BOOK --> [*]: detection_failed
    
    PICKUP_TO_CARRIER --> MOVE_TO_STORAGE: book_in_carrier
    PICKUP_TO_CARRIER --> [*]: pickup_failed
    
    MOVE_TO_STORAGE --> PLACE_TO_STORAGE_BOX: arrived_at_storage
    MOVE_TO_STORAGE --> [*]: move_failed
    
    PLACE_TO_STORAGE_BOX --> VERIFY_PLACEMENT: book_placed
    PLACE_TO_STORAGE_BOX --> [*]: place_failed
    
    VERIFY_PLACEMENT --> [*]: verified_success
    VERIFY_PLACEMENT --> [*]: verify_failed
```

**Sub-State 정의:**

| Sub-State | 코드값 | 동작 | Interface 사용 | Timeout |
|-----------|--------|------|----------------|---------|
| **MOVE_TO_PICKUP** | 101 | DDC에 책장 위치로 이동 명령 | drive.move_to_target(shelf_pose) | 60초 |
| **DETECT_BOOK** | 102 | AIS에 도서 감지 요청, DAC 관측 자세 | ai.detect_book(book_id), arm.change_pose(OBSERVATION) | 30초 |
| **PICKUP_TO_CARRIER** | 103 | 도서 픽업 → 내부 운반함에 적재 | arm.pick_book(book_id, pose, carrier_slot) | 30초 |
| **MOVE_TO_STORAGE** | 104 | 픽업 보관함 위치로 이동 | drive.move_to_target(storage_pose) | 60초 |
| **PLACE_TO_STORAGE_BOX** | 105 | 운반함에서 픽업 → 픽업 보관함에 배치 | arm.place_book(book_id, carrier_slot, storage_id) | 30초 |
| **VERIFY_PLACEMENT** | 106 | 픽업 보관함 정위치 확인 | ai.verify_book_position(book_id) | 10초 |

**전체 타임아웃:** 240초 (4분)

#### 3.2.2 반납 도서 정리 (ReshelvingBookSM)

**시나리오 정정 (문서 기반):**
```
1. 반납대로 이동 (스케줄링: 1시간마다)
2. 반납대 도서 스캔 (AIS)
3. 여러 권의 도서를 내부 운반함에 수거 (DAC)
4. RCS에 도서 정보 요청 (책장 위치 확인)
5. 책장 A로 이동
6. 책장 A 도서들을 운반함에서 픽업 → 책장에 배치
7. 책장 B로 이동 (남은 도서가 있으면)
8. 반복...
```
```
stateDiagram-v2
    [*] --> MOVE_TO_RETURN_DESK
    
    MOVE_TO_RETURN_DESK --> SCAN_RETURNED_BOOKS: arrived
    MOVE_TO_RETURN_DESK --> [*]: move_failed
    
    SCAN_RETURNED_BOOKS --> COLLECT_TO_CARRIER: books_found
    SCAN_RETURNED_BOOKS --> [*]: no_books_or_scan_failed
    
    COLLECT_TO_CARRIER --> REQUEST_BOOK_INFO: all_collected
    COLLECT_TO_CARRIER --> [*]: collect_failed
    
    REQUEST_BOOK_INFO --> MOVE_TO_SHELF: info_received
    REQUEST_BOOK_INFO --> [*]: info_request_failed
    
    MOVE_TO_SHELF --> PLACE_BOOKS_TO_SHELF: arrived_at_shelf
    
    PLACE_BOOKS_TO_SHELF --> VERIFY_SHELF_PLACEMENT: books_placed
    PLACE_BOOKS_TO_SHELF --> [*]: place_failed
    
    VERIFY_SHELF_PLACEMENT --> MOVE_TO_SHELF: more_shelves (다른 책장 도서)
    VERIFY_SHELF_PLACEMENT --> [*]: all_books_placed
```

**Sub-State 정의:**

| Sub-State | 코드값 | 동작 | Interface 사용 | Timeout |
|-----------|--------|------|----------------|---------|
| **MOVE_TO_RETURN_DESK** | 107 | 반납대로 이동 | drive.move_to_target(return_desk_pose) | 60초 |
| **SCAN_RETURNED_BOOKS** | 108 | 반납대 도서 스캔 | ai.detect_books_on_desk(), arm.change_pose(OBSERVATION) | 60초 |
| **COLLECT_TO_CARRIER** | 109 | 여러 권 수거 → 내부 운반함 | arm.collect_books(book_list, carrier_slots) | 60초 |
| **REQUEST_BOOK_INFO** | 110 | RCS에 도서 정보 요청 (책장 위치) | rcs_client.request_book_info(book_ids) | 30초 |
| **MOVE_TO_SHELF** | 111 | 책장 위치로 이동 | drive.move_to_target(shelf_pose) | 60초 |
| **PLACE_BOOKS_TO_SHELF** | 112 | 해당 책장 도서들을 운반함→책장 | arm.place_books_to_shelf(book_list, shelf_id) (loop) | 30초/권 |
| **VERIFY_SHELF_PLACEMENT** | 113 | 배치된 도서 정위치 확인 | ai.verify_books_on_shelf(book_ids) | 20초 |

**전체 타임아웃:** 600초 (10분, 책 수에 따라 가변)

#### 3.2.3 길안내 (GuidingSM)
```
stateDiagram-v2
    [*] --> SELECT_DEST
    
    SELECT_DEST --> SCAN_USER: dest_selected
    SELECT_DEST --> [*]: timeout_60s
    
    SCAN_USER --> REGISTER_USER: user_in_camera
    SCAN_USER --> [*]: scan_timeout_30s
    
    REGISTER_USER --> GUIDING_TO_DEST: user_registered
    REGISTER_USER --> [*]: register_failed
    
    GUIDING_TO_DEST --> [*]: arrived_at_dest
    GUIDING_TO_DEST --> FIND_USER: user_lost_10s
    GUIDING_TO_DEST --> [*]: guidance_timeout
    
    FIND_USER --> GUIDING_TO_DEST: user_found
    FIND_USER --> [*]: search_timeout_30s
    
    note right of GUIDING_TO_DEST
        DDC: follow_mode 활성화
        AIS: tracking_status 실시간 구독
        - person_detected = true: 계속 안내
        - is_lost = true (10초): FIND_USER
        - 3회 실패: 안내 종료
    end note
```

**Sub-State 정의:**

| Sub-State | 코드값 | 동작 | Interface 사용 | Timeout |
|-----------|--------|------|----------------|---------|
| **SELECT_DEST** | 114 | GUI에서 목적지 선택 대기 | gui.update_screen(DEST_SELECT), gui.wait_event() | 60초 |
| **SCAN_USER** | 115 | 사용자 카메라 앞 서기 대기 | gui.update_screen(USER_SCAN) | 30초 |
| **REGISTER_USER** | 116 | AIS에 사용자 등록 | ai.register_person() | 10초 |
| **GUIDING_TO_DEST** | 117 | 목적지까지 추종 안내 | drive.enable_follow_mode(), ai.subscribe_tracking_status() | 무제한 (거리 기반) |
| **FIND_USER** | 118 | 사용자 재탐색 (360도 회전) | drive.rotate_in_place(360), ai.try_reregister_user() | 30초 |

**전체 타임아웃:** 600초 (10분)

**특이사항:**
- `GUIDING_TO_DEST`에서 `user_lost` 카운터: 3회 누적 시 작업 실패
- DDC의 `follow_mode`는 AIS의 `/tracking/status` Topic을 실시간 구독

#### 3.2.4 좌석 정리 (CleaningDeskSM)
```
stateDiagram-v2
    [*] --> MOVE_TO_DESK
    
    MOVE_TO_DESK --> SCAN_DESK: arrived_at_desk
    MOVE_TO_DESK --> [*]: move_failed
    
    SCAN_DESK --> COLLECT_TRASH: trash_detected
    SCAN_DESK --> [*]: desk_is_clean
    
    COLLECT_TRASH --> MOVE_TO_BIN: all_trash_collected
    COLLECT_TRASH --> [*]: collect_failed
    
    MOVE_TO_BIN --> DUMP_TRASH: arrived_at_bin
    MOVE_TO_BIN --> [*]: move_failed
    
    DUMP_TRASH --> [*]: disposal_complete
```

**Sub-State 정의:**

| Sub-State | 코드값 | 동작 | Interface 사용 | Timeout |
|-----------|--------|------|----------------|---------|
| **MOVE_TO_DESK** | 119 | 좌석 위치로 이동 | drive.move_to_target(desk_pose) | 60초 |
| **SCAN_DESK** | 120 | 좌석 쓰레기 탐지 | ai.detect_trash(desk_pose), arm.change_pose(OBSERVATION) | 30초 |
| **COLLECT_TRASH** | 121 | 쓰레기 수거 (반복) | arm.collect_trash(trash_pose) (loop) | 30초/개 |
| **MOVE_TO_BIN** | 122 | 쓰레기통으로 이동 | drive.move_to_target(bin_pose) | 60초 |
| **DUMP_TRASH** | 123 | 쓰레기 배출 | arm.dispose_trash(bin_pose) | 30초 |

**전체 타임아웃:** 300초 (5분)

#### 3.2.5 서가 정리 (SortingShelvesSM)
```
stateDiagram-v2
    [*] --> MOVE_TO_SHELF
    
    MOVE_TO_SHELF --> SCAN_SHELF: arrived_at_shelf
    MOVE_TO_SHELF --> [*]: move_failed
    
    SCAN_SHELF --> PICKUP_MISPLACED: misplaced_found
    SCAN_SHELF --> MOVE_TO_SHELF: move_to_next_shelf
    SCAN_SHELF --> [*]: all_shelves_done
    
    PICKUP_MISPLACED --> MOVE_TO_CORRECT_SHELF: book_picked_to_carrier
    PICKUP_MISPLACED --> [*]: pickup_failed
    
    MOVE_TO_CORRECT_SHELF --> PLACE_BOOK_TO_SHELF: arrived
    MOVE_TO_CORRECT_SHELF --> [*]: move_failed
    
    PLACE_BOOK_TO_SHELF --> VERIFY_PLACEMENT: placed
    PLACE_BOOK_TO_SHELF --> [*]: place_failed
    
    VERIFY_PLACEMENT --> SCAN_SHELF: verified (다음 도서 확인)
    VERIFY_PLACEMENT --> [*]: verify_failed
Sub-State 정의:
Sub-State코드값동작Interface 사용TimeoutMOVE_TO_SHELF124서가 위치로 이동drive.move_to_target(shelf_pose)60초SCAN_SHELF125책장 스캔, 잘못 꽂힌 도서 감지ai.scan_bookshelf_for_errors(shelf_id)60초PICKUP_MISPLACED126잘못된 도서 픽업 → 운반함arm.pick_book(book_id, pose, carrier_slot)30초MOVE_TO_CORRECT_SHELF127올바른 책장으로 이동drive.move_to_target(correct_shelf_pose)60초PLACE_BOOK_TO_SHELF128운반함 → 올바른 책장에 배치arm.place_book_to_shelf(book_id, carrier_slot, shelf_id)30초VERIFY_PLACEMENT129배치 정위치 확인ai.verify_book_position(book_id)10초
전체 타임아웃: 600초 (10분, 책장 수에 따라 가변)

4. 인터페이스 명세
4.1 RCS ↔ DMC
4.1.1 Action: dobby1/assign_task
목적: RCS가 DMC에게 작업 할당
Goal (RCS → DMC):
yaml# AssignTask.action Goal
uint8 task_type
  uint8 PICKUP_BOOK = 1
  uint8 RESHELVING_BOOK = 2
  uint8 GUIDING = 3
  uint8 CLEANING_DESK = 4
  uint8 SORTING_SHELVES = 5

int32 task_id
string task_data_json  # JSON 직렬화된 작업 데이터

# 예시 (도서 픽업):
# {
#   "book_id": "ISBN-12345",
#   "shelf_pose": {"x": 10.5, "y": 3.2, "theta": 0.0},
#   "carrier_slot_id": 1,
#   "storage_id": 3,
#   "storage_pose": {"x": 5.0, "y": 8.0, "theta": 1.57}
# }
Result (DMC → RCS):
yaml# AssignTask.action Result
bool accepted
string reject_reason  # "BATTERY_LOW", "ALREADY_BUSY", "INVALID_TASK", "ERROR_STATE"

# 작업 수락 시
bool success
string result_message  # "OK", "PARTIAL_SUCCESS", "FAILED"
float32 total_time_sec
float32 total_distance_m
string error_details  # 실패 시 상세 정보
Feedback (DMC → RCS, 1Hz):
yaml# AssignTask.action Feedback
uint8 main_state
uint8 sub_state
float32 progress_percent  # 0-100
float32 battery_level     # 0-100
string status_message
builtin_interfaces/Time timestamp
4.1.2 Service: dobby1/request_book_info (반납 정리 시)
Request (DMC → RCS):
yamlstring[] book_ids  # 반납된 도서 ID 목록
Response (RCS → DMC):
yaml# BookInfo.msg
struct BookInfo:
  string book_id
  string shelf_id
  geometry_msgs/Pose shelf_pose
  int32 priority  # 배치 우선순위

BookInfo[] book_infos
bool success
4.2 DMC ↔ DDC (Drive Control)
4.2.1 Action: dobby1/drive/move_to_target
Goal:
yamlgeometry_msgs/Pose target_pose
string location_name  # "충전소", "반납대", "책장_A1" 등
Result:
yamlbool success
geometry_msgs/Pose final_pose
float32 distance_traveled  # meters
string error_code  # "OBSTACLE_BLOCKED", "PATH_NOT_FOUND", "TIMEOUT"
Feedback:
yamlgeometry_msgs/Pose current_pose
float32 distance_remaining  # meters
float32 estimated_time  # seconds
4.2.2 Service: dobby1/drive/enable_follow_mode
Request:
yamlstring tracking_topic  # "/dobby1/ai/tracking/status"
geometry_msgs/Pose destination  # 최종 목적지
float32 follow_distance  # 사용자와의 거리 (default: 1.5m)
Response:
yamlbool success
string message
4.2.3 Service: dobby1/drive/disable_follow_mode
Request:
yaml# Empty
Response:
yamlbool success
string message
4.2.4 Service: dobby1/drive/rotate_in_place
Request:
yamlfloat32 angle  # degrees (360 = 전체 회전)
Response:
yamlbool success
4.2.5 Topic: dobby1/status/current_pose (DDC → DMC, All)
Message:
yamlgeometry_msgs/Pose pose
builtin_interfaces/Time timestamp
Publish Rate: 10Hz
4.3 DMC ↔ DAC (Arm Control)
4.3.1 Action: dobby1/arm/pick_book
Goal:
yamlstring book_id
geometry_msgs/Pose book_pose
int32 carrier_slot_id  # 내부 운반함 슬롯 번호
Result:
yamlbool success
string book_id
string message  # "OK", "BOOK_NOT_FOUND", "GRIPPER_ERROR", "CARRIER_FULL"
Feedback:
yamlstring status  # "moving_to_book", "grasping", "moving_to_carrier"
string current_action
4.3.2 Action: dobby1/arm/place_book
Goal:
yamlstring book_id
int32 carrier_slot_id  # 내부 운반함 슬롯 번호
geometry_msgs/Pose target_pose  # 픽업 보관함 또는 책장 위치
int32 target_id  # storage_box_id 또는 shelf_id
uint8 target_type  # 0=STORAGE_BOX, 1=BOOKSHELF
Result:
yamlbool success
string book_id
string message
4.3.3 Action: dobby1/arm/collect_books
Goal:
yamlstring[] book_ids
geometry_msgs/Pose[] book_poses
int32[] carrier_slot_ids
Result:
yamlbool success
int32 books_collected
string[] collected_book_ids
string message
Feedback:
yamlstring current_book_id
int32 books_collected_count
string status
4.3.4 Action: dobby1/arm/collect_trash
Goal:
yamlstring trash_type  # "cup", "can", "paper" 등
geometry_msgs/Pose trash_pose
Result:
yamlbool success
string message
4.3.5 Action: dobby1/arm/dispose_trash
Goal:
yamlgeometry_msgs/Pose trash_bin_pose
Result:
yamlbool success
string message
4.3.6 Service: dobby1/arm/change_pose
Request:
yamluint8 pose_type
  uint8 OBSERVATION_POSE = 0
  uint8 INITIAL_POSE = 1
  uint8 CUSTOM_POSE = 2

geometry_msgs/Pose target_location  # CUSTOM_POSE일 때만
Response:
yamlbool success
string message
4.4 DMC ↔ AIS (Vision)
4.4.1 Service: dobby1/ai/detect_book
Request:
yamlstring book_id
Response:
yamlbool detected
string book_id
geometry_msgs/Pose book_pose
float32 confidence  # 0.0 ~ 1.0
4.4.2 Service: dobby1/ai/check_storage_box
Request:
yamlint32 storage_box_id
Response:
yamlbool is_valid
int32 storage_box_id
geometry_msgs/Pose storage_box_pose
bool is_empty
string status_message
4.4.3 Service: dobby1/ai/verify_book_position
Request:
yamlstring book_id
Response:
yamlbool is_correct_position
string book_id
string error_type  # "tilted", "upside_down", "out_of_slot", ""
string message
4.4.4 Service: dobby1/ai/identify_bookshelf
Request:
yaml# Empty (현재 위치 기준)
Response:
yamlbool detected
string bookshelf_id
geometry_msgs/Pose bookshelf_pose
geometry_msgs/Pose[] available_slots  # 빈 슬롯 위치 목록
4.4.5 Service: dobby1/ai/detect_trash
Request:
yamlsensor_msgs/Image camera_image
geometry_msgs/Pose seat_location
Response:
yamlbool trash_found
int32 trash_count
string[] trash_types
geometry_msgs/Pose[] trash_poses
float32[] confidence_scores
4.4.6 Service: dobby1/ai/register_person
Request:
yamlsensor_msgs/Image camera_image  # Optional, auto-capture if empty
float32 registration_timeout  # seconds
Response:
yamlbool success
string tracking_id
string message
4.4.7 Service: dobby1/ai/change_tracking_mode
Request:
yamluint8 mode
  uint8 REGISTRATION_MODE = 0
  uint8 TRACKING_MODE = 1
  uint8 IDLE_MODE = 2

string tracking_id  # TRACKING_MODE일 때 필요
Response:
yamlbool success
string message
4.4.8 Topic: dobby1/ai/tracking/status (AIS → DMC, DDC)
Message:
yamlstd_msgs/Header header
bool person_detected
string tracking_id
geometry_msgs/Pose person_pose
float32 distance_to_person  # meters
float32 confidence  # 0.0 ~ 1.0
bool is_lost
float32 time_since_last_seen  # seconds
Publish Rate: 10Hz (사용자 추적 중)
4.4.9 Topic: dobby1/ai/obstacles (AIS → DDC)
Message:
yamlbool dynamic  # true: 동적 장애물, false: 정적
float32 x  # 화면 상 정규화 좌표 (0~1)
float32 y  # 화면 상 정규화 좌표 (0~1)
float32 depth  # meters
Publish Rate: 10Hz
4.5 DMC ↔ Dobby GUI
4.5.1 Service: dobby1/gui/update_screen
Request:
yamluint8 screen_type
  uint8 IDLE_SCREEN = 0
  uint8 GUIDING_DEST_SELECT = 1
  uint8 GUIDING_USER_SCAN = 2
  uint8 GUIDING_IN_PROGRESS = 3
  uint8 TASK_IN_PROGRESS = 4
  uint8 CHARGING_SCREEN = 5
  uint8 ERROR_SCREEN = 6

string screen_data_json  # 화면별 추가 데이터
Response:
yamlbool success
string message
4.5.2 Topic: dobby1/gui/screen_event (GUI → DMC)
Message:
yamluint8 event_type
  uint8 DEST_SELECTED = 0
  uint8 BUTTON_PRESSED = 1
  uint8 TIMEOUT = 2

string event_data_json
builtin_interfaces/Time timestamp
Publish Rate: Event-driven
4.6 DMC ↔ 음성 인식 서비스 (NEW)
4.6.1 Topic: dobby1/stt/result (STT Service → DMC)
Message:
yamlstring text  # STT 결과 텍스트
float32 confidence  # 0.0 ~ 1.0
builtin_interfaces/Time timestamp
Publish Rate: Event-driven (음성 입력 시)
4.6.2 Service: dobby1/tts/speak
Request:
yamlstring text
string language  # "ko-KR", "en-US"
float32 speed  # 0.5 ~ 2.0 (default: 1.0)
Response:
yamlbool success
float32 duration  # seconds (예상 출력 시간)
4.6.3 LLM Service (HTTP POST, DMC → External)
Endpoint 1: POST /LLM/books (도서 검색)
Request:
json{
  "query": "어린 왕자 책 찾아줘",
  "sessionId": "user_12345_session_abc"
}
Response:
json{
  "query": "어린왕자는 A구역 3번 선반에 있습니다.",
  "action": "book_voice"
}
Endpoint 2: POST /LLM/guide (길안내)
Request:
json{
  "query": "열람실 알려줘",
  "sessionId": "user_12345_session_abc"
}
Response:
json{
  "response": "열람실은 2층 좌측에 있습니다.",
  "action": "road_voice"
}
Endpoint 3: POST /LLM/order (음료 주문, Kreacher용)
Request:
json{
  "query": "아메리카노 주문할게요",
  "sessionId": "user_12345_session_abc"
}
Response:
json{
  "response": "아메리카노 주문이 접수되었습니다.",
  "action": "drink_voice"
}
DMC의 처리:

action: "book_voice" → TTS 출력만 (작업 생성 안 함)
action: "road_voice" → TTS 출력 후 GuidingSM 시작
action: "drink_voice" → Kreacher에 전달 (DMC는 무시)

4.7 DMC → All (상태 발행)
4.7.1 Topic: dobby1/status/robot_state
Message:
yaml# DobbyState.msg
uint8 main_state
  uint8 INITIALIZING = 0
  uint8 CHARGING = 1
  uint8 IDLE = 2
  uint8 EXECUTING_TASK = 3
  uint8 FORCE_MOVE_TO_CHARGER = 4
  uint8 MOVING_TO_CHARGER = 9
  uint8 LISTENING = 10
  uint8 ERROR = 99

uint8 sub_state
  uint8 NONE = 100
  # 101~129: Sub-states (위에 정의)

bool is_error
string error_message
int32 current_task_id  # -1 if no task
builtin_interfaces/Time timestamp
Publish Rate: 1Hz (상태 변경 시 즉시 발행)
4.7.2 Topic: dobby1/status/battery_status
Message:
yaml# BatteryStatus.msg
float32 charge_percentage  # 0.0 ~ 100.0
bool is_charging
bool is_critical  # < 20%
bool is_warning   # < 40%
builtin_interfaces/Time timestamp
```

**Publish Rate:** 1Hz

---

## 5. 데이터 흐름

### 5.1 작업 할당 플로우
```
┌─────┐                                    ┌─────┐
│ RCS │                                    │ DMC │
└──┬──┘                                    └──┬──┘
   │                                          │
   │ 1. Action Call: assign_task              │
   ├─────────────────────────────────────────>│
   │    task_type=PICKUP_BOOK                │
   │    task_id=12345                         │
   │    task_data={"book_id":"ISBN-123",...}  │
   │                                          │
   │                                          ├─> Check battery >= 20%?
   │                                          ├─> Check main_state == IDLE?
   │                                          ├─> Check no current_task?
   │                                          │
   │ 2. Goal Response: ACCEPTED              │
   │<─────────────────────────────────────────┤
   │    accepted=true                         │
   │                                          │
   │                                          ├─> [SMACH Transition]
   │                                          │   IDLE → EXECUTING_TASK
   │                                          │
   │                                          ├─> [Battery Manager]
   │                                          │   start_draining()
   │                                          │
   │                                          ├─> [Start Task SM]
   │                                          │   PickupBookSM.execute()
   │                                          │
   │ 3. Feedback (1Hz)                        │
   │<─────────────────────────────────────────┤
   │    main_state=EXECUTING_TASK (3)        │
   │    sub_state=MOVE_TO_PICKUP (101)       │
   │    progress=15%                          │
   │    battery=82%                           │
   │                                          │
   │ 4. Feedback                              │
   │<─────────────────────────────────────────┤
   │    sub_state=PICKUP_TO_CARRIER (103)    │
   │    progress=50%                          │
   │    battery=80%                           │
   │                                          │
   │ 5. Result                                │
   │<─────────────────────────────────────────┤
   │    success=true                          │
   │    result_message="OK"                   │
   │    total_time=145.3 sec                  │
   │    total_distance=25.4 m                 │
   │                                          │
   │                                          ├─> [SMACH Transition]
   │                                          │   EXECUTING_TASK → IDLE
   │                                          │   (battery >= 40%)
```

### 5.2 배터리 긴급 상황 플로우
```
┌─────┐  ┌─────┐  ┌─────┐  ┌─────┐  ┌─────┐
│ DMC │  │ RCS │  │ DDC │  │ DAC │  │ AIS │
└──┬──┘  └──┬──┘  └──┬──┘  └──┬──┘  └──┬──┘
   │        │        │        │        │
   │ [작업 실행 중]
   │ main_state=EXECUTING_TASK
   │ sub_state=MOVE_TO_STORAGE
   │        │        │        │        │
   ├─> [Battery Manager]
   │   update(): 82% → 79% → ... → 19.5%
   │        │        │        │        │
   ├─> [Battery Manager]
   │   is_critical() = True  ⚠️
   │        │        │        │        │
   ├─> [SMACH Trigger]
   │   battery_critical
   │        │        │        │        │
   ├─> [Before Callback: emergency_abort_all]
   │   • Cancel all Actions
   │        │        │        │        │
   ├───────────────> CancelGoal (move_to_target)
   │        │        │        │        │
   │        │        ├─> [Nav2] Stop
   │        │        │        │        │
   ├────────────────────────> CancelGoal (place_book)
   │        │        │        │        │
   │        │        │        ├─> [MoveIt] Stop
   │        │        │        │        │
   │        │        │        │        │
   ├────────> Result: task_aborted
   │        │   reason="BATTERY_CRITICAL"
   │        │   success=false
   │        │        │        │        │
   ├─> [SMACH Transition]
   │   EXECUTING_TASK → FORCE_MOVE_TO_CHARGER
   │        │        │        │        │
   ├─> [After Callback: emergency_move_to_charger]
   │        │        │        │        │
   ├───────────────> move_to_target(charger_pose)
   │        │        │        │        │
   │        │        ├─> [Nav2] Plan & Execute
   │        │        │   Emergency route
   │        │        │        │        │
   │        │        └───> Arrived
   │        │        │        │        │
   ├─> [SMACH Transition]
   │   FORCE_MOVE_TO_CHARGER → CHARGING
   │        │        │        │        │
   ├─> [Battery Manager]
   │   start_charging()
   │   19.5% → 29.5% → 39.5% → ...
```

### 5.3 음성 인식 플로우
```
┌──────┐  ┌────────┐  ┌─────┐  ┌──────┐  ┌─────┐  ┌─────┐
│ User │  │Speaker │  │ STT │  │ DMC  │  │ LLM │  │ TTS │
└───┬──┘  └────┬───┘  └──┬──┘  └───┬──┘  └──┬──┘  └──┬──┘
    │          │         │         │        │       │
    │ "도비야"  │         │         │        │       │
    ├─────────>│         │         │        │       │
    │          │ Audio   │         │        │       │
    │          ├────────>│         │        │       │
    │          │         │ STT     │        │       │
    │          │         ├────────>│        │       │
    │          │         │ "도비야" │        │       │
    │          │         │         │        │       │
    │          │         │         ├─> [VoiceCommandHandler]
    │          │         │         │   is_wake_word("도비야") = True
    │          │         │         │        │       │
    │          │         │         ├─> [SMACH Trigger]
    │          │         │         │   wake_word_detected
    │          │         │         │        │       │
    │          │         │         ├─> [SMACH Transition]
    │          │         │         │   IDLE → LISTENING
    │          │         │         │        │       │
    │          │         │         ├─> [On Enter: LISTENING]
    │          │         │         │   start_voice_session()
    │          │         │         │   start_timer(20s)
    │          │         │         │        │       │
    │          │         │         ├────────────────>│
    │          │         │         │   speak("네, 무엇을 도와드릴까요?")
    │          │         │         │        │       │
    │<─────────┼─────────┼─────────┼────────┼───────┤
    │ "네, 무엇을 도와드릴까요?" (음성 출력)         │
    │          │         │         │        │       │
    │ "화장실   │         │         │        │       │
    │ 어디있어?"│         │         │        │       │
    ├─────────>│         │         │        │       │
    │          │ Audio   │         │        │       │
    │          ├────────>│         │        │       │
    │          │         │ STT     │        │       │
    │          │         ├────────>│        │       │
    │          │         │ "화장실  │        │       │
    │          │         │ 어디있어?"│       │       │
    │          │         │         │        │       │
    │          │         │         ├─> [VoiceCommandHandler]
    │          │         │         │   is_wake_word() = False
    │          │         │         │   parse_intent(text)
    │          │         │         │        │       │
    │          │         │         ├───────>│       │
    │          │         │         │ POST /LLM/guide
    │          │         │         │ {"query":"화장실..."}
    │          │         │         │        │       │
    │          │         │         │<───────┤       │
    │          │         │         │ {"response":"열람실은...",
    │          │         │         │  "action":"road_voice"}
    │          │         │         │        │       │
    │          │         │         ├─> Intent = NAVIGATION
    │          │         │         │   destination = "화장실"
    │          │         │         │        │       │
    │          │         │         ├────────────────>│
    │          │         │         │   speak("열람실은 2층...")
    │          │         │         │        │       │
    │<─────────┼─────────┼─────────┼────────┼───────┤
    │ "열람실은 2층 좌측에 있습니다." (음성)          │
    │          │         │         │        │       │
    │          │         │         ├─> [SMACH Trigger]
    │          │         │         │   valid_task_command
    │          │         │         │        │       │
    │          │         │         ├─> [SMACH Transition]
    │          │         │         │   LISTENING → EXECUTING_TASK
    │          │         │         │        │       │
    │          │         │         ├─> [Start Task SM]
    │          │         │         │   GuidingSM.execute()
    │          │         │         │   (destination="화장실")
```

---

## 6. 배터리 관리 전략

### 6.1 배터리 상태 정의
```
┌─────────────────────────────────────────────────┐
│         BatteryState (Enum)                     │
├─────────────────────────────────────────────────┤
│  IDLE = 0        # 대기 중 (소모 없음)           │
│  CHARGING = 1    # 충전 중 (+10%/min)            │
│  DRAINING = 2    # 작업 중 (-1%/min)             │
└─────────────────────────────────────────────────┘
6.2 배터리 레벨별 동작 정책
배터리 레벨상태 분류DMC 동작RCS 작업 수락비고100% - 80%정상 (충전 완료)CHARGING → IDLE✅ 수락충전 목표 도달79% - 40%정상 (작업 가능)작업 수행✅ 수락-39% - 20%주의작업 수행 가능, 완료 후 충전소 이동✅ 수락EXECUTING_TASK → MOVING_TO_CHARGER19% - 5%위험모든 작업 중단, 강제 충전 복귀❌ 거부 (reject_reason="BATTERY_LOW")* → FORCE_MOVE_TO_CHARGER< 5%긴급시스템 정지 경고, 관리자 알림❌ 거부ERROR 상태 고려
6.3 배터리 변화 속도 (Config)
파일: config/battery_config.yaml
yamlbattery:
  # 초기값
  initial_level: 100.0  # %
  
  # 변화 속도
  rates:
    charging: 10.0      # %/min (충전 중)
    idle: 0.0           # %/min (대기 중)
    working: -1.0       # %/min (작업 중)
  
  # 임계값
  thresholds:
    critical: 20.0      # % (강제 복귀)
    warning: 40.0       # % (작업 후 충전)
    charge_target: 80.0 # % (충전 완료)
    emergency: 5.0      # % (관리자 알림)
  
  # 테스트 모드
  test_mode:
    enabled: false      # true시 배터리 소모/충전 비활성화
    manual_control: true # Test GUI에서 직접 설정 가능
```

### 6.4 배터리 업데이트 로직

**업데이트 주기:** 1초마다 (1Hz Timer)

**Pseudo-code:**
```
every 1 second:
  1. dt = 1/60  # 1초 = 1/60분
  
  2. current_main_state = sm_manager.get_current_state()
  
  3. if current_main_state == CHARGING:
       battery_level += charge_rate * dt
       battery_state = CHARGING
     
     elif current_main_state == EXECUTING_TASK:
       if not test_mode.enabled:
         battery_level -= abs(work_rate) * dt
       battery_state = DRAINING
     
     else:  # IDLE, LISTENING, etc.
       battery_state = IDLE
       # battery_level 변화 없음
  
  4. battery_level = clamp(battery_level, 0, 100)
  
  5. # 임계값 체크
     if battery_level < critical_threshold (20%):
       sm_manager.trigger('battery_critical')
     
     if battery_level >= charge_target (80%) and battery_state == CHARGING:
       sm_manager.trigger('charge_complete')
     
     if battery_level < emergency_threshold (5%):
       notify_admin("BATTERY_EMERGENCY")
  
  6. # 발행
     state_publisher.publish_battery(battery_level, is_charging)
```

### 6.5 Test GUI에서 배터리 제어

**기능:**
- 슬라이더로 배터리 레벨 직접 설정 (0~100%)
- "Set Battery" 버튼 클릭 시 즉시 반영
- "Enable Test Mode" 체크박스
  - ON: 배터리 자동 변화 비활성화
  - OFF: 정상 동작 (충전/소모)

**인터페이스:**
```
# Service: /dobby1/test/set_battery
Request:
  float32 level  # 0.0 ~ 100.0
  bool freeze    # true시 자동 변화 중지

Response:
  bool success
  float32 current_level

7. 에러 처리 & 타임아웃
7.1 에러 분류 체계
에러 레벨코드 범위설명DMC 동작RCS 보고INFO1xx정상 동작 정보로그만 기록-WARNING2xx복구 가능한 경고재시도 시도Feedback에 포함ERROR3xx작업 실패Task SM 중단, IDLE 복귀Result: success=falseCRITICAL4xx시스템 레벨 오류ERROR 상태 진입, 모든 동작 정지관리자 알림
7.2 에러 코드 정의
yamlerror_codes:
  # 1xx: INFO
  100: "Task started"
  101: "State transition: {from} → {to}"
  102: "Sub-state changed: {sub_state}"
  
  # 2xx: WARNING (재시도 가능)
  200: "Action timeout, retrying ({retry_count}/1)"
  201: "Book not found, re-scanning"
  202: "User lost during guidance, searching"
  203: "Storage box occupied, trying next slot"
  204: "Gripper error, re-attempting"
  
  # 3xx: ERROR (작업 실패)
  300: "Task timeout exceeded ({total_time}s)"
  301: "Book not found after retries"
  302: "User not found after search timeout"
  303: "Navigation failed: {error_code}"
  304: "Arm control failed: {error_code}"
  305: "Invalid task data: {reason}"
  306: "Storage box full, no available slots"
  307: "Verification failed: {error_type}"
  
  # 4xx: CRITICAL (시스템 오류)
  400: "Hardware failure: {component}"
  401: "Communication loss: {interface}"
  402: "Battery system error"
  403: "Emergency stop triggered"
  404: "SMACH execution error: {exception}"
7.3 타임아웃 정책
7.3.1 작업별 전체 타임아웃
작업전체 타임아웃초과 시 동작에러 코드도서 픽업240초 (4분)Task SM 중단, Result: success=false300반납 정리600초 (10분)Task SM 중단, 부분 성공 처리 가능300길안내600초 (10분)안내 종료, Result: success=false300좌석 정리300초 (5분)Task SM 중단300서가 정리600초 (10분)Task SM 중단, 부분 성공 처리 가능300
SMACH 구현:

StateMachine에 timeout 파라미터 설정
Timeout 시 outcomes=['timeout'] 반환

7.3.2 Sub-State별 타임아웃
이동 관련:

MOVE_TO_* 모든 State: 60초
초과 시: Nav2 Action Cancel → 재시도 1회 → 실패

암 제어 관련:

PICKUP_*, PLACE_*: 30초
COLLECT_*: 30초 × 개수
초과 시: MoveIt Action Cancel → 재시도 1회 → 실패

비전 관련:

DETECT_*, SCAN_*: 30초
VERIFY_*: 10초
초과 시: Service Timeout → 재시도 1회 → 실패

음성 관련:

SELECT_DEST (목적지 선택): 60초
SCAN_USER (사용자 스캔): 30초
LISTENING (음성 명령 대기): 20초
초과 시: TTS "시간 초과" → IDLE 복귀

7.3.3 Action/Service 타임아웃
파일: config/action_timeouts.yaml
yamlactions:
  move_to_target:
    timeout: 60.0
    retry_count: 1
    cancel_on_timeout: true
  
  pick_book:
    timeout: 30.0
    retry_count: 1
    cancel_on_timeout: true
  
  place_book:
    timeout: 30.0
    retry_count: 1
    cancel_on_timeout: true
  
  collect_books:
    timeout_per_book: 30.0
    retry_count: 0  # 수거는 재시도 안 함 (부분 성공 허용)
  
  collect_trash:
    timeout: 30.0
    retry_count: 0

services:
  detect_book:
    timeout: 5.0
    retry_count: 1
  
  register_person:
    timeout: 10.0
    retry_count: 2
  
  llm_parse:
    timeout: 5.0
    retry_count: 0  # LLM은 재시도 안 함
```

### 7.4 에러 복구 플로우
```
┌────────────────────────────────────────┐
│ Action/Service 실행                    │
└───────────────┬────────────────────────┘
                │
                ├─> [Result 수신 또는 Timeout]
                │
                ├─> [Check] Result.success?
                │
         ┌──────┴──────┐
         │ YES         │ NO
         │             │
         ▼             ▼
    다음 단계       에러 발생
                      │
                      ├─> [Check] retry_count < max_retry?
                      │
               ┌──────┴──────┐
               │ YES         │ NO
               │             │
               ▼             ▼
          [Retry]     [에러 레벨 확인]
               │             │
               └─────┬───────┘
                     │
                     ├─> [에러 코드 범위]
                     │
          ┌──────────┼──────────┬──────────┐
          │ 1xx      │ 2xx      │ 3xx      │ 4xx
          │          │          │          │
        INFO      WARNING     ERROR    CRITICAL
          │          │          │          │
      로그만   로그 + 계속   Task 중단   ERROR 상태
                              ↓             ↓
                        IDLE 복귀    모든 동작 정지
                        RCS 보고     관리자 알림
```

### 7.5 재시도 전략

**원칙:**
- 하드웨어 관련 (이동, 암): 1회 재시도
- 비전 관련: 1-2회 재시도 (감지 실패는 흔함)
- 통신 관련 (LLM, RCS): 재시도 안 함 (빠른 실패)
- 수거 작업: 재시도 안 함 (부분 성공 허용)

**재시도 간격:** 1초 (하드웨어 안정화)

**예시 (도서 감지):**
```
1차 시도: detect_book(book_id) → detected=false
  ↓ 1초 대기
2차 시도: detect_book(book_id) → detected=false
  ↓
에러 코드 301: "Book not found after retries"
Task SM 중단

8. 주요 시나리오
8.1 정상 시나리오: 도서 픽업
sequencetitle 도서 픽업 (정상)

RCS->DMC: assign_task(PICKUP_BOOK)
note over DMC: Check: battery>=20%, state==IDLE
DMC->RCS: ACCEPTED

note over DMC: State: IDLE → EXECUTING_TASK
note over DMC: Battery: start_draining()
note over DMC: Start PickupBookSM

DMC->DDC: move_to_target(shelf_pose)
note over DDC: Nav2 planning & execution
DDC->DMC: Feedback: distance_remaining
DDC->DMC: Result: success, arrived

note over DMC: SubState: MOVE_TO_PICKUP → DETECT_BOOK

DMC->AIS: detect_book(book_id)
AIS->DMC: detected=true, book_pose

note over DMC: SubState: DETECT_BOOK → PICKUP_TO_CARRIER

DMC->DAC: pick_book(book_id, pose, carrier_slot=1)
note over DAC: MoveIt planning, approach, grasp
DAC->DMC: Result: success

note over DMC: SubState: PICKUP_TO_CARRIER → MOVE_TO_STORAGE

DMC->DDC: move_to_target(storage_pose)
DDC->DMC: Result: success

note over DMC: SubState: MOVE_TO_STORAGE → PLACE_TO_STORAGE_BOX

DMC->DAC: place_book(book_id, carrier_slot=1, storage_id=3)
note over DAC: MoveIt planning, place, release
DAC->DMC: Result: success

note over DMC: SubState: PLACE_TO_STORAGE_BOX → VERIFY_PLACEMENT

DMC->AIS: verify_book_position(book_id)
AIS->DMC: is_correct=true

note over DMC: PickupBookSM outcome: 'succeeded'
note over DMC: State: EXECUTING_TASK → IDLE
note over DMC: Battery: 78% (작업 완료, 충전 불필요)

DMC->RCS: Result: success=true, time=145s, distance=28m
8.2 예외 시나리오: 배터리 부족
sequencetitle 반납 정리 중 배터리 긴급 복귀

RCS->DMC: assign_task(RESHELVING_BOOK)
DMC->RCS: ACCEPTED

note over DMC: State: IDLE → EXECUTING_TASK
note over DMC: Battery: 25% → start_draining()

DMC->DDC: move_to_target(return_desk_pose)
note over DDC: Navigating...
note over DMC: Battery: 25% → 24% → ... → 19.8%

note over DMC: BatteryManager.update()
note over DMC: is_critical() = True ⚠️

note over DMC: [SMACH Trigger] battery_critical

note over DMC: [Before Callback] emergency_abort_all()

DMC->DDC: CancelGoal(move_to_target)
DDC->DMC: Goal canceled

DMC->DAC: CancelGoal (if any)
DMC->AIS: Stop any active service

note over DMC: State: EXECUTING_TASK → FORCE_MOVE_TO_CHARGER

DMC->RCS: Result: success=false, error="BATTERY_CRITICAL"

note over DMC: [After Callback] emergency_move_to_charger()

DMC->DDC: move_to_target(charger_pose)
note over DDC: Emergency route, max speed
DDC->DMC: Result: success

note over DMC: State: FORCE_MOVE_TO_CHARGER → CHARGING
note over DMC: Battery: 19.5% → 29.5% → 39.5% → ...
note over DMC: Battery: 80.0% (charge_target)

note over DMC: [SMACH Trigger] charge_complete

note over DMC: State: CHARGING → IDLE


## 9. 테스트 전략

### 9.1 테스트 레벨 구조
```
┌───────────────────────────────────────────────────────────┐
│ Level 4: 시스템 통합 테스트 (End-to-End)                  │
│  ────────────────────────────────────────────────────     │
│  • 실제 RCS + DMC + Mock 하위 컨트롤러                    │
│  • 전체 시나리오 검증                                      │
│  • 멀티 로봇 동시 실행 테스트                              │
│  • 장시간 스트레스 테스트                                  │
└───────────────────────────────────────────────────────────┘
                            ▲
┌───────────────────────────────────────────────────────────┐
│ Level 3: 컴포넌트 통합 테스트                             │
│  ────────────────────────────────────────────────────     │
│  • DMC + Mock Interfaces                                  │
│  • SMACH State Machine + Interface Manager                │
│  • 배터리 시나리오 (충전/소모/긴급복귀)                    │
│  • 음성 인식 플로우 (Wake word → LLM → Task)              │
└───────────────────────────────────────────────────────────┘
                            ▲
┌───────────────────────────────────────────────────────────┐
│ Level 2: 모듈 단위 테스트                                  │
│  ────────────────────────────────────────────────────     │
│  • StateMachineManager 독립 테스트                        │
│  • BatteryManager 독립 테스트                             │
│  • InterfaceManager 독립 테스트                           │
│  • VoiceCommandHandler 독립 테스트                        │
│  • Task SM 독립 실행 (PickupBookSM, GuidingSM 등)         │
└───────────────────────────────────────────────────────────┘
                            ▲
┌───────────────────────────────────────────────────────────┐
│ Level 1: 유닛 테스트 (Unit Test)                          │
│  ────────────────────────────────────────────────────     │
│  • SMACH State 클래스 (execute 메서드)                    │
│  • Transition 조건 함수 (is_critical, can_accept_task)    │
│  • 배터리 계산 로직 (update, clamp)                        │
│  • LLM 파싱 로직 (parse_book_query, parse_guide_query)   │
└───────────────────────────────────────────────────────────┘
```

### 9.2 Test GUI 기능 명세

#### 9.2.1 UI 레이아웃
```
┌─────────────────────────────────────────────────────────────┐
│           Dobby DMC Test GUI (RQt Plugin)                   │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌─── Control Panel ────────────────────────────────────┐  │
│  │                                                       │  │
│  │  [Battery Control]                                    │  │
│  │  Current: 45.2%  [▓▓▓▓▓▓▓▓▓░░░░░░░░░░░]               │  │
│  │  Set Level: [60] % [Apply]                            │  │
│  │  ☑ Enable Test Mode (freeze battery)                 │  │
│  │                                                       │  │
│  │  [State Control]                                      │  │
│  │  Current Main: EXECUTING_TASK (3)                     │  │
│  │  Current Sub:  MOVE_TO_PICKUP (101)                   │  │
│  │                                                       │  │
│  │  Force Main State:                                    │  │
│  │  ( ) IDLE  ( ) CHARGING  ( ) EXECUTING_TASK           │  │
│  │  ( ) LISTENING  ( ) ERROR                             │  │
│  │  [Apply State]                                        │  │
│  │                                                       │  │
│  │  [Task Assignment]                                    │  │
│  │  Task Type: [Pickup Book ▼]                           │  │
│  │  Task Data: {"book_id":"TEST-001",...}                │  │
│  │  [Assign Task]                                        │  │
│  │                                                       │  │
│  │  Quick Tasks:                                         │  │
│  │  [Pickup Book] [Reshelving] [Guiding]                 │  │
│  │  [Cleaning]    [Sorting]                              │  │
│  │                                                       │  │
│  │  [Simulate Events]                                    │  │
│  │  Voice: [Wake Word] [Valid Command] [Invalid]         │  │
│  │  Vision: [Book Detected] [Book Not Found]             │  │
│  │  Motion: [Arrived] [Navigation Failed]                │  │
│  │  Other: [User Lost] [Gripper Error]                   │  │
│  │                                                       │  │
│  └───────────────────────────────────────────────────────┘  │
│                                                             │
│  ┌─── State Monitor ────────────────────────────────────┐  │
│  │                                                       │  │
│  │  Main State:    EXECUTING_TASK (3)                    │  │
│  │  Sub State:     MOVE_TO_PICKUP (101)                  │  │
│  │  Battery:       45.2%  [⚠️ WARNING]                   │  │
│  │  Current Task:  12345 (PICKUP_BOOK)                   │  │
│  │  Progress:      32%                                   │  │
│  │  Status:        "Navigating to shelf..."              │  │
│  │  Elapsed Time:  00:00:45                              │  │
│  │                                                       │  │
│  │  Active Interfaces:                                   │  │
│  │  DDC: ✅ Action in progress                           │  │
│  │  DAC: ⏸️ Idle                                         │  │
│  │  AIS: ✅ Subscribed                                   │  │
│  │  GUI: ✅ Connected                                    │  │
│  │                                                       │  │
│  └───────────────────────────────────────────────────────┘  │
│                                                             │
│  ┌─── Timeline (최근 10개 이벤트) ──────────────────────┐  │
│  │                                                       │  │
│  │  12:34:56  [INFO] State: IDLE → EXECUTING_TASK        │  │
│  │  12:35:12  [INFO] SubState: MOVE_TO_PICKUP            │  │
│  │  12:35:45  [INFO] DDC Action: move_to_target started  │  │
│  │  12:36:20  [WARNING] Battery: 48% → 45%               │  │
│  │  12:36:35  [INFO] DDC Action: arrived                 │  │
│  │  12:36:40  [INFO] SubState: DETECT_BOOK               │  │
│  │  ...                                                  │  │
│  │                                                       │  │
│  │  [Clear] [Export Log]                                 │  │
│  └───────────────────────────────────────────────────────┘  │
│                                                             │
│  ┌─── Mock Interface Responses ─────────────────────────┐  │
│  │                                                       │  │
│  │  [DDC Mock]                                           │  │
│  │  Next move_to_target will:                            │  │
│  │  ( ) Succeed  ( ) Fail  ( ) Timeout                   │  │
│  │  Delay: [2.0] seconds                                 │  │
│  │                                                       │  │
│  │  [DAC Mock]                                           │  │
│  │  Next pick_book will:                                 │  │
│  │  ( ) Succeed  ( ) Fail  ( ) Timeout                   │  │
│  │  Delay: [1.5] seconds                                 │  │
│  │                                                       │  │
│  │  [AIS Mock]                                           │  │
│  │  Next detect_book will:                               │  │
│  │  ( ) Succeed (detected=true)                          │  │
│  │  ( ) Fail (detected=false)                            │  │
│  │  Confidence: [0.95]                                   │  │
│  │                                                       │  │
│  └───────────────────────────────────────────────────────┘  │
│                                                             │
│  ┌─── Scenario Playback ────────────────────────────────┐  │
│  │                                                       │  │
│  │  Load Scenario: [pickup_success.yaml ▼] [Load]       │  │
│  │                                                       │  │
│  │  Steps:                                               │  │
│  │  1. Set battery=80%                                   │  │
│  │  2. Assign pickup_book task                           │  │
│  │  3. Mock DDC: success, 5s                             │  │
│  │  4. Mock AIS: detected=true                           │  │
│  │  5. Mock DAC: success, 3s                             │  │
│  │  ...                                                  │  │
│  │                                                       │  │
│  │  [▶️ Play] [⏸️ Pause] [⏹️ Stop] [⏭️ Step]              │  │
│  │                                                       │  │
│  └───────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

#### 9.2.2 Test GUI 기능 목록

| 기능 | 설명 | 구현 방법 |
|------|------|----------|
| **배터리 제어** | 슬라이더로 0~100% 설정, 즉시 반영 | Service Call: `/dobby1/test/set_battery` |
| **Test Mode** | 배터리 자동 변화 중지 | Service Call에 `freeze=true` 파라미터 |
| **강제 상태 전환** | Main State 직접 변경 (디버깅용) | SMACH Trigger 직접 호출 |
| **작업 할당** | GUI에서 작업 생성 및 할당 | Action Call: `/dobby1/assign_task` |
| **Quick Tasks** | 미리 정의된 작업 버튼 (JSON 자동 생성) | 템플릿 기반 Task 생성 |
| **이벤트 시뮬레이션** | Wake word, 감지 성공/실패 등 | Topic Publish 또는 Mock 응답 설정 |
| **상태 모니터** | 실시간 Main/Sub State, 배터리, 진행률 | Topic Subscribe: `/dobby1/status/*` |
| **Interface 상태** | 각 인터페이스 연결 상태 및 활동 | Service Call로 health check |
| **Timeline** | 최근 이벤트 로그 (시간순) | ROS Log Subscriber + 필터링 |
| **Mock 응답 설정** | 다음 Action/Service 결과 미리 정의 | Mock Interface 파라미터 설정 |
| **Scenario Playback** | YAML 시나리오 파일 자동 재생 | 스크립트 기반 자동화 |

### 9.3 테스트 케이스 목록

#### 9.3.1 Main State Machine 테스트

| Test ID | 시나리오 | 초기 상태 | 트리거 | 기대 결과 | 검증 방법 |
|---------|---------|----------|--------|----------|----------|
| **SM-01** | 시스템 부팅 → 초기화 | 전원 OFF | 전원 ON | INITIALIZING → CHARGING (2초) | State Topic |
| **SM-02** | 충전 완료 | CHARGING, battery=35% | 시간 경과 | battery=80% → IDLE | Battery Topic |
| **SM-03** | 작업 할당 (정상) | IDLE, battery=60% | assign_task | IDLE → EXECUTING_TASK, accepted=true | Action Response |
| **SM-04** | 작업 할당 (배터리 부족) | IDLE, battery=15% | assign_task | IDLE 유지, accepted=false, reason="BATTERY_LOW" | Action Response |
| **SM-05** | 작업 중 배터리 긴급 | EXECUTING_TASK, battery=22% | battery → 19% | EXECUTING_TASK → FORCE_MOVE_TO_CHARGER | State Topic, RCS Result |
| **SM-06** | 작업 완료 (충전 불필요) | EXECUTING_TASK, battery=55% | task_complete | EXECUTING_TASK → IDLE | State Topic |
| **SM-07** | 작업 완료 (충전 필요) | EXECUTING_TASK, battery=35% | task_complete | EXECUTING_TASK → MOVING_TO_CHARGER | State Topic |
| **SM-08** | 음성 호출 | IDLE | wake_word_detected | IDLE → LISTENING | State Topic, TTS 출력 |
| **SM-09** | 음성 타임아웃 | LISTENING | 20초 경과 | LISTENING → IDLE | State Topic, TTS "시간 초과" |
| **SM-10** | 음성 명령 (유효) | LISTENING | valid_task_command | LISTENING → EXECUTING_TASK | State Topic |
| **SM-11** | 치명적 에러 | EXECUTING_TASK | critical_error | * → ERROR | State Topic, Admin 알림 |

#### 9.3.2 Task State Machine 테스트

| Test ID | 작업 | 시나리오 | 기대 결과 | 검증 항목 |
|---------|------|---------|----------|----------|
| **TSM-01** | 도서 픽업 | 정상 완료 | 모든 Sub-State 순차 전환, outcome='succeeded' | Sub-State Topic, Action Results |
| **TSM-02** | 도서 픽업 | 책 미발견 (2회 실패) | DETECT_BOOK에서 outcome='failed', ERROR 301 | AIS Service Response, Error Log |
| **TSM-03** | 도서 픽업 | 이동 실패 | MOVE_TO_PICKUP에서 outcome='failed', ERROR 303 | DDC Action Result |
| **TSM-04** | 도서 픽업 | Timeout (240초) | Task SM 중단, ERROR 300 | SMACH Timeout, RCS Result |
| **TSM-05** | 반납 정리 | 3권, 2개 책장 | MOVE_TO_SHELF 2회, 모든 책 배치, outcome='succeeded' | Sub-State 전환 횟수, Action 호출 |
| **TSM-06** | 반납 정리 | 도서 정보 요청 실패 | REQUEST_BOOK_INFO에서 outcome='failed', ERROR 305 | RCS Service Response |
| **TSM-07** | 길안내 | 정상 완료 | 사용자 인식 → 안내 → 도착, outcome='succeeded' | AIS Tracking Status, DDC Follow Mode |
| **TSM-08** | 길안내 | 사용자 놓침 → 재발견 | GUIDING → FIND_USER → GUIDING, outcome='succeeded' | Sub-State 전환, AIS Reregister |
| **TSM-09** | 길안내 | 사용자 3회 미발견 | 3회 FIND_USER 실패, outcome='failed', ERROR 302 | user_lost_count, Error Log |
| **TSM-10** | 길안내 | 목적지 선택 타임아웃 | SELECT_DEST에서 60초 후 outcome='timeout' | SMACH Timeout |
| **TSM-11** | 좌석 정리 | 쓰레기 없음 | SCAN_DESK에서 outcome='succeeded' (clean) | AIS Trash Detection |
| **TSM-12** | 좌석 정리 | 2개 수거 → 배출 | COLLECT_TRASH 2회, outcome='succeeded' | DAC Action 호출 횟수 |
| **TSM-13** | 서가 정리 | 잘못 꽂힌 도서 재배치 | PICKUP → MOVE → PLACE → VERIFY, outcome='succeeded' | Sub-State 순서, AIS Verify |

#### 9.3.3 배터리 관리 테스트

| Test ID | 시나리오 | 초기값 | 동작 | 기대 결과 | 검증 |
|---------|---------|--------|------|----------|------|
| **BAT-01** | 충전 속도 | 50%, CHARGING | 1분 대기 | 60% (+10%/min) | Battery Topic |
| **BAT-02** | 작업 소모 속도 | 50%, EXECUTING_TASK | 1분 대기 | 49% (-1%/min) | Battery Topic |
| **BAT-03** | 대기 중 소모 없음 | 50%, IDLE | 1분 대기 | 50% (변화 없음) | Battery Topic |
| **BAT-04** | Test Mode (freeze) | 50%, EXECUTING_TASK, test_mode=true | 1분 대기 | 50% (변화 없음) | Battery Topic |
| **BAT-05** | Test GUI 설정 | 50% | set_battery(70%) | 즉시 70% 반영 | Service Response |
| **BAT-06** | Critical 트리거 | 21% → 19% | EXECUTING_TASK | battery_critical 트리거 | SMACH Transition |
| **BAT-07** | Charge Complete | 78% → 80% | CHARGING | charge_complete 트리거 | SMACH Transition |
| **BAT-08** | Emergency 알림 | 6% → 4% | - | Admin 알림 발송 | Admin Topic |

#### 9.3.4 인터페이스 테스트 (Mock 기반)

| Test ID | 인터페이스 | 시나리오 | Mock 설정 | 기대 결과 | 검증 |
|---------|----------|---------|----------|----------|------|
| **IF-01** | DDC | move_to_target 성공 | Mock: succeed, 3s | Result.success=true, 3초 후 | Action Result |
| **IF-02** | DDC | move_to_target 실패 | Mock: fail, error="PATH_NOT_FOUND" | Result.success=false, error_code | Action Result |
| **IF-03** | DDC | move_to_target 타임아웃 | Mock: timeout | 60초 후 timeout 처리 | Action Timeout |
| **IF-04** | DDC | follow_mode 활성화 | Mock: succeed | Service.success=true | Service Response |
| **IF-05** | DAC | pick_book 성공 | Mock: succeed, 2s | Result.success=true | Action Result |
| **IF-06** | DAC | pick_book 그리퍼 에러 | Mock: fail, error="GRIPPER_ERROR" | Result.success=false, message | Action Result |
| **IF-07** | AIS | detect_book 발견 | Mock: detected=true, confidence=0.95 | Response.detected=true | Service Response |
| **IF-08** | AIS | detect_book 미발견 | Mock: detected=false | Response.detected=false | Service Response |
| **IF-09** | AIS | tracking_status 구독 | Mock: person_detected=true, 10Hz | Topic 수신 확인 | Topic Subscriber |
| **IF-10** | LLM | parse_guide_query | Mock HTTP Response: {"action":"road_voice"} | Intent=NAVIGATION | HTTP Response |
| **IF-11** | TTS | speak 성공 | Mock: duration=3.5s | Service.success=true | Service Response |
| **IF-12** | STT | stt_result 발행 | Mock Topic: text="도비야" | VoiceHandler 콜백 호출 | Topic Callback |

#### 9.3.5 통합 시나리오 테스트

| Test ID | 시나리오 | 설명 | 검증 항목 |
|---------|---------|------|----------|
| **INT-01** | 연속 작업 (3개) | 도서 픽업 → 반납 정리 → 좌석 정리 | 모든 작업 성공, State 전환 정상 |
| **INT-02** | 배터리 부족 중단 | 반납 정리 시작 (battery=25%) → 중단 → 충전 → 재할당 | 긴급 복귀, 충전 완료 후 작업 가능 |
| **INT-03** | 음성 + 작업 | "도비야" → "화장실" → 길안내 완료 | 음성 인식 → Task 생성 → 정상 완료 |
| **INT-04** | 에러 복구 | 도서 픽업 실패 → 다른 작업 할당 가능 | ERROR 상태 아님, IDLE 복귀 |
| **INT-05** | 멀티 로봇 (2대) | dobby1, dobby2 동시 작업 | Namespace 충돌 없음, 독립 동작 |
| **INT-06** | 장시간 스트레스 | 1시간 동안 작업 반복 | 메모리 누수 없음, 안정성 유지 |

### 9.4 Mock Interface 구현 전략

**Mock 기능:**
- 각 Interface의 Action/Service를 즉시 응답
- Test GUI에서 응답 타입 설정 (성공/실패/타임아웃)
- 응답 지연 시간 설정 (0~10초)
- Topic은 주기적 발행 (예: tracking_status 10Hz)

**Mock 클래스 구조:**
MockDriveInterface(DriveInterface)

move_to_target() → Future (즉시 or 지연 후 응답)
enable_follow_mode() → 설정된 결과 반환
내부: response_queue (Test GUI에서 설정)

MockArmInterface(ArmInterface)

pick_book() → 설정된 결과 반환
...

MockAIInterface(AIInterface)

detect_book() → 설정된 결과 반환
publish_tracking_status() → Topic 발행 (10Hz)
...

MockLLMInterface(LLMInterface)

parse_*_query() → JSON 응답 (고정 또는 설정)

