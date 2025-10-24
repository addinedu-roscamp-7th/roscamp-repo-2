JAVIS DMC (Dobby Main Controller) 상세 설계서 v6.0

📋 목차

- 1. 시스템 컨텍스트
- 2. 전체 아키텍처 다이어그램
- 3. 패키지 구조
- 4. 클래스 다이어그램
- 5. 상태 머신 설계
- 6. 배터리 관리
- 7. 음성 상호작용 & LISTENING 모드
- 8. 작업별 구현 전략
- 9. 설계 요약


## 1. 시스템 컨텍스트

### 1.1 설계 확정 사항

| 항목 | 결정 사항 |
| :--- | :--- |
| DDC 위치 정보 | DDC → RCS 직접 송신 + DMC 구독 (병렬 구조 유지) |
| TF 프레임 | 공통 `map`, 개별 `odom/base_link` (dobby1/, dobby2/) |
| 메시지 패키지 | `javis_interfaces` (운영) + `javis_dmc_test_msgs` (테스트) |
| 음성 호출 파이프라인 | `voice_recognition_controller`가 아날로그 음성을 수집해 Wake Word(“도비야”) 검출 후 `set_listening_mode` 호출 |
| 자연어 이해 | Wake Word 후 `voice_api_service`에 STT/LLM/TTS 요청, `request_task` → `create_task` → `guide_person` Action 순으로 연계 |
| GUI 목적지 선택 | GUIDING 모드 진입 시 60초 타임아웃, 미선택 시 RCS에 `aborted(destination_timeout)` 결과 반환 |
| 테스트 전략 | Test GUI 기반 수동 검증 + 최소 단위 테스트. 과도한 모의 환경/프레임워크 도입 금지 |

### 1.2 DMC 역할

```
┌─────────────────────────────────────────────────────────────┐
│                Robot Control Service (RCS)                  │
│  - 작업 큐 관리                                               │
│  - 작업 수락 판단 (DMC 상태 + 배터리 확인)                        │
│  - 위치 정보 수신 (DDC Topic 직접 구독)                          │
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
    │ - 배터리  │         │ - 배터리 │
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

### 1.3 연관 문서

- `docs/SoftwareArchitecture.md`: 전체 시스템 컴포넌트 관계,음성 파이프라인, 서비스/액션 명칭, 계층 구조
- `docs/StateDefinition.md`: dobby 로봇의 모드/상태 정의, LISTENING 및 GUIDING 전환 규칙
- `docs/SystemScenario/GuidingScenario.md`: dobby로봇의 음성 인식 Wake Word → 목적지 안내 , 화면 클릭 -> 목적지 안내 전체 시퀀스


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
│   ├── change_tracking_mode (Service)
│   └── tracking/status (Topic)
├── gui/
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

### 2.3 음성 상호작용 계층

```
[사용자] → [마이크] → voice_recognition_controller (WWD + 아날로그 오디오 스트리밍)
    ├─ Wake Word 감지 시 DMC.set_listening_mode(True)
    ├─ 오디오 패킷을 voice_api_service(HTTP)로 전달해 STT/LLM/TTS 처리
    ├─ voice_api_service 응답(TTS 오디오 + 작업 메타데이터)을 수신해 스피커로 출력
    └─ 세션 종료 시 DMC.set_listening_mode(False) 및 오디오 스트림 종료

DMC
    ├─ LISTENING 진입: 현재 작업/주행 일시 정지, 20초 타이머
    ├─ voice_api_service가 전달한 작업 의도를 기반으로 request_task → RCS.create_task 호출
    └─ guide_person Action Goal 수신 후 GUIDING 상태로 전환
```


## 3. 패키지 구조

```
javis_dmc/
├── javis_dmc/                      # Python 패키지
│   ├── __init__.py
│   ├── dmc_node.py                 # Main Node
│   ├── battery_manager.py          # 배터리 관리
│   │
│   ├── states/
│   │   ├── __init__.py
│   │   ├── main_states.py          # Main State 클래스
│   │   └── state_enums.py          # State Enum (javis_interfaces 기반)
│   │
│   ├── task_executors/
│   │   ├── __init__.py
│   │   ├── base_executor.py
│   │   ├── pickup_executor.py
│   │   ├── reshelving_executor.py
│   │   ├── guiding_executor.py
│   │   ├── cleaning_executor.py
│   │   └── sorting_executor.py
│   │
│   ├── interfaces/
│   │   ├── __init__.py
│   │   ├── base_interface.py
│   │   ├── drive_interface.py
│   │   ├── arm_interface.py
│   │   ├── ai_interface.py
│   │   └── gui_interface.py
│   │
│   ├── mock/
│   │   ├── __init__.py
│   │   ├── mock_drive.py
│   │   ├── mock_arm.py
│   │   ├── mock_ai.py
│   │   └── mock_gui.py
│   │
│   └── utils/
│       ├── __init__.py
│       ├── logger.py
│       └── ros_utils.py
│
├── test_gui/
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

javis_dmc_test_msgs/
├── srv/
│   ├── SetBattery.srv
│   └── SetMockResponse.srv
├── package.xml
└── CMakeLists.txt
```

> **참고:** 주요 디렉터리 용도 요약
> - `battery_manager.py`: 배터리 시뮬레이션과 경계값 로직. `DmcStateMachine`과 결합되어 작업 수락 조건을 결정합니다.
> - `states/`: `main_states.py`, `state_enums.py`로 구성된 메인/서브 상태 정의 및 전이 규칙. `StateDefinition.md`와 반드시 동기화합니다.
> - `task_executors/`: 액션 종류별 실행기(픽업·반납·길안내·청소·정리). 공통 베이스 클래스로 서브 상태/피드백 처리를 표준화합니다.
> - `interfaces/`: Drive/Arm/AI/GUI/Voice의 ROS 2 액션·서비스 어댑터. 실 하드웨어와 Mock 구현이 동일 API를 사용하도록 추상화합니다.
> - `sessions/`: LISTENING과 목적지 선택 타임아웃을 관리하는 dataclass. `dmc_node`에서 세션 관리 코드를 분리하기 위한 핵심 보조 모듈입니다.
> - `mock/`: Test GUI·단위 테스트용 가짜 하위 시스템. 실제 배포 시에는 Launch에서 import하지 않도록 주의합니다.
> - `test_gui/`: QA용 rqt 기반 GUI와 헬퍼. 시나리오 실행, 상태 모니터링, Mock 주입을 지원합니다.
> - `config/`: ROS 파라미터 YAML. 타임아웃·충전소 위치 등 실행 시점 설정을 보관합니다.
>   - `action_timeouts.yaml`, `patrol_routes.yaml` 등
> - `launch/`: 단일/다중 로봇 및 테스트용 ROS 2 Launch 스크립트.
> - `test/`: 단위 테스트 모듈. 상태 머신·배터리·실행자 단위 검증을 유지합니다.
> - `javis_dmc_test_msgs/`: 테스트 전용 서비스 정의. 운영 빌드에는 포함하지 않습니다.
> - Test GUI 상세 설계는 `DevelopmentPlan/TestGuiDesign.md`를 참고합니다.
> - Mock/실 장비 전환은 `use_mock_interfaces` 파라미터(Launch 또는 ros2 param)를 통해 제어하며, Test GUI에서 해당 값을 토글할 수 있도록 한다.

### 3.2 소스 레이어 세부 역할

| 계층 | 포함 모듈 | 책임 | 핵심 진입점 |
| :--- | :--- | :--- | :--- |
| 오케스트레이션 | `dmc_node.JavisDmcNode` | 상태 머신 + 액션 서버 + 서비스 핸들러 통합 | `main()` → `JavisDmcNode.__init__` |
| 상태 관리 | `states/main_states.py`, `states/state_enums.py` | 모드/메인/서브 상태 정의, 전이, 작업 수락 판단 | `DmcStateMachine` |
| 배터리 | `battery_manager.BatteryManager` | 충전/방전 시뮬레이션, 경고·위험 임계치 계산 | `_on_battery_timer`, `_sync_battery_state` |
| 세션 | `sessions/listening_session.py`, `sessions/destination_session.py` | LISTENING 및 목적지 선택 타이머/메타데이터 관리 | `_activate_listening_mode`, `_run_guiding_sequence` |
| 하위 장치 인터페이스 | `interfaces/*.py` | Drive/Arm/AI/GUI/Voice 액션·서비스 클라이언트 래핑 | `_initialize_interfaces`, 각 실행 흐름 |
| 작업 실행기 | `task_executors/*.py` | 액션별 서브 상태 진행, 피드백/결과 생산 | `_configure_executors`, `_execute_task` |
| Mock/QA | `mock/*.py`, `test_gui/` | 통합 테스트, 실패 주입, 상태 모니터링 UI | Test GUI 시나리오 |

> **리딩 순서 가이드:** 새 담당자는 (1) `StateDefinition.md` → (2) `states/` → (3) `battery_manager.py` → (4) `sessions/` → (5) `interfaces/` → (6) `task_executors/` → (7) `dmc_node.py` 순으로 확인하면 빠르게 흐름을 파악할 수 있습니다.

### 3.3 `dmc_node.py` 모듈 분해

`JavisDmcNode`는 약 1,500줄로 구성되어 있으며 아래 5개 그룹으로 나눠 읽을 수 있습니다.

| 구간 | 범위 (행 기준) | 내용 | 리팩토링 메모 |
| :--- | :--- | :--- | :--- |
| 초기화 | 54-206 | 파라미터 선언, 상태 머신/세션/인터페이스, 서비스·액션 서버 생성 | 추후 `BootstrapConfig` 클래스로 분리 추천 |
| LISTENING/모드 서비스 | 211-389 | LISTENING 토글, 모드 전환, 비상 정지/재개, 수동 상태 설정 | `ListeningController`, `AdminServiceHandler`로 추출 가능 |
| 액션 서비스 | 751-989 | Goal 수락/취소/실행, 피드백/결과 구성 | 실행자별 헬퍼 클래스로 분할 |
| 작업 런타임 | 872-1289 | 길안내·도서 픽업 시퀀스, Future 타임아웃, Pose 변환 | `GuidingWorkflow`, `PickupWorkflow` 등 별도 모듈화 |
| 공통 헬퍼 | 610-867, 1126-1289 | 배터리 동기화, 세션 대기, Pose/Quaternion 변환 | `navigation_helpers.py` 등 유틸 분리 고려 |

> **리팩토링 원칙**
> 1. 서비스 핸들러/액션 콜백을 `@dataclass` 또는 전용 클래스로 분리하여 테스트 가능한 단위로 만든다.  
> 2. 세션/배터리 판단 로직은 Pure Function으로 유지하여 시뮬레이션 테스트를 쉽게 한다.  
> 3. 인터페이스 실패 처리(`Future` 타임아웃 등)를 공통 함수로 묶어 로그 메시지를 일관되게 관리한다.  
> 4. Mock와 실제 구현이 동일 메서드 시그니처를 갖도록 인터페이스 계층을 확장한다.

### 3.4 빠른 코드 이해 체크리스트

1. `config/action_timeouts.yaml`에서 LISTENING·GUIDING 시간 제약 확인  
2. `DmcStateMachine`에서 모드/작업 전이 조건 정리  
3. `BatteryManager` 임계값이 작업 수락/후속 전이 트리거라는 점 이해  
4. `_activate_listening_mode` → `_run_guiding_sequence` 흐름 추적 (음성 → GUI → 주행)  
5. `_wait_future_success`에서 타임아웃 시 로그만 남기고 성공으로 처리되는 현재 동작을 숙지 (개선 필요)  
6. 액션별 실행자(`task_executors/*`)가 서브 상태와 피드백을 어떻게 업데이트하는지 확인  
7. Mock 인터페이스 인젝션 방법(`mock/mock_*.py`) 파악 — Test GUI에서 실패 시나리오를 재현할 때 사용

### 3.1 ROS 인터페이스 매핑 표

| 인터페이스 | 타입 | 네임스페이스 포함 토픽/서비스 | 주요 동작 | 연계 문서 |
| :--- | :--- | :--- | :--- | :--- |
| DriveInterface | ActionClient | `{robot}/drive/move_to_target`, `{robot}/drive/guide_navigation` | 이동 명령, 사람 추종 액션 전송 | `InterfaceSpecification/dmc_to_ddc.md` |
| DriveInterface | ServiceClient | `{robot}/drive/control_command` | 비상 정지, 재개 명령 | `InterfaceSpecification/dmc_to_ddc.md` |
| GUIInterface | Subscription | `{robot}/gui/screen_event` | 목적지 선택/취소 등 GUI 입력 수신 | `SequenceDiagram/GuidingScenario.md` |
| VoiceRecognitionInterface | Subscription | `{robot}/voice_recognition_controller/stt_result` | LISTENING 중 텍스트 이벤트 수신 | `SequenceDiagram/GuidingScenario.md` |
| VoiceRecognitionInterface | ServiceClient | `voice_recognition_controller/set_listening_mode` | Wake Word 세션 제어 및 오디오 스트림 토글 | `SequenceDiagram/GuidingScenario.md` |
| BatteryManager | Publisher | `{robot}/status/battery_status` | 배터리 퍼블리시 | `InterfaceSpecification/rcs_to_dmc.md` |
| StateMachine | Publisher | `{robot}/status/robot_state` | Main/Sub 상태 브로드캐스트 | `InterfaceSpecification/rcs_to_dmc.md` |

> **설계 의도:** 네임스페이스 접두어 `{robot}`는 `robot_namespace` 파라미터 또는 노드 네임스페이스에서 결정된다. 신규 토픽을 추가할 때는 위 표와 문서를 동시에 갱신해 다른 담당자가 참조 경로를 혼동하지 않도록 한다.

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
│ - listening_active: bool                                │
│ - listening_deadline: Optional[rclpy.time.Time]         │
│ - destination_deadline: Optional[rclpy.time.Time]       │
│                                                         │
│ - battery: BatteryManager                               │
│ - drive: DriveInterface                                 │
│ - arm: ArmInterface                                     │
│ - ai: AIInterface                                       │
│ - gui: GUIInterface                                     │
│ - stt_session: Optional[VoiceSession]                   │
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
│   ├── current_pose: Pose2D
│   ├── listening_active: bool
│   ├── listening_deadline: Optional[Time]
│   └── destination_deadline: Optional[Time]
│
├── [컴포넌트]
│   ├── battery: BatteryManager
│   ├── drive: DriveInterface
│   ├── arm: ArmInterface
│   ├── ai: AIInterface
│   ├── gui: GUIInterface
│   └── voice_session: VoiceSession (voice_recognition_controller 연동 데이터)
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
    ├── sm_timer (100Hz)
    └── listening_timer (20s window 확인)
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
    │   ├── cancel_all_actions()
    │   └── control_command(...)
    │
    ├── ArmInterface
    │   ├── pick_book(...)
    │   ├── place_book(...)
    │   ├── cancel_all_actions()
    │   └── is_action_active()
    │
    ├── AIInterface
    │   ├── change_tracking_mode(...)
    │   ├── detect_book(...)  # 스켈레톤 유지
    │   └── subscribe_tracking_status(...)
    │
    └── GUIInterface
        └── subscribe_screen_event(...)
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
    ├── CleaningExecutor   # 스켈레톤
    └── SortingExecutor    # 스켈레톤
```

GuidingExecutor 업데이트 포인트
- LISTENING 해제 시점에서 `SELECT_DEST` → `SCAN_USER` → `GUIDING_TO_DEST` 서브 상태 전환.
- 목적지 미선택 시 `aborted(destination_timeout)` 결과를 RCS에 전달하고 Executor 종료.
- Vision Service 이벤트 수신 시 `_publish_feedback`으로 추적 상태를 갱신한다.


> **참고:** LISTENING ↔ GUIDING 전환 로직은 `SequenceDiagram/GuidingScenario.md` 흐름과 1:1 대응합니다. 구현 시 타이머 값과 Vision 이벤트 토픽명을 해당 문서와 일치시키는지 점검하세요.

### 4.5 상태·세션 제어 구성 요소

| 구성 요소 | 타입 | 책임 | 주요 메서드/속성 | 비고 |
| :--- | :--- | :--- | :--- | :--- |
| `DmcStateMachine` | 클래스 | 메인/서브 상태 전이 및 작업 수락 판정 | `set_main_state`, `can_accept_task`, `determine_post_task_state` | `StateDefinition.md` 값과 동기화 |
| `ListeningSession` (신규) | dataclass | Wake Word 지속 시간 관리, Voice API 스트리밍 파이프라인 연결 | `start`, `cancel`, `handle_timeout`, `update_deadline` | GUIDING 중 재진입 지원, 20초 제한 |
| `DestinationSession` (신규) | dataclass | GUI 목적지 선택 타이머 및 결과 캐싱 | `begin_selection`, `resolve_selection`, `abort_timeout` | 60초 타임아웃, Vision 등록 트리거 |
| `BatteryManager` | 클래스 | 배터리 상태 추적 및 경계 확인 | `start_draining`, `is_warning`, `is_critical` | 전이 조건: 40% / 20% |
| `ActionBinding` (개선) | dataclass | MainState ↔ Action 타입/토픽 매핑 | `goal_type`, `feedback_builder`, `result_builder` | 신규 작업 추가 시 테이블만 보강 |
| `InterfaceRegistry` | 컬렉션 | ROS 인터페이스 일괄 초기화/종료 | `initialize`, `shutdown` | 실패 시 로깅 후 계속 진행 |

> **설계 의도:** 세션 전용 구조체를 두어 `JavisDmcNode`가 “세션 시작/중단” 메서드만 부르면 되도록 단순화한다. dataclass + 최소 메서드 조합으로 유지해 오버엔지니어링을 피하고, 별도 상태 머신 라이브러리를 도입하지 않는다.

#### 4.5.1 LISTENING 세션 처리 순서

1. Wake Word 감지 → `ListeningSession.start()` 호출, `VoiceRecognitionInterface.start_stream()` 활성화, 20초 타이머 등록.
2. 타이머 동안 수신된 발화는 `VoiceRecognitionInterface.subscribe_dialog_event` 콜백을 통해 세션으로 전달되며, Voice API 응답 메타데이터를 기반으로 후속 동작을 결정한다.
3. Voice API 응답에 `require_task=True`가 포함되면 세션이 `request_task`를 호출하고 결과를 큐에 보관한다.
4. RCS `guide_person` Goal 수신 시 세션을 종료하고 GUIDING 메인 상태로 전환한다.
5. 타임아웃/취소 발생 시 `handle_timeout`이 음성 스트림을 중단하고 상태를 IDLE 또는 기존 작업 상태로 복귀시킨다.

#### 4.5.2 목적지 선택 타이머 시퀀스

1. LISTENING → GUIDING 전환 직후 `DestinationSession.begin_selection()`으로 GUI 지도 열람을 요청하고 60초 타이머를 설정한다.
2. 사용자가 목적지를 선택하면 `resolve_selection`이 Vision 등록 단계(`SCAN_USER`)로 전환하도록 상태 머신을 갱신한다.
3. 타이머 만료 시 `abort_timeout`이 RCS Action Result `aborted(destination_timeout)`을 반환하고, `determine_post_task_state`를 호출해 후속 상태를 정한다.
4. Vision 등록/추적 완료 이벤트 수신 시 `DestinationSession`을 완료 상태로 표시해 중복 실행을 방지한다.

> **설계 의도:** Wake Word 세션과 목적지 선택 세션을 분리해 동시에 동작할 때 충돌을 피한다. 두 타이머 값(20초/60초)은 `config/action_timeouts.yaml`에서 관리해 테스트가 용이하도록 한다.

## 5. 상태 머신 설계

### 5.1 로봇 모드 계층

| Mode ID | 한글명 | 설명 | 기본 진입 경로 | 주요 해제 조건 | 관리자 제어 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `MODE_STANDBY` | 대기 모드 | 충전소에서 대기하며 음성 호출과 작업 할당을 준비한다. | 관리자 GUI에서 `Standby` 명령, 배터리 충전 완료(≥ 40%) | 배터리 ≤ 40% → `CHARGING`, 긴급 정지 → `EMERGENCY_STOP`, 관리자 `Autonomy` 전환 | Admin GUI `set_robot_mode` |
| `MODE_AUTONOMY` | 자율이동 모드 | 웨이포인트 순찰(ROAMING) 상태에서 작업/음성 호출을 수락한다. | 관리자 GUI에서 `Autonomy` 명령, 충전 완료 후 자동 복귀 | 배터리 ≤ 40% → `MOVING_TO_CHARGER`, 긴급 정지 | Admin GUI `set_robot_mode` |

> **설계 의도:** 모드 전환은 항상 관리자 GUI 트리거를 따라간다. 내부 로직은 배터리/긴급 상황에 따라 모드를 강제 종료할 수 있지만, 임의 전환은 허용하지 않는다.

### 5.2 모드 전환 및 Admin GUI 연동

- **모드 변경:** Admin GUI는 `dobby_admin/set_robot_mode`(예정) 서비스를 호출해 `standby` 또는 `autonomy` 값을 전달한다. DMC는 현재 작업/세션이 없을 때만 즉시 전환하며, 진행 중이라면 `busy` 응답을 반환한다.
- **웨이포인트 제어:** AUTONOMY 모드 진입 시 DMC는 `ROAMING` 상태로 전환하고 `DDC.start_patrol`을 호출한다. STANDBY 복귀 시 `DDC.control_command(STOP)` 후 `IDLE`로 정착한다.
- **긴급 정지:** Admin GUI는 `dobby_admin/emergency_stop` 서비스를 통해 어느 모드에서나 즉시 `EMERGENCY_STOP`으로 전환한다. 해제 시 `resume_navigation` 명령으로 이전 모드/상태를 복원한다.
- **모드별 작업 수락 정책:** STANDBY에서는 `IDLE`에서만 새 작업을 수락하며, AUTONOMY에서는 `ROAMING` 중 작업을 수락한다. 두 모드 모두 배터리 경고 이하(≤ 40%)에서는 신규 작업을 거부한다.
- **상태 브로드캐스트:** 모드 변경 결과는 `status/robot_state` 토픽과 Admin GUI 피드백 채널(`dobby_admin/mode_feedback`)에 동시에 반영해 UI가 즉시 갱신되도록 한다.

### 5.3 Main State 정의 (`DobbyState.msg` 기준)

| State 값 | 식별자 | 설명 | 배터리 변화 | 작업 수락 |
| :--- | :--- | :--- | :--- | :--- |
| 0 | INITIALIZING | 시스템 초기화 | 없음 | ❌ |
| 1 | CHARGING | 충전 중 (`battery < 40%`) | +10%/min | ❌ |
| 2 | IDLE | 대기 (충전소, `battery ≥ 40%`) | +10%/min (충전소), 0 | ✅ |
| 3 | MOVING_TO_CHARGER | 충전소 이동 | -1%/min | ✅ (≥ 40%) |
| 4 | PICKING_UP_BOOK | 도서 픽업 | -1%/min | ❌ |
| 5 | RESHELVING_BOOK | 반납 정리 | -1%/min | ❌ |
| 6 | GUIDING | 길 안내 | -1%/min | ❌ |
| 7 | CLEANING_DESK | 좌석 정리 | -1%/min | ❌ |
| 8 | SORTING_SHELVES | 서가 정리 | -1%/min | ❌ |
| 9 | FORCE_MOVE_TO_CHARGER | 강제 충전 복귀 | -1%/min | ❌ |
| 10 | LISTENING | 음성 인식 대기 (Wake Word 후 20초) | 0 | ❌ |
| 11 | ROAMING | 웨이포인트 순찰 (AUTONOMY 모드) | -1%/min | ✅ (≥ 40%) |
| 98 | EMERGENCY_STOP | 긴급 정지 (Admin) | 0 | ❌ |
| 99 | MAIN_ERROR | 치명적 오류 | 없음 | ❌ |

### 5.4 Sub State 정의 (공통 + 작업)

- 공통: `NONE = 100`, `SUB_ERROR = 199`
- Book Pickup: `MOVE_TO_PICKUP`, `PICKUP_BOOK`, `MOVE_TO_STORAGE`, `STOWING_BOOK`
- Reshelving: `MOVE_TO_RETURN_DESK`, `COLLECT_RETURN_BOOKS`, `MOVE_TO_PLACE_SHELF`, `PLACE_RETURN_BOOK`
- Guiding: `SELECT_DEST`, `SCAN_USER`, `GUIDING_TO_DEST`, `FIND_USER`
- Cleaning (스켈레톤): `MOVE_TO_DESK`, `SCAN_DESK`, `CLEANING_TRASH`, `MOVE_TO_BIN`, `TIDYING_SHELVES`
- Sorting (스켈레톤): `MOVE_TO_SHELF`, `SCAN_BOOK`, `SORT_BOOK`

### 5.5 Main State Diagram

```
stateDiagram-v2
    [*] --> INITIALIZING

    INITIALIZING --> CHARGING: init_complete
    CHARGING --> IDLE: battery >= 40

    IDLE --> LISTENING: wake_word_detected
    IDLE --> PICKING_UP_BOOK: task_assigned(PICKUP)
    IDLE --> RESHELVING_BOOK: task_assigned(RESHELVING)
    IDLE --> GUIDING: task_assigned(GUIDING)
    IDLE --> CLEANING_DESK: task_assigned(CLEANING)
    IDLE --> SORTING_SHELVES: task_assigned(SORTING)
    IDLE --> FORCE_MOVE_TO_CHARGER: battery < 20

    LISTENING --> GUIDING: guidance_confirmed
    LISTENING --> IDLE: timeout_20s or cancelled
    LISTENING --> FORCE_MOVE_TO_CHARGER: battery < 20

    GUIDING --> IDLE: complete & at_charger
    GUIDING --> MOVING_TO_CHARGER: complete & !at_charger
    GUIDING --> LISTENING: wake_word_detected (안내 중 호출)
    GUIDING --> FORCE_MOVE_TO_CHARGER: battery < 20
    GUIDING --> IDLE: aborted(destination_timeout or user_cancel)

    MOVING_TO_CHARGER --> IDLE: arrived & battery >= 40
    MOVING_TO_CHARGER --> CHARGING: arrived & battery < 40
    MOVING_TO_CHARGER --> FORCE_MOVE_TO_CHARGER: battery < 20

    FORCE_MOVE_TO_CHARGER --> CHARGING: arrived
    FORCE_MOVE_TO_CHARGER --> MAIN_ERROR: failure

    MAIN_ERROR --> IDLE: error_resolved
```

### 5.6 STANDBY MODE 상태 다이어그램

```
stateDiagram-v2
    [*] --> INITIALIZING
    INITIALIZING --> CHARGING: init_complete
    CHARGING --> IDLE: battery >= 40 & admin=STANDBY
    IDLE --> LISTENING: wake_word_detected
    IDLE --> TASK_IN_PROGRESS: task_assigned
    IDLE --> FORCE_MOVE_TO_CHARGER: battery <= 20
    LISTENING --> IDLE: timeout_20s or cancel
    LISTENING --> TASK_IN_PROGRESS: guidance_confirmed

    state TASK_IN_PROGRESS {
        direction LR
        [*] --> CLEANING_DESK
        [*] --> GUIDING
        [*] --> SORTING_SHELVES
        [*] --> PICKING_UP_BOOK
        [*] --> RESHELVING_BOOK
        CLEANING_DESK --> [*]: clean_complete
        GUIDING --> [*]: guide_complete
        SORTING_SHELVES --> [*]: sort_complete
        PICKING_UP_BOOK --> [*]: pickup_complete
        RESHELVING_BOOK --> [*]: reshelving_complete
    }

    TASK_IN_PROGRESS --> MOVING_TO_CHARGER: battery_warning
    MOVING_TO_CHARGER --> CHARGING: charger_arrived
    MOVING_TO_CHARGER --> IDLE: battery >= 80 & admin=STANDBY

    INITIALIZING --> EMERGENCY_STOP: emergency
    CHARGING --> EMERGENCY_STOP: emergency
    IDLE --> EMERGENCY_STOP: emergency
    TASK_IN_PROGRESS --> EMERGENCY_STOP: emergency
```

> **참고:** Admin GUI가 `autonomy`로 전환하면 `IDLE` 또는 `CHARGING` 상태에서 AUTONOMY 모드로 이탈하고, 이후 상태 전이는 5.7을 따른다.

### 5.7 AUTONOMY MODE 상태 다이어그램

```
stateDiagram-v2
    [*] --> ROAMING: admin=AUTONOMY & battery >= 40
    ROAMING --> LISTENING: wake_word_detected
    ROAMING --> TASK_IN_PROGRESS: task_assigned
    ROAMING --> MOVING_TO_CHARGER: battery <= 40
    ROAMING --> FORCE_MOVE_TO_CHARGER: battery <= 20

    LISTENING --> ROAMING: timeout_20s or cancel
    LISTENING --> TASK_IN_PROGRESS: guidance_confirmed

    state TASK_IN_PROGRESS {
        direction LR
        [*] --> CLEANING_DESK
        [*] --> GUIDING
        [*] --> SORTING_SHELVES
        [*] --> PICKING_UP_BOOK
        [*] --> RESHELVING_BOOK
        CLEANING_DESK --> [*]: clean_complete
        GUIDING --> [*]: guide_complete
        SORTING_SHELVES --> [*]: sort_complete
        PICKING_UP_BOOK --> [*]: pickup_complete
        RESHELVING_BOOK --> [*]: reshelving_complete
    }

    TASK_IN_PROGRESS --> ROAMING: task_complete & battery >= 40
    TASK_IN_PROGRESS --> MOVING_TO_CHARGER: battery <= 40
    MOVING_TO_CHARGER --> CHARGING: charger_arrived
    CHARGING --> ROAMING: battery >= 80 & admin=AUTONOMY

    ROAMING --> EMERGENCY_STOP: emergency
    TASK_IN_PROGRESS --> EMERGENCY_STOP: emergency
    LISTENING --> EMERGENCY_STOP: emergency
```

> **관리자 요청:** AUTONOMY 모드에서 `Standby` 명령이 들어오면 현재 작업 종료 후 `MOVING_TO_CHARGER` → `CHARGING` → `IDLE` 순으로 복귀한다.

### 5.8 GUIDING Sub State Diagram

```
stateDiagram-v2
    [*] --> SELECT_DEST

    SELECT_DEST --> SCAN_USER: dest_selected
    SELECT_DEST --> [*]: destination_timeout

    SCAN_USER --> GUIDING_TO_DEST: user_registered
    SCAN_USER --> [*]: scan_failed (3회 시)

    GUIDING_TO_DEST --> FIND_USER: user_lost
    GUIDING_TO_DEST --> [*]: arrived

    FIND_USER --> GUIDING_TO_DEST: user_found
    FIND_USER --> [*]: find_timeout
```

타임아웃 이벤트는 모두 RCS Action Result(`aborted`, reason 포함)로 보고하고, `GuidingExecutor`는 `_publish_feedback`을 통해 GUI/RCS에 진행 상황을 공유한다.


## 6. 배터리 관리

### 6.1 배터리 상태

```
BatteryState (Enum)
├── IDLE = 0
├── CHARGING = 1
└── DRAINING = 2
```

### 6.2 배터리 레벨별 동작

| 레벨 | 상태 | DMC 동작 | 작업 수락 |
| :--- | :--- | :--- | :--- |
| 100% ~ 80% | 정상 (충전 완료) | CHARGING → IDLE | ✅ |
| 79% ~ 40% | 정상 | 작업 수행 | ✅ |
| 39% ~ 20% | 경고 | 작업 완료 후 MOVING_TO_CHARGER | ✅ |
| 19% ~ 0% | 위험 | FORCE_MOVE_TO_CHARGER 진입 | ❌ |

### 6.3 BatteryManager 구조

```
BatteryManager
├── level: float
├── state: BatteryState
├── charge_rate: float (10.0)
├── work_rate: float (-1.0)
├── critical_threshold: float (20.0)
├── warning_threshold: float (40.0)
├── charge_target: float (80.0)
├── test_mode_enabled: bool
└── 메서드: update, start_charging, start_draining, set_idle, is_critical, is_warning,
            is_sufficient, force_set, enable_test_mode, disable_test_mode
```


## 7. 음성 상호작용 & LISTENING 모드

1. **Wake Word 감지:** `voice_recognition_controller`가 “도비야”를 감지하면 `set_listening_mode(True)` 서비스 호출. DMC는 현재 실행 중인 Drive Action을 일시 정지하고 LISTENING 상태로 진입, 20초 타이머를 설정한다.
2. **사용자 의도 파악:** LISTENING 상태에서 수집한 오디오는 `voice_recognition_controller`를 통해 `voice_api_service`로 전송된다. Voice API는 STT/LLM을 처리해 의도를 판단하고 `require_task=True`일 경우 작업 페이로드를 반환한다.
3. **작업 생성:** DMC는 Voice API 결과를 바탕으로 `request_task` 서비스를 호출하고, RCS `create_task`로 연계한다. RCS가 `guide_person` Action Goal을 발행하면 LISTENING 상태를 종료하고 GUIDING 메인 상태로 전환한다.
4. **세션 종료:** 20초 내 추가 입력이 없거나 사용자가 “취소”를 말하면 LISTENING 타이머가 만료되고, DMC는 LISTENING → IDLE 전환과 함께 `set_listening_mode(False)`를 호출해 오디오 스트림을 중단한다.
5. **안내 중 호출:** GUIDING 상태에서도 Wake Word가 감지되면 LISTENING 상태로 재진입하되, 기존 안내 작업은 일시 정지하고 20초 내 응답이 없으면 다시 GUIDING으로 복귀한다.

해당 흐름은 `GuidingScenario.md`의 시퀀스를 기준으로 하며, 상태 전환/서비스 이름은 `SoftwareArchitecture.md`와 1:1로 대응된다.


## 8. 작업별 구현 전략

### 8.1 핵심 구현 원칙

- **GuidingExecutor:** 최신 시나리오 구현의 핵심. GUI 목적지 선택 60초 타이머, Vision Service 연동, RCS Action Result 전송까지 포함한다.
- **Pickup/ReshelvingExecutor:** 기존 로직 유지. 추후 InterfaceSpecification 개정 시 데이터 스키마만 업데이트한다.
- **CleaningExecutor & SortingExecutor:** 아직 스켈레톤 유지. 상태 전이, 기본 피드백, Action 취소 처리만 구현하고 세부 동작은 TODO로 남긴다.
- **Interface 계층:** Drive/Arm/AI/GUI 인터페이스는 `snake_case` 명명 규칙을 따른다. 신규 메서드 추가 시 관련 문서를 함께 갱신한다.
- **테스트 철학:** Test GUI와 필수 단위 테스트만 활용한다. End-to-End 시뮬레이터는 도입하지 않고 로그/ros2 CLI 기반 수동 검증을 포함한다.

### 8.2 구현 체크리스트 (협업용)

| 체크 항목 | 설명 | 담당 | 상태 | 문서 동기화 포인트 |
| :--- | :--- | :--- | :--- | :--- |
| VoiceRecognitionInterface 리팩터링 | 아날로그 오디오 스트림 → Voice API 전송, `set_listening_mode` 서비스 구현 |  | 대기 | §2.3, §4.5.1, `Architecture/SoftwareArchitecture.md` |
| LISTENING 세션 매니저 | Wake Word 타이머, 음성 스트림 제어, Voice API 응답 큐 구현 |  | 대기 | §4.5.1, `SequenceDiagram/GuidingScenario.md` |
| 목적지 선택 타이머 | GUI 이벤트 60초 제한, Vision 연동, RCS 취소 플로우 |  | 대기 | §4.5.2 |
| 모드 전환/관리자 서비스 | `set_robot_mode`, `emergency_stop`, `resume_navigation` 서비스 처리 및 상태 브로드캐스트 |  | 대기 | §5.1~5.7 |
| 상태 머신/Enum 동기화 | `ROAMING`, `EMERGENCY_STOP` 등 MainState 확장 및 전이 로직 구현 |  | 대기 | §5.3~5.7, `StateDefinition/StateDefinition.md` |
| 타이머/파라미터 설정 | `action_timeouts.yaml`에 20s/60s 및 Voice API 엔드포인트 정의, 파라미터 로딩 |  | 대기 | §4.5, §7 |
| 로그/모니터링 보강 | LISTENING/모드 전환/긴급 정지 로그와 Admin GUI 피드백 토픽 업데이트 |  | 대기 | §5.2, §7 |
| 테스트 플랜 업데이트 | 단위 테스트/시나리오 테스트 확장 (모드 전환, Voice API 페이크) |  | 대기 | §8.1, `test/` |
| Task Executor 정비 | 서브 상태 전환/피드백 구조 유지 여부 검토, 겹치는 로직 정리 |  | 대기 | §3.2, §8.1 |
| Nav2 Waypoint 연동 | 순찰(ROAMING)용 Waypoint/Path 메시지 정의 및 DDC 인터페이스 확장 |  | 대기 | §3.2, §5.6, `DevelopmentPlan/NavWaypointDesign.md` |
| Mock & Test GUI 정합성 | Mock API와 Test GUI 제어 경로 일치 여부 검토, 시나리오 Runner 설계 반영 |  | 대기 | §3.2, `DevelopmentPlan/TestGuiDesign.md` |
| 상태 그래프 서비스 | DMC가 상태/전이 메타데이터를 서비스/토픽으로 제공, GUI가 시각화 |  | 대기 | §4.5, §5.3, `DevelopmentPlan/StateIntrospection.md` |

> **체크 방법:** 담당자는 `상태` 열에 `진행중`, `완료` 등을 기입하고 완료 시 관련 문서 섹션 번호를 검토한다. 항목 추가가 필요하면 테이블에 바로 반영한다.

### 8.3 진행 로그 작성 가이드

1. 변경 착수 시 `상태` 값을 `진행중`으로 갱신하고, 간단한 작업 노트는 `docs/DevelopmentPlan/changelog/yyyymmdd.md`(신규)에 기록한다.
2. 코드 수정 후 관련 섹션(§4.5, §5, §7 등)의 내용과 동기화되었는지 확인하고 필요 시 요약 문장을 갱신한다.
3. 리뷰 완료 후 `상태`를 `완료`로 전환하며, 후속 과제가 있다면 테이블 하단에 새 항목을 추가한다.

## 9. 설계 요약

| 항목 | 내용 |
| :--- | :--- |
| Main State | 11개 (LISTENING 추가) |
| Sub State | 23개 (GUIDING 타임아웃 분기 반영) |
| 작업 타입 | 5개 (Pickup, Reshelving, Guiding, Cleaning, Sorting) |
| 인터페이스 | 4개 (Drive, Arm, AI, GUI) |
| 음성 파이프라인 | WWD → voice_recognition_controller → voice_api_service → request_task → guide_person |
| 실패 처리 | Wake Word 타임아웃, 목적지 미선택, 사용자 이탈, 배터리 임계값 |
| 문서 연계 | SoftwareArchitecture / StateDefinition / GuidingScenario |
| 테스트 전략 | Test GUI + 핵심 단위 테스트, 오버 엔지니어링 금지 |

본 설계서는 최신 아키텍처 문서를 근거로 Dobby Main Controller의 구현 기준을 정의한다. 모든 코드 변경 시 본 문서와 참조 문서를 함께 갱신하여 정합성을 유지한다.
