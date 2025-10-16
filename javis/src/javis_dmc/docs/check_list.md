: "도비야"
Speaker->STT: Audio Stream
STT->DMC: Topic: stt_result, text="도비야"
note over DMC: VoiceCommandHandler.on_stt_result()
note over DMC: is_wake_word("도비야") = True
note over DMC: [SMACH Trigger] wake_word_detected
note over DMC: State: IDLE → LISTENING
note over DMC: [On Enter] start_voice_session()
note over DMC: Timer: 20초 시작
DMC->TTS: speak("네, 무엇을 도와드릴까요?")
TTS->Speaker: Audio Output
Speaker->User: "네, 무엇을 도와드릴까요?"
User->Speaker: "화장실 어디 있어?"
Speaker->STT: Audio Stream
STT->DMC: text="화장실 어디 있어?"
note over DMC: VoiceCommandHandler.parse_intent()
DMC->LLM: POST /LLM/guide
note over LLM: {"query":"화장실 어디 있어?"}
LLM->DMC: {"response":"열람실은 2층 좌측...", "action":"road_voice"}
note over DMC: Intent = NAVIGATION
note over DMC: destination = "화장실"
DMC->TTS: speak("열람실은 2층 좌측에 있습니다. 안내해드릴까요?")
TTS->User: (음성 출력)
User->Speaker: "응"
Speaker->STT: Audio
STT->DMC: text="응"
note over DMC: 긍정 응답 확인
note over DMC: [SMACH Trigger] valid_task_command
note over DMC: State: LISTENING → EXECUTING_TASK
note over DMC: Start GuidingSM
DMC->GUI: update_screen(GUIDING_USER_SCAN)
GUI->User: (화면: "카메라를 보세요")
User->DMC: (카메라 앞에 섬)
note over DMC: SubState: SELECT_DEST → SCAN_USER
DMC->AIS: register_person()
note over AIS: 사용자 얼굴 등록 중...
AIS->DMC: success=true, tracking_id="USER_001"
note over DMC: SubState: SCAN_USER → GUIDING_TO_DEST
DMC->DDC: enable_follow_mode("/dobby1/ai/tracking/status")
DMC->DDC: set_destination(bathroom_pose)
note over DDC: Nav2 + Follow behavior 활성화
loop 주기적 체크 (1Hz)
AIS->DMC: tracking_status(person_detected=true)
note over DMC: 정상 안내 중...
DMC->RCS: Feedback (progress=30%, sub_state=GUIDING_TO_DEST)
end
note over User: (사용자가 갑자기 옆으로 비킴)
AIS->DMC: tracking_status(person_detected=false, is_lost=true)
note over DMC: 10초 동안 미감지 확인
note over DMC: SubState: GUIDING_TO_DEST → FIND_USER
DMC->DDC: disable_follow_mode()
DMC->DDC: rotate_in_place(360)
note over DDC: 제자리 회전 시작
DMC->AIS: try_reregister_user("USER_001")
note over AIS: 회전하며 사용자 재탐색...
AIS->DMC: success=true, tracking_id="USER_001"
note over DMC: SubState: FIND_USER → GUIDING_TO_DEST
DMC->DDC: enable_follow_mode()
note over DMC: 안내 재개...
note over DDC: 목적지 도착
DDC->DMC: Result: arrived
DMC->DDC: disable_follow_mode()
note over DMC: GuidingSM outcome: 'succeeded'
note over DMC: State: EXECUTING_TASK → IDLE
DMC->TTS: speak("도착했습니다. 좋은 시간 되세요!")
TTS->User: (음성 출력)
DMC->RCS: Result: success=true, time=185s, distance=42m

### 8.4 복잡 시나리오: 반납 정리 (여러 책장)
```sequence
title 반납 도서 정리 (3권, 2개 책장)

note over RCS: 스케줄링 (1시간마다)
RCS->DMC: assign_task(RESHELVING_BOOK)
DMC->RCS: ACCEPTED

note over DMC: State: IDLE → EXECUTING_TASK
note over DMC: Start ReshelvingBookSM

DMC->DDC: move_to_target(return_desk_pose)
DDC->DMC: Result: success

note over DMC: SubState: MOVE_TO_RETURN_DESK → SCAN_RETURNED_BOOKS

DMC->AIS: scan_returned_books(return_desk_pose)
AIS->DMC: books_found=3, book_ids=["B1","B2","B3"]

note over DMC: SubState: SCAN_RETURNED_BOOKS → COLLECT_TO_CARRIER

DMC->DAC: collect_books(["B1","B2","B3"], carrier_slots=[1,2,3])
note over DAC: 순차 수거: B1→slot1, B2→slot2, B3→slot3
DAC->DMC: Result: success, books_collected=3

note over DMC: SubState: COLLECT_TO_CARRIER → REQUEST_BOOK_INFO

DMC->RCS: request_book_info(["B1","B2","B3"])
RCS->DMC: book_infos=[
note over RCS: B1→ShelfA, B2→ShelfA, B3→ShelfB
RCS->DMC: ]

note over DMC: 그룹핑: ShelfA=[B1,B2], ShelfB=[B3]

note over DMC: SubState: REQUEST_BOOK_INFO → MOVE_TO_SHELF

DMC->DDC: move_to_target(shelfA_pose)
DDC->DMC: Result: success

note over DMC: SubState: MOVE_TO_SHELF → PLACE_BOOKS_TO_SHELF

loop ShelfA 도서 (B1, B2)
  DMC->DAC: place_book(B1, carrier_slot=1, shelfA)
  DAC->DMC: Result: success
  DMC->AIS: verify_book_position(B1)
  AIS->DMC: is_correct=true
  
  DMC->DAC: place_book(B2, carrier_slot=2, shelfA)
  DAC->DMC: Result: success
  DMC->AIS: verify_book_position(B2)
  AIS->DMC: is_correct=true
end

note over DMC: SubState: VERIFY_SHELF_PLACEMENT
note over DMC: Check: 남은 도서? ShelfB=[B3]

note over DMC: SubState: VERIFY_SHELF_PLACEMENT → MOVE_TO_SHELF

DMC->DDC: move_to_target(shelfB_pose)
DDC->DMC: Result: success

note over DMC: SubState: MOVE_TO_SHELF → PLACE_BOOKS_TO_SHELF

DMC->DAC: place_book(B3, carrier_slot=3, shelfB)
DAC->DMC: Result: success
DMC->AIS: verify_book_position(B3)
AIS->DMC: is_correct=true

note over DMC: SubState: VERIFY_SHELF_PLACEMENT
note over DMC: Check: 남은 도서? 없음

note over DMC: ReshelvingBookSM outcome: 'succeeded'
note over DMC: State: EXECUTING_TASK → IDLE
note over DMC: Battery: 62% (작업 완료, 충전 불필요)

DMC->RCS: Result: success=true, time=425s, distance=55m
```

### 8.5 에러 시나리오: 도서 미발견
```sequence
title 도서 픽업 실패 (도서 미발견)

RCS->DMC: assign_task(PICKUP_BOOK, book_id="MISSING-123")
DMC->RCS: ACCEPTED

note over DMC: State: IDLE → EXECUTING_TASK

DMC->DDC: move_to_target(shelf_pose)
DDC->DMC: Result: success

note over DMC: SubState: MOVE_TO_PICKUP → DETECT_BOOK

DMC->AIS: detect_book("MISSING-123")
note over AIS: 스캔 중...
AIS->DMC: detected=false

note over DMC: WARNING 201: "Book not found, re-scanning"
note over DMC: 1초 대기 후 재시도

DMC->AIS: detect_book("MISSING-123")
AIS->DMC: detected=false

note over DMC: ERROR 301: "Book not found after retries"
note over DMC: PickupBookSM outcome: 'failed'

note over DMC: State: EXECUTING_TASK → IDLE

DMC->RCS: Result: success=false
note over RCS: result_message="Book not found"
note over RCS: error_details="ERROR 301"
```

---

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


---

## 10. 검증 체크리스트

이 설계서가 완성되었는지 확인하는 체크리스트입니다.

### 10.1 시스템 컨텍스트 검증

- [  ] DMC의 역할과 책임이 명확한가?
- [  ] RCS, DDC, DAC, AIS, GUI와의 관계가 정의되었는가?
- [  ] 외부 서비스(STT, TTS, LLM)와의 통신 방법이 명시되었는가?
- [  ] 도비 로봇 하드웨어 구성(내부 운반함 포함)이 반영되었는가?
- [  ] DMC가 관리하는 것과 관리하지 않는 것이 구분되었는가?

### 10.2 컴포넌트 아키텍처 검증

- [  ] TaskManager가 제거되고 current_task만 사용하는가?
- [  ] 5개 주요 컴포넌트(SM Manager, Interface Manager, Battery Manager, State Publisher, Voice Handler)가 정의되었는가?
- [  ] 각 컴포넌트의 역할과 메서드가 명확한가?
- [  ] 컴포넌트 간 의존성이 최소화되었는가?
- [  ] 클래스 다이어그램이 이해하기 쉬운가?

### 10.3 SMACH State Machine 검증

- [  ] Main State가 8개(INITIALIZING, CHARGING, IDLE, LISTENING, EXECUTING_TASK, MOVING_TO_CHARGER, FORCE_MOVE_TO_CHARGER, ERROR)인가?
- [  ] 모든 State 전환 조건이 명확한가?
- [  ] 각 State의 진입/퇴장 동작이 정의되었는가?
- [  ] LISTENING State가 음성 인식을 위해 추가되었는가?
- [  ] Task SM 5개(Pickup, Reshelving, Guiding, Cleaning, Sorting)가 모두 정의되었는가?
- [  ] 각 Task SM의 Sub-State 흐름이 문서(시퀀스 다이어그램)와 일치하는가?
- [  ] 도서 픽업/반납 시나리오가 "내부 운반함(carrier)"을 반영하는가?
- [  ] 배터리 긴급 상황이 모든 State에서 처리 가능한가?

### 10.4 인터페이스 명세 검증

- [  ] RCS ↔ DMC 인터페이스가 정의되었는가? (Action, Service)
- [  ] DMC ↔ DDC 인터페이스가 정의되었는가? (Action, Service, Topic)
- [  ] DMC ↔ DAC 인터페이스가 정의되었는가? (Action, Service)
- [  ] DMC ↔ AIS 인터페이스가 정의되었는가? (Service, Topic)
- [  ] DMC ↔ GUI 인터페이스가 정의되었는가? (Service, Topic)
- [  ] DMC ↔ 음성 서비스 인터페이스가 추가되었는가? (STT, TTS, LLM)
- [  ] 모든 Action에 Goal/Result/Feedback가 정의되었는가?
- [  ] 모든 Service에 Request/Response가 정의되었는가?
- [  ] Topic 메시지 타입과 발행 주기가 명시되었는가?
- [  ] 네임스페이스 처리가 멀티로봇 대응 가능한가?

### 10.5 데이터 흐름 검증

- [  ] 작업 할당 플로우가 명확한가?
- [  ] 배터리 긴급 상황 플로우가 상세한가?
- [  ] 음성 인식 플로우가 추가되었는가?
- [  ] 각 플로우에서 컴포넌트 간 통신이 명확한가?

### 10.6 배터리 관리 검증

- [  ] 배터리 상태(IDLE, CHARGING, DRAINING)가 정의되었는가?
- [  ] 배터리 레벨별 동작 정책이 명확한가?
- [  ] 충전/소모 속도가 설정 가능한가?
- [  ] 임계값(critical=20%, warning=40%, target=80%)이 적절한가?
- [  ] Test GUI에서 배터리 제어가 가능한가?
- [  ] Test Mode로 자동 변화를 중지할 수 있는가?

### 10.7 에러 처리 검증

- [  ] 에러 레벨(INFO/WARNING/ERROR/CRITICAL)이 명확한가?
- [  ] 에러 코드 체계가 정의되었는가?
- [  ] 각 에러에 대한 DMC 동작이 명시되었는가?
- [  ] 타임아웃 정책이 작업별/Sub-State별로 정의되었는가?
- [  ] 재시도 전략(횟수, 간격)이 정의되었는가?
- [  ] 에러 복구 플로우가 이해하기 쉬운가?

### 10.8 시나리오 검증

- [  ] 8개 주요 시나리오가 시퀀스 다이어그램으로 표현되었는가?
  - 도서 픽업 (정상)
  - 배터리 긴급 복귀
  - 음성 인식 길안내
  - 반납 정리 (여러 책장)
  - 도서 미발견 (에러)
- [  ] 정상/예외 경로가 모두 고려되었는가?
- [  ] 시퀀스가 인터페이스 명세와 일치하는가?
- [  ] 음성 인식 시나리오가 LLM 통신을 포함하는가?

### 10.9 테스트 전략 검증

- [  ] 4단계 테스트 레벨(Unit → Module → Integration → System)이 정의되었는가?
- [  ] Test GUI 기능이 충분한가?
  - 배터리 제어
  - 상태 강제 전환
  - 작업 할당
  - 이벤트 시뮬레이션
  - 상태 모니터
  - Timeline
  - Mock 응답 설정
  - Scenario Playback
- [  ] Test GUI UI 레이아웃이 구체적인가?
- [  ] Mock Interface 구현 전략이 명확한가?
- [  ] 테스트 케이스가 충분한가? (50+ 케이스)
  - Main SM: 11개
  - Task SM: 13개
  - Battery: 8개
  - Interface: 12개
  - Integration: 6개

### 10.10 문서 완성도 검증

- [  ] 모든 섹션이 작성되었는가?
- [  ] 다이어그램이 읽기 쉬운가?
- [  ] 표와 리스트가 체계적인가?
- [  ] 코드 스니펫 없이 설계만 명시되었는가?
- [  ] 기술 용어가 일관되게 사용되었는가?
- [  ] v2.0 변경 사항이 명확하게 반영되었는가?
  - TaskManager 제거
  - 음성 인식 추가
  - 도비 내부 운반함 반영
  - 시나리오 정정

---