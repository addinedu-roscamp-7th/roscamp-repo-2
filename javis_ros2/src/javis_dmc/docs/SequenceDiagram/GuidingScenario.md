## 길안내 시나리오 (음성 호출 + GUI 목적지 선택)

### 1. 음성 호출 및 LISTENING 상태 진입

```
sequenceDiagram
    actor User as 사용자
    participant Mic as 로봇 마이크
    participant VRC as voice_recognition_controller
    participant DMC as Dobby Main Controller
    participant DDC as Dobby Drive Controller
    participant VoiceAPI as voice_api_service
    participant Speaker as 로봇 스피커

    User->>Mic: "도비야"
    Mic->>VRC: 아날로그 음성 스트림
    activate VRC
    VRC->>VRC: 로컬 WWD 감지
    VRC->>DMC: set_listening_mode(True) [ROS2 Service]
    deactivate VRC

    activate DMC
    DMC->>DDC: 주행 일시정지 명령 [ROS2 Action Cancel]
    DMC->>DMC: 현재 작업 일시정지<br/>State: ROAMING/IDLE → LISTENING<br/>20초 타이머 시작
    DMC-->>VRC: {success: true}
    deactivate DMC

    activate VRC
    VRC->>VoiceAPI: 안내 시작 멘트 요청 [HTTP]
    VoiceAPI-->>VRC: TTS 오디오("네, 무엇을 도와드릴까요?")
    VRC->>Speaker: 오디오 재생
    deactivate VRC
    Speaker-->>User: 음성 출력
```

### 2. 자연어 이해 및 길안내 작업 생성

```
sequenceDiagram
    actor User as 사용자
    participant Mic as 로봇 마이크
    participant VRC as voice_recognition_controller
    participant VoiceAPI as voice_api_service
    participant DMC as Dobby Main Controller
    participant RCS as Robot Control Service
    participant Speaker as 로봇 스피커

    User->>Mic: "화장실 어디 있어?"
    Mic->>VRC: 음성 스트림
    activate VRC
    VRC->>VoiceAPI: 오디오 청크 업로드 [HTTP Stream]
    deactivate VRC

    activate VoiceAPI
    VoiceAPI->>VoiceAPI: STT + Intent 분석<br/>intent="navigation", target="화장실"
    VoiceAPI-->>VRC: {tts_audio: "화장실로 안내해드릴까요?", require_confirmation: true}
    deactivate VoiceAPI

    activate VRC
    VRC->>Speaker: "화장실로 안내해드릴까요?" TTS 재생
    deactivate VRC
    Speaker-->>User: 음성 출력

    User->>Mic: "응"
    Mic->>VRC: 음성 스트림
    activate VRC
    VRC->>VoiceAPI: 오디오 청크 업로드 [HTTP Stream]
    deactivate VRC

    activate VoiceAPI
    VoiceAPI->>VoiceAPI: 의도 파싱<br/>intent="confirmation", confirmed=true
    VoiceAPI-->>VRC: {tts_audio: "앞으로 나와 주시면 안내를 시작할게요", require_task: true, task_payload:{destination:"화장실"}}
    deactivate VoiceAPI

    activate VRC
    VRC->>Speaker: "앞으로 나와 주시면 안내를 시작할게요" TTS 재생
    deactivate VRC
    Speaker-->>User: 음성 출력

    activate VRC
    VRC->>DMC: submit_voice_task(destination="화장실") [ROS2 Service]
    deactivate VRC

    activate DMC
    DMC->>DMC: 서비스 요청 수신 → request_task 준비
    DMC->>RCS: create_task(guide_person, destination) [ROS2 Service]
    deactivate DMC

    activate RCS
    RCS->>RCS: 작업 검증 및 큐 추가
    RCS-->>DMC: assign_task{id, destination} [ROS2 Action Goal]
    deactivate RCS

    activate DMC
    DMC->>DMC: State: LISTENING → GUIDING<br/>listening 타이머 종료
    DMC->>VRC: set_listening_mode(False) 응답 [ROS2 Service]
    DMC-->>RCS: 작업 수락 {accepted: true}
    deactivate DMC
```

### 3-A. GUI 기반 목적지 입력 (새로운 설계)

```
sequenceDiagram
    actor User as 사용자
    participant GUI as Dobby Test GUI
    participant DMC as Dobby Main Controller
    participant NAV as Navigation

    Note over DMC: State: IDLE 또는 ROAMING

    User->>GUI: 지도에서 목적지 터치 (예: "화장실")
    activate GUI
    GUI->>DMC: QueryLocationInfo("화장실") [ROS2 Service]
    deactivate GUI

    activate DMC
    DMC->>DMC: library_locations.yaml 검색
    DMC-->>GUI: Response {found: true, pose: {x,y,theta}, description}
    deactivate DMC

    activate GUI
    GUI->>User: 확인 팝업 표시<br/>"화장실로 안내를 시작하시겠습니까?"
    deactivate GUI

    User->>GUI: "확인" 버튼 터치
    activate GUI
    GUI->>DMC: RequestGuidance("화장실", pose, "gui") [ROS2 Service]
    deactivate GUI

    activate DMC
    DMC->>DMC: State: IDLE/ROAMING → WAITING_DEST_INPUT → GUIDING
    DMC->>NAV: GuidePerson Action Goal {dest_location: pose}
    DMC-->>GUI: Response {success: true, task_id, message}
    deactivate DMC

    GUI->>User: "안내 시작됨" 상태 표시
```

### 3-B. 음성 기반 목적지 입력 (새로운 설계)

```
sequenceDiagram
    actor User as 사용자
    participant VRC as Voice Recognition Controller
    participant DMC as Dobby Main Controller
    participant NAV as Navigation
    participant Speaker as 로봇 스피커

    Note over DMC: State: LISTENING

    User->>VRC: "화장실로 안내해줘"
    activate VRC
    VRC->>VRC: 음성 인식 → "화장실" 추출
    VRC->>DMC: QueryLocationInfo("화장실") [ROS2 Service]
    deactivate VRC

    activate DMC
    DMC->>DMC: library_locations.yaml 검색
    DMC-->>VRC: Response {found: true, pose: {x,y,theta}, description}
    deactivate DMC

    activate VRC
    VRC->>DMC: RequestGuidance("화장실", pose, "voice") [ROS2 Service]
    deactivate VRC

    activate DMC
    DMC->>DMC: State: LISTENING → GUIDING
    DMC->>NAV: GuidePerson Action Goal {dest_location: pose}
    DMC-->>VRC: Response {success: true, task_id, message}
    deactivate DMC

    activate VRC
    VRC->>Speaker: TTS: "화장실로 안내를 시작합니다"
    deactivate VRC
    Speaker-->>User: 음성 출력
```

### 3-C. (DEPRECATED) 구 설계: 60초 타이머 기반 목적지 선택

**이 섹션은 구 설계로 제거되었습니다.**
- **문제점**: DMC가 destination_session으로 60초 동안 대기하는 설계
- **변경**: GUI/VRC가 QueryLocationInfo와 RequestGuidance로 직접 좌표 제공
- **상태 변경**: SELECT_DEST 서브 상태 제거 → WAITING_DEST_INPUT 메인 상태 추가
- **참고 문서**: `GuidanceFlowRefactor.md`, `DevelopmentPlan.md` Section 4.5.2

### 4. 피안내자 인식 및 추적 전환

```
sequenceDiagram
    participant DMC as Dobby Main Controller
    participant DVS as Dobby Vision Service
    participant GUI as Dobby GUI

    activate DMC
    DMC->>DVS: change_tracking_mode(REGISTRATION_MODE) [ROS2 Service]
    deactivate DMC

    GUI->>User: "카메라 앞에 서 주세요"

    loop 피안내자 등록 반복
        DVS->>DVS: 얼굴 추적 & 특징 추출
        DVS-->>DMC: tracking/status {person_detected, tracking_id} [ROS2 Topic]
    end

    activate DMC
    DMC->>DVS: change_tracking_mode(TRACKING_MODE) [ROS2 Service]
    DMC->>DMC: GUIDING(NAVIGATION 단계) 전환
    deactivate DMC
```

### 5. 목적지 안내 주행 및 종료

```
sequenceDiagram
    participant DMC as Dobby Main Controller
    participant DDC as Dobby Drive Controller
    participant DVS as Dobby Vision Service
    participant RCS as Robot Control Service

    activate DMC
    DMC->>DDC: 목적지 이동 명령 {destination} [ROS2 Action Goal]
    deactivate DMC

    activate DDC
    DDC->>DDC: Nav2 경로 계획 및 주행
    DDC-->>DMC: 주행 피드백 (현재 위치, 진행률) [ROS2]
    deactivate DDC

    loop 주기적 상태 확인
        activate DVS
        DVS->>DVS: 피안내자 추적 / 장애물 감지
        DVS-->>DMC: 추적 및 장애물 이벤트 [ROS2 Topic]
        deactivate DVS
    end

    opt 장애물 감지 시
        DMC->>DDC: 회피/정지 명령 [ROS2]
        DDC-->>DMC: 처리 결과 보고
    end

    opt 피안내자 이탈 시
        DMC->>DDC: 정지 명령
        DMC->>DMC: 30초 응답 대기<br/>미응답 시 작업 종료 판단
        DMC->>DDC: 대기 지점 이동 또는 작업 취소
    end

    DDC-->>DMC: 목적지 도착 알림 [ROS2]
    activate DMC
    DMC->>RCS: 작업 완료 보고 {task_id, result} [ROS2]
    DMC->>DDC: 정지 및 대기 명령
    DMC->>DMC: State: GUIDING → ROAMING/IDLE 또는 MOVING_TO_CHARGER (배터리≤40%)
    deactivate DMC
```

### 6. 정리

- **음성 호출**: `set_listening_mode(True)`를 사용하여 LISTENING 상태(10)와 20초 타이머를 준수한다.
- **목적지 입력 (새 설계)**:
  - GUI/VRC가 `QueryLocationInfo` 서비스로 좌표 조회
  - `RequestGuidance` 서비스로 DMC에 좌표 전달
  - DMC는 즉시 GuidePerson Action Goal 생성 (destination_session 제거)
  - **State 전환**: IDLE/LISTENING → **WAITING_DEST_INPUT(11)** → GUIDING(6)
- **피안내자 인식**: GUIDING 내부 서브 상태로 SCAN_USER(110) → GUIDING_TO_DEST(111)
- **타임아웃 제거**: 60초 목적지 선택 타이머 삭제됨 (구 설계 deprecated)
- **상태 복귀**: 작업 완료 시 GUIDING → IDLE/ROAMING, 배터리 ≤40% 시 MOVING_TO_CHARGER
- **에러 처리**: 이탈, 배터리 부족, 충돌 시 Action Result로 RCS에 전달

**주요 변경사항 (v6.0 → v7.0)**:
1. SELECT_DEST 서브 상태(109) 제거
2. WAITING_DEST_INPUT 메인 상태(11) 추가
3. ROAMING 상태 번호 11 → 12로 변경
4. destination_session 로직 제거
5. QueryLocationInfo, RequestGuidance 서비스 추가
6. library_locations.yaml 단일 소스 사용
