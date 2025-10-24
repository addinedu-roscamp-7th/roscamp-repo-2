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

### 3. 목적지 입력 및 피안내자 인식 준비

```
sequenceDiagram
    actor User as 사용자
    participant GUI as Dobby GUI
    participant DMC as Dobby Main Controller
    participant RCS as Robot Control Service

    GUI->>DMC: 지도 승인 요청 [ROS2]
    activate DMC
    DMC->>DMC: 내부 상태 갱신<br/>GUIDING(DESTINATION_SELECTION 단계)<br/>60초 타이머 시작
    DMC-->>GUI: 승인 완료 & 지도 데이터 [ROS2]
    deactivate DMC

    alt 60초 내 목적지 선택
        User->>GUI: 목적지 버튼 선택
        activate GUI
        GUI->>DMC: 목적지 저장 요청{destination} [ROS2]
        deactivate GUI

        activate DMC
        DMC->>DMC: 목적지 캐시<br/>GUIDING(PASSENGER_IDENTIFICATION 단계)
        DMC->>RCS: 로봇 상태 업데이트 {state: GUIDING} [ROS2 Topic]
        DMC-->>GUI: 목적지 저장 완료 [ROS2]
        deactivate DMC

        GUI->>GUI: 피안내자 인식 화면 표시
    else 60초 타이머 만료
        activate DMC
        DMC->>DMC: 타임아웃 발생<br/>GUIDING 작업 중단 결정
        DMC->>RCS: guide_person result{status: aborted, reason: "destination_timeout"} [ROS2 Action Result]
        DMC->>DMC: State: GUIDING → ROAMING/IDLE 또는 MOVING_TO_CHARGER
        DMC-->>GUI: 시간 초과 안내 요청 [ROS2]
        DMC->>RCS: 로봇 상태 업데이트 {state: ROAMING/IDLE} [ROS2 Topic]
        deactivate DMC
    end
```

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

- 음성 호출 단계에서 `set_listening_mode(True)`를 사용하여 StateDefinition의 LISTENING 상태와 20초 타이머를 준수한다.
- 길안내 확정 시 `request_task` → `create_task` 흐름을 통해 SoftwareArchitecture의 노드/서비스 연결을 그대로 따른다.
- GUIDING 내부 단계(목적지 입력 → 피안내자 인식 → 이동)는 StateDefinition의 Task State(GUIDING)를 기준으로 단계별로 기술하였다.
- 타임아웃, 배터리 조건, 이탈 처리 등은 StateDefinition의 규칙에 맞춰 ROAMING/IDLE 또는 MOVING_TO_CHARGER로 복귀하도록 명시한다.
- GUI 타이머 만료 시 ROS2 Action Result를 통해 `guide_person` 작업을 `aborted` 상태로 RCS에 전달해 작업 큐 일관성을 유지한다.
