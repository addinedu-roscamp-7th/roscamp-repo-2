## JAVIS 시스템  소프트웨어 아키텍쳐 설계


### 소프트웨어  역할
---

| 그룹 (대분류) | 장치/서버 | 구성 요소 (ID) | 역할 (여기에 작성) |
| :--- | :--- | :--- | :--- |
| **Client GUIs** | Dobby | `dobby_gui` | 도비 로봇 터치스크린 인터페이스. 길안내 목적지 선택, 도서 픽업 진행상황 표시, 사용자 인터랙션 화면 제공 |
| | Information Desk PC | `information_desk_gui` | 도서관 안내데스크 키오스크. 도서 조회/예약, 좌석 예약, 회원 인증, 보관함 상태 표시 |
| | Admin PC | `admin_gui` | 관리자 모니터링 인터페이스. 로봇 상태/위치 실시간 모니터링, 작업 할당, 시스템 제어, 알림 수신 |
| | Cafe Order PC | `cafe_order_gui` | 카페 주문 키오스크. 메뉴 조회/주문, 주문 상태 표시, 픽업 알림 |
| **Servers** | LLM Server | `llm_service` | 음성 명령 처리 및 대화형 인터페이스 제공. 자연어 이해, 의도 파악, 응답 생성 |
| | JAVIS Server | `robot_control_service` | 로봇 작업 관리 및 스케줄링. 작업 생성/할당/모니터링, 로봇 상태 추적, 작업 우선순위 관리 (RCS) |
| | | `application_service` | 비즈니스 로직 처리 및 데이터 관리. 도서/회원/좌석 정보 관리, GUI-RCS 간 중계, 외부 시스템(ILS) 연동 |
| | | `db` | 시스템 데이터베이스. 도서/회원/좌석/작업 이력 등 영구 데이터 저장 |
| **Robots & Devices**| Dobby | `dobby_main_controller` | 도비 로봇 통합 제어 노드. 상태 관리, 하위 컨트롤러 조율, 작업 실행 오케스트레이션, 의사결정 (DMC) |
| | | `dobby_vision_service` | 도비 비전 AI 처리. 도서/쓰레기/사람 감지, 객체 위치 추정, 추적 대상 등록/추적 (AIS - Arm & Navigation) |
| | | `dobby_arm_controller` | 도비 로봇팔 제어. 매니퓰레이터 동작 계획/실행, 그리퍼 제어, 픽앤플레이스 작업 수행 (DAC) |
| | | `dobby_drive_controller` | 도비 주행 제어. 자율 내비게이션, 경로 계획, 장애물 회피, 사람 추종 주행 (DDC) |
| | | `stt_tts_manager` | 음성 인식 및 합성 관리. 사용자 음성 → 텍스트 변환, LLM 응답 → 음성 출력, 로컬 처리 |
| | Dobby Hardware | `d_depth`, `d_cam` | 도비 비전 센서. Depth 카메라(물체 거리 측정), RGB 카메라(객체 인식용) |
| | | `d_joint` | 도비 로봇팔 액추에이터. 6축 관절 모터 제어 인터페이스 |
| | | `d_wheel`, `d_lidar` | 도비 주행 하드웨어. 구동 휠 모터, LiDAR(2D/3D SLAM 및 장애물 감지) |
| | | `d_mic`, `d_speaker` | 도비 오디오 장치. 마이크(음성 입력), 스피커(TTS 출력 및 알림음) |
| | Kreacher | `kreacher_controller` | 크리처 로봇팔 통합 제어. 음료 제조 시퀀스 실행, 비전 시스템 조율, 작업 상태 보고 |
| | | `kreacher_vision_service` | 크리처 비전 AI 처리. 그리퍼/컵/버튼 좌표 감지 |
| | Kreacher Hardware | `k_joint` | 크리처 로봇팔 액추에이터. 6축 관절 모터 제어 인터페이스 |
| | | `k_depth` | 크리처 비전 센서. Depth 카메라(작업 공간 3D 인식) |
| | Information Desk | `authentication_controller` | 안내데스크 인증 장치 제어. RFID 리더 관리, 회원증 스캔, 보관함 도어락 제어 |
| | Desk Hardware | `desk_rfid`, `door_motor`| 안내데스크 하드웨어. RFID 리더(회원 인증), 보관함 도어 모터(잠금/해제) |
| | Cafe Order Payment | `payment_controller` | 카페 결제 처리. 결제 단말 연동, 거래 승인, 영수증 발행, 주문 확정 트리거 |
| | Cafe Hardware | `cafe_rfid` | 카페 RFID 리더. 회원증 기반 간편 결제 처리 |




### 다이어그램

```mermaid

   graph LR
    %% === 1. Style Definitions (스타일 정의) ===
    classDef yellowBox fill:#fff2cc,stroke:#d6b656,stroke-width:2px
    classDef whiteBox fill:#f9f9f9,stroke:#666,stroke-width:1px
    classDef dbBox fill:#fff2cc,stroke:#d6b656,shape:cylinder
    classDef tcpLink stroke:#82b366,stroke-width:3px,color:#82b366
    classDef ros2Link stroke:#6c8ebf,stroke-width:3px,color:#6c8ebf
    classDef serialLink stroke:#333,stroke-width:3px,color:#333

    %% === 2. Node & Group Definitions (노드 및 그룹 정의) ===

    %% Group 1: Client GUIs (Green Box)
    subgraph G1 [Client GUIs]
        direction TB
        subgraph Dobby_PC [Dobby]
            dobby_gui[Dobby GUI]
        end
        subgraph Info_PC [Information Desk PC]
            info_gui[Information Desk GUI]
        end
        subgraph Admin_PC [Admin PC]
            admin_gui[Admin GUI]
        end
        subgraph Cafe_PC [Cafe Order PC]
            cafe_gui[Cafe Order GUI]
        end
    end

    %% Group 2: Servers (Blue Box)
    subgraph G2 [Servers]
        direction TB
        subgraph LLM_Server [LLM Server]
            llm_svc[LLM Service]
        end
        subgraph JAVIS_Server [JAVIS Server]
            robot_svc[Robot Control Service]
            app_svc[Application Service]
            db[(DB)]
            %% Internal Server Links
            robot_svc --> app_svc
            app_svc --> db
        end
    end

    %% Group 3: Robots & Devices (Orange Box)
    subgraph G3 [Robots & Devices]
        direction TB
        subgraph Dobby_Robot [Dobby]
            dobby_main[Dobby Main Controller]
            dobby_vision[Dobby Vision Service]
            dobby_arm[Dobby Arm Controller]
            dobby_drive[Dobby Drive Controller]
            stt_tts[STT/TTS Manager]
        end
        subgraph Dobby_HW [Dobby Hardware]
            d_depth[DEPTH CAMERA]
            d_cam[CAM]
            d_joint[JOINT MOTOR]
            d_wheel[WHEEL MOTOR]
            d_lidar[LIDAR]
            d_mic[MIC]
            d_speaker[Speaker]
        end
        subgraph Kreacher_Robot [Kreacher]
            kreacher_ctrl[Kreacher Controller]
            kreacher_vision[Kreacher Vision Service]
        end
        subgraph Kreacher_HW [Kreacher Hardware]
            k_joint[JOINT MOTOR]
            k_depth[DEPTH CAMERA]
        end
        subgraph Info_Desk [Information Desk]
            auth_ctrl[Authentication Controller]
        end
        subgraph Desk_HW [Desk Hardware]
            desk_rfid[RFID 리더기]
            door_motor[DOOR MOTOR]
        end
        subgraph Cafe_Payment [Cafe Order Payment]
            payment_ctrl[Payment Controller]
        end
        subgraph Cafe_HW [Cafe Hardware]
            cafe_rfid[RFID 리더기]
        end
    end

    %% === 3. Connection Definitions (연결 관계 정의) ===

    %% TCP (Green Links)
    dobby_gui -- "TCP" --> llm_svc :::tcpLink
    info_gui -- "TCP" --> llm_svc :::tcpLink
    info_gui -- "TCP" --> app_svc :::tcpLink
    admin_gui -- "TCP" --> robot_svc :::tcpLink
    admin_gui -- "TCP" --> app_svc :::tcpLink
    cafe_gui -- "TCP" --> llm_svc :::tcpLink
    cafe_gui -- "TCP" --> app_svc :::tcpLink
    app_svc -- "TCP" --> auth_ctrl :::tcpLink

    %% ROS2 (Blue Links)
    dobby_gui -- "ROS2" --> dobby_main :::ros2Link
    llm_svc -- "ROS2" --> dobby_main :::ros2Link
    llm_svc -- "ROS2" --> stt_tts :::ros2Link
    dobby_main -- "ROS2" --> robot_svc :::ros2Link
    robot_svc -- "ROS2" --> kreacher_ctrl :::ros2Link
    
    %% Internal Dobby (ROS2)
    dobby_main -- "ROS2" --> dobby_vision :::ros2Link
    dobby_main -- "ROS2" --> dobby_arm :::ros2Link
    dobby_main -- "ROS2" --> dobby_drive :::ros2Link
    dobby_main -- "ROS2" --> stt_tts :::ros2Link
    dobby_vision -- "ROS2" --> dobby_arm :::ros2Link
    dobby_vision -- "ROS2" --> dobby_drive :::ros2Link

    %% Internal Kreacher (ROS2)
    kreacher_ctrl -- "ROS2" --> kreacher_vision :::ros2Link

    %% Serial (Black Links)
    info_gui -- "Serial" --> auth_ctrl :::serialLink
    cafe_gui -- "Serial" --> payment_ctrl :::serialLink
    
    %% Dobby HW (Serial)
    d_depth --> dobby_vision :::serialLink
    d_cam --> dobby_vision :::serialLink
    dobby_arm -- "Serial" --> d_joint :::serialLink
    dobby_drive -- "Serial" --> d_wheel :::serialLink
    dobby_drive -- "Serial" --> d_lidar :::serialLink
    d_mic --> stt_tts :::serialLink
    stt_tts -- "Serial" --> d_speaker :::serialLink
    
    %% Kreacher HW (Serial)
    k_depth --> kreacher_vision :::serialLink
    kreacher_ctrl -- "Serial" --> k_joint :::serialLink
    
    %% Desk HW (Serial)
    desk_rfid --> auth_ctrl :::serialLink
    auth_ctrl -- "Serial" --> door_motor :::serialLink
    
    %% Cafe HW (Serial)
    cafe_rfid --> payment_ctrl :::serialLink

    %% === 4. Apply Styles to Nodes (노드에 스타일 적용) ===
    class dobby_gui,info_gui,admin_gui,cafe_gui yellowBox
    class llm_svc,robot_svc,app_svc yellowBox
    class dobby_main,dobby_vision,dobby_arm,dobby_drive,stt_tts yellowBox
    class kreacher_ctrl,kreacher_vision yellowBox
    class auth_ctrl,payment_ctrl yellowBox
    class db dbBox
    class d_depth,d_cam,d_joint,d_wheel,d_lidar,d_mic,d_speaker whiteBox
    class k_joint,k_depth whiteBox
    class desk_rfid,door_motor whiteBox
    class cafe_rfid whiteBox
    
```