AVIS 음성 인식 통합 시스템 아키텍처 v3.0 (최종 확정안)

📋 목차

시스템 개요
소프트웨어 컴포넌트 역할
시스템 아키텍처 다이어그램
통신 프로토콜 매트릭스
계층 구조


## 1. 시스템 개요
JAVIS(Library Automation System)는 도서관 자동화를 위한 통합 로봇 시스템으로, 음성 인식 기반 사용자 상호작용, 도서 관리, 좌석 정리, 음료 제조 등의 서비스를 제공합니다.
핵심 구성:

2대의 Dobby 로봇: 도서 픽업/반납, 길안내, 좌석 정리, 서가 정리
Kreacher 로봇: 도서관 카페 음료 제조
중앙 서버: 작업 스케줄링, 데이터 관리, LLM 기반 대화 처리
다중 GUI: 사용자/관리자 인터페이스

주요 특징:

음성 인식 상호작용: Wake Word("도비야") 기반 자연어 대화
자율 이동: 웨이포인트 순찰 및 동적 경로 계획
작업 스케줄링: 우선순위 기반 다중 로봇 작업 할당
실시간 모니터링: 로봇 상태, 작업 진행률, 배터리 관리

## 2. 소프트웨어 컴포넌트 역할

### 2.1 Client GUIs
| 구성 요소                    | 위치                  | 역할 및 책임    |
| ------------------------ | ------------------- | ------------- |
| **dobby_gui**            | Dobby Robot         | - 도비 로봇 터치스크린 인터페이스<br>  - 길안내 목적지 선택 및 경로 표시<br>  - 도비 상태 실시간 표시 <br>  - 사용자 인터랙션 화면 제공 (음성 인식 시작 버튼 등)<br>  - 피안내자 인식 안내 화면<br>  - 로봇 상태 및 배터리 표시<br>**통신:** ROS2 ↔ DMC                                                                                                              |
| **information_desk_gui** | Information Desk PC | - 도서관 안내데스크 키오스크<br>  - 도서 검색 및 조회 (제목, 저자, ISBN)<br>  - 도서 예약 및 픽업 요청<br>  - 좌석 예약 및 현황 조회<br>  - 회원 인증 (RFID 스캔)<br>  - 픽업 보관함 상태 표시 및 개폐 제어<br>**통신:** TCP ↔ App Service, Serial ↔ Authentication Controller                                                                            |
| **admin_gui**            | Admin PC            | - 관리자 모니터링 및 제어 인터페이스<br>  - 로봇 모드 관리 (standby/autonomy)<br>  - 로봇 상태/위치 실시간 모니터링 (지도 기반)<br>  - 작업 큐 및 진행 상황 모니터링<br>  - 로봇 모드 전환 (대기/자율이동)<br>  - 긴급 제어 (정지, 작업 취소, IDLE 복귀)<br>  - 시스템 통계 및 작업 이력 조회<br>  - 알림 수신 (에러, 배터리 경고 등)<br>**통신:** TCP ↔ App Service & RCS, ROS2 ↔ DMC (긴급 제어) |
| **cafe_order_gui**       | Cafe Order PC       | - 카페 주문 키오스크<br>  - 메뉴 조회 및 주문<br>  - 주문 상태 표시 (대기, 제조 중, 완료)<br>  - 픽업 알림<br>  - 결제 처리 (RFID 간편결제)<br>**통신:** TCP ↔ App Service, Serial ↔ Payment Controller  |


### 2.2 Servers

| 구성 요소                           | 위치                 | 역할 및 책임             |
| ------------------------------- | ------------------ | -------------------------------- |
| **llm_service**                 | LLM Server (독립 서버) | - 음성 명령 처리 및 대화형 인터페이스 제공<br>  - 자연어 이해 및 의도 파악 (Intent Parsing)<br>  - 대화 문맥 관리 (로봇별 세션 유지)<br>  - 자연어 응답 생성<br>  - OpenAI API 또는 로컬 LLM 호출<br>  - DMC에 작업 요청 (필요 시)<br>  - Application Service 정보 조회 (Phase 2)<br>**타입:** HTTP REST Server (FastAPI/Flask)<br>**통신:** HTTP ↔ STT/TTS Manager, TCP ↔ App Service (Phase 2) |
| **robot_control_service (RCS)** | JAVIS Server       | - 로봇 작업 관리 및 스케줄링<br>  - 작업 생성, 검증 및 큐 관리 (우선순위 기반)<br>  - 작업 할당 및 로봇 선택 (가용성, 배터리, 위치 기반)<br>  - 로봇 상태 모니터링 (Main/Sub State, 배터리, 위치)<br>  - 작업 실패 시 재할당<br>  - 작업 이력 및 통계 관리<br>  - Admin GUI에 실시간 상태 발행<br>**타입:** ROS2 Python Node<br>**통신:** ROS2 Action/Topic ↔ DMC, TCP ↔ App Service    |
| **application_service**         | JAVIS Server       | - 비즈니스 로직 처리 및 데이터 관리<br>  - 도서 정보 관리 (CRUD, 위치, 재고)<br>  - 회원 정보 관리 (인증, 대출 이력)<br>  - 좌석 예약 및 현황 관리<br>  - 도서관 공간 정보 제공 (시설 위치, 좌표)<br>  - GUI 요청 처리 및 응답<br>  - RCS 작업 검증 지원<br>  - LLM Service 정보 제공 (Phase 2)<br>  - 외부 시스템(ILS) 연동<br>  - 작업 이력 로깅<br>**타입:** TCP HTTP Server (FastAPI / Spring Boot)<br>**통신:** TCP ↔ GUIs, RCS, LLM Service / SQL ↔ DB                                                                    |
| **db**                          | JAVIS Server       | - 시스템 데이터베이스<br>  - 도서/회원/좌석 영구 데이터 저장<br>  - 작업 이력 및 통계 저장<br>  - 위치 정보 (책장, 시설) 저장<br>  - 트랜잭션 관리 및 무결성 보장<br>**타입:** PostgreSQL / MySQL<br>**통신:** SQL ↔ App Service       |

### 2.3 Dobby Robot

| 구성 요소                            | 타입               | 역할 및 책임    |
| -------------------------------- | ---------------- | --------------------------------------------- |
| **dobby_main_controller (DMC)**  | ROS2 Python Node | - 도비 로봇 통합 제어 및 오케스트레이션<br>  - Main/Sub State 관리 (SMACH 기반 State Machine)<br>  - 작업 실행 (Executor 패턴)<br>  - 하위 컨트롤러 조율 (DDC, DAC, DVS)<br>  - 음성 세션 상태 관리 (listening_mode)<br>  - 배터리 자동 관리 및 충전 제어<br>  - RCS 작업 할당 수락 (Action Server)<br>  - STT/TTS 상태 전환,작업  요청 수락<br>  - 긴급 제어 처리 (정지, 취소, 복귀)<br>  - 로봇 상태 실시간 발행 (10Hz)<br>  - 지원 작업: Pickup Book, Reshelving Book, Guide Person, Clean Seat, Sorting Shelves<br>  - 길안내 작업 요청(rcs) <br>**통신:** ROS2 ↔ 하위 컨트롤러, RCS, STT/TTS, GUI |
| **stt_tts_manager**              | ROS2 Python Node | - 음성 인식 및 합성 관리<br>  - 마이크 입력 → 텍스트 변환 (STT)<br>  - Wake Word 감지 ("도비야", 로컬 처리)<br>  - DMC 상태 모니터링 (음성 인식 활성화 조건 판단)<br>  - DMC listening 모드 제어 (Service Call)<br>  - TTS 출력 및 타임아웃 관리 (20초)<br>  - 작업관련 llm service에서 수신 후 dmc에 전달 (Ros2 Service) <br>  - 세션 종료 처리<br>**통신:** Serial ↔ d_mic & d_speaker, HTTP ↔ LLM Service, ROS2 ↔ DMC       |
| **dobby_vision_service (DVS)**   | ROS2 Python Node | - 객체 감지 및 인식 (도서, 쓰레기, 사람)<br>  - 6D Pose Estimation (도서 위치 추정)<br>  - 피안내자 등록 및 추적<br>  - 장애물 감지 (동적/정적)<br>  - 책장/보관함 위치 식별<br>  - DMC에 피안내자 등록,추적모드 변경 요청 받음 <br>  - DAC에 객체 좌표 제공 / DDC에 장애물 정보 제공<br>**통신:** Serial ↔ Camera & Depth Camera, ROS2 ↔ DAC & DDC      |
| **dobby_arm_controller (DAC)**   | ROS2 Python Node | - 매니퓰레이터 동작 계획 및 실행<br>  - 그리퍼 제어 (개폐, 파지력 조절)<br>  - 픽앤플레이스 작업 수행 (도서 픽업/배치, 쓰레기 수거)<br>  - DVS 좌표 기반 정밀 제어 및 충돌 회피<br>  - 관측 자세 제어<br>**통신:** Serial ↔ Robot Arm, ROS2 ↔ DMC & DVS    |
| **dobby_drive_controller (DDC)** | ROS2 Python Node | - 자율 내비게이션 (Nav2 기반)<br>  - 경로 계획 및 실행<br>  - 장애물 회피 (동적/정적)<br>  - 사람 추종 주행 (Guide Navigation)<br>  - 웨이포인트 순찰 (AUTONOMY 모드)<br>  - SLAM 및 Localization, Docking<br>  - 책장, 반납대 , 보관함 근접 시 도킹 모드  (충전소)<br>**통신:** Serial ↔ Wheels & LiDAR & Depth Camera, ROS2 ↔ DMC & DVS    |




### 2.4 Dobby Hardware
```
d_mic (Microphone) — Hardware : 사용자 음성 입력 캡처
d_speaker (Speaker) — Hardware : TTS 음성 출력 및 알림음 재생
d_cam (Camera1, RGB) — Hardware : 객체 인식(도서, 사람, 쓰레기)용 RGB 카메라
d_depth (RGBDCamera1, 작업용) — Hardware : 로봇팔 작업용 거리 측정 및 3D 인식
RGBDCamera2 (주행용) — Hardware : 주행 장애물 감지 및 사람 추적
d_joint (Arm1, 6축) — Hardware : 도서 픽업, 쓰레기 수거 등 매니퓰레이션
d_wheel — Hardware : 차동 구동 휠 모터
d_lidar — Hardware : 2D/3D SLAM 및 장애물 감지
```

### 2.5 Kreacher Robot

**kreacher_controller (KC)** — ROS2 Python Node 
- 음료 제조 시퀀스 실행
- 비전 시스템 조율(그리퍼, 컵, 버튼 좌표 획득)
- 작업 상태를 RCS로 보고
- 정밀 동작 제어(컵 배치, 버튼 누르기)
- 통신: Serial ↔ k_joint, ROS2 ↔ KVS, RCS
- kreacher_vision_service (KVS) — ROS2 Python Node
- 그리퍼, 컵, 버튼 좌표 감지
- KC에 좌표 정보 제공
- 통신: Serial ↔ k_depth, Camera1 / ROS2 ↔ KC

### 2.6 Kreacher Hardware
```
k_joint (Arm2, 6축) — Hardware : 음료 제조용 로봇팔
k_depth (Depthcamera1) — Hardware : 객체 인식, 작업 공간 3D 인식(컵, 그리퍼, 버튼 위치)
```

### 2.7 Information Desk
**authentication_controller** — ROS2/Embedded Node
- RFID 리더 관리
- 회원증 스캔 및 ID 전송
- 보관함 도어락 제어(개폐)
- App Service에 인증 요청
- 통신: Serial ↔ desk_rfid, door_motor / TCP ↔ App Service

### 2.8 Desk Hardware
```
desk_rfid — Hardware : 회원 인증용 RFID 리더(회원증 태깅)
door_motor — Hardware : 보관함 잠금/해제 모터
```

### 2.9 Cafe Order Payment
**payment_controller** — Embedded Controller
- 결제 단말 연동
- RFID 간편 결제 처리
- 거래 승인 및 영수증 발행
- 주문 확정 트리거
- App Service로 결제 정보 전송
- 통신: Serial ↔ cafe_rfid / TCP ↔ App Service

### 2.10 Cafe Hardware
```
cafe_rfid — Hardware : 회원증 기반 간편 결제용 RFID 리더
```

## 3. 시스템 아키텍처 다이어그램
```
mermaid
    graph TB
        %% 스타일 정의
        classDef userNode fill:#e1f5ff,stroke:#0288d1,stroke-width:2px
        classDef adminNode fill:#e1f5ff,stroke:#0288d1,stroke-width:2px
        classDef guiNode fill:#fff9c4,stroke:#f57f17,stroke-width:2px
        classDef serverNode fill:#c8e6c9,stroke:#388e3c,stroke-width:2px
        classDef robotNode fill:#ffccbc,stroke:#e64a19,stroke-width:2px
        classDef hwNode fill:#f5f5f5,stroke:#757575,stroke-width:1px
        classDef aiNode fill:#e1bee7,stroke:#8e24aa,stroke-width:2px
        
        %% 사용자
        User[👤 사용자]:::userNode
        Admin[👨‍💼 관리자]:::adminNode
        
        %% Client GUIs
        subgraph GUIs[Client GUIs]
            DobbyGUI[Dobby GUI]:::guiNode
            InfoGUI[Information Desk GUI]:::guiNode
            AdminGUI[Admin GUI]:::guiNode
            CafeGUI[Cafe Order GUI]:::guiNode
        end
        
        %% Servers
        subgraph Servers[JAVIS Servers]
            subgraph LLMServer[LLM Server]
                LLMService[LLM Service<br/>HTTP REST Server<br/>대화 처리]:::serverNode
            end
            
            RCS[Robot Control Service<br/>ROS2 Node<br/>작업 스케줄링]:::robotNode
            AppService[Application Service<br/>TCP Server<br/>데이터 관리]:::serverNode
            DB[(Database<br/>PostgreSQL)]:::serverNode
        end
        
        %% Dobby Robot
        subgraph DobbyRobot[Dobby Robot - ROS2 Domain]
            DMC[Dobby Main Controller<br/>ROS2 Node<br/>통합 제어]:::robotNode
            STTTTS[STT/TTS Manager<br/>ROS2 Node<br/>음성 입출력]:::robotNode
            DVS[Dobby Vision Service<br/>ROS2 Node<br/>객체 인식]:::robotNode
            DAC[Dobby Arm Controller<br/>ROS2 Node<br/>로봇팔 제어]:::robotNode
            DDC[Dobby Drive Controller<br/>ROS2 Node<br/>주행 제어]:::robotNode
        end

        %% Kreacher Robot
        subgraph KreacherRobot[Kreacher Robot - ROS2 Domain]
            KC[Kreacher Controller<br/>ROS2 Node<br/>음료 제조]:::robotNode
            KVS[Kreacher Vision Service<br/>ROS2 Node<br/>좌표 감지]:::robotNode
        end
        
        %% Hardware
        subgraph Hardware[Dobby Hardware]
            Mic[🎤 Microphone]:::hwNode
            Speaker[🔊 Speaker]:::hwNode
            Camera1[📷 Camera1<br/>RGB]:::hwNode
            Depthcamera2[📷 RGBDCamera2<br/>주행용]:::hwNode
            Depthcamera1[📷 RGBDCamera1<br/>작업용]:::hwNode
            Arm1[🦾 Robot Arm1<br/>6축]:::hwNode
            Arm2[🦾 Robot Arm2<br/>6축]:::hwNode
            Wheels[⚙️ Wheels<br/>차동구동]:::hwNode
        end
        
        subgraph KreacherHW[Kreacher Hardware]
            KCamera[📷 Camera1]:::hwNode
            KDepth[📷 Depthcamera1]:::hwNode
            KArm[🦾 Arm2]:::hwNode
        end
        
        %% 연결 관계
        
        %% 사용자 ↔ GUI
        User -->|음성 입력| Mic
        Speaker -->|음성 출력| User
        User -->|터치| DobbyGUI
        User -->|터치| InfoGUI
        User -->|터치| CafeGUI
        Admin -->|터치| AdminGUI
        
        %% 하드웨어 ↔ 소프트웨어
        Mic -->|Serial| STTTTS
        STTTTS -->|Serial| Speaker
        Camera1 -->|Serial| DVS
        Depthcamera1 -->|Serial| DVS
        Depthcamera2 -->|Serial| DDC
        Arm1 -->|Serial| DAC
        Wheels -->|Serial| DDC
        
        %% Kreacher Hardware
        KCamera -->|Serial| KVS
        KDepth -->|Serial| KVS
        KArm -->|Serial| KC
        
        %% 음성 처리 핵심 흐름 (강조)
        STTTTS <-->|HTTP<br/>REST API| LLMService
        STTTTS <-->|ROS2<br/>Service/Topic| DMC
        
        %% LLM 내부
        LLMService <-.->|HTTPS<br/>API Call| LLMModel
        LLMService <-.->|TCP<br/>Phase 2| AppService
        
        %% Dobby Robot 내부 (ROS2)
        DMC <-->|ROS2| DAC
        DMC <-->|ROS2| DDC
        DVS <-->|ROS2| DAC
        DVS <-->|ROS2| DDC
        
        %% Kreacher Robot 내부 (ROS2)
        KC <-->|ROS2| KVS
        
        %% 작업 관리
        DMC <-->|ROS2<br/>Action/Topic| STTTTS
        DMC <-->|ROS2<br/>Action/Topic| RCS
        KC <-->|ROS2<br/>Action/Topic| RCS
        RCS <-->|TCP| AppService
        AppService <-->|SQL| DB
        
        %% GUI 연결
        DobbyGUI <-->|ROS2| DMC
        InfoGUI <-->|TCP| AppService
        AdminGUI <-->|TCP| AppService
        AdminGUI <-->|TCP| RCS
        AdminGUI <-->|ROS2<br/>긴급 제어, 로봇 모드 전환(standby/roaming)| DMC
        CafeGUI <-->|TCP| AppService
        
        %% 스타일 적용
        linkStyle 14,15,16 stroke:#e91e63,stroke-width:4px
        linkStyle 17,18 stroke:#9c27b0,stroke-width:2px,stroke-dasharray:5
```
---

## 4. 통신 프로토콜 매트릭스

| 통신 구간                 | 프로토콜          | 데이터 타입        | 용도            | 비고            |
| ------------------------- | ----------------------------- | ------------------------ | ------------------------ | ------------------ |
| **User ↔ Hardware**       | Physical | Audio / Touch | 사용자 입력 및 출력 | -           |
| **Hardware ↔ Robot SW**   | Serial | Binary | 센서 / 액추에이터 제어 | UART, USB          |
| **Dobby Robot 내부**        | ROS2  | Topic / Service / Action | 실시간 제어 및 통신  | DDS 기반  |
| **Kreacher Robot 내부**     | ROS2  | Topic / Service / Action | 실시간 제어 및 통신  | DDS 기반  |
| **STT/TTS ↔ LLM Service** | HTTP REST  | JSON  | 음성 → 텍스트 변환, 의도 파싱 요청/응답 | 동기 방식, 5초 타임아웃     |
| **STT/TTS ↔ DMC**         | ROS2 Service / Topic  | -  | Listening 모드 제어, 상태 수신 , 파싱된 작업 요청  | -  |
| **DMC ↔ RCS**             | ROS2 Service / Action / Topic | JSON  | 작업 생성·할당, 상태 보고  | -  |
| **KC ↔ RCS**              | ROS2 Action / Topic  | JSON  | 음료 제조 작업 할당  | -  |
| **INFO GUI, ORDER GUI ↔ App Service**     | TCP HTTP  | JSON  | 데이터 조회 및 수정  | REST API  |
| **Admin GUI ↔ RCS**             | TCP  | JSON  | 작업 상태 모니터링  | -  
| **Admin GUI ↔ DMC**       | ROS2 Service  | JSON  | 긴급 제어(정지, 취소 등)  | -   |
| **LLM ↔ App Service**     | TCP HTTP  | JSON  | 정보 조회 (Phase 2)  | -    |
| **App Service ↔ DB**      | SQL  | SQL Query   | 데이터 영속화   | PostgreSQL / MySQL |
| **RCS ↔ App Service**     | TCP HTTP   | JSON   | 작업 검증, 이력 로깅   | -  |



## 5. 계층 구조
```┌──────────────────────────────────────────────────────────┐
│                Layer 0: User Interface                   │
│  • 음성 명령 (Wake Word "도비야", 자연어 대화)                 │
│  • 터치 입력 (GUI 버튼, 메뉴 선택)                            │
│  • 시각적 피드백 (화면 표시, 음성 출력)                         │
└──────────────────────────────────────────────────────────┘
                          ↓
┌──────────────────────────────────────────────────────────┐
│         Layer 1: Presentation (Client GUIs)              │
│  • Dobby GUI  · 터치 스크린 · 경로/진행상황 · 상태/배터리       │
│  • Information Desk GUI  · 도서/좌석 · 예약 · 인증             │
│  • Admin GUI  · 모니터링 · 모드 전환 · 긴급 제어              │
│  • Cafe Order GUI  · 메뉴/주문 · 결제 · 픽업 알림              │
└──────────────────────────────────────────────────────────┘
                          ↓
┌──────────────────────────────────────────────────────────┐
│   Layer 2: Application Services (Business Logic)         │
│  • LLM Service  · 의도 파싱 · 대화 관리 · 응답 생성            │
│  • RCS  · 작업 생성/검증 · 우선순위 큐 · 로봇 할당             │
│  • Application Service  · 도서/회원/좌석/공간 데이터           │
│                         · 외부 ILS · 로그/이력                │
└──────────────────────────────────────────────────────────┘
                          ↓
┌──────────────────────────────────────────────────────────┐
│   Layer 3: Robot Coordination (Main Controllers)         │
│  • DMC  · 통합 오케스트레이션 · 상태/모드 관리 · 액션 수락     │
│  • KC   · 음료 제조 시퀀스 · RCS 연동                        │
│  • STT/TTS Manager  · 세션/타임아웃 · listening 모드          │
└──────────────────────────────────────────────────────────┘
                          ↓
┌──────────────────────────────────────────────────────────┐
│  Layer 4: Device Control (Specialized Controllers)       │
│  • Vision Service  · 객체 인식 · 포즈 추정 · 추적              │
│  • Arm Controller  · 플래닝 · 파지력 · 충돌 회피              │
│  • Drive Controller · Nav2 · SLAM/Localization · 도킹        │
└──────────────────────────────────────────────────────────┘
                          ↓
┌──────────────────────────────────────────────────────────┐
│     Layer 5: Hardware (Sensors & Actuators)              │
│  • 센서: Mic, RGB/RGBD Cameras, LiDAR                    │
│  • 액추에이터: Speaker, Robot Arm, Wheels, Door Motor    │
│  • 인증/결제: RFID Readers, Payment Terminal             │
└──────────────────────────────────────────────────────────┘

```


## 6. 주요 데이터 흐름

### 6.1 음성 기반 길안내 요청 플로우
```
[사용자] "도비야"
    ↓ 음성
[Microphone]
    ↓ Serial
[STT/TTS Manager]
    ├─ Wake Word 감지 ("도비야")
    ├─ DMC State 확인 (cached: ROAMING)
    ├─ voice_active = True (ROAMING이므로)
    └─ ROS2 Service: set_listening_mode(True)
         ↓
[DMC]
    ├─ listening_mode = True
    ├─ DDC.control_command(STOP) - 순찰 정지
    └─ 응답: success
         ↓
[STT/TTS Manager]
    └─ TTS: "말씀하세요"
         ↓
[Speaker] → [사용자] "화장실 어디야?"
    ↓
[Microphone]
    ↓ Serial
[STT/TTS Manager]
    ├─ STT 변환: "화장실 어디야?"
    └─ HTTP POST http://llm-server:8000/api/v1/parse
         ↓
[LLM Service]
    ├─ OpenAI API 호출 (HTTPS)
    ├─ 의도 파싱: intent="navigation", target="화장실"
    └─ HTTP Response: {response: "화장실로 안내해드릴까요?", session_end: false}
         ↓
[STT/TTS Manager]
    └─ TTS: "화장실로 안내해드릴까요?"
         ↓
[Speaker] → [사용자] "응"
    ↓
[Microphone]
    ↓ Serial
[STT/TTS Manager]
    ├─ STT 변환: "응"
    └─ HTTP POST /api/v1/parse
         ↓
[LLM Service]
    ├─ 의도 파싱: intent="confirmation", confirmed=true
    └─ HTTP Response: 
        {
          response: "앞에 서서 인식을 기다려주세요",
          intent: "confirmation",
          confirmed: true,
          require_task: true,
          task_type: "guide_person",
          destination: "화장실",
          session_end: true
        }
         ↓
[STT/TTS Manager]
    ├─ require_task == true 확인
    ├─ ROS2 Service: /dobby1/request_task(
    │       task_type="guide_person",
    │       destination="화장실"
    │   )
    │    ↓
    │  [DMC]
    │    ├─ RCS Service: create_task()
    │    ├─ listening_mode = False
    │    └─ 응답: {success: true}
    │         ↓
    │  [RCS]
    │    ├─ 작업 검증 (App Service)
    │    ├─ 작업 큐 추가
    │    └─ ROS2 Action: /dobby1/main/guide_person (Goal 전송)
    │         ↓
    │  [DMC]
    │    ├─ Action 수락
    │    ├─ State: ROAMING → GUIDING
    │    └─ GuidingExecutor 실행
    │
    └─ TTS: "앞에 서서 인식을 기다려주세요" (HTTP Response에서 받은 텍스트)
         ↓
[Speaker] → [사용자]
```

---

### 6.2 정보 조회 플로우 (Phase 2)
```
[사용자] "도비야"
    ↓
[음성 인식 시작] (위와 동일)
    ↓
[사용자] "해리포터 어디 있어?"
    ↓
[STT/TTS Manager]
    └─ HTTP POST /api/v1/parse
         ↓
[LLM Service]
    ├─ 의도 파싱: intent="query", target="해리포터"
    ├─ TCP GET http://javis-server:3000/app/query_book?title=해리포터
    │    ↓
    │  [App Service]
    │    ├─ DB 조회: SELECT * FROM books WHERE title LIKE '%해리포터%'
    │    └─ 응답: {location: "3층 판타지 코너", shelf_id: "F-305", available: true}
    │         ↓
    └─ 응답 생성: "해리포터는 3층 판타지 코너에 있습니다. 안내해드릴까요?"
         ↓
[STT/TTS Manager]
    └─ TTS: "해리포터는 3층 판타지 코너에 있습니다. 안내해드릴까요?"
         ↓
[사용자] "아니야"
    ↓
[LLM Service]
    ├─ intent="confirmation", confirmed=false
    └─ 응답: {response: "알겠습니다", session_end: true}
         ↓
[STT/TTS Manager]
    ├─ TTS: "알겠습니다"
    └─ ROS2 Service: set_listening_mode(False)
         ↓
[DMC]
    ├─ listening_mode = False
    └─ DDC.control_command(RESUME) - 순찰 재개
```

---

### 6.3 관리자 긴급 제어 플로우
```
[Admin GUI]
    └─ "긴급 정지" 버튼 클릭
         ↓
    ROS2 Service: /dobby1/emergency_stop
         ↓
[DMC]
    ├─ 모든 Action 취소
    ├─ DDC.control_command(STOP)
    ├─ DAC.stop()
    ├─ listening_mode = False
    ├─ State: 현재 상태 → EMERGENCY_STOP  <-- (수정됨)
    └─ 응답: {success: true, message: "Emergency stop executed"}
         ↓
[Admin GUI]
    └─ 알림: "Dobby1 긴급 정지 완료"
```

---

### 6.4 도서 픽업 작업 플로우
```
[Information Desk GUI]
    └─ 사용자: 도서 검색 → "해리포터" 선택 → "픽업 요청" 클릭
         ↓
    TCP POST http://javis-server:3000/app/create_book_task
         ↓
[App Service]
    ├─ 도서 정보 조회 (DB)
    ├─ 작업 정보 구성
    └─ TCP HTTP: RCS create_task 호출
         ↓
[RCS]
    ├─ 작업 생성: task_type="pickup_book", book_id=12345
    ├─ 할당 가능한 로봇 선택
    │   - dobby1: IDLE, battery=75%, 거리=10m
    │   - dobby2: CLEANING_DESK, 제외
    │   → dobby1 선택
    └─ ROS2 Action: /dobby1/main/pickup_book (Goal 전송)
         ↓
[DMC (dobby1)]
    ├─ Action 수락
    ├─ State: IDLE → PICKING_UP_BOOK
    └─ PickupExecutor 실행
         ├─ Sub State: MOVE_TO_PICKUP
         ├─ DDC: move_to_target(책장 위치)
         ├─ Sub State: PICKUP_BOOK
         ├─ DAC: pick_book(book_id, pose)
         ├─ Sub State: MOVE_TO_STORAGE
         ├─ DDC: move_to_target(픽업대)
         ├─ Sub State: STOWING_BOOK
         └─ DAC: place_book(storage_id, pose)
              ↓
         Action Result: {success: true, total_distance_m: 20.5, total_time_sec: 180}
              ↓
[RCS]
    ├─ 작업 완료 처리
    └─ TCP: App Service 작업 로깅
         ↓
[App Service]
    ├─ 작업 이력 DB 저장
    ├─ 도서 상태 업데이트 (available → reserved)
    └─ Information Desk GUI에 알림
         ↓
[Information Desk GUI]
    └─ "도서가 1층 픽업대에 준비되었습니다" 표시
```

### 6.5 관리자 로봇 모드 변경 플로우
```
[Admin GUI]
    └─ "Dobby1" 선택 → "roaming" 모드 버튼 클릭
         ↓
    ROS2 Service: /dobby1/set_mode
    (Request: {mode: "roaming"})
         ↓
[DMC (dobby1)]
    ├─ 모드 변경 요청 수락
    ├─ (mode == "roaming" and battery >= 40% and State == IDLE)
    ├─ State: IDLE → ROAMING
    ├─ DDC.control_command(RESUME_WAYPOINT) - 순찰 시작
    ├─ (mode == "standby")
    ├─ State: ROAMING → IDLE
    ├─ DDC.control_command(STOP)
    └─ 응답: {success: true}
         ↓
[Admin GUI]
    └─ "Dobby1 모드 변경 완료" 알림

### 6.6 긴급 정지 해제 플로우
[Admin GUI]
    └─ "긴급 정지 해제" 버튼 클릭 (Dobby1 대상)
         ↓
    ROS2 Service: /dobby1/clear_emergency_stop
         ↓
[DMC (dobby1)]
    ├─ State == EMERGENCY_STOP 확인
    ├─ (확인 시) State: EMERGENCY_STOP → IDLE(이전 상태로 복귀)
    ├─ DDC.control_command(RESUME) - 순찰 재개
    ├─ DAC.stop()
    ├─ listening_mode = False
    └─ 응답: {success: true, message: "Emergency stop cleared. Returning to IDLE."}
         ↓
[Admin GUI]
    └─ 알림: "Dobby1 긴급 정지 해제됨. IDLE 상태로 복귀."
```
---

## 7. 상태 관리

### 7.1 DMC Main State 정의

| State | 값 | 설명 | 배터리 변화 | 작업 수락 | 음성 인식 |
|-------|---|------|-----------|---------|----------|
| INITIALIZING | 0 | 시스템 초기화 | - | ❌ | ❌ |
| CHARGING | 1 | 충전 중 (battery < 40%) | +10%/min | ❌ | ❌ |
| IDLE | 2 | 대기 (충전소, battery ≥ 40%) | +10%/min (충전소) | ✅ | ✅ |
| MOVING_TO_CHARGER | 3 | 충전소로 이동 | -1%/min | ✅ (battery ≥ 40%) | ❌ |
| PICKING_UP_BOOK | 4 | 도서 픽업 실행 | -1%/min | ❌ | ❌ |
| RESHELVING_BOOK | 5 | 반납 정리 실행 | -1%/min | ❌ | ❌ |
| GUIDING | 6 | 길안내 실행 | -1%/min | ❌ | ❌ |
| CLEANING_DESK | 7 | 좌석 정리 실행 | -1%/min | ❌ | ❌ |
| SORTING_SHELVES | 8 | 서가 정리 실행 | -1%/min | ❌ | ❌ |
| FORCE_MOVE_TO_CHARGER | 9 | 긴급 충전 복귀 | -1%/min | ❌ | ❌ |
| LISTENING | 10 | 음성인식중 | -1%/min | ❌ |✅ |
| ROAMING | 11 | 자율 순찰 중 | -1%/min | ✅ |✅ |
| EMERGENCY_STOP | 98 | 관리자 긴급 정지 (해제 필요) | - | ❌ | ❌ |
| MAIN_ERROR | 99 | 에러 상태 | - | ❌ | ❌ |

---

### 7.2 로봇 모드 (Robot Mode)

| 모드 | 설명 | Main State 전환 | 동작 |
|------|------|----------------|------|
| **standby** | 대기 모드 | IDLE ↔ CHARGING 만 | 충전소에서 대기, 작업 할당만 수락, 음성 인식 가능 |
| **roaming** | 자율이동 모드 | IDLE → ROAMING (battery ≥ 40%) | 웨이포인트 순찰, 작업 할당 가능, 음성 인식 가능 (순찰 일시정지) |

---

### 7.3 Listening Mode

| listening_mode | DMC State | 동작 |
|---------------|----------|------|
| **True** | IDLE or ROAMING | 음성 대화 세션 중, ROAMING이면 DDC 정지 |
| **False** | Any | 정상 동작, ROAMING이면 DDC 재개 |

---
