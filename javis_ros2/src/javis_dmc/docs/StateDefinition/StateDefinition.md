🤖 Dobby Robot State Diagram Specification v3.0

Version: 3.1
Last Updated: 2025-01-21
Architecture: Mode-Based State Architecture

📋 목차

- 개요

- 모드(Mode) 정의

- 상태(State) 정의 테이블

- 대기 모드 (STANDBY MODE) 상태 다이어그램

- 자율이동 모드 (ROAMING MODE) 상태 다이어그램

- 작업 상태(Task States) 상세 다이어그램

- 상태 전환 규칙

- 버전 히스토리

1. 개요

본 문서는 Dobby 로봇의 모드 기반 상태(State) 체계를 정의합니다.

주요 변경사항 (v3.1)

배터리 임계값 조정:

CHARGING → IDLE: ≥ 40%
IDLE → CHARGING: ≤ 40%
FORCE_MOVE_TO_CHARGER 트리거: ≤ 20% (유지)
ROAMING 진입 조건: ≥ 40%


모드와 상태 관계 명확화:

AUTONOMY와 STANDBY는 모드(Mode)
ROAMING은 AUTONOMY 모드에서의 상태(State)
AUTONOMY 모드에서는 IDLE 상태가 존재하지 않음

계층 구조
Robot Mode (로봇 모드)
├── STANDBY MODE (대기 모드)
│   ├── Common States (공통 상태)
│   │   ├── INITIALIZING
│   │   ├── CHARGING
│   │   ├── IDLE
│   │   ├── MOVING_TO_CHARGER
│   │   ├── FORCE_MOVE_TO_CHARGER
│   │   ├── LISTENING
│   │   └── EMERGENCY_STOP
│   └── Task States (작업 상태)
└── AUTONOMY MODE (자율이동 모드)
    ├── Common States (공통 상태)
    │   ├── INITIALIZING
    │   ├── CHARGING
    │   ├── MOVING_TO_CHARGER
    │   ├── FORCE_MOVE_TO_CHARGER
    │   ├── LISTENING
    │   └── EMERGENCY_STOP
    ├── ROAMING (순찰 상태)
    └── Task States (작업 상태)
```
---

### 2. 모드(Mode) 정의


| Mode ID      | 한글명     | 영문명          | 설명                          | 진입 조건                           |
| ------------ | ------- | ------------ | --------------------------- | ------------------------------- |
| MODE_STANDBY | 대기 모드   | STANDBY MODE | 충전소에서 대기하며 작업 할당 및 음성 인식 가능       | 관리자 모드 변경          |
| MODE_ROAMING | 자율이동 모드 | ROAMING MODE | 웨이포인트 순찰하며 작업 할당 및 음성 인식 가능 | 관리자 모드 변경  |


#### 2.1 모드별 동작 차이

| 구분            | STANDBY MODE        | AUTONOMY MODE               |
| ------------- | ------------------- | -------------------------- |
| **주행**        | 작업 할당 시에만 이동        | 웨이포인트 순찰 (ROAMING 상태)           |
| **작업 수락**     | ✅ 가능 (IDLE 상태에서)   | ✅ 가능 (ROAMING 상태에서)                       |
| **음성 인식**     | ✅ 가능 (IDLE → LISTENING) | ✅ 가능 (ROAMING → LISTENING) |
| **배터리 관리**    | IDLE ↔ CHARGING (40% 기준)  | CHARGING → ROAMING (≥ 40%) |
| **충전 후 동작**  | CHARGING → IDLE     | CHARGING → ROAMING           |
| **IDLE 존재 여부** | ✅ 있음 (주 상태)      | ❌ 없음 (ROAMING으로 대체)    |

---

---
### 3. 상태(State) 정의 테이블
#### 3.1 공통 상태 (Common States)

| State ID | 한글명       | 영문명                   | 설명                  | 배터리 조건  | 모드 호환성 |
| -------- | --------- | --------------------- | ------------------- | ------- | ------- |
| DB_S00   | 초기화       | INITIALIZING          | 시스템 시작 및 초기화        | -       | 모든 모드 |
| DB_S01   | 충전 중      | CHARGING              | 충전소에서 배터리 충전 중      | < 40%    | 모든 모드 |
| DB_S02   | 작업 대기     | IDLE                  | 작업 지시 대기 (충전소 위치)   | ≥ 40%   | STANDBY 전용 |
| DB_S03   | 충전소 이동 중  | MOVING_TO_CHARGER     | 작업 완료 후 충전소 복귀      | 20~40% | 모든 모드 |
| DB_S04   | 강제 충전소 이동 | FORCE_MOVE_TO_CHARGER | 긴급 충전 복귀 (모든 작업 중단) | ≤ 20%   | 모든 모드 |
| DB_S05   | 음성 인식 중   | LISTENING             | 사용자 음성 명령 대기 중      | -       | 모든 모드 |
| DB_S99   | 긴급 정지     | EMERGENCY_STOP        | 관리자 긴급 정지           | -       | 모든 모드 |

#### 3.2 자율이동 모드 전용 상태
| State ID | 한글명   | 영문명     | 설명          | 배터리 조건 |
| -------- | ----- | ------- | ----------- | -------- |
| DB_S06   | 자율이동중 | ROAMING | 웨이포인트 기반 순찰 | ≥ 40% |

#### 3.3 작업 상태(Task States)
| State ID | 한글명      | 영문명             | 설명                | 모드 호환성 |
| -------- | -------- | --------------- | ----------------- | ------- |
| DB_S10   | 책장 정리중   | SORTING_SHELVES | 책장 정리 작업 수행 중     | 모든 모드 |
| DB_S20   | 반납도서 정리중 | RESHELVING_BOOK | 반납 도서 정리 작업 수행 중  | 모든 모드 |
| DB_S30   | 도서 픽업중   | PICKING_UP_BOOK | 도서 픽업 작업 수행 중     | 모든 모드 |
| DB_S40   | 길 안내중    | GUIDING         | 사용자 길안내 수행 중      | 모든 모드 |
| DB_S50   | 좌석 정리중   | CLEANING_DESK   | 좌석 쓰레기 수거 작업 수행 중 | 모든 모드 |

---
### 4. 대기 모드 (STANDBY MODE) 상태 다이어그램
```
stateDiagram-v2
    direction TB
    [*] --> INITIALIZING: 시작
    INITIALIZING --> CHARGING: 초기화완료
    CHARGING --> IDLE: battery ≥ 40%
    IDLE --> CHARGING: battery ≤ 40%
    IDLE --> FORCE_MOVE_TO_CHARGER: battery ≤ 20%
    IDLE --> LISTENING: wakeWordDetected
    IDLE --> TASK_IN_PROGRESS: 작업요청

    LISTENING --> IDLE: timeout(20s)
    LISTENING --> TASK_IN_PROGRESS: guidanceRequested

    state TASK_IN_PROGRESS {
        direction LR
        [*] --> CLEANING_DESK: cleaningDeskRequested
        [*] --> GUIDING: guidanceRequested
        [*] --> SORTING_SHELVES: sortingShelfRequested
        [*] --> PICKING_UP_BOOK: pickupBookRequested
        [*] --> RESHELVING_BOOK: reshelvingBookRequested
        CLEANING_DESK --> [*]: cleaningDeskComplete
        GUIDING --> [*]: guidanceComplete
        SORTING_SHELVES --> [*]: sortingShelfComplete
        PICKING_UP_BOOK --> [*]: pickupBookComplete
        RESHELVING_BOOK --> [*]: reshelvingBookComplete
    }

    TASK_IN_PROGRESS --> MOVING_TO_CHARGER: 작업 완료
    MOVING_TO_CHARGER --> CHARGING: chargerArrived
    
    TASK_IN_PROGRESS --> FORCE_MOVE_TO_CHARGER: battery ≤ 20%
```
---

### 5. 자율이동 모드 (AUTONOMY MODE) 상태 다이어그램
```
stateDiagram-v2
    direction TB
    [*] --> INITIALIZING: 시작
    INITIALIZING --> CHARGING: 초기화완료
    CHARGING --> ROAMING: battery ≥ 40%
    ROAMING --> CHARGING: battery ≤ 40%
    ROAMING --> LISTENING: wakeWordDetected
    ROAMING --> TASK_IN_PROGRESS: 작업요청
    ROAMING --> FORCE_MOVE_TO_CHARGER: battery ≤ 20%
    LISTENING --> ROAMING: timeout(20s) or 대화종료
    
    state TASK_IN_PROGRESS {
        direction LR
        [*] --> CLEANING_DESK: cleaningDeskRequested
        [*] --> GUIDING: guidanceRequested
        [*] --> SORTING_SHELVES: sortingShelfRequested
        [*] --> PICKING_UP_BOOK: pickupBookRequested
        [*] --> RESHELVING_BOOK: reshelvingBookRequested
        CLEANING_DESK --> [*]: cleaningDeskComplete
        GUIDING --> [*]: guidanceComplete
        SORTING_SHELVES --> [*]: sortingShelfComplete
        PICKING_UP_BOOK --> [*]: pickupBookComplete
        RESHELVING_BOOK --> [*]: reshelvingBookComplete
    }
    
    TASK_IN_PROGRESS --> ROAMING: 작업 완료
    TASK_IN_PROGRESS --> MOVING_TO_CHARGER: 작업 완료 & battery ≤ 40%
    TASK_IN_PROGRESS --> FORCE_MOVE_TO_CHARGER: battery ≤ 20%
    MOVING_TO_CHARGER --> CHARGING: chargerArrived
    FORCE_MOVE_TO_CHARGER --> CHARGING: chargerArrived
```
---

### 6. 작업 상태(Task States) 상세 다이어그램

#### 6.1 책장정리 (SORTING_SHELVES)
```
stateDiagram-v2
    direction LR
    [*] --> MOVE_TO_SHELF
    MOVE_TO_SHELF --> SCAN_BOOK: 도착
    SCAN_BOOK --> SORT_BOOK: 정리 필요
    SORT_BOOK --> SCAN_BOOK: 완료
    SCAN_BOOK --> [*]: 전체 완료

6.2 반납 도서 정리 (RESHELVING_BOOK)
stateDiagram-v2
    direction LR
    [*] --> MOVE_TO_RETURN_DESK
    MOVE_TO_RETURN_DESK --> COLLECT_RETURN_BOOKS
    COLLECT_RETURN_BOOKS --> MOVE_TO_PLACE_SHELF
    MOVE_TO_PLACE_SHELF --> PLACE_RETURN_BOOK
    PLACE_RETURN_BOOK --> [*]: 완료

6.3 도서 픽업 (PICKING_UP_BOOK)
stateDiagram-v2
    direction LR
    [*] --> MOVE_TO_PICKUP
    MOVE_TO_PICKUP --> PICKING_BOOK
    PICKING_BOOK --> MOVE_TO_STORAGE
    MOVE_TO_STORAGE --> STOWING_BOOK
    STOWING_BOOK --> [*]: 완료

6.4 길 안내 (GUIDING)
stateDiagram-v2
    direction LR
    [*] --> SELECT_DEST
    SELECT_DEST --> SCAN_USER
    SCAN_USER --> GUIDING_TO_DEST
    GUIDING_TO_DEST --> [*]: 목적지 도착

6.5 좌석 정리 (CLEANING_DESK)
stateDiagram-v2
    direction LR
    [*] --> MOVE_TO_DESK
    MOVE_TO_DESK --> SCAN_DESK
    SCAN_DESK --> COLLECT_TRASH
    COLLECT_TRASH --> MOVE_TO_BIN
    MOVE_TO_BIN --> DUMP_TRASH
    DUMP_TRASH --> [*]

---

### 7. 상태 전환 규칙
#### 7.1 공통 전환 규칙
| Priority     | From                  | To                    | Trigger       | Condition                  |
| ------------ | --------------------- | --------------------- | ------------- | -------------------------- |
| **CRITICAL** | ANY                   | EMERGENCY_STOP        | Admin 긴급정지 명령 | `/dobby<N>/emergency_stop` |
| **HIGH**     | ANY                   | FORCE_MOVE_TO_CHARGER | 배터리 ≤ 20%     | 배터리 경고                     |
| **MEDIUM**   | FORCE_MOVE_TO_CHARGER | CHARGING              | 충전소 도착        | Navigation Complete        |
| **LOW**      | INITIALIZING          | CHARGING              | 초기화 완료        | Subsystem Ready            |

#### 7.2 STANDBY 모드 전환 규칙
| Priority     | From                  | To                    | Trigger       | Condition                  |
| ------------ | --------------------- | --------------------- | ------------- | -------------------------- |
| **MEDIUM**   | CHARGING              | IDLE                  | 충전 완료         | Battery ≥ 40%              |
| **MEDIUM**   | IDLE                  | CHARGING              | 배터리 부족        | Battery ≤ 40%              |

#### 7.3 AUTONOMY 모드 전환 규칙
| Priority     | From                  | To                    | Trigger       | Condition                  |
| ------------ | --------------------- | --------------------- | ------------- | -------------------------- |
| **MEDIUM**   | CHARGING              | ROAMING               | 충전 완료         | Battery ≥ 40%              |
| **MEDIUM**   | ROAMING               | CHARGING              | 배터리 부족        | Battery ≤ 40%              |

---
#### 7.4 배터리 관리 우선순위
```
≤ 20%  → FORCE_MOVE_TO_CHARGER (모든 모드) (작업 긴급중단 상태)
21~39% → MOVING_TO_CHARGER (작업 완료 후) 또는 CHARGING (직접 전환)(작업 할당 불가능 상태들)
≥ 40%  → IDLE(STANDBY 모드) 또는 ROAMING(AUTONOMY 모드)(작업 할당 가능 상태들)
```


### 8. 버전 히스토리
```
Version  Date        Changes                                                   Author
v3.1     2025-10-22  배터리 임계값 조정, 모드-상태 관계 명확화, 다이어그램 업데이트     System Architect
v3.0     2025-01-21  모드 기반 아키텍처 도입, 배터리 임계값 변경, Mermaid 다이어그램 개편  System Architect
v2.0     2025-01-21  State ID 체계 변경, Sub-state 다이어그램 추가               System Architect
v1.0     2024-10-17  초기 문서 생성    
```

Document End