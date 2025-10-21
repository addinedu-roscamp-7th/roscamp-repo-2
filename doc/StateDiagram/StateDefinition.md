# Dobby Robot State Diagram Specification v3.0

Version: 3.0

Last Updated: 2025-01-21

Architecture: Mode-Based State Architecture

---

## 목차

개요

모드(Mode) 정의

상태(State) 정의 테이블

대기 모드 (STANDBY MODE) 상태 다이어그램

자율이동 모드 (ROAMING MODE) 상태 다이어그램

작업 상태(Task States) 상세 다이어그램

상태 전환 규칙

버전 히스토리

---

### 1. 개요

본 문서는 Dobby 로봇의 모드(Mode) 기반 상태(State) 체계를 정의 합니다.

주요 변경사항 (v3.0)

모드(Mode) 개념 도입: STANDBY MODE / ROAMING MODE

배터리 임계값 변경:

CHARGING → IDLE: ≥ 40%

FORCE_MOVE_TO_CHARGER 트리거: ≤ 20%

ROAMING 진입 조건: ≥ 60%

모드별 상태 전환 규칙 분리


### 계층 구조
```
Robot Mode (로봇 모드)
├── STANDBY MODE (대기 모드)
│   ├── Common States (공통 상태)
│   └── Task States (작업 상태)
└── ROAMING MODE (자율이동 모드)
    ├── Common States (공통 상태)
    ├── Task States (작업 상태)
    └── ROAMING (순찰 상태)
```
---

### 2. 모드(Mode) 정의


| Mode ID      | 한글명     | 영문명          | 설명                          | 진입 조건                           |
| ------------ | ------- | ------------ | --------------------------- | ------------------------------- |
| MODE_STANDBY | 대기 모드   | STANDBY MODE | 충전소에서 대기하며 작업 할당 및 음성 인식 가능       | 관리자 모드 변경          |
| MODE_ROAMING | 자율이동 모드 | ROAMING MODE | 웨이포인트 순찰하며 작업 할당 및 음성 인식 가능 | 관리자 모드 변경  |


#### 2.1 모드별 동작 차이

| 구분            | STANDBY MODE        | ROAMING MODE               |
| ------------- | ------------------- | -------------------------- |
| **주행**        | 작업 할당 시에만 이동        | 웨이포인트 순찰 (자율 이동)           |
| **작업 수락**     | ✅ 가능                | ✅ 가능                       |
| **음성 인식**     | ✅ 가능 (LISTENING 상태) | ✅ 가능 (순찰 일시정지 후 LISTENING) |
| **배터리 관리**    | IDLE ↔ CHARGING 반복  | ROAMING → 배터리 부족 40%시    |
| **IDLE 후 동작** | 충전소 대기              | ROAMING 상태로 순찰 재개            |

---

### 3. 상태(State) 정의 테이블
#### 3.1 공통 상태 (Common States)

| State ID | 한글명       | 영문명                   | 설명                  | 배터리 조건  |
| -------- | --------- | --------------------- | ------------------- | ------- |
| DB_S00   | 초기화       | INITIALIZING          | 시스템 시작 및 초기화        | -       |
| DB_S01   | 충전 중      | CHARGING              | 충전소에서 배터리 충전 중      | 충전 중    |
| DB_S02   | 작업 대기     | IDLE                  | 작업 지시 대기 (충전소 위치)   | ≥ 60%   |
| DB_S03   | 충전소 이동 중  | MOVING_TO_CHARGER     | 작업 완료 후 충전소 복귀      | 40~100% |
| DB_S04   | 강제 충전소 이동 | FORCE_MOVE_TO_CHARGER | 긴급 충전 복귀 (모든 작업 중단) | ≤ 20%   |
| DB_S05   | 음성 인식 중   | LISTENING             | 사용자 음성 명령 대기 중      | -       |
| DB_S99   | 긴급 정지     | EMERGENCY_STOP        | 관리자 긴급 정지           | -       |
#### 3.2 자율이동 모드 전용 상태
| State ID | 한글명   | 영문명     | 설명          |
| -------- | ----- | ------- | ----------- |
| DB_S06   | 자율이동중 | ROAMING | 웨이포인트 기반 순찰 |

#### 3.3 작업 상태(Task States)
| State ID | 한글명      | 영문명             | 설명                |
| -------- | -------- | --------------- | ----------------- |
| DB_S10   | 책장 정리중   | SORTING_SHELVES | 책장 정리 작업 수행 중     |
| DB_S20   | 반납도서 정리중 | RESHELVING_BOOK | 반납 도서 정리 작업 수행 중  |
| DB_S30   | 도서 픽업중   | PICKING_UP_BOOK | 도서 픽업 작업 수행 중     |
| DB_S40   | 길 안내중    | GUIDING         | 사용자 길안내 수행 중      |
| DB_S50   | 좌석 정리중   | CLEANING_DESK   | 좌석 쓰레기 수거 작업 수행 중 |


---

### 4. 대기 모드 (STANDBY MODE) 상태 다이어그램
```
stateDiagram-v2
    direction TB
    [*] --> INITIALIZING: 시작
    INITIALIZING --> CHARGING: 초기화완료
    CHARGING --> IDLE: battery ≥ 40%
    IDLE --> FORCE_MOVE_TO_CHARGER: 배터리 ≤ 20%
    IDLE --> LISTENING: wakeWordDetected
    IDLE --> CHARGING: 배터리 ≤ 40%
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
```
---

### 5. 자율운영 모드 (AUTONOMY MODE) 상태 다이어그램
```
stateDiagram-v2
    direction TB
    [*] --> INITIALIZING
    INITIALIZING --> CHARGING
    CHARGING --> ROAMING: battery ≥ 40%
    ROAMING --> LISTENING: wakeWordDetected
    ROAMING --> TASK_IN_PROGRESS: 작업요청
    ROAMING --> FORCE_MOVE_TO_CHARGER: battery ≤ 20%
    LISTENING --> ROAMING: timeout(20s)

```
---

### 6. 작업 상태(Task States) 상세 다이어그램

#### 6.1 책장정리 (SORTING_SHELVES)
```
stateDiagram-v2
    direction LR
    [*] --> MOVE_TO_SHELF
    MOVE_TO_SHELF --> SCAN_BOOK: 책장 도착(Arrived at Desk)
    SCAN_BOOK --> SORT_BOOK: 정리 도서 발견(Misplaced Book Found)
    SCAN_BOOK --> MOVE_TO_SHELF: 정리 필요한 다음 책장으로 이동 (Move to Next Shelf)
    SORT_BOOK --> SCAN_BOOK: 도서 정리 완료 (Sort Book Complete)
    SCAN_BOOK --> [*]: 전체 정리 완료 (All Shelves Sorted)
```

#### 6.2 반납도서정리 (RESHELVING_BOOK)
```
stateDiagram-v2
    direction LR
    [*] --> MOVE_TO_RETURN_DESK : 반납도서 정리 작업 시작 (Assign Reshelving Task)
    MOVE_TO_RETURN_DESK --> COLLECT_RETURN_BOOKS : 반납대 도착(Arrived at Return Desk)
    COLLECT_RETURN_BOOKS --> MOVE_TO_PLACE_SHELF : 반납도서 수거 (Collection Complete)
    MOVE_TO_PLACE_SHELF -->  [*]: 반납도서 인식 실패 (Scan Failed)
    MOVE_TO_PLACE_SHELF --> PLACE_RETURN_BOOK : 책장 도착(Arrived at Shelf)
    PLACE_RETURN_BOOK --> [*]: 반납도서 정리 완료(Place Book Complete)
```

#### 6.3 도서픽업 (PICKING_UP_BOOK)
```
stateDiagram-v2
    direction LR
    [*] --> MOVE_TO_PICKUP
    MOVE_TO_PICKUP --> PICKING_BOOK
    PICKING_BOOK --> MOVE_TO_STORAGE
    MOVE_TO_STORAGE --> STOWING_BOOK
    STOWING_BOOK --> [*]: 완료
```

#### 6.4 길안내 (GUIDING)
```
stateDiagram-v2
    direction LR
    [*] --> SELECT_DEST
    SELECT_DEST --> SCAN_USER
    SCAN_USER --> GUIDING_TO_DEST
    GUIDING_TO_DEST --> [*]: 목적지 도착
```

#### 6.5 좌석정리 (CLEANING_DESK)
```
stateDiagram-v2
    direction LR
    [*] --> MOVE_TO_DESK
    MOVE_TO_DESK --> SCAN_DESK
    SCAN_DESK --> COLLECT_TRASH
    COLLECT_TRASH --> MOVE_TO_BIN
    MOVE_TO_BIN --> DUMP_TRASH
    DUMP_TRASH --> [*]
```

---

### 7. 상태 전환 규칙
#### 7.1 공통 전환 규칙
| Priority     | From                  | To                    | Trigger       | Condition                  |
| ------------ | --------------------- | --------------------- | ------------- | -------------------------- |
| **CRITICAL** | ANY                   | EMERGENCY_STOP        | Admin 긴급정지 명령 | `/dobby<N>/emergency_stop` |
| **HIGH**     | ANY                   | FORCE_MOVE_TO_CHARGER | 배터리 ≤ 20%     | 배터리 경고                     |
| **MEDIUM**   | FORCE_MOVE_TO_CHARGER | CHARGING              | 충전소 도착        | Navigation Complete        |
| **LOW**      | INITIALIZING          | CHARGING              | 초기화 완료        | Subsystem Ready            |
| **LOW**      | CHARGING              | IDLE                  | 충전 완료         | Battery ≥ 60%              |

---
#### 7.4 배터리 관리 우선순위
```
≤ 20%  → FORCE_MOVE_TO_CHARGER
21~39% → MOVING_TO_CHARGER
40~59% → CHARGING
≥ 60%  → IDLE 또는 ROAMING
```

### 8. 버전 히스토리
```
Version	Date	Changes	Author
v3.0	2025-01-21	모드 기반 아키텍처 도입, 배터리 임계값 변경, Mermaid 다이어그램 개편	System Architect
v2.0	2025-01-21	State ID 체계 변경, Sub-state 다이어그램 추가	System Architect
v1.0	2025-10-17	초기 문서 생성	-
```
Document End