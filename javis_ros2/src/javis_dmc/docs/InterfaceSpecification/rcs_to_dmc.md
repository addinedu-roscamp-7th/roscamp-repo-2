# RCS <-> DMC 통신 정리 문서

> 작성자: 김우영  
> 용도: RCS와 DMC 간의 Action/Service/Topic 인터페이스 정리  
> 대상: 도서관 로봇 Dobby

---

## 📦 인터페이스 요약

| From | To   | Protocol | Interface 항목       | 메시지 형식                     |
|------|------|----------|----------------------|---------------------------------|
| RCS  | DMC  | Action   | 도서 픽업 작업        | `dobby1/main/pickup_book`       |
| RCS  | DMC  | Action   | 반납도서 정리 작업     | `dobby1/main/reshelving_book`   |
| RCS  | DMC  | Action   | 길안내 작업            | `dobby1/main/guide_person`      |
| RCS  | DMC  | Action   | 좌석 정리 작업         | `dobby1/main/clean_seat`        |
| DMC  | RCS  | Service  | 사용자 작업 생성 요청  | `/rcs/create_user_task`         |
| DMC  | All  | Topic    | 로봇의 상태            | `dobby1/status/robot_state`     |
| RMS  | DMC  | Topic    | 배터리 잔량            | `dobby1/status/battery_status`  |

---

## 🎯 Action 정의

### 1. 도서 픽업 작업 – `PickupBook.action`

```action
# Goal
string book_id
int32 storage_id
geometry_msgs/Pose2D shelf_approach_location
geometry_msgs/Pose book_pick_pose
geometry_msgs/Pose2D storage_approach_location
geometry_msgs/Pose storage_slot_pose
---
# Result
string book_id
int32 storage_id
bool success
string message
float64 total_distance_m
float64 total_time_sec
---
# Feedback (토픽으로 대체 고려)
int32 progress_percent
상태값 예: "OK" | "BOOK_NOT_FOUND" | "STORAGE_FULL" | "ARM_ERROR"
```

### 2. 반납도서 정리 작업 – ReshelvingBook.action
```action
# Goal
int32 return_desk_id
geometry_msgs/Pose2D return_desk_location
geometry_msgs/Pose return_desk_pose
---
# Result
bool success
int32 books_processed
string[] failed_book_ids
float64 total_distance_m
float64 total_time_sec
string message
---
# Feedback
int32 progress_percent
```
### 3. 길안내 작업 – GuidePerson.action
```action
# Goal
geometry_msgs/Pose2D dest_location
bool user_initiated  # True: 사용자 주도 작업 (상태 체크 무시), False: RCS 주도 작업 (상태 체크 필요)
---
# Result
bool success
int32 error_code
float64 total_distance_m
float64 total_time_sec
string message
---
# Feedback
float64 distance_remaining_m
bool person_detected
```

> **v4.0 업데이트**: `user_initiated` 플래그 추가
> - `True`: 사용자가 VRC/GUI를 통해 직접 요청한 길안내 (LISTENING/WAITING_DEST_INPUT 상태에서도 허용)
> - `False`: RCS가 작업으로 할당하는 길안내 (IDLE/ROAMING 상태에서만 허용)

### 4. 좌석 정리 작업 – CleanSeat.action
```action
# Goal
int32 seat_id
geometry_msgs/Pose2D return_desk_location
geometry_msgs/Pose return_desk_pose
geometry_msgs/Pose2D bin_location
geometry_msgs/Pose bin_pose
---
# Result
int32 seat_id
bool success
int32 trash_collected_count
string[] trash_types
float64 total_distance_m
float64 total_time_sec
string message
---
# Feedback
int32 progress_percent
```

---

## 🔧 Service 정의

### 1. 사용자 작업 생성 요청 – `CreateUserTask.srv`

**용도**: DMC가 사용자 주도 작업(길안내)을 RCS에 생성 요청

**Service Name**: `/rcs/create_user_task`

**Message Definition**:
```srv
# Request
string robot_id              # 요청 로봇 ID (예: "dobby1")
string task_type             # 작업 타입 (현재는 "GUIDING"만 지원)
geometry_msgs/Pose2D dest_location  # 목적지 좌표
string destination_name      # 목적지 이름 (예: "화장실")
bool user_initiated          # True (사용자 주도 작업)

---

# Response
bool success                 # 성공 여부
string task_id               # 생성된 작업 ID (예: "guidance_20250127_143022")
string message               # 상태 메시지
```

**사용 시나리오**:
1. 사용자가 VRC/GUI를 통해 RequestGuidance 호출
2. DMC가 배터리 체크 (≥40%)
3. DMC가 RCS에 CreateUserTask Service 호출 (user_initiated=True)
4. RCS가 즉시 GuidePerson Action 호출 (goal.user_initiated=True)
5. DMC가 Action Goal 수신 → LISTENING/WAITING_DEST_INPUT → GUIDING 전환

**RCS 로직**:
```python
def handle_create_user_task(request):
    if request.user_initiated:
        # 사용자 주도 작업 = DMC 상태 체크 무시
        task_id = generate_task_id()
        
        # 즉시 GuidePerson Action 호출
        goal = GuidePerson.Goal()
        goal.dest_location = request.dest_location
        goal.user_initiated = True  # 플래그 전달
        
        send_action_goal(request.robot_id, goal)
        
        return Response(success=True, task_id=task_id)
```

---

## 📡 Topic 정의
### 1. 로봇 상태 – DobbyState.msg
```msg
uint8 INITIALIZING = 0
uint8 CHARGING = 1
uint8 IDLE = 2
uint8 MOVING_TO_CHARGER = 3
uint8 PICKING_UP_BOOK = 4
uint8 RESHELVING_BOOK = 5
uint8 GUIDING = 6
uint8 CLEANING_DESK = 7
uint8 SORTING_SHELVES = 8
uint8 FORCE_MOVE_TO_CHARGER = 9
uint8 LISTENING = 10
uint8 WAITING_DEST_INPUT = 11
uint8 ROAMING = 12
uint8 EMERGENCY_STOP = 98
uint8 MAIN_ERROR = 99
# -------------------------
uint8 main_state

# Sub State Constants
uint8 NONE = 100
uint8 MOVE_TO_PICKUP = 101
uint8 PICKUP_BOOK = 102
uint8 MOVE_TO_STORAGE = 103
uint8 STOWING_BOOK = 104
uint8 MOVE_TO_RETURN_DESK = 105
uint8 COLLECT_RETURN_BOOKS = 106
uint8 MOVE_TO_PLACE_SHELF = 107
uint8 PLACE_RETURN_BOOK = 108
# uint8 SELECT_DEST = 109  # DEPRECATED: 제거됨 (WAITING_DEST_INPUT 메인 상태로 대체)
uint8 SCAN_USER = 110
uint8 GUIDING_TO_DEST = 111
uint8 FIND_USER = 112
uint8 MOVE_TO_DESK = 113
uint8 SCAN_DESK = 114
uint8 CLEANING_TRASH = 115
uint8 MOVE_TO_BIN = 116
uint8 TIDYING_SHELVES = 117
uint8 MOVE_TO_SHELF = 118
uint8 SCAN_BOOK = 119
uint8 SORT_BOOK = 120
uint8 SUB_ERROR = 199
# ----------------------------
uint8 sub_state


# Error Info
bool is_error
string error_message
```
### 2. 배터리 상태 – BatteryStatus.msg
```msg
float32 charge_percentage
bool is_charging
```
## ⚙️ 네임스페이스 및 TF Frame 설정
Namespace 예시: dobby1, dobby2

TF 충돌 방지: frame_prefix를 <robot_name>/ 으로 설정

```python
robot_state_publisher_node = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    namespace='dobby1',
    parameters=[
        {'robot_description': loaded_urdf_content},
        {'use_sim_time': use_sim_time},
        {'frame_prefix': 'dobby1/'}
    ]
)
```

---
## 🔁 상태 변화 범위
| 구분 | 범위 |
|------|------|
| Main | 0 ~ 99 |
| Sub  | 100 ~ 199 |

---

## 📋 작업 할당 조건 (v4.0)

### RCS → DMC 작업 할당 가능 상태

RCS가 DMC에게 작업을 할당할 수 있는 상태:

**✅ 작업 할당 가능 (Work Assignable)**
- `IDLE (2)`: STANDBY 모드 대기 중
- `ROAMING (12)`: AUTONOMY 모드 순찰 중

**❌ 작업 할당 불가 (Work Not Assignable)**

**사용자 인터랙션 중:**
- `LISTENING (10)`: VRC 음성 인식 처리 중
- `WAITING_DEST_INPUT (11)`: GUI 목적지 선택 대기 중

**작업 수행 중:**
- `GUIDING (6)`: 길안내 중
- `SORTING_SHELVES (8)`: 책장 정리 중
- `RESHELVING_BOOK (5)`: 반납도서 정리 중
- `PICKING_UP_BOOK (4)`: 도서 픽업 중
- `CLEANING_DESK (7)`: 좌석 정리 중

**배터리 관리 중:**
- `CHARGING (1)`: 충전 중
- `MOVING_TO_CHARGER (3)`: 충전소 이동 중
- `FORCE_MOVE_TO_CHARGER (9)`: 긴급 충전소 이동 중

**시스템 상태:**
- `INITIALIZING (0)`: 초기화 중
- `EMERGENCY_STOP (98)`: 긴급 정지

### RCS 작업 할당 체크 로직 예시

```python
WORK_ASSIGNABLE_STATES = [
    DobbyState.IDLE,           # 2
    DobbyState.ROAMING         # 12
]

def can_assign_task_to_dmc(robot_id, task_type):
    """
    RCS가 DMC에 작업을 할당할 수 있는지 체크
    
    Args:
        robot_id: 로봇 ID (예: "dobby1")
        task_type: 작업 타입 (예: "SORTING_SHELVES")
        
    Returns:
        bool: 작업 할당 가능 여부
    """
    current_state = get_robot_state(robot_id)
    
    # 일반 작업 할당 (RCS 주도)
    return current_state in WORK_ASSIGNABLE_STATES
```

### 사용자 주도 작업 예외 처리

**길안내(GUIDING) 작업의 특수성:**

사용자가 VRC/GUI를 통해 직접 요청한 길안내 작업은 예외적으로 허용:

```python
def handle_create_user_task(request):
    """DMC로부터 사용자 작업 생성 요청"""
    
    if request.user_initiated and request.task_type == "GUIDING":
        # 사용자 주도 길안내 = 상태 체크 무시
        # LISTENING, WAITING_DEST_INPUT 상태에서도 허용
        
        task_id = generate_task_id()
        goal = GuidePerson.Goal()
        goal.dest_location = request.dest_location
        goal.user_initiated = True  # 플래그 설정
        
        send_action_goal(request.robot_id, goal)
        
        return Response(success=True, task_id=task_id)
    else:
        # 일반 작업 = 상태 체크 필수
        if can_assign_task_to_dmc(request.robot_id, request.task_type):
            # ... 작업 할당 로직
            pass
        else:
            return Response(
                success=False,
                message="로봇이 작업 불가 상태입니다"
            )
```

### 작업 시작 주체 구분

| 주체 | 작업 타입 | 상태 체크 | 허용 상태 |
|------|-----------|-----------|-----------|
| **RCS 주도** | 책장 정리, 도서 픽업, 좌석 정리, 반납도서 정리 | 필수 | IDLE, ROAMING |
| **사용자 주도** | 길안내 (GUIDING) | 무시 | LISTENING, WAITING_DEST_INPUT 포함 |

**핵심:**
- RCS가 먼저 작업을 할당할 때는 IDLE/ROAMING만 허용
- DMC가 사용자 요청으로 작업 생성을 요청할 때는 상태 무관하게 허용
- `user_initiated` 플래그로 구분

---