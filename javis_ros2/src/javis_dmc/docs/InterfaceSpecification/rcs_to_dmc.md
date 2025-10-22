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

📡 Topic 정의
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
uint8 ROAMING = 11
uint8 EMERGENCYC_STOP = 98
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
uint8 SELECT_DEST = 109
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
🔁 상태 변화 범위
구분	범위
Main	0 ~ 99
Sub	100 ~ 199

