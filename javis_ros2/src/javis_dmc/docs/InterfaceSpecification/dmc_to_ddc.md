# DMC <-> DDC 통신 정리 문서

> 작성자: 김우영  
> 용도: DMC와 DDC 간의 주행, 안내, 도킹 등 Action/Service/Topic 정의  
> 대상: 도서관 로봇 Dobby

---

## 📦 인터페이스 요약

| From | To   | Protocol | 인터페이스 항목 | 메시지 형식                                   |
|------|------|----------|------------------|-----------------------------------------------|
| DMC  | DDC  | Action   | 목적지 이동       | `dobby1/drive/navigate_to_pose` (NAV2 표준)   |
| DMC  | DDC  | Action   | 안내 주행         | `dobby1/drive/guide_navigation`               |
| DDC  | DMC  | Service  | 주행 제어 명령     | `dobby1/drive/control_command`                |
| DDC  | RMS/RCS | Topic | 현재 위치 발행      | `dobby1/status/current_pose`                  |

---

## 🎯 Action 정의

### 목적지 이동 – `NavigateToPose.action` (nav2_msgs)

```action
# Goal
geometry_msgs/PoseStamped pose      # 목적지 위치/자세 + frame_id
string behavior_tree                # 사용할 BT (기본값: 빈 문자열 → 기본 트리)
---
# Result
uint16 error_code                   # nav2 오류 코드 (0: NONE)
string error_msg                    # 오류 상세
---
# Feedback
geometry_msgs/PoseStamped current_pose
builtin_interfaces/Duration navigation_time
builtin_interfaces/Duration estimated_time_remaining
int16 number_of_recoveries
float32 distance_remaining
```

> **비고:**  
> - `pose.header.frame_id`는 기본적으로 `map`을 사용하며, `pose.header.stamp`는 Goal 생성 시각을 기록한다.  
> - NAV2 기본 BT를 사용할 경우 `behavior_tree`를 빈 문자열로 둔다.  
> - DMC 내부에서는 `Pose2D` 기반 좌표를 관리하되, NAV2에 전달하기 전에 `PoseStamped`로 변환한다.

### 안내 주행 – `GuideNavigation.action`

```action
# Goal
geometry_msgs/Pose destination              # 목적지 위치 및 방향
float32 max_speed                           # 최대 주행 속도 (m/s)
float32 person_follow_distance              # 사람과의 거리 유지 (m)
---
# Result
bool success
geometry_msgs/Pose2D final_location
string termination_reason                   # 작업 종료 사유 (예: "목표 도달")
---
# Feedback
geometry_msgs/Pose2D current_location
float32 distance_remaining
string status                               # "경로 계획 중", "이동 중", 등
bool person_detected
```

## 🛠️ Service 정의

### 주행 제어 명령 – `DriveControlCommand.srv`

```srv
uint8 STOP = 0
uint8 RESUME = 1

# Request
uint8 command                   # 0: 정지, 1: 재개
string reason                   # 명령 사유
---
# Response
bool success
string current_state
```

## 📡 Topic 정의

### 현재 로봇 위치 – `CurrentPose.msg`

```msg
geometry_msgs/Pose2D pose       # 로봇의 현재 위치 및 방향
```
