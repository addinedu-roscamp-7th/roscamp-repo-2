# DMC <-> DDC 통신 정리 문서

> 작성자: 김우영  
> 용도: DMC와 DDC 간의 주행, 안내, 도킹 등 Action/Service/Topic 정의  
> 대상: 도서관 로봇 Dobby

---

## 📦 인터페이스 요약

| From | To   | Protocol | 인터페이스 항목 | 메시지 형식                         |
|------|------|----------|------------------|-------------------------------------|
| DMC  | DDC  | Action   | 목적지 이동       | `dobby1/drive/move_to_target`       |
| DMC  | DDC  | Action   | 안내 주행         | `dobby1/drive/guide_navigation`     |
| DDC  | DMC  | Service  | 주행 제어 명령     | `dobby1/drive/control_command`      |
| DDC  | RMS/RCS | Topic | 현재 위치 발행      | `dobby1/status/current_pose`        |

---

## 🎯 Action 정의

### 목적지 이동 – `MoveToTarget.action`

```action
# Goal
geometry_msgs/Pose target_pose              # 목적지 위치 및 자세
string location_name                        # 위치 이름 (예: "Shelf_A1", "Return_Desk")
---
# Result
bool success
geometry_msgs/Pose final_pose               # 최종 도착 위치
---
# Feedback
geometry_msgs/Pose current_pose
float32 distance_remaining
```

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
