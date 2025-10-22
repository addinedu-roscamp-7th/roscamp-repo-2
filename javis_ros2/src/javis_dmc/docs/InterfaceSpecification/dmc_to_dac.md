# DMC <-> DAC 통신 정리 문서

> 작성자: 김우영  
> 용도: DMC와 DAC 간의 Arm Control Action/Service 인터페이스 정리  
> 대상: 도서관 로봇 Dobby

---

## 📦 인터페이스 요약

| From | To   | Protocol | 인터페이스 항목     | 메시지 형식                              |
|------|------|----------|----------------------|------------------------------------------|
| DMC  | DAC  | Action   | 도서 픽업 동작        | `dobby1/arm/pick_book`                   |
| DMC  | DAC  | Action   | 도서 보관 동작        | `dobby1/arm/place_book`                  |
| DMC  | DAC  | Action   | 반납도서 회수         | `dobby1/arm/collect_returned_books`      |
| DMC  | DAC  | Action   | 도서 재배치           | `dobby1/arm/sort_book`                   |
| DMC  | DAC  | Action   | 좌석 정리 작업         | `dobby1/arm/clean_desk`                  |
| DMC  | DAC  | Action   | 쓰레기 수거           | `dobby1/arm/collect_trash`               |
| DMC  | DAC  | Action   | 쓰레기 배출           | `dobby1/arm/dispose_trash`               |
| DMC  | DAC  | Service  | 암 자세 변경           | `dobby1/arm/change_pose`                 |

---

## 🎯 Action 정의

### 도서 픽업 – `PickBook.action`

```action
# Goal
string book_id
geometry_msgs/Pose book_pose
int32 carrier_slot_id
---
# Result
bool success
string book_id
string message
---
# Feedback
string status
string current_action
```

### 도서 보관 – `PlaceBook.action`

```action
# Goal
string book_id
int32 carrier_slot_id
geometry_msgs/Pose storage_box_pose
int32 storage_box_id
---
# Result
bool success
string book_id
string message
---
# Feedback
string status
string current_action
```

### 반납도서 회수 – `CollectReturnedBooks.action`

```action
# Goal
string[] book_ids
geometry_msgs/Pose[] book_poses
int32 carrier_slot_ids
---
# Result
bool success
int32[] books_collected
string[] collected_book_ids
string message
---
# Feedback
string current_book_id
int32 books_collected_count
```

### 도서 재배치 – `RearrangeBook.action`

```action
# Goal
string book_id
int32 carrier_slot_id
geometry_msgs/Pose bookshelf_pose
string bookshelf_id
---
# Result
bool success
string book_id
string message
---
# Feedback
string status
string current_action
```

### 좌석 정리 – `CleanDesk.action`

```action
# Goal
int32 task_id
int32 seat_id
geometry_msgs/Pose seat_pose
---
# Result
int32 task_id
bool success
int32 trash_collected
string[] trash_types
string message
---
# Feedback
string current_action
int32 trash_collected_count
```

### 쓰레기 수거 – `CollectTrash.action`

```action
# Goal
string trash_type
geometry_msgs/Pose trash_pose
---
# Result
bool success
string message
---
# Feedback
# 정의 예정
```

### 쓰레기 배출 – `DisposeTrash.action`

```action
# Goal
geometry_msgs/Pose trash_bin_pose
---
# Result
bool success
string message
---
# Feedback
# 정의 예정
```

## 🛠️ Service 정의

### 암 자세 변경 – `ChangeArmPose.srv`

```srv
uint8 INITIAL_POSE = 0
uint8 CUSTOM_POSE = 1

# Request
uint8 pose_type
geometry_msgs/Pose target_pose
---
# Response
bool success
string message
```
