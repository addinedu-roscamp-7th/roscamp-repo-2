# DMC <-> AIS(DVS) 통신 정리 문서

> 작성자: 김종명  
> 용도: DMC와 AIS 간의 주행 카메라 기반 피안내자 인식 및 추적 인터페이스 정의  
> 대상: 도서관 로봇 Dobby

---

## 📦 인터페이스 요약

| From | To   | Protocol | 인터페이스 항목     | 메시지 형식                             |
|------|------|----------|----------------------|-----------------------------------------|
| DMC  | AIS  | Service  | 추적 모드 변경        | `dobby1/ai/change_tracking_mode` (`REGISTRATION_MODE`, `TRACKING_MODE`, `IDLE_MODE`) |
| AIS  | DMC  | Topic    | 피안내자 추적 상태     | `dobby1/ai/tracking/status`             |

> **Note:** 피안내자 등록은 별도 서비스 없이 `change_tracking_mode` 요청에서 `REGISTRATION_MODE`로 전환하면 AIS가 카메라 프레임을 기반으로 추적 대상을 확보하고, 이후 `TRACKING_MODE`로 전환해 추적을 지속합니다. 등록 여부와 추적 ID는 `tracking/status` 토픽을 통해 확인합니다. 네임스페이스만 `dobby2/…`로 바뀌는 동일한 인터페이스를 두 번째 로봇에 적용합니다.

---

## 🛠️ Service 정의

### 추적 모드 변경 – `ChangeTrackingMode.srv`

```srv
uint8 REGISTRATION_MODE = 0
uint8 TRACKING_MODE = 1
uint8 IDLE_MODE = 2

# Request
uint8 mode                             # 추적 모드 설정
string tracking_id                     # 추적 대상 ID
---
# Response
bool success                           # 성공 여부
string message                         # 결과 메시지
```

## 📡 Topic 정의

### 피안내자 추적 상태 – `TrackingStatus.msg`

```msg
std_msgs/Header header                 # 메시지 헤더
bool person_detected                   # 사람 감지 여부
string tracking_id                     # 추적 대상 ID
geometry_msgs/Pose person_pose         # 사람 위치 (Pose)
float32 distance_to_person             # 거리 (m)
float32 confidence                     # 감지 신뢰도 (0~1)
bool is_lost                           # 추적 손실 여부
float32 time_since_last_seen           # 마지막 감지 후 경과 시간 (초)
```
