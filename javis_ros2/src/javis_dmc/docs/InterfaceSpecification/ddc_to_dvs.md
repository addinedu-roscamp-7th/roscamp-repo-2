# DDC <-> DVS 통신 정리 문서

> 작성자: 김종명  
> 용도: 주행 시스템(DDC)과 AI 비전 시스템(DVS/AIS) 간의 장애물 감지 및 피안내자 추적 상태 공유  
> 대상: 도서관 로봇 Dobby

---

## 📡 Topic 인터페이스 요약

| From | To   | Protocol | 인터페이스 항목       | 메시지 형식                         |
|------|------|----------|------------------------|-------------------------------------|
| DVS  | DDC  | Topic    | 장애물 감지             | `dobby1/ai/obstacles`               |
| AIS  | DDC  | Topic    | 피안내자 추적 상태       | `dobby1/ai/tracking/status`         |

---

## 📍 Topic 메시지 정의

### 1. 장애물 감지 – `Obstacles.msg`

```msg
bool dynamic            # 장애물 유형 (정적/동적)
float32 x               # 정규화된 화면 좌표 (X: 0.0 ~ 1.0)
float32 y               # 정규화된 화면 좌표 (Y: 0.0 ~ 1.0)
float32 depth           # 깊이 정보 (단위: 미터)
📌 설명

dynamic:

false – 정적 장애물 (예: 벽, 책장 등)

true – 동적 장애물 (예: 사람, 움직이는 객체 등)

x, y:

화면 상에서 정규화된 좌표 (0.0 ~ 1.0)

depth:

뎁스 카메라 기준 거리 정보 (단위: m)

🕒 발행 주기

장애물 감지 시마다 실시간 발행

2. 피안내자 추적 상태 – TrackingStatus.msg
msg
코드 복사
std_msgs/Header header             # ROS 표준 메시지 헤더
bool person_detected               # 사람 감지 여부
string tracking_id                 # 추적 중인 대상 ID
geometry_msgs/Pose person_pose     # 사람의 위치 (3D 좌표)
float32 distance_to_person         # 로봇과 사람 간 거리 (m)
float32 confidence                 # 감지 신뢰도 (0.0 ~ 1.0)
bool is_lost                       # 추적 실패 여부
float32 time_since_last_seen       # 마지막 감지 이후 경과 시간 (초)
📌 주요 상태 필드

person_detected: 현재 사람을 시야 내에서 인식 중인지 여부

is_lost: 추적 대상을 놓쳤는지 여부

confidence: 인식 정확도

time_since_last_seen: 마지막 감지 이후 시간