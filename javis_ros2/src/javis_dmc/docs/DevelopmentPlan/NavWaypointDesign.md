# Dobby 순찰(ROAMING) NAV2 연동 설계 v0.1

## 1. 목표

- AUTONOMY 모드에서 로봇이 웨이포인트 경로를 순찰하면서 작업 할당 요청을 수락할 수 있도록 한다.
- 차기 개발자가 `Drive Interface`와 DDC(Node) 사이의 책임을 명확히 이해하고 재사용 가능한 메시지 포맷을 확보한다.
- Mock/실 장비가 동일한 인터페이스로 동작해 시나리오 테스트(Test GUI + Mock)와 실제 NAV2 환경을 쉽게 전환하도록 한다.

## 2. 구성요소 역할

| 구성요소 | 책임 | 비고 |
| :--- | :--- | :--- |
| DMC (`JavisDmcNode`) | 순찰 시작/정지 트리거, 웨이포인트 목록 로딩, Drive 인터페이스 호출 | 순찰 상태 전이 로직은 `DmcStateMachine`에 추가 |
| Drive Interface (`RosDriveInterface`) | NAV2 Waypoint 액션 클라이언트 생성, Goal 전송/취소, 피드백 전달 | 기존 `move_to_target`, `guide_navigation`과 동일한 callback group 공유 |
| Dobby Drive Controller (DDC) | NAV2 Waypoint 실행 서버, 경로 리스트 소비, 주행 상태/도착 이벤트 발행 | 실행 실패 시 오류 코드/메시지를 Result로 제공 |
| NAV2 | Waypoint Follower 플러그인, 위치 추정/경로 계획 | 기존 Nav2 스택 활용, Waypoint 벡터로 이동 |

## 3. 메시지/인터페이스 안

### 3.1 Waypoint Action 정의 제안

- 패키지: `javis_interfaces/action/PatrolWaypoints.action` (신규)

```
# Goal
geometry_msgs/PoseStamped[] waypoints
float32 loop_pause_sec
bool loop_forever

---
# Result
bool success
uint8 error_code
string message
int32 completed_loops

---
# Feedback
uint32 current_index
float32 progress  # 0~1, 전체 경로 대비
```

- `DriveControlCommand` 서비스와 동일 네임스페이스(`{robot}/drive/patrol_waypoints`)에서 Action 서버 제공.
- DDC는 NAV2 Waypoint Follower 액션(`nav2_msgs/action/FollowWaypoints`)을 내부에서 호출하고, 진행 상황을 `PatrolWaypoints` 피드백으로 변환해 전달한다.

### 3.2 컨피그 파일 구조

`config/patrol_routes.yaml` (신규)

```yaml
routes:
  default:
    loop: true
    loop_pause_sec: 5.0
    waypoints:
      - { x: 1.2, y: 2.4, theta: 1.57 }
      - { x: 4.0, y: 2.4, theta: 3.14 }
      - { x: 4.0, y: -1.2, theta: -1.57 }
      - { x: 1.2, y: -1.2, theta: 0.0 }
  lobby:
    loop: false
    loop_pause_sec: 2.0
    waypoints:
      - { x: 0.5, y: 0.0, theta: 0.0 }
      - { x: 0.5, y: -3.5, theta: -1.57 }
```

- `JavisDmcNode`는 로봇 파라미터(`patrol_route_name`)로 사용할 경로 ID를 선택한다.
- Mock 환경에서는 동일 YAML을 파싱해 `MockDriveInterface`가 가짜 Pose 업데이트에 사용한다.

## 4. DMC 연동 흐름

1. AUTONOMY 모드 진입 시 `patrol_routes.yaml` 로드 → `PatrolRoute` 데이터 클래스에 캐시.
2. `DmcStateMachine`이 `ROAMING` 상태로 전환되면 `_start_patrol()` 호출:
   - Drive 인터페이스 초기화 확인.
   - `RosDriveInterface.start_patrol(route)` (신규) 호출해 Action Goal 전송.
3. `PatrolWaypoints` 결과/피드백 → 순찰 종료 로그 기록 및 `describe_state_machine` 런타임 정보(`patrol_active`) 갱신.
   - 성공 시 INFO, 실패 시 WARN으로 기록하고 GUI는 `patrol_active` 변화를 구독한다.
4. 목표 작업 할당(예: Guide) 수신 시 `cancel_patrol()` 호출 → NAV2 Action cancel → DMC 상태 변경.
5. 주행 실패 시 Result의 오류 코드를 기반으로 `MainState.MAIN_ERROR` 또는 `ROAMING`으로 복귀 결정.

> 실행 시 `patrol_route` ROS 파라미터(기본값 `default`)를 통해 사용할 경로를 선택하고, `use_mock_interfaces`가 `true`일 때는 Mock 순찰 로직이 활성화된다.

## 5. 인터페이스 확장 개요

### 5.1 `DriveInterface`/`RosDriveInterface`

- 신규 메서드
  - `start_patrol(route: PatrolRoute) -> Future`
  - `cancel_patrol(reason: str = "interrupt") -> bool`
  - `is_patrol_active() -> bool`
- 내부 상태
  - `_patrol_client: ActionClient` → `PatrolWaypoints`
  - `_patrol_goal_handle`
  - `_patrol_route_cache`
- Mock 구현
  - Waypoint 목록을 순회하며 `_current_pose` 업데이트.
  - `loop_forever` 설정에 따라 반복/종료 제어.

### 5.2 `JavisDmcNode`

- `_load_patrol_routes()` 함수를 추가해 YAML 파싱.
- `_start_patrol()` / `_stop_patrol()`로 순찰 시작·중단 로직을 분리하고, 상태 머신은 기존 `ROAMING` 상태를 그대로 사용한다.
- 작업 종료 후 로봇이 AUTONOMY 모드라면 `_cleanup_after_task()`에서 `_start_patrol`을 재호출한다.

## 6. 오류 처리/Failover

| 상황 | Drive Result | DMC 대응 | 후속 상태 |
| :--- | :--- | :--- | :--- |
| NAV2 액션 서버 없음 | Goal rejected | 로그 경고 + `MainState.IDLE` (STANDBY) | 작업 수락 제한 |
| 경로 상 충돌/전도 | `error_code = 2`, message 포함 | `MainState.MAIN_ERROR`, Admin GUI에 알림 | 수동 복구 |
| 작업 할당으로 중단 | cancel → success true | `ROAMING`에서 유지 | 작업 완료 후 자동 재개 |
| 배터리 경고 | `_sync_battery_state` → Drain 상태 | `determine_post_task_state`로 `MOVING_TO_CHARGER` | 순찰 중단 |
| Mock 실패 | `set_mock_response`로 실패 설정 | 경고 로그 후 순찰 미시작 | QA 확인 |

## 7. 문서/테스트 영향

- `DevelopmentPlan.md` §5.6, §8.2 갱신 (완료 트리거).
- `InterfaceSpecification/dmc_to_ddc.md`에 `PatrolWaypoints` 인터페이스 추가.
- Mock/실 장비 모두 `patrol_routes.yaml`을 사용하도록 Launch/파라미터 조정.
- Test GUI에서 “순찰 시작/중단” 버튼과 시나리오 정의를 추가해 QA가 확인할 수 있도록 한다.

## 8. TODO

- `javis_interfaces` 패키지에 `PatrolWaypoints.action` 추가.
- DDC 측 NAV2 연결 로직 구현 및 문서화.
- Mock 테스트: Waypoint를 순회하는 동안 `current_pose`가 업데이트되는지 확인.
- Test GUI: 순찰 중 피드백/현재 인덱스 표시.
