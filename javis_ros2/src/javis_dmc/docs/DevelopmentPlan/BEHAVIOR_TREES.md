# JAVIS DMC 행동 트리 설계 (v1)

이 문서는 JAVIS DMC의 새로운 아키텍처인 행동 트리의 전체 구조를 정의합니다.

## 제어 노드 범례

- **`Fallback` (또는 `Selector`):** OR 조건. 자식 노드를 순서대로 실행하며, 하나라도 **성공**하면 즉시 멈추고 **성공**을 반환. (플랜 B)
- **`Sequence`:** AND 조건. 자식 노드를 순서대로 실행하며, 하나라도 **실패**하면 즉시 멈추고 **실패**를 반환.
- **`Parallel`:** 자식 노드들을 동시에 실행.

---

## 1. 최상위 트리: `RootTree`

로봇의 모든 의사결정을 총괄하는 최상위 트리입니다. 가장 우선순위가 높은 행동(비상 정지)부터 차례대로 확인합니다.

```
Fallback (ID: Root)
|
|-- Sequence (1순위: 비상 정지 처리)
|   |-- IsEmergencyStopped?
|   `-- HandleEmergencyStop
|
|-- Sequence (2순위: 배터리 위험 강제 충전)
|   |-- IsBatteryCritical?
|   `-- RunSubTree(tree_name="ForceChargeTree")
|
|-- Sequence (3순위: 로밍 중 자발적 충전)
|   |-- IsCurrentMode(modes="ROAMING")
|   |-- ShouldGoToChargerFromRoaming?
|   `-- RunSubTree(tree_name="GoToChargerTree")
|
|-- Sequence (4순위: 새로운 작업 처리)
|   |-- IsNewTaskFromRCS?
|   |-- IsBatterySufficientForNewTask?
|   |-- AcceptNewTaskFromRCS
|   `-- RunSubTree(tree_name="TaskDispatcherTree")
|
`-- RunSubTree(tree_name="PatrolOrIdleTree") // 5순위: 순찰 또는 대기
```

---

## 2. 작업 분기 트리: `TaskDispatcherTree`

`AcceptNewTaskFromRCS` 노드가 블랙보드에 저장한 `current_task.type`에 따라 적절한 작업 서브 트리를 실행시키는 분기점입니다.

```
Fallback (ID: TaskDispatcher)
|
|-- Sequence (길안내 작업인가?)
|   |-- CheckBlackboardValue(key="current_task.type", value="GUIDING")
|   `-- RunSubTree(tree_name="GuidingTaskTree")
|
|-- Sequence (책 수집 작업인가?)
|   |-- CheckBlackboardValue(key="current_task.type", value="PICKUP_BOOK")
|   `-- RunSubTree(tree_name="PickupTaskTree")
|
|-- Sequence (책 정리 작업인가?)
|   |-- CheckBlackboardValue(key="current_task.type", value="RESHELVING_BOOK")
|   `-- RunSubTree(tree_name="ReshelvingTaskTree")
|
`-- ReportTaskFailure(message="Unknown task type") // 지원하지 않는 작업 타입
```
*참고: `CheckBlackboardValue`는 블랙보드의 특정 키 값이 주어진 값과 일치하는지 확인하는 새로운 조건 노드입니다.*

---

## 3. 시나리오별 작업 트리

### 3.1 길 안내 트리: `GuidingTaskTree`

길 안내 작업의 전체 과정을 정의합니다. 작업 시작점(음성/GUI)을 처리하는 로직이 포함됩니다.

```
Sequence (ID: GuidingTask)
|
|-- Fallback (1. 목적지 설정)
|   |-- Sequence (GUI로 목적지 선택)
|   |   |-- WaitForGuiInput(timeout=60.0)
|   |   `-- SetBlackboard(key="current_task.destination", value_from="gui_selected_destination")
|   `-- Sequence (음성으로 목적지 설정)
|       |-- RunListeningSequence(timeout=20.0)
|       `-- SetBlackboard(key="current_task.destination", value_from="user_entities.destination")
|
|-- Sequence (2. 사용자 스캔 및 등록)
|   |-- PlayTTS(message="안내를 시작하겠습니다. 잠시 저를 바라봐주세요.")
|   |-- SetAITrackingMode(mode="registration")
|   `-- Wait(seconds=5) // 사용자가 로봇을 바라볼 시간
|
|-- Sequence (3. 목적지로 이동)
|   |-- SetAITrackingMode(mode="tracking")
|   |-- FollowUser(destination_key="current_task.destination")
|   `-- SetAITrackingMode(mode="idle")
|
|-- PlayTTS(message="목적지에 도착했습니다. 안내를 종료합니다.")
`-- ClearCurrentTask(success=true)
```
*참고: `SetBlackboard`는 특정 키에 값을 설정하는 새로운 액션 노드입니다.*

### 3.2 책 수집 트리: `PickupTaskTree`

책을 수집하여 로봇 내부 보관함에 넣는 과정입니다.

```
Sequence (ID: PickupTask)
|
|-- MoveTo(destination_key="current_task.shelf_approach_location") // 1. 책장으로 이동
|-- ApproachTo(target_key="current_task.shelf_id", distance=0.1) // 2. 책장에 정밀 접근
|-- MoveArmToPose(pose_name="observe") // 3. 팔을 관찰 자세로
|-- PickObject // 4. 책 집기 (AI 인식과 연동)
|-- MoveArmToPose(pose_name="carry") // 5. 팔을 운반 자세로
|-- MoveTo(destination_key="current_task.storage_approach_location") // 6. 내부 보관함으로 이동
|-- PlaceObject // 7. 책 내려놓기
|-- MoveArmToPose(pose_name="home") // 8. 팔을 기본 자세로
`-- ClearCurrentTask(success=true)
```

---

## 4. 기타 보조 트리

### 4.1 충전 트리: `GoToChargerTree` / `ForceChargeTree`

두 트리는 거의 동일하며, 이동 중 다른 작업을 허용할지 여부만 다릅니다.

```
Sequence (ID: GoToCharger)
|
|-- StopMotion // 현재 주행 중단
|-- MoveTo(destination_key="charger_location_pose") // 충전소로 이동
|-- ApproachTo(target_key="charger_id", distance=0.05) // 충전기에 정밀 접근
`-- StartChargingAction // 충전 시작 (물리적 연결 등)
```

### 4.2 순찰/대기 트리: `PatrolOrIdleTree`

`RootTree`의 가장 마지막에 실행되며, 로봇의 기본 상태를 담당합니다.

```
Fallback (ID: PatrolOrIdle)
|
|-- Sequence (로밍 모드일 경우)
|   |-- IsCurrentMode(modes="ROAMING")
|   `-- StartPatrol
|
`-- Wait(seconds=1.0) // IDLE 모드일 경우, 1초 대기 후 다시 RootTree 처음부터 반복
```
