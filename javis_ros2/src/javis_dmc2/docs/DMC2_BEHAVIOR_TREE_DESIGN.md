# JAVIS DMC2 행동트리 기반 재설계 문서

**패키지명**: `javis_dmc2`
**노드명**: `dmc2_node`
**작성일**: 2025-03-15
**기반 프레임워크**: BehaviorTree.CPP (py_trees 또는 BehaviorTree.CPP Python binding)
**목적**: 현재 State Machine 기반 DMC를 Behavior Tree로 전환하여 가독성, 유지보수성, 확장성 향상

---

## 목차

1. [개요](#1-개요)
2. [현재 아키텍처의 한계와 개선 방향](#2-현재-아키텍처의-한계와-개선-방향)
3. [행동트리 기술 스택 선정](#3-행동트리-기술-스택-선정)
4. [패키지 구조](#4-패키지-구조)
5. [행동트리 설계](#5-행동트리-설계)
6. [커스텀 노드 구현 명세](#6-커스텀-노드-구현-명세)
7. [Blackboard 데이터 구조](#7-blackboard-데이터-구조)
8. [인터페이스 통합 전략](#8-인터페이스-통합-전략)
9. [마이그레이션 로드맵](#9-마이그레이션-로드맵)
10. [테스트 계획](#10-테스트-계획)

---

## 1. 개요

### 1.1 배경

현재 `javis_dmc`는 **State Machine (DmcStateMachine)** 기반으로 구현되어 있으며, 각 작업은 **Executor 패턴**으로 처리됩니다.

**문제점**:
- 복잡한 조건 분기가 코드 곳곳에 산재 (if-else 중첩)
- 새로운 작업 추가 시 여러 파일 수정 필요 (`dmc_node.py`, `state_enums.py`, `main_states.py`)
- 작업 흐름을 파악하기 어려움 (코드 1800줄 이상)
- 디버깅 및 시각화 도구 부족

### 1.2 행동트리 도입 효과

- ✅ **가시성**: Groot 등의 시각화 도구로 로봇 동작 흐름을 한눈에 파악
- ✅ **모듈화**: 작업별 서브트리로 분리하여 독립적 개발/테스트 가능
- ✅ **재사용성**: 공통 로직(충전, 오류 처리)을 서브트리로 정의하여 재사용
- ✅ **확장성**: 새로운 작업 추가 시 XML 트리 파일만 수정
- ✅ **디버깅**: 실시간 트리 상태 모니터링 및 로깅

### 1.3 설계 원칙

1. **기존 인터페이스 재사용**: `RosDriveInterface`, `RosArmInterface`, `RosAIInterface` 등을 그대로 활용
2. **점진적 마이그레이션**: 기존 DMC와 병행 운영 가능하도록 별도 패키지 생성
3. **ROS2 Action 호환**: RCS의 작업 할당 메커니즘(Action Server)을 유지
4. **Groot 호환**: BehaviorTree.CPP XML 포맷으로 작성하여 Groot에서 편집 가능

---

## 2. 현재 아키텍처의 한계와 개선 방향

### 2.1 State Machine의 복잡도

**현재 구조** (`main_states.py`):
```python
class DmcStateMachine:
    def determine_post_task_state(self, success, at_charger, battery_warning, battery_critical):
        if battery_critical:
            return MainState.FORCE_MOVE_TO_CHARGER
        if battery_warning:
            return MainState.CHARGING if at_charger else MainState.MOVING_TO_CHARGER
        if not success and self.mode == RobotMode.AUTONOMY:
            return MainState.ROAMING
        # ... 더 많은 조건들
```

**문제점**:
- 상태 전이 조건이 메서드 내부에 하드코딩
- 우선순위 변경 시 코드 수정 필요
- 시각화 불가능

**행동트리 개선안**:
```xml
<Fallback name="PostTaskHandler">
  <Sequence>
    <IsBatteryCritical/>
    <RunSubTree tree="ForceChargeTree"/>
  </Sequence>
  <Sequence>
    <IsBatteryWarning/>
    <RunSubTree tree="GoToChargerTree"/>
  </Sequence>
  <Sequence>
    <IsAutonomyMode/>
    <RunSubTree tree="PatrolTree"/>
  </Sequence>
  <SetState state="IDLE"/>
</Fallback>
```

### 2.2 Executor 패턴의 한계

**현재 구조**:
- `PickupExecutor`, `GuidingExecutor` 등이 각각 실행 로직 구현
- 서브 상태 전환이 Executor 내부에 숨겨져 있음
- 공통 로직(오류 처리, 재시도) 중복 코드

**행동트리 개선안**:
- 각 Executor의 로직을 서브트리로 변환
- 공통 로직을 Decorator 노드로 구현 (Retry, Timeout 등)

---

## 3. 행동트리 기술 스택 선정

### 3.1 후보 기술

| 라이브러리 | 언어 | ROS2 통합 | Groot 지원 | 평가 |
|-----------|------|-----------|-----------|------|
| **BehaviorTree.CPP** | C++ | ✅ (ros2_behavior_tree) | ✅ | **권장** |
| py_trees | Python | ✅ (py_trees_ros) | ❌ | 대안 |
| py_trees_ros_viewer | Python | ✅ | ⚠️ (제한적) | 보조 |

### 3.2 최종 선정: BehaviorTree.CPP + Python Binding

**선정 이유**:
1. **Groot 완벽 지원**: XML 기반 트리 편집 및 실시간 모니터링
2. **ROS2 Community Package**: `ros2_behavior_tree` 패키지로 ROS2 Action/Service 노드 제공
3. **Python Binding 가능**: pybind11 기반 Python wrapper 존재
4. **풍부한 예제**: Navigation2, MoveIt2 등이 사용 중

**설치**:
```bash
# BehaviorTree.CPP
sudo apt install ros-jazzy-behaviortree-cpp

# Groot (시각화 도구)
sudo snap install groot

# Python binding (선택)
pip install py-trees py-trees-ros
```

### 3.3 대안: py_trees (Pure Python)

**선정 이유**:
- 기존 DMC가 Python이므로 학습 곡선 낮음
- ROS2 통합 패키지 (`py_trees_ros`) 제공
- Blackboard, Decorator 등 기본 기능 완비

**단점**:
- Groot 미지원 (자체 웹 뷰어 사용)
- 성능이 C++ 대비 낮음 (실시간성 요구 시)

**권장 방안**:
- **Phase 1**: py_trees로 프로토타입 개발 (Python 친화적)
- **Phase 2**: 성능 이슈 발생 시 BehaviorTree.CPP로 포팅

---

## 4. 패키지 구조

### 4.1 디렉토리 구조

```
javis_dmc2/
├── javis_dmc2/
│   ├── __init__.py
│   ├── dmc2_node.py                    # 메인 노드 (BT 실행 엔진)
│   ├── bt_nodes/                       # 커스텀 BT 노드 구현
│   │   ├── __init__.py
│   │   ├── conditions/                 # 조건 노드
│   │   │   ├── battery_conditions.py  # IsBatteryCritical, IsBatteryWarning
│   │   │   ├── mode_conditions.py     # IsCurrentMode, IsNewTaskFromRCS
│   │   │   └── sensor_conditions.py   # IsUserInView, IsWakeWordDetected
│   │   ├── actions/                    # 액션 노드
│   │   │   ├── drive_actions.py       # MoveTo, ApproachTo, StartPatrol
│   │   │   ├── arm_actions.py         # MoveArmToPose, PickObject, PlaceObject
│   │   │   ├── interaction_actions.py # PlayTTS, RunListeningSequence
│   │   │   └── task_actions.py        # AcceptNewTaskFromRCS, ClearCurrentTask
│   │   └── decorators/                 # 데코레이터 노드
│   │       ├── retry.py                # Retry(N)
│   │       └── timeout.py              # Timeout(seconds)
│   ├── interfaces/                     # 기존 인터페이스 재사용
│   │   ├── __init__.py
│   │   └── ... (심볼릭 링크 또는 복사)
│   └── blackboard/
│       ├── __init__.py
│       └── blackboard_manager.py       # Blackboard 데이터 관리
├── config/
│   ├── dmc2_params.yaml                # ROS2 파라미터
│   ├── battery_config.yaml             # 배터리 설정 (재사용)
│   └── bt_config.yaml                  # BT 설정 (트리 파일 경로 등)
├── trees/                              # 행동트리 XML 파일
│   ├── root_tree.xml                   # 최상위 트리
│   ├── task_dispatcher.xml             # 작업 분기 트리
│   ├── tasks/
│   │   ├── guiding_task.xml
│   │   ├── pickup_task.xml
│   │   ├── reshelving_task.xml
│   │   ├── cleaning_task.xml
│   │   └── sorting_task.xml
│   └── utilities/
│       ├── go_to_charger.xml
│       ├── force_charge.xml
│       └── patrol_or_idle.xml
├── launch/
│   ├── dmc2_single.launch.py
│   ├── dmc2_multi.launch.py
│   └── dmc2_test.launch.py
├── test/
│   ├── test_bt_nodes.py                # 노드 유닛 테스트
│   ├── test_subtrees.py                # 서브트리 통합 테스트
│   └── test_blackboard.py
├── docs/
│   └── bt_node_reference.md            # 커스텀 노드 레퍼런스
├── package.xml
├── setup.py
└── README.md
```

### 4.2 setup.py 설정

```python
from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'javis_dmc2'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'trees'), glob('trees/**/*.xml', recursive=True)),
    ],
    install_requires=['setuptools', 'rclpy', 'py_trees', 'py_trees_ros'],
    zip_safe=True,
    maintainer='kim jong myung',
    maintainer_email='jongbob1918@gmail.com',
    description='JAVIS DMC with Behavior Tree',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dmc2_node = javis_dmc2.dmc2_node:main',
        ],
    },
)
```

---

## 5. 행동트리 설계

### 5.1 최상위 트리: `root_tree.xml`

```xml
<?xml version="1.0"?>
<root main_tree_to_execute="RootTree">
  <BehaviorTree ID="RootTree">
    <Fallback name="RootFallback">

      <!-- 1순위: 비상 정지 처리 -->
      <Sequence name="EmergencyHandler">
        <IsEmergencyStopped/>
        <HandleEmergencyStop/>
      </Sequence>

      <!-- 2순위: 배터리 위험 강제 충전 -->
      <Sequence name="CriticalBatteryHandler">
        <IsBatteryCritical/>
        <SubTree ID="ForceChargeTree"/>
      </Sequence>

      <!-- 3순위: 로밍 중 자발적 충전 -->
      <Sequence name="VoluntaryCharging">
        <IsCurrentMode modes="ROAMING"/>
        <ShouldGoToChargerFromRoaming/>
        <SubTree ID="GoToChargerTree"/>
      </Sequence>

      <!-- 4순위: 새로운 작업 처리 -->
      <Sequence name="NewTaskHandler">
        <IsNewTaskFromRCS/>
        <IsBatterySufficientForNewTask/>
        <AcceptNewTaskFromRCS/>
        <SubTree ID="TaskDispatcherTree"/>
      </Sequence>

      <!-- 5순위: 순찰 또는 대기 (기본 동작) -->
      <SubTree ID="PatrolOrIdleTree"/>

    </Fallback>
  </BehaviorTree>
</root>
```

### 5.2 작업 분기 트리: `task_dispatcher.xml`

```xml
<?xml version="1.0"?>
<root main_tree_to_execute="TaskDispatcherTree">
  <BehaviorTree ID="TaskDispatcherTree">
    <Fallback name="TaskDispatcher">

      <Sequence name="GuidingTask">
        <CheckBlackboard key="current_task.type" value="GUIDING"/>
        <SubTree ID="GuidingTaskTree"/>
      </Sequence>

      <Sequence name="PickupTask">
        <CheckBlackboard key="current_task.type" value="PICKUP_BOOK"/>
        <SubTree ID="PickupTaskTree"/>
      </Sequence>

      <Sequence name="ReshelvingTask">
        <CheckBlackboard key="current_task.type" value="RESHELVING_BOOK"/>
        <SubTree ID="ReshelvingTaskTree"/>
      </Sequence>

      <Sequence name="CleaningTask">
        <CheckBlackboard key="current_task.type" value="CLEANING_DESK"/>
        <SubTree ID="CleaningTaskTree"/>
      </Sequence>

      <Sequence name="SortingTask">
        <CheckBlackboard key="current_task.type" value="SORTING_SHELVES"/>
        <SubTree ID="SortingTaskTree"/>
      </Sequence>

      <!-- 지원하지 않는 작업 타입 -->
      <ReportTaskFailure message="Unknown task type"/>

    </Fallback>
  </BehaviorTree>
</root>
```

### 5.3 작업 서브트리 예시: `guiding_task.xml`

```xml
<?xml version="1.0"?>
<root main_tree_to_execute="GuidingTaskTree">
  <BehaviorTree ID="GuidingTaskTree">
    <Sequence name="GuidingSequence">

      <!-- 1. 목적지 설정 -->
      <Fallback name="DestinationInput">
        <Sequence name="GUIInput">
          <WaitForGuiInput timeout="60.0"/>
          <SetBlackboard key="current_task.destination" value_from="gui_selected_destination"/>
        </Sequence>
        <Sequence name="VoiceInput">
          <RunListeningSequence timeout="20.0"/>
          <SetBlackboard key="current_task.destination" value_from="user_entities.destination"/>
        </Sequence>
      </Fallback>

      <!-- 2. 사용자 스캔 및 등록 -->
      <Sequence name="UserRegistration">
        <PlayTTS message="안내를 시작하겠습니다. 잠시 저를 바라봐주세요."/>
        <SetAITrackingMode mode="registration"/>
        <Wait seconds="5.0"/>
        <IsUserInView/>  <!-- 사용자 등록 확인 -->
      </Sequence>

      <!-- 3. 목적지로 이동 (사용자 추종) -->
      <Sequence name="Navigation">
        <SetAITrackingMode mode="tracking"/>
        <FollowUser destination_key="current_task.destination"/>
        <SetAITrackingMode mode="idle"/>
      </Sequence>

      <!-- 4. 작업 완료 -->
      <PlayTTS message="목적지에 도착했습니다. 안내를 종료합니다."/>
      <ClearCurrentTask success="true"/>

    </Sequence>
  </BehaviorTree>
</root>
```

### 5.4 도서 픽업 서브트리: `pickup_task.xml`

```xml
<?xml version="1.0"?>
<root main_tree_to_execute="PickupTaskTree">
  <BehaviorTree ID="PickupTaskTree">
    <Sequence name="PickupSequence">

      <!-- 1. 책장으로 이동 -->
      <MoveTo destination_key="current_task.shelf_approach_location"/>

      <!-- 2. 책장에 정밀 접근 -->
      <ApproachTo target_key="current_task.shelf_id" distance="0.1"/>

      <!-- 3. 팔을 관찰 자세로 -->
      <MoveArmToPose pose_name="observe"/>

      <!-- 4. 책 집기 (재시도 3회) -->
      <Retry num_attempts="3">
        <PickObject/>
      </Retry>

      <!-- 5. 팔을 운반 자세로 -->
      <MoveArmToPose pose_name="carry"/>

      <!-- 6. 보관함으로 이동 -->
      <MoveTo destination_key="current_task.storage_approach_location"/>

      <!-- 7. 책 내려놓기 -->
      <PlaceObject/>

      <!-- 8. 팔을 기본 자세로 -->
      <MoveArmToPose pose_name="home"/>

      <!-- 9. 작업 완료 -->
      <ClearCurrentTask success="true"/>

    </Sequence>
  </BehaviorTree>
</root>
```

### 5.5 충전 서브트리: `go_to_charger.xml`

```xml
<?xml version="1.0"?>
<root main_tree_to_execute="GoToChargerTree">
  <BehaviorTree ID="GoToChargerTree">
    <Sequence name="ChargingSequence">

      <StopMotion/>
      <MoveTo destination_key="charger_location_pose"/>
      <ApproachTo target_key="charger_id" distance="0.05"/>
      <StartChargingAction/>

      <!-- 충전 완료 대기 -->
      <WaitForBatteryLevel level="80.0" timeout="3600.0"/>

    </Sequence>
  </BehaviorTree>
</root>
```

### 5.6 순찰/대기 서브트리: `patrol_or_idle.xml`

```xml
<?xml version="1.0"?>
<root main_tree_to_execute="PatrolOrIdleTree">
  <BehaviorTree ID="PatrolOrIdleTree">
    <Fallback name="PatrolOrIdle">

      <Sequence name="PatrolMode">
        <IsCurrentMode modes="ROAMING"/>
        <StartPatrol/>
      </Sequence>

      <!-- IDLE 모드일 경우 1초 대기 후 RootTree 재시작 -->
      <Wait seconds="1.0"/>

    </Fallback>
  </BehaviorTree>
</root>
```

---

## 6. 커스텀 노드 구현 명세

### 6.1 조건 노드 (Conditions)

#### 6.1.1 `IsBatteryCritical`

```python
# javis_dmc2/bt_nodes/conditions/battery_conditions.py
import py_trees
from javis_dmc2.blackboard import BlackboardManager

class IsBatteryCritical(py_trees.behaviour.Behaviour):
    '''배터리가 위험 수준인지 확인'''

    def __init__(self, name: str = "IsBatteryCritical"):
        super().__init__(name=name)
        self.blackboard = BlackboardManager.get_instance()

    def update(self) -> py_trees.common.Status:
        battery_level = self.blackboard.get('battery.level', 100.0)
        critical_threshold = self.blackboard.get('battery.critical_threshold', 10.0)

        if battery_level <= critical_threshold:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE
```

#### 6.1.2 `IsCurrentMode`

```python
class IsCurrentMode(py_trees.behaviour.Behaviour):
    '''현재 로봇 모드가 특정 모드인지 확인'''

    def __init__(self, name: str = "IsCurrentMode", modes: str = "IDLE"):
        super().__init__(name=name)
        self.allowed_modes = [m.strip().upper() for m in modes.split(',')]
        self.blackboard = BlackboardManager.get_instance()

    def update(self) -> py_trees.common.Status:
        current_mode = self.blackboard.get('robot.mode', 'STANDBY')

        if current_mode in self.allowed_modes:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE
```

#### 6.1.3 `IsNewTaskFromRCS`

```python
class IsNewTaskFromRCS(py_trees.behaviour.Behaviour):
    '''RCS로부터 새로운 작업이 할당되었는지 확인'''

    def __init__(self, name: str = "IsNewTaskFromRCS"):
        super().__init__(name=name)
        self.blackboard = BlackboardManager.get_instance()

    def update(self) -> py_trees.common.Status:
        pending_task = self.blackboard.get('rcs.pending_task', None)

        if pending_task is not None:
            self.logger.info(f"New task detected: {pending_task.get('type')}")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE
```

### 6.2 액션 노드 (Actions)

#### 6.2.1 `MoveTo`

```python
# javis_dmc2/bt_nodes/actions/drive_actions.py
import py_trees
from javis_dmc2.interfaces import RosDriveInterface
from javis_dmc2.blackboard import BlackboardManager

class MoveTo(py_trees.behaviour.Behaviour):
    '''지정된 위치로 이동'''

    def __init__(self, name: str = "MoveTo", destination_key: str = "current_task.destination"):
        super().__init__(name=name)
        self.destination_key = destination_key
        self.blackboard = BlackboardManager.get_instance()
        self.drive_interface = None
        self.future = None

    def setup(self, **kwargs):
        # ROS 노드 인스턴스 전달받음
        self.drive_interface = kwargs.get('drive_interface')
        if self.drive_interface is None:
            raise RuntimeError("DriveInterface not provided to MoveTo node")

    def initialise(self):
        destination = self.blackboard.get(self.destination_key)
        if destination is None:
            self.logger.error(f"Destination not found in blackboard: {self.destination_key}")
            self.feedback_message = "No destination"
            return

        self.logger.info(f"Moving to {destination}")
        self.future = self.drive_interface.move_to_target(destination, reason=self.name)

    def update(self) -> py_trees.common.Status:
        if self.future is None:
            return py_trees.common.Status.FAILURE

        if not self.future.done():
            return py_trees.common.Status.RUNNING

        try:
            result = self.future.result()
            if result.success:
                self.logger.info("MoveTo succeeded")
                return py_trees.common.Status.SUCCESS
            else:
                self.logger.warn(f"MoveTo failed: {result.message}")
                return py_trees.common.Status.FAILURE
        except Exception as exc:
            self.logger.error(f"MoveTo exception: {exc}")
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID:
            # 작업 취소
            if self.future and not self.future.done():
                self.drive_interface.cancel_navigation()
```

#### 6.2.2 `AcceptNewTaskFromRCS`

```python
# javis_dmc2/bt_nodes/actions/task_actions.py
class AcceptNewTaskFromRCS(py_trees.behaviour.Behaviour):
    '''RCS의 pending_task를 current_task로 승인'''

    def __init__(self, name: str = "AcceptNewTaskFromRCS"):
        super().__init__(name=name)
        self.blackboard = BlackboardManager.get_instance()

    def update(self) -> py_trees.common.Status:
        pending_task = self.blackboard.get('rcs.pending_task', None)

        if pending_task is None:
            self.logger.warn("No pending task to accept")
            return py_trees.common.Status.FAILURE

        # pending_task를 current_task로 이동
        self.blackboard.set('current_task.type', pending_task.get('type'))
        self.blackboard.set('current_task.goal', pending_task.get('goal'))
        self.blackboard.set('current_task.timestamp', pending_task.get('timestamp'))

        # pending_task 삭제
        self.blackboard.set('rcs.pending_task', None)

        self.logger.info(f"Accepted task: {pending_task.get('type')}")
        return py_trees.common.Status.SUCCESS
```

#### 6.2.3 `ClearCurrentTask`

```python
class ClearCurrentTask(py_trees.behaviour.Behaviour):
    '''현재 작업을 완료/실패 처리하고 Blackboard 정리'''

    def __init__(self, name: str = "ClearCurrentTask", success: bool = True, message: str = ""):
        super().__init__(name=name)
        self.success = success
        self.message = message
        self.blackboard = BlackboardManager.get_instance()

    def update(self) -> py_trees.common.Status:
        current_task = self.blackboard.get('current_task', {})

        # 작업 결과를 RCS에 보고
        result_status = 'SUCCESS' if self.success else 'FAILURE'
        self.blackboard.set('rcs.last_task_result', {
            'type': current_task.get('type'),
            'status': result_status,
            'message': self.message,
            'timestamp': time.time(),
        })

        # current_task 삭제
        self.blackboard.set('current_task', {})

        self.logger.info(f"Task cleared: {result_status}")
        return py_trees.common.Status.SUCCESS
```

### 6.3 데코레이터 노드 (Decorators)

#### 6.3.1 `Retry`

```python
# javis_dmc2/bt_nodes/decorators/retry.py
import py_trees

class Retry(py_trees.decorators.Decorator):
    '''자식 노드 실패 시 N회 재시도'''

    def __init__(self, name: str = "Retry", child: py_trees.behaviour.Behaviour = None, num_attempts: int = 3):
        super().__init__(name=name, child=child)
        self.num_attempts = num_attempts
        self.current_attempt = 0

    def initialise(self):
        self.current_attempt = 0

    def update(self) -> py_trees.common.Status:
        if self.decorated.status == py_trees.common.Status.SUCCESS:
            return py_trees.common.Status.SUCCESS

        if self.decorated.status == py_trees.common.Status.FAILURE:
            self.current_attempt += 1
            if self.current_attempt < self.num_attempts:
                self.logger.info(f"Retry attempt {self.current_attempt}/{self.num_attempts}")
                self.decorated.stop(py_trees.common.Status.INVALID)
                self.decorated.setup_with_descendants()
                return py_trees.common.Status.RUNNING
            else:
                self.logger.warn(f"Retry failed after {self.num_attempts} attempts")
                return py_trees.common.Status.FAILURE

        return self.decorated.status
```

---

## 7. Blackboard 데이터 구조

### 7.1 Blackboard 키 정의

```python
# javis_dmc2/blackboard/blackboard_manager.py

BLACKBOARD_SCHEMA = {
    # 로봇 상태
    'robot.mode': 'STANDBY',  # STANDBY | AUTONOMY
    'robot.main_state': 'IDLE',
    'robot.sub_state': 'NONE',

    # 배터리
    'battery.level': 100.0,
    'battery.is_charging': False,
    'battery.warning_threshold': 40.0,
    'battery.critical_threshold': 10.0,

    # 작업 (RCS)
    'rcs.pending_task': None,  # {'type': 'GUIDING', 'goal': {...}, 'timestamp': ...}
    'rcs.last_task_result': None,

    # 현재 작업
    'current_task.type': None,  # GUIDING | PICKUP_BOOK | ...
    'current_task.goal': {},
    'current_task.destination': None,
    'current_task.shelf_id': None,
    'current_task.book_id': None,
    'current_task.storage_id': None,

    # 사용자 입력
    'gui_selected_destination': None,
    'user_entities.destination': None,

    # 센서
    'sensor.user_in_view': False,
    'sensor.wake_word_detected': False,

    # 내비게이션
    'navigation.charger_location_pose': {'x': 0.0, 'y': 0.0, 'theta': 0.0},
    'navigation.current_pose': None,

    # 시스템
    'system.emergency_stop': False,
}
```

### 7.2 BlackboardManager 클래스

```python
class BlackboardManager:
    '''Blackboard 싱글톤 관리'''

    _instance = None

    @classmethod
    def get_instance(cls):
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance

    def __init__(self):
        if BlackboardManager._instance is not None:
            raise RuntimeError("BlackboardManager is a singleton!")
        self.data = dict(BLACKBOARD_SCHEMA)
        self.lock = threading.Lock()

    def get(self, key: str, default=None):
        with self.lock:
            keys = key.split('.')
            value = self.data
            for k in keys:
                value = value.get(k, {})
                if value == {}:
                    return default
            return value if value != {} else default

    def set(self, key: str, value):
        with self.lock:
            keys = key.split('.')
            target = self.data
            for k in keys[:-1]:
                if k not in target:
                    target[k] = {}
                target = target[k]
            target[keys[-1]] = value

    def reset(self):
        with self.lock:
            self.data = dict(BLACKBOARD_SCHEMA)
```

---

## 8. 인터페이스 통합 전략

### 8.1 기존 인터페이스 재사용

기존 `javis_dmc/interfaces/` 코드를 그대로 사용:
- `RosDriveInterface`
- `RosArmInterface`
- `RosAIInterface`
- `RosGUIInterface`
- `RosVoiceRecognitionInterface`

**통합 방법**:
```python
# javis_dmc2/dmc2_node.py
from javis_dmc.interfaces import (
    RosDriveInterface,
    RosArmInterface,
    RosAIInterface,
    RosGUIInterface,
    RosVoiceRecognitionInterface,
)

class DMC2Node(Node):
    def __init__(self):
        super().__init__('dmc2_node')

        # 인터페이스 초기화
        self.drive = RosDriveInterface(self, self.namespace)
        self.arm = RosArmInterface(self, self.namespace)
        self.ai = RosAIInterface(self, self.namespace)
        self.gui = RosGUIInterface(self, self.namespace)
        self.voice = RosVoiceRecognitionInterface(self, self.namespace)

        # BT 노드에 인터페이스 전달
        self.setup_tree(
            drive_interface=self.drive,
            arm_interface=self.arm,
            ai_interface=self.ai,
            gui_interface=self.gui,
            voice_interface=self.voice,
        )
```

### 8.2 ROS2 Action Server 연동

RCS의 작업 할당을 받기 위해 Action Server 유지:

```python
class DMC2Node(Node):
    def __init__(self):
        # ... (위와 동일)

        # Action Server 생성
        self.action_servers = {
            'guiding': ActionServer(
                self,
                GuidePerson,
                self._ns('main/guide_person'),
                execute_callback=self._on_guiding_goal,
            ),
            'pickup': ActionServer(
                self,
                PickupBook,
                self._ns('main/pickup_book'),
                execute_callback=self._on_pickup_goal,
            ),
            # ... 나머지 작업들
        }

    def _on_guiding_goal(self, goal_handle):
        '''길 안내 작업 Goal 수신'''
        goal = goal_handle.request

        # Blackboard에 작업 등록
        self.blackboard.set('rcs.pending_task', {
            'type': 'GUIDING',
            'goal': {
                'destination': goal.dest_location,
                'destination_name': goal.destination_name,
            },
            'timestamp': time.time(),
            'goal_handle': goal_handle,  # 결과 전송용
        })

        # BT가 작업을 처리할 때까지 대기
        while self.blackboard.get('current_task.type') != 'GUIDING':
            time.sleep(0.1)

        # BT가 작업 완료할 때까지 피드백 전송
        while self.blackboard.get('current_task.type') == 'GUIDING':
            feedback = GuidePerson.Feedback()
            feedback.distance_remaining_m = self.blackboard.get('current_task.distance_remaining', 0.0)
            goal_handle.publish_feedback(feedback)
            time.sleep(0.5)

        # 작업 결과 전송
        result = GuidePerson.Result()
        task_result = self.blackboard.get('rcs.last_task_result', {})
        result.success = (task_result.get('status') == 'SUCCESS')
        result.message = task_result.get('message', '')

        if result.success:
            goal_handle.succeed(result)
        else:
            goal_handle.abort(result)

        return result
```

---

## 9. 마이그레이션 로드맵

### Phase 1: 프로토타입 개발 (1주)

- [ ] `javis_dmc2` 패키지 생성
- [ ] py_trees 설치 및 기본 트리 구성
- [ ] 조건 노드 5개 구현 (IsBatteryCritical, IsCurrentMode, IsNewTaskFromRCS, IsUserInView, IsEmergencyStopped)
- [ ] 액션 노드 10개 구현 (MoveTo, MoveArmToPose, PlayTTS, AcceptNewTaskFromRCS, ClearCurrentTask, ...)
- [ ] `root_tree.xml`, `task_dispatcher.xml` 작성
- [ ] `guiding_task.xml` 작성 및 테스트

### Phase 2: 작업 트리 완성 (1주)

- [ ] `pickup_task.xml` 구현
- [ ] `reshelving_task.xml` 구현
- [ ] `cleaning_task.xml` 구현
- [ ] `sorting_task.xml` 구현
- [ ] 충전/순찰 서브트리 구현
- [ ] Blackboard 데이터 흐름 검증

### Phase 3: 인터페이스 통합 (3일)

- [ ] 기존 인터페이스(`javis_dmc/interfaces/`)를 DMC2에서 import
- [ ] Mock 인터페이스 동작 확인
- [ ] Action Server 연동 (RCS 작업 할당 수신)
- [ ] Action Feedback/Result 전송 구현

### Phase 4: 테스트 및 디버깅 (1주)

- [ ] 유닛 테스트 작성 (각 BT 노드)
- [ ] 통합 테스트 (서브트리 시나리오)
- [ ] Groot 시각화 테스트
- [ ] 실제 로봇 하드웨어 테스트 (또는 Mock)

### Phase 5: 문서화 및 인수인계 (2일)

- [ ] BT 노드 레퍼런스 문서 작성
- [ ] 사용자 가이드 작성 (Groot 사용법)
- [ ] 기존 DMC와 비교 문서 작성
- [ ] 인수인계 세션

---

## 10. 테스트 계획

### 10.1 유닛 테스트

```python
# test/test_bt_nodes.py
import pytest
from javis_dmc2.bt_nodes.conditions.battery_conditions import IsBatteryCritical
from javis_dmc2.blackboard import BlackboardManager

def test_is_battery_critical():
    bb = BlackboardManager.get_instance()
    bb.reset()

    node = IsBatteryCritical()

    # 배터리 정상
    bb.set('battery.level', 50.0)
    bb.set('battery.critical_threshold', 10.0)
    assert node.update() == py_trees.common.Status.FAILURE

    # 배터리 위험
    bb.set('battery.level', 5.0)
    assert node.update() == py_trees.common.Status.SUCCESS
```

### 10.2 통합 테스트

```python
# test/test_subtrees.py
def test_guiding_task_tree():
    '''길 안내 서브트리 전체 시나리오 테스트'''
    bb = BlackboardManager.get_instance()
    bb.reset()

    # 작업 설정
    bb.set('current_task.type', 'GUIDING')
    bb.set('gui_selected_destination', {'x': 10.0, 'y': 5.0, 'theta': 0.0})

    # 트리 로드 및 실행
    tree = load_tree('trees/tasks/guiding_task.xml')
    tree.setup_with_descendants()

    # 트리 실행 (타임아웃 60초)
    for _ in range(60):
        tree.tick_once()
        if tree.status == py_trees.common.Status.SUCCESS:
            break
        time.sleep(1.0)

    # 검증
    assert tree.status == py_trees.common.Status.SUCCESS
    assert bb.get('current_task.type') is None  # 작업 완료 후 삭제됨
```

---

## 부록 A: Groot 사용법

### A.1 Groot 실행

```bash
groot
```

### A.2 XML 파일 로드

1. Groot 실행 후 `File` → `Open Tree`
2. `trees/root_tree.xml` 선택
3. 트리 구조가 그래픽으로 표시됨

### A.3 실시간 모니터링

```bash
# DMC2 노드 실행
ros2 launch javis_dmc2 dmc2_test.launch.py

# Groot Monitor 모드
groot --mode monitor --address localhost --port 1667
```

---

## 부록 B: 기존 DMC vs DMC2 비교

| 항목 | javis_dmc (State Machine) | javis_dmc2 (Behavior Tree) |
|------|---------------------------|----------------------------|
| 코드 라인 수 | ~1800 (dmc_node.py) | ~800 (dmc2_node.py) + XML |
| 작업 추가 시 수정 파일 | 5개 (node, state_enums, executor, ...) | 1개 (XML 트리 파일) |
| 시각화 | 불가능 | Groot으로 실시간 모니터링 |
| 디버깅 | 로그 기반 | 트리 상태 + 로그 |
| 재사용성 | 낮음 (코드 중복) | 높음 (서브트리) |
| 학습 곡선 | 중간 (State Machine 이해 필요) | 높음 (BT 개념 학습) |

---

**작성자**: Claude Code
**검토 필요**: 팀원 검토 및 BT 프레임워크 선정 회의
