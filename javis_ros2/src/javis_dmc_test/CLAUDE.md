# CLAUDE.md

이 파일은 Claude Code(claude.ai/code)가 이 리포지토리의 코드 작업 시 참고할 가이드를 제공합니다.

## 역할 및 임무

당신은 **ROS 2 Jazzy, Python, 그리고 PyQt6**를 전문으로 다루는 **시니어 로봇 소프트웨어 아키텍트**입니다.

이 프로젝트에서는 ROS 2 `dobby_main_controller(DMC)` 노드의 통합 테스트를 위한 Mock 시스템을 개발합니다. 최종 설계는 `javis_dmc_test/docs/DevelopmentPlan.md` 파일에 완벽하게 정의되어 있습니다.

### 핵심 원칙

**`docs/DevelopmentPlan.md`는 '단 하나의 진실의 원천(Single Source of Truth)'**입니다. 이 설계 문서를 기반으로 `javis_dmc_test_msgs`와 `javis_dmc_test` 두 패키지의 모든 코드를 작성해야 합니다.

### 엄격한 규칙

1. **설계 명세 준수**: 제공된 `DevelopmentPlan.md`의 설계 명세를 **절대** 변경, 제안, 또는 이탈하지 마십시오. **100%** 명세에 따라 코드를 작성해야 합니다.

2. **패키지 분리**: `javis_dmc_test_msgs`와 `javis_dmc_test` 패키지가 분리된 이유를 정확히 이해하십시오. (`.msg` 파일은 별도 패키지에 있어야 합니다.)

3. **GUI 역할 분리**: `gui_node.py` (ROS 2 로직)와 `main_window.py` (PyQt6 UI 로직)의 역할을 명확히 분리해야 합니다.

4. **GUI 프레임워크**: GUI는 **PyQt6**를 사용해야 합니다.

5. **ROS 스레딩**: `rclpy.spin()`은 **`threading.Thread`**를 사용해 GUI의 메인 스레드를 막지 않고 백그라운드에서 실행되어야 합니다.

6. **스레드 안전성**: ROS 2 콜백(예: `status_callback`)에서 GUI 위젯을 직접 수정하지 않고, **`pyqtSignal`**과 **`pyqtSlot`**을 사용해 스레드 세이프(thread-safe)하게 처리해야 합니다.

7. **코드 구조**:
   - `mocks/` 폴더: Mock 서버 **클래스 정의**를 포함
   - `nodes/` 폴더: 클래스들을 임포트하여 노드를 실행시키는 **'실행기(Entry Point)'** 역할

8. **상속 패턴**: 모든 Mock 서버는 `mocks/mock_server_base.py`의 `MockServerBase` 클래스를 **상속(Inheritance)**받아 중복을 최소화해야 합니다.

9. **리소스 로딩**: RCS 작업 요청은 `resource/dmc_test_goals.yaml` 파일을 로드하여 전송해야 합니다. **`ament_index_python`**을 사용해 리소스 파일 경로를 찾아야 합니다.

10. **Timer 제어**: Topic Publisher Mock(`mock_dvs_tracking_status`)은 `on`/`off` 모드에 따라 **`rclpy.Timer`**를 `reset()`하거나 `cancel()`해야 합니다.

11. **문서화**: 모든 코드는 **한국어**로 작성된 주석을 포함하여 완전하고 실행 가능해야 합니다.

## 프로젝트 개요

**JAVIS DMC Test Tools**는 JAVIS DMC(Dobby Main Controller)를 위한 테스트 유틸리티를 제공하는 ROS 2 Python 패키지입니다. 이 패키지를 사용하면 실제 로봇 하드웨어나 종속 서비스 없이도 DMC를 테스트하고 디버그할 수 있습니다.

현재 제공되는 기능:
- **Status GUI**: DMC 상태 시각화를 위한 Tkinter 기반 모니터링 및 제어 인터페이스
- **Test Messages**: 테스트용 커스텀 메시지/서비스 정의 (`javis_dmc_test_msgs` 패키지)

이는 DMC가 관리하는 Dobby 도서관 도우미 로봇을 포함하는 더 큰 JAVIS 도서관 자동화 시스템의 일부입니다.

## 개발 명령어

### 빌드

이 패키지와 종속성 빌드:
```bash
cd /home/mac/dev_ws/roscamp-repo-2/javis_ros2
colcon build --packages-select javis_dmc_test javis_dmc_test_msgs
```

심볼릭 링크 설치로 빌드 (개발 시 권장):
```bash
colcon build --packages-select javis_dmc_test javis_dmc_test_msgs --symlink-install
```

빌드 후 워크스페이스 소싱:
```bash
source install/setup.bash
```

### 실행

DMC 모니터링을 위한 Status GUI 실행:
```bash
ros2 run javis_dmc_test status_gui_node
```

Status GUI는 DMC 노드가 실행 중이어야 합니다. 먼저 DMC 실행:
```bash
ros2 launch javis_dmc dmc_single.launch.py robot_namespace:=dobby1
```

### 테스트

린팅 체크 실행:
```bash
# PEP8 스타일 체크
ament_flake8 javis_dmc_test/

# Docstring 스타일 체크
ament_pep257 javis_dmc_test/

# 저작권 헤더 체크
ament_copyright javis_dmc_test/
```

pytest 실행:
```bash
pytest test/
```

## 코드 아키텍처

### Status GUI 아키텍처 (구현 완료)

**Status GUI**(`javis_dmc_test/status_gui/`)는 DMC 상태 모니터링을 위해 ROS 2와 Tkinter를 안전하게 통합하는 **이중 스레딩 아키텍처**를 사용합니다:

**GUI 스레드** (메인 스레드, `status_gui_widget.py`):
- Tkinter 메인 루프 실행 (`tk.mainloop()`)
- UI 컴포넌트 관리: 상태 그래프 캔버스, 제어 패널, 로그 디스플레이
- 스레드 안전 `Queue`에서 읽어 UI 요소 업데이트
- ROS 2 API를 직접 호출하지 않음

**ROS 스레드** (백그라운드 스레드, `status_gui_node.py`):
- `MultiThreadedExecutor`를 사용하여 `rclpy.spin()` 실행
- DMC 토픽 구독: `DobbyState`, `BatteryStatus`, `/rosout`, 모드 피드백, 상태 전환
- 서비스 클라이언트 제공: `set_robot_mode`, `emergency_stop`, `resume_navigation`, `set_listening_mode`, 파라미터 업데이트
- GUI 소비를 위해 스레드 안전 `Queue`에 이벤트 푸시

**통신 패턴**:
```
ROS 콜백 → event_queue.put() → GUI 주기적 폴링 (after_idle) → UI 업데이트
GUI 버튼 클릭 → ROS 서비스/파라미터 호출 (블로킹) → 응답
```

**주요 컴포넌트**:

1. **StatusGuiRosNode** (`status_gui_node.py`):
   - 모든 ROS 통신을 관리하는 ROS 2 노드
   - 상태, 배터리, 로그, 모드 피드백 토픽 구독
   - 관리 서비스 클라이언트 노출 (모드 변경, 비상 정지 등)
   - GUI 스레드를 위해 큐에 이벤트 발행

2. **StatusGuiApp** (`status_gui_widget.py`):
   - UI 레이아웃과 위젯을 관리하는 Tkinter 애플리케이션
   - 상태 그래프 시각화를 포함한 다크 테마 인터페이스
   - 모드 전환, 비상 제어, 리스닝 모드를 위한 제어 패널
   - 색상으로 코딩된 심각도 레벨을 가진 실시간 로그 디스플레이

3. **StateGraphCanvas** (`status_gui_widget.py`):
   - 상태 머신을 시각화하는 커스텀 Canvas 위젯
   - 상태 사각형과 전환 화살표 그리기
   - 현재 활성 상태 강조
   - 큰 상태 그래프를 위한 스크롤 가능

**Status GUI vs Test GUI 비교**:

| 항목 | Status GUI (현재 구현) | Test GUI (설계 문서) |
| :--- | :--- | :--- |
| **위치** | `status_gui/` | `gui/` |
| **목적** | DMC 상태 모니터링 | Mock 시스템 제어 및 테스트 |
| **UI 프레임워크** | Tkinter | PyQt6 |
| **제어 대상** | DMC 노드 | Mock 서버 + DMC 노드 |
| **주요 기능** | 상태 시각화, 모드 전환, 비상 정지 | Mock mode 제어, 작업 전송, YAML 로딩 |
| **구현 상태** | ✅ 완료 | ⏳ 설계 완료, 구현 예정 |

### 메시지/서비스 아키텍처

`javis_dmc_test_msgs` 패키지는 커스텀 인터페이스를 정의합니다:

**서비스** (`javis_dmc_test_msgs/srv/`):
- `SendTask.srv`: DMC에 테스트 작업 전송
- `SetBattery.srv`: 배터리 상태 변경 시뮬레이션
- `SetMockMethod.srv`: Mock 메서드 동작 구성
- `SetMockResponse.srv`: 미리 정의된 Mock 응답 설정

이러한 서비스는 실제 하드웨어 없이 자동화된 테스트 및 시나리오 시뮬레이션을 가능하게 합니다.

## DMC와의 통합

Status GUI는 다음을 통해 DMC와 통합됩니다:

**토픽 (구독)**:
- `/{namespace}/status/robot_state` (DobbyState): Main/sub state, mode, active task
- `/{namespace}/status/battery_status` (BatteryStatus): 배터리 레벨, 충전 상태
- `/{namespace}/admin/mode_feedback` (String): 모드 변경 확인
- `/{namespace}/debug/state_transitions` (String): 실시간 상태 변경 로그
- `/rosout` (Log): 통합 로그 디스플레이를 위한 시스템 전체 로깅

**서비스 (클라이언트)**:
- `/{namespace}/admin/set_robot_mode`: STANDBY ↔ AUTONOMY 전환
- `/{namespace}/admin/emergency_stop`: 모든 작업 즉시 중지
- `/{namespace}/admin/resume_navigation`: 비상 정지에서 재개
- `/{namespace}/set_listening_mode`: 음성 리스닝 활성화/비활성화
- `/{namespace}/debug/describe_state_machine`: 상태 머신 인트로스펙션 가져오기
- `/{dmc_node_fqn}/set_parameters`: DMC 런타임 파라미터 업데이트

기본 네임스페이스는 `dobby1`이며, `robot_namespace` 파라미터를 통해 구성 가능합니다.

## 개발 계획 컨텍스트

완전한 Mock 시스템 아키텍처 설계(버전 7.0)는 `docs/DevelopmentPlan.md`를 참조하십시오. 주요 아키텍처 결정:

**설계 원칙**:
- 1 Mock = 1 Node: 각 인터페이스는 독립적인 ROS 2 노드가 됩니다
- `mode` 파라미터 제어: 테스트를 위한 `active`/`error`/`on`/`off` 모드
- GUI는 Push & Trigger 사용: 서버에 파라미터 설정, 클라이언트 트리거
- Mock 상태 발행: 모든 Mock은 `dobby1/mock_system/status`에 상태 발행

**계획된 Mock 시스템** (DevelopmentPlan.md에서, 아직 완전히 구현되지 않음):
- Mock 서버: DDC (드라이브), DAC (암), DVS (AI/비전) 인터페이스 시뮬레이터
- Mock 클라이언트: RCS 작업 전송자, VRC 음성 이벤트 전송자
- 테스트 GUI 제어 패널: RCS 트리거, VRC 트리거, Mock 서버 제어를 위한 탭

**현재 구현 상태**:
- ✅ **Status GUI 구현 완료** (`status_gui/status_gui_node.py`, `status_gui/status_gui_widget.py`)
  - Tkinter 기반 이중 스레딩 아키텍처
  - DMC 상태 모니터링, 배터리 표시, 로그 통합, 원격 제어
  - `ros2 run javis_dmc_test status_gui_node`로 실행
- ✅ 테스트 메시지 패키지 생성됨 (`javis_dmc_test_msgs`)
- ⏳ Mock 시스템 아키텍처 설계 완료, 구현 예정:
  - Mock 서버 노드 (`mocks/`)
  - PyQt6 기반 Test GUI (`gui/`)
  - 실행 스크립트 (`nodes/`)
  - 리소스 파일 및 런치 파일

## 코딩 컨벤션

**중요**: 모든 코드는 상위 `javis_dmc` 패키지의 `AGENTS.md`에 있는 컨벤션을 준수해야 합니다. 주요 포인트:

### 명명 규칙

**Python**:
- 모듈/패키지: `snake_case`
- 클래스/예외: `PascalCase`
- 함수/변수: `snake_case`
- 상수: `SCREAMING_SNAKE_CASE`
- 들여쓰기: 4칸 스페이스
- 문자열: 작은따옴표 선호 (`'text'`)

**ROS 2 요소**:
- 패키지/노드/토픽/서비스 이름: `snake_case`
- 메시지/서비스/액션 타입 이름: `PascalCase`
- 메시지 필드 이름: `snake_case`
- 메시지 상수: `SCREAMING_SNAKE_CASE`

### 문서화

- **주석**: 한국어로 작성
- **Python docstring**: 모든 공개 함수/클래스에 사용
- **형식**: 첫 번째 줄에 간단한 요약, 필요시 세부사항

### 코드 구성

- **Import**: 한 줄에 하나씩, 표준 라이브러리 → 서드파티 → 로컬로 그룹화
- **간격**: 함수 사이 1줄, import 후 2줄
- **로깅**: ROS 노드에 `self.get_logger().info/warn/error()` 사용

### 설계 정합성

코드는 `/docs`의 설계 문서와 일치해야 합니다:
- `DevelopmentPlan.md`: 테스트 시스템 아키텍처 및 Mock 노드 명세
- 전체 문서 구조는 상위 `javis_dmc` 패키지 참조

## 커밋 메시지 형식

프로젝트 템플릿을 따르십시오:

```
[TYPE/SCOPE]: 간단한 설명

TYPE: FEAT, FIX, REFACTOR, DOCS, TEST, ENVIR, STYLE
SCOPE: 기능/모듈 이름 (전역이면 생략)
```

git 히스토리에서 예시:
- `[REFACTORFEAT/DMC]: test gui 개선 및 리팩토링`
- `[TEST/DMC]: GIT 테스트 추가`
- `[ENVIR]: commit message template 추가`

## 패키지 구조

### 설계된 구조 (DevelopmentPlan.md 기준)

```
javis_dmc_test/
├── javis_dmc_test/              # 메인 Python 패키지
│   ├── __init__.py
│   ├── nodes/                   # 실행 스크립트 (Entry Points)
│   │   ├── start_gui.py         # [구현 예정]
│   │   ├── start_mock_ddc.py    # [구현 예정]
│   │   ├── start_mock_dac.py    # [구현 예정]
│   │   └── start_mock_dvs.py    # [구현 예정]
│   ├── gui/                     # PyQt6 기반 Mock 제어 GUI [구현 예정]
│   │   ├── gui_node.py
│   │   ├── main_window.py
│   │   └── widgets/
│   ├── status_gui/              # Tkinter 기반 DMC 모니터링 GUI [✅ 구현 완료]
│   │   ├── __init__.py
│   │   ├── status_gui_node.py   # ROS 노드 (백그라운드 스레드)
│   │   └── status_gui_widget.py # Tkinter 앱 (메인 스레드)
│   └── mocks/                   # Mock 서버 클래스들 [구현 예정]
│       ├── mock_server_base.py
│       ├── mock_ddc_servers.py
│       ├── mock_dac_servers.py
│       └── mock_dvs_servers.py
├── resource/
│   ├── javis_dmc_test
│   ├── main_window.ui           # [구현 예정]
│   └── dmc_test_goals.yaml      # [구현 예정]
├── launch/
│   └── dmc_mock.launch.py       # [구현 예정]
├── docs/
│   └── DevelopmentPlan.md       # Mock 시스템 설계 v7.0
├── setup.py                     # Entry point가 있는 패키지 설정
├── package.xml                  # ROS 패키지 매니페스트
└── README.md                    # 빠른 시작 가이드 (한국어)
```

### 분리된 메시지 패키지

```
javis_dmc_test_msgs/
├── msg/                         # Mock 상태 메시지
│   └── MockStatus.msg           # [구현 예정]
├── srv/                         # 서비스 정의
│   ├── SendTask.srv
│   ├── SetBattery.srv
│   ├── SetMockMethod.srv
│   └── SetMockResponse.srv
├── CMakeLists.txt
└── package.xml
```

## Entry Points

`setup.py`에 정의됨:

```python
entry_points={
    'console_scripts': [
        # ✅ 구현 완료
        'status_gui_node = javis_dmc_test.status_gui.status_gui_node:main',

        # ⏳ 구현 예정 (설계 문서 기준)
        'start_gui = javis_dmc_test.nodes.start_gui:main',
        'start_mock_ddc = javis_dmc_test.nodes.start_mock_ddc:main',
        'start_mock_dac = javis_dmc_test.nodes.start_mock_dac:main',
        'start_mock_dvs = javis_dmc_test.nodes.start_mock_dvs:main',
    ],
}
```

**현재 사용 가능한 실행 명령**:
```bash
# DMC 상태 모니터링 GUI
ros2 run javis_dmc_test status_gui_node
```

## 주요 구현 노트

### Status GUI의 스레드 안전성

**핵심 패턴**: ROS 콜백에서 Tkinter 메서드를 호출하지 말고, Tkinter 콜백에서 블로킹 ROS 메서드를 호출하지 마십시오:

```python
# ✅ 올바름: ROS 콜백이 큐에 푸시
def _on_state(self, msg: DobbyState):
    self._put_event({'type': 'state', 'data': msg})  # 스레드 안전

# ✅ 올바름: GUI가 주기적으로 큐 폴링
def _poll_events(self):
    while not self._event_queue.empty():
        event = self._event_queue.get_nowait()
        self._handle_event(event)  # Tkinter 위젯 업데이트
    self._root.after(50, self._poll_events)  # 재스케줄
```

### 상태 머신 시각화

`StateGraphCanvas`는 인트로스펙션 데이터에서 상태 다이어그램을 동적으로 구축합니다:

```python
state_graph = {
    'main_states': [
        {'name': 'INITIALIZING'},
        {'name': 'IDLE'},
        {'name': 'PICKING_UP_BOOK'},
        # ...
    ],
    'transitions': [
        {'from': 'IDLE', 'to': 'PICKING_UP_BOOK'},
        # ...
    ]
}
```

DMC의 `debug/describe_state_machine` 서비스를 통해 검색됩니다.

### 네임스페이스 처리

GUI는 `robot_namespace` 파라미터를 통해 구성 가능한 네임스페이스를 지원합니다 (기본값: `dobby1`):

```python
def _ns(self, topic: str) -> str:
    if not self.robot_namespace:
        return topic
    return f'{self.robot_namespace}/{topic}'
```

파라미터 서비스를 위한 전체 노드 이름은 완전 한정 이름을 사용합니다:

```python
def _dmc_node_fqn(self) -> str:
    node_name = self._ns('javis_dmc_node')
    if not node_name.startswith('/'):
        node_name = f'/{node_name}'
    return node_name
```

## 관련 패키지

- **javis_dmc**: 테스트 중인 메인 컨트롤러 패키지
- **javis_interfaces**: JAVIS 시스템을 위한 액션/메시지 정의
- **javis_dmc_test_msgs**: 커스텀 테스트 메시지/서비스
- **javis_rcs**: Robot Control Service (작업 할당)
- **javis_rcs_test**: RCS 테스팅 유틸리티

## 향후 개발

`DevelopmentPlan.md` 섹션 7.2에 기반한 계획된 추가 사항:

### 구현 우선순위

1. **메시지 정의** (`MockStatus.msg`) - 다른 모든 구현의 기반
2. **Mock 서버 베이스** (`mock_server_base.py`) - 공통 로직
3. **Mock 서버 구현**:
   - `mock_ddc_servers.py`: Drive 인터페이스 시뮬레이터
   - `mock_dac_servers.py`: Arm 인터페이스 시뮬레이터
   - `mock_dvs_servers.py`: AI/Vision 인터페이스 시뮬레이터
4. **실행 스크립트** (`nodes/`):
   - `start_mock_ddc.py`, `start_mock_dac.py`, `start_mock_dvs.py`
5. **리소스 파일**:
   - `dmc_test_goals.yaml`: RCS 작업 Goal 정의
6. **Test GUI** (`gui/`):
   - `gui_node.py`: PyQt6 기반 ROS 2 노드
   - `main_window.py`: Mock 제어 인터페이스
7. **런치 파일**:
   - `dmc_mock.launch.py`: 전체 Mock 환경 통합 실행

### 주요 차이점

- **Status GUI** (✅ 완료): DMC 상태 모니터링 및 기본 제어 - 개발자가 DMC 동작을 관찰하는 도구
- **Test GUI** (⏳ 예정): Mock 시스템 제어 및 테스트 시나리오 실행 - 개발자가 DMC를 테스트하는 도구

두 GUI는 서로 다른 목적으로 함께 사용됩니다:
- Status GUI로 DMC 상태 모니터링
- Test GUI로 Mock 노드 제어 및 테스트 시나리오 실행
