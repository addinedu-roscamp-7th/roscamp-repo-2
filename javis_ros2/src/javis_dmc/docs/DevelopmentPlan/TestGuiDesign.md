# JAVIS DMC Test GUI 설계 문서 v1.0

본 문서는 `javis_dmc/test_gui` 모듈의 요구사항·구조·확장 전략을 정리해 QA 팀과 개발자가 동일한 기준으로 도구를 유지·보강할 수 있도록 한다.

## 1. 목표

- **상태 관찰**: `JavisDmcNode`의 모드/메인·서브 상태/배터리/관리자 피드백을 실시간으로 확인한다.
- **행동 제어**: LISTENING, 모드 전환, 긴급 정지, 수동 상태 설정 등 관리자 서비스 호출을 UI에서 수행한다.
- **시나리오 재현**: 길 안내·도서 픽업 등 복합 시나리오를 Mock 또는 실 장비와 함께 재현하고 실패 케이스를 빠르게 재검증한다.
- **실패 주입**: 특정 컴포넌트(Drive, Arm, AI 등) 통신 실패를 시뮬레이션하고 GUI에서 간단히 재시도/회복을 지시한다.
- **로그 관찰성**: 중요 이벤트를 레벨별로 표시해 QA가 상황을 빠르게 파악하고 공유할 수 있도록 한다.

## 2. 요구사항

### 2.1 기능 요구사항

1. 상태 모니터 패널  
   - 모드/메인 상태/서브 상태/배터리/LISTENING 상태/최근 관리자 피드백 표시  
   - 상태 값에 따라 색상·힌트를 바꿔 오퍼레이터가 즉시 판단할 수 있도록 한다.
2. 제어 패널  
   - `set_listening_mode`, `admin/set_robot_mode`, `admin/emergency_stop`, `admin/resume_navigation`, `admin/force_task_result`, `admin/set_manual_state` 서비스 호출 버튼.  
   - 호출 결과/오류를 로그 영역과 토스트(알림)로 동시에 노출한다.
3. 시나리오/실패 처리 패널  
   - (향후) “길 안내 시작”, “도서 픽업 시작” 등 미리 정의된 시나리오를 선택해 필요한 서비스/액션 호출 시퀀스를 실행한다.  
   - 실행 중 Drive/Arm/AI 통신 실패가 발생하면 팝업으로 “성공으로 간주하고 다음 단계로 넘어갈지” 또는 “실패로 종료할지”를 선택할 수 있게 한다(테스트 모드).  
   - 운영 모드에서는 팝업 없이 즉시 실패로 처리하며, Test/Actual 모드를 UI에서 전환할 수 있어야 한다.
4. 이벤트 로그  
   - INFO/WARN/ERROR/STATUS/SUCCESS 레벨별 색상 태그.  
   - 최근 N개의 메시지를 유지하고, 파일로 내보내기 옵션을 제공하도록 확장 가능하게 설계한다.

### 2.2 비기능 요구사항

- **모듈화**: ROS Node(`TestGuiRosNode`)와 Tk UI(`TestGuiApp`)를 분리해 로직/표시층 간 의존성을 최소화한다.
- **확장성**: 시나리오 정의·실패 주입 동작을 Config(JSON/YAML)로 불러오도록 준비한다.
- **시험 독립성**: Mock 모듈과 결합 시에도 실제 하위 시스템 호출과 동일한 인터페이스를 사용한다.
- **로깅 투명성**: ROS 로그(`rclpy` logger)와 GUI 로그가 서로 누락되지 않도록 이벤트 큐를 단일 진입점으로 사용한다.

## 3. 아키텍처

```
[Tk Root] ──> TestGuiApp  ── UI 이벤트 → TestGuiRosNode 서비스 헬퍼
                    ▲                             │
                    │ 상태 업데이트 큐 (Queue)    │ ROS2 서비스/액션/토픽
                    │                             ▼
               TestGuiRosNode  <── ROS Topic 구독 (DobbyState, BatteryStatus, admin/mode_feedback)
                    │
                    └─ rclpy MultiThreadedExecutor (별도 스레드)
```

- **TestGuiRosNode**
  - ROS 2 서비스 클라이언트: 모드 전환, LISTENING, 긴급 정지, 강제 작업 결과, 수동 상태.
  - ROS 2 구독자: `status/robot_state`, `status/battery_status`, `admin/mode_feedback`.
  - 이벤트 큐에 표준화된 딕셔너리 payload를 넣어 UI와 분리.
- **TestGuiApp**
  - Tkinter로 패널 구성 (`_build_status_panel`, `_build_control_panel`, `_build_task_panel`, `_build_log_panel`).
  - `_schedule_queue_poll`로 주기적으로 큐를 폴링해 상태/로그 업데이트.
  - 향후 시나리오 실행 버튼은 `TestGuiRosNode`의 헬퍼 메서드(예: `run_scenario(config, interactive)` )를 호출하도록 확장한다.
  - 팝업 UI는 Tkinter `messagebox` 기반으로 구현하고, 사용자가 “성공으로 간주”를 선택하면 해당 Step 결과에 `override_success=True` 플래그를 남긴다.
  - 상태 다이어그램 데이터는 `describe_state_machine`(§4.4) 서비스를 통해 로드한다.

## 4. 시나리오·실패 주입 설계

### 4.1 시나리오 정의 포맷 제안

```yaml
scenarios:
  guide_person_success:
    description: "길 안내 성공 시나리오"
    steps:
      - action: call_service
        target: set_listening_mode
        args: { enabled: true }
      - action: publish_mock
        target: mock_gui.resolve_destination
        args: { destination: "화장실" }
      - action: wait_for_state
        args: { main: GUIDING, sub: GUIDING_TO_DEST }
      - action: call_service
        target: force_task_result
        args: { success: true }
```

- `TestGuiRosNode`에 `run_scenario(scenario_id, interactive=False)` 메서드를 추가해 YAML을 로드하고 step을 순차 실행한다.
- `publish_mock` 타입은 `mock/mock_*` 모듈에서 제공하는 서비스/액션을 호출해 특정 컴포넌트 실패를 시뮬레이션한다.
- `interactive=True`일 때는 각 Step 실행 후 Future 결과가 실패하면 GUI에 팝업을 띄워 “성공 간주 / 실패 유지 / 재시도” 중 하나를 선택하도록 한다.
- 기본 시나리오는 `config/test_gui_scenarios.yaml`에 저장하며, 운영자는 필요 시 YAML 파일을 수정하거나 새로운 시나리오를 추가한다.
- Mock 응답 강제는 `test/set_mock_response` 서비스를 호출해 인터페이스별 성공/실패 응답을 설정한다(포맷: `method` 또는 `method:error_code`).
- 순찰 상태는 상태 다이어그램의 `ROAMING` 노드와 이벤트 로그를 통해 확인하며, `describe_state_machine`의 `runtime.patrol_active` 플래그를 UI에 표시한다.
- Mock 응답 패널은 인터페이스/메서드 프리셋을 드롭다운 목록으로 제공하고, 필요 시 사용자 정의 입력도 지원한다.

### 4.2 실패 분기 설계

- **동시 통신 실패 감시**: 각 Step 수행 후 `wait_for_state` 또는 `wait_for_feedback(status=busy/error)` 조건을 통한 검증.
- **재시도 정책**: Step에 `retry` 필드를 추가해 UI에서 재실행 여부를 선택할 수 있도록 한다.
- **ARM/Drive/AI 분기**: 시나리오에서 특정 컴포넌트 상태를 토글하기 위해 Mock 서비스(예: `mock_drive/set_response`) 호출을 표준화한다.
- **상태 기반 분기**: Test GUI가 현재 메인 상태를 모니터링하고 조건문(`when`)을 사용해 경로를 나누도록 한다.
- **성공 간주 플래그**: 팝업에서 “성공으로 간주”를 선택하면 Step 결과에 `override: success`를 기록하고 로그에 명시한다.

### 4.3 설계 패턴

- 시나리오 실행 엔진은 `dataclass ScenarioStep`을 사용해 타입 안정성과 IDE 지원을 확보한다.
- 실패 이벤트는 큐에 `{'type': 'scenario', 'step': ..., 'status': 'failed'}` 형태로 기록해 UI에서 별도 색상으로 표시한다.

### 4.4 상태 인트로스펙션 연동

- `describe_state_machine` 서비스(JSON 또는 전용 srv)를 호출해 메인/서브 상태, 전이 규칙, 설명을 로드한다.
- 렌더링: Tkinter Canvas 또는 서드파티 그래프 위젯으로 상태 박스/화살표를 그리고, GUI는 `status/robot_state` 토픽을 기반으로 현재 상태 박스를 하이라이트한다.
- 디버그 토픽(선택)을 구독해 실시간 전이 이벤트를 기록하고 로그 패널과 연동한다.

## 5. 로깅 및 관찰성

- 모든 이벤트는 `TestGuiRosNode._put_event()`를 통해 큐에 등록하며 `level` 필드를 필수로 포함한다.
- UI 로그 패널은 레벨별 태그 색상을 가진 Text 위젯으로 구성하며, 향후 `Export` 버튼을 추가해 CSV/JSON으로 저장할 수 있도록 한다.
- ROS 로그(`self.get_logger()`)와 GUI 로그를 동시에 남겨야 할 경우 `_put_event` 호출 전후에 `get_logger().info(...)`를 호출하여 ROS bag에서도 추적 가능하게 한다.
- 모드 피드백(성공/경고/에러)과 같은 중요 이벤트는 `status` 값과 함께 저장되어 QA 보고서 작성 시 활용할 수 있다.

## 6. UI 구성 개요

| 패널 | 구성요소 | 설명 |
| :--- | :--- | :--- |
| 상태 패널 | 모드/메인/서브/LISTENING/배터리/순찰/피드백 라벨 | 상태별 색상·힌트 제공 |
| 제어 패널 | 모드 전환, LISTENING 토글, 긴급 정지/해제, 작업 강제 완료/실패, 수동 상태 버튼 | 각 버튼은 서비스 호출 전/후 알림을 로그에 push |
| 작업 패널 | (확장 예정) 시나리오 목록, 작업 진행률, 실패 카운트 | YAML 기반 시나리오 정의를 바인딩 |
| 상태 다이어그램 패널 | 메인/서브 상태 노드를 박스로 표현, 전이 화살표 표시, 현재 상태 하이라이트 | 초기 버전에서는 정적 레이아웃, 추후 동적 렌더링 개선 필요 |
| Mock 응답 패널 | 인터페이스/메서드/에러 코드 입력 후 성공·실패 응답을 설정 | 메서드/에러 코드는 선택식 목록 제공 + 직접 입력도 지원 |
| 로그 패널 | Text 위젯 + Treeview(향후) | 레벨별 색상 태깅 및 검색/필터 지원 계획 |

## 7. 확장 가이드

1. **시나리오 엔진 추가**  
   - `test_gui/scenario_runner.py`를 신규 생성하여 YAML 로딩과 Step 실행을 담당.  
   - `TestGuiRosNode`가 해당 Runner를 소유하고 UI 요청 시 실행하도록 한다.
2. **Mock 연동 표준화**  
   - `mock/mock_base.py`에 “응답 프로파일” 설정 API를 정의하고 GUI에서 이를 호출해 실패를 주입한다.  
   - 예: `mock_drive`에 `set_next_result(success=False, error_code=...)`.  
   - 장치 소스 선택 토글을 제공해 Mock ↔ 실 장비 인터페이스를 Test GUI에서 전환할 수 있게 한다(Launch 파라미터 또는 ROS 파라미터로 전달).  
   - Mock 사용 여부를 `describe_state_machine` 응답에도 포함해 상태 그래프 상단에 현재 실행 환경을 표시한다.
3. **로그 Export/필터**  
   - 로그 패널에 “레벨 필터” 콤보박스와 “저장” 버튼을 추가한다.  
   - `event_queue` payload에 `timestamp` 필드를 포함해 정렬/저장을 단순화한다.
4. **멀티 로봇 지원**  
   - `robot_namespace` 파라미터를 UI에서 변경할 수 있도록 입력창/콤보박스를 제공한다.  
   - 여러 로봇을 동시에 모니터링해야 할 경우 탭(Tab) 구조로 확장한다.

## 8. 향후 개선 제안

- **시각화**: 상태 전환 히스토리를 타임라인 그래프로 표현하여 Debrief 속도를 높인다.
- **자동화 스크립트**: Test GUI에서 시나리오 실행 로그를 JUnit/XML 등으로 변환해 CI에서 재사용한다.
- **관찰 포인트 추가**: `DmcStateMachine.snapshot()`을 주기적으로 호출하는 Debug 토픽을 추가해 GUI에서 상세 상태를 표시한다.
- **리팩토링**: Tkinter 대신 Qt/웹 기반 UI를 사용할 경우에도 ROS Node → 이벤트 큐 → 렌더러 구조를 유지하도록 한다.

## 9. 가독성 향상을 위한 권장 기능

코드 검토 결과, 다음 항목을 Test GUI에 추가하면 인수 인계 담당자와 QA가 상황을 더 빠르게 이해할 수 있다.

1. **인터페이스별 상태 배너**: Drive/Arm/AI/Voice 인터페이스의 초기화 여부, 최근 호출, 마지막 오류 메시지를 패널 상단에 표시한다. (각 인터페이스의 Mock/실 장비 구분, 최근 응답 시간 포함)
2. **세션 타이머 표시**: LISTENING과 목적지 선택 세션의 남은 시간을 Progress Bar로 노출해 타임아웃 임박 상황을 직관적으로 알 수 있게 한다.
3. **작업 타임라인 로그**: `task_executors`에서 발행하는 서브 상태/피드백을 시간순으로 시각화하여 특정 단계에서 실패했을 때 즉시 파악할 수 있도록 한다.
4. **배터리 임계치 알림**: `BatteryManager`의 warning/critical 상태 전환 시 팝업 및 색상 Highlight로 QA가 충전 상태를 놓치지 않게 한다.
5. **환경 정보 요약**: `config/patrol_routes.yaml` 등에서 로딩한 순찰 경로/중요 위치를 GUI에 리스트업해 어떤 환경 설정으로 테스트 중인지 공유한다.
6. **Mock 제어 창**: `mock/mock_*` 인터페이스의 응답 프로파일을 GUI에서 즉시 변경하거나 초기화할 수 있는 패널을 제공한다.
7. **Nav2 순찰 모니터링**: ROAMING 모드에서 현재 웨이포인트 index, 진행률, 반복 횟수를 상태 패널에 추가한다(순찰 기능 도입 후).

---

담당자는 본 문서를 기반으로 Test GUI 요구사항을 명세하고, 기능 추가 시 섹션을 갱신하여 QA·운영 인수인계를 용이하게 해야 한다.
