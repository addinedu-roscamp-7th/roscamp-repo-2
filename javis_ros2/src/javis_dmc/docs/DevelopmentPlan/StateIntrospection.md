# DMC 상태 그래프 인트로스펙션 설계 v0.1

## 1. 목표

- `JavisDmcNode`가 현재 지원하는 메인/서브 상태와 전이 규칙을 외부 도구(Test GUI, Admin 도구)가 동적으로 조회할 수 있게 한다.
- 테스트/운영 모드에 따라 상태 디스플레이와 전이 하이라이트를 자동 갱신해 문서-코드 불일치를 줄인다.
- Nav2 순찰, LISTENING, 작업 전환 등 복잡한 상태 흐름을 GUI에서 직관적으로 파악하게 한다.

## 2. 제공 방식

### 2.1 ROS 서비스: `describe_state_machine`

- 네임스페이스: `{robot}/debug/describe_state_machine`
- 타입: `javis_dmc_test_msgs/srv/DescribeStateMachine` (신규) 또는 `std_srvs/Trigger` + JSON 문자열
- 응답 필드(예시):

```json
{
  "modes": [
    {"id": 0, "name": "STANDBY", "description": "충전소 대기"},
    {"id": 1, "name": "AUTONOMY", "description": "순찰 및 자동 작업"}
  ],
  "main_states": [
    {
      "id": 2,
      "name": "IDLE",
      "description": "작업 대기",
      "available_from": ["STANDBY"],
      "transitions": ["LISTENING", "PICKING_UP_BOOK", "MOVING_TO_CHARGER"]
    },
    ...
  ],
  "sub_states": [
    {"id": 100, "name": "NONE"},
    {"id": 109, "name": "SELECT_DEST", "parent": "GUIDING"},
    ...
  ],
  "transition_rules": [
    {
      "from": "GUIDING",
      "to": "LISTENING",
      "trigger": "wake_word",
      "conditions": ["mode in [AUTONOMY, STANDBY]"]
    },
    ...
  ],
  "runtime": {
    "use_mock_interfaces": true
  }
}
```

- `DmcStateMachine` 초기화 시 상태/전이 정보를 테이블로 구성하고 서비스에서 그대로 반환한다.
- 배터리 경고/위험 조건, 세션 타이머 등 추가 조건도 `conditions` 필드에 문자열 배열로 명시한다.

### 2.2 디버그 토픽(선택)

- 토픽: `{robot}/debug/state_transitions`
- 메시지: `javis_dmc_test_msgs/msg/StateTransitionEvent` (from, to, reason, timestamp)
- GUI가 실시간으로 상태 변경 이벤트를 수신해 다이어그램 하이라이트를 즉시 갱신할 수 있도록 한다.

## 3. Test GUI 연동

### 3.1 초기 로딩 흐름

1. GUI 시작 시 `describe_state_machine` 서비스 호출 → 상태 그래프 데이터 캐시
2. Tkinter Canvas 또는 SVG 렌더러를 사용해 노드/화살표를 그린다
   - `main_states` 배열 순서대로 박스 생성
   - `transition_rules`를 순회하면서 화살표/조건 레이블 추가
   - `sub_states`는 메인 상태 박스 내부 또는 별도 섹션으로 표시
3. `status/robot_state` 구독으로 현재 상태를 받으면 해당 노드/화살표를 강조(색상, 깜빡임 등)
4. 디버그 토픽이 활성화된 경우 이벤트 기반으로 다이어그램을 즉시 업데이트

### 3.2 UI 동작 예시

- 노드 클릭 시 상태 설명/조건 팝업
- 화살표 클릭 시 트리거/조건 표시
- 배터리 경계, LISTENING 세션 등 특별 조건은 다이어그램에 아이콘으로 표시

### 3.3 모드 전환

- Test/Actual 모드 토글 버튼을 GUI에 두고, 선택 값에 따라 Mock 스위치/팝업 동작이 달라지도록 한다.
- 상태 그래프는 두 모드에서 동일하게 사용하지만, Test 모드에서는 `override_success` 같은 플래그를 표시할 수 있다.

## 4. 구현 가이드

1. `javis_dmc_test_msgs` 패키지에 새로운 srv/msg 정의 추가 (또는 JSON 기반 Trigger 사용)
2. `JavisDmcNode`에 서비스 서버 등록 (`_describe_state_machine`)
3. Test GUI에서 서비스 호출 + Canvas 렌더러 구현 (예: `test_gui/state_graph_widget.py`)
4. 디버그 토픽을 선택적으로 활성화 (Launch 파라미터)
5. 문서 `StateDefinition.md` 업데이트 시 서비스 응답도 동일한 데이터를 참조하도록 유지

## 5. 문서 연계

- `DevelopmentPlan/DevelopmentPlan.md` §4.5, §8.2
- `DevelopmentPlan/TestGuiDesign.md` UI 구성/확장 가이드
- `docs/StateDefinition/StateDefinition.md` 상태 정의 테이블

---

향후 구현 담당자는 본 문서를 기준으로 상태 인트로스펙션 기능을 개발하고 Test GUI와 연결한다. 서비스/토픽 명세가 변경될 경우 문서를 먼저 수정한 뒤 코드를 따라 업데이트한다.
