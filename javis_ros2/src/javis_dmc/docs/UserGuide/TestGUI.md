# JAVIS DMC Test GUI 사용자 가이드 v1.0

## 1. 개요
Test GUI는 DMC Node를 수동 제어하고 상태/시나리오를 검증하기 위한 운영·QA 도구입니다. 본 문서는 기본 사용 절차, 시나리오 실행, Mock 응답 제어, 상태 다이어그램 활용 방법을 설명합니다.

## 2. 실행 방법
```bash
ros2 run javis_dmc test_gui_node --ros-args -p robot_namespace:=dobby1
```
- `robot_namespace`는 대상 DMC 인스턴스(예: `dobby1`)와 동일하게 설정합니다.
- Mock 인터페이스를 사용하려면 DMC를 `use_mock_interfaces:=true` 파라미터와 함께 실행하십시오.

## 3. UI 구성
### 3.1 좌측 패널
- **로봇 상태**: 현재 모드, 메인/서브 상태, 배터리, LISTENING, 관리자 피드백이 표시됩니다. 상태에 따라 색상이 변화합니다.
- **제어 패널**: 모드 전환, 긴급 정지/해제, LISTENING 토글, 작업 강제 완료/실패, 수동 상태 설정, Test/Actual 모드 토글, Mock/Real 라디오 버튼, 시나리오 실행, Mock 응답 설정을 제공합니다.
- **작업 패널**: QA 메모용 작업 목록을 관리합니다.
- **이벤트 로그**: 전체 로그가 레벨별 색상 태그로 표시됩니다.

### 3.2 우측 패널
- **상태 다이어그램**: `describe_state_machine` 서비스로 로드한 상태 그래프를 Canvas에 렌더링합니다.
  - 현재 상태는 하이라이트(파란색)로 표시됩니다.
  - `debug/state_transitions` 토픽을 통해 수신한 전이 이벤트는 다이어그램에 반영됩니다.
  - 모드 정보(`use_mock_interfaces`)가 상단에 표시됩니다.

## 4. 시나리오 실행
- `config/test_gui_scenarios.yaml`에서 시나리오를 정의합니다(예: `guiding_smoke`).
- 제어 패널의 "시나리오 실행"에서 선택 후 `시작` 버튼을 클릭합니다.
- Test 모드가 활성화되어 있으면 Step 실패 시 팝업이 표시되며 `성공 간주/실패/취소`를 선택할 수 있습니다.
- Step 액션 종류:
  - `call_service`: `set_listening_mode`, `set_robot_mode`, `emergency_stop`, `resume_navigation`, `force_task_success/failure`
  - `wait_for_state`: 지정된 메인/서브 상태가 될 때까지 대기
  - `sleep`: 지정 시간 대기
  - `publish_mock`: Mock 환경에서 임의 이벤트를 삽입(현재 로그만 기록)
  - `log`: 메시지를 로그에 기록

## 5. Mock 응답 제어
- 제어 패널 하단의 "Mock 응답 제어" 섹션을 사용합니다.
  - 인터페이스(`drive`, `arm`, `ai`, `gui`)와 메서드명을 입력합니다.
  - 에러 코드가 필요하면 `method:error_code` 형식으로 작성합니다.
  - `성공으로 설정`/`실패로 설정` 버튼을 클릭하면 `test/set_mock_response` 서비스가 호출됩니다.
- Mock 모드가 아닌 경우 설정이 적용되지 않으므로 주의하십시오.

## 6. Test/Actual 모드 토글
- "Test 모드" 체크박스를 활성화하면 시나리오 실패 시 팝업이 표시되어 흐름을 제어할 수 있습니다.
- "인터페이스" 라디오 버튼은 Mock ↔ Real 전환 서비스(`set_parameters`)를 호출합니다. 현재 실행 중인 DMC에는 즉시 적용되지 않으므로 필요 시 노드를 재시작하십시오.

## 7. 상태 다이어그램 사용
- `describe_state_machine` 호출 실패 시 GUI가 경고를 표시합니다. 이 경우 DMC 노드가 실행 중인지, 네임스페이스가 올바른지 확인하십시오.
- 하이라이트가 갱신되지 않는다면 `debug/state_transitions` 구독 여부를 확인 (`ros2 topic echo`)하세요.

## 8. 테스트 시나리오 예시
1. **LISTENING 타임아웃 확인**
   - `set_listening_mode` 실행 후 사용자 입력 없이 20초 대기.
   - 상태 다이어그램에서 `LISTENING → IDLE` 전이가 나타나는지 확인.
2. **GUIDING 시나리오**
   - Mock 환경에서 `guiding_smoke` 실행.
   - 각 Step이 SUCCESS로 기록되는지 로그 확인.
3. **Mock 실패 주입**
   - Mock 패널에서 `drive` / `navigate_to_pose` / `ERROR` 입력 후 실패 설정.
   - 시나리오 실행 시 팝업이 뜨고 로그가 기록되는지 확인.

## 9. 순찰 상태 모니터링
- AUTONOMY 모드 전환 시 로그에 순찰 시작 메시지가 표시되고, `describe_state_machine` 응답의 `runtime.patrol_active` 값이 true가 됩니다.
- `debug/state_transitions` 토픽을 통해 `ROAMING` 상태 유지 동안 순찰 관련 이벤트를 확인할 수 있습니다 (별도 서브 상태는 사용하지 않습니다).
- Standby 모드로 전환하거나 작업이 시작되면 순찰이 자동으로 취소되고 `patrol_active`가 false로 바뀌어야 합니다.
- `patrol_routes.yaml`을 수정한 뒤 DMC를 재시작하면 즉시 새로운 경로로 순찰이 시작됩니다.

## 10. 문제 해결
- 서비스 미준비 경고: DMC가 실행 중인지, 네임스페이스가 일치하는지 확인.
- 상태 다이어그램이 비어 있음: `describe_state_machine` 서비스 응답이 0인지 확인.
- Mock 설정 적용되지 않음: DMC가 `use_mock_interfaces:=true`로 실행됐는지 확인.

## 11. 참고 문서
- `docs/DevelopmentPlan/TestGuiDesign.md`
- `docs/DevelopmentPlan/StateIntrospection.md`
- `docs/DevelopmentPlan/changelog/checklist.md`
- `docs/DevelopmentPlan/changelog/test_checklist.md`
- `docs/DevelopmentPlan/NavWaypointDesign.md`
