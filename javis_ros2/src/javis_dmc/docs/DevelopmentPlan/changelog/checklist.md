# JAVIS DMC 개선 체크리스트 (v2025-03)

본 문서는 DevelopmentPlan/TestGuiDesign/StateIntrospection 설계에 따라 구현을 진행하기 위한 실천 항목을 정리한다. 각 항목은 `상태` 값과 `비고`를 갱신하면서 추적한다.

| 구분 | 항목 | 작업 내용 | 담당 | 상태 | 비고 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| 상태 인트로스펙션 | describe_state_machine 서비스 구현 | `JavisDmcNode`에 상태 메타데이터 제공 서비스 추가, `javis_dmc_test_msgs` 업데이트 |  | 완료 | `StateIntrospection.md` 참고, Trigger JSON 버전 구현 |
| 상태 인트로스펙션 | describe_state_machine 서비스 구현 | `JavisDmcNode`에 상태 메타데이터 제공 서비스 추가, `javis_dmc_test_msgs` 업데이트 |  | 완료 | `StateIntrospection.md` 참고, Trigger JSON 버전 구현 |
| 상태 인트로스펙션 | state_transitions 디버그 토픽 | 상태 전이 이벤트 퍼블리시(선택), Test GUI 하이라이트 연동 |  | 완료 | JSON String 형태로 구현 |
| Test GUI | 상태 다이어그램 렌더링 | 서비스 응답 기반 Canvas/SVG 구현, 현재 상태 하이라이트/깜빡임 |  | 완료 | `TestGuiDesign.md` §4.4 |
|  | Test/Actual 모드 토글 | Mock ↔ 실 장비 인터페이스 선택 UI, Launch/파라미터 연동 |  | 완료 |  |
|  | 실패 팝업 처리 | 시나리오 실행 중 실패 시 “성공 간주/실패 유지/재시도” 선택 팝업 구현 |  | 완료 |  |
|  | 시나리오 Runner | YAML 기반 Step 실행, `interactive=True` 옵션 처리 |  | 완료 |  |
|  | Mock 제어 패널 | `mock/mock_*` 응답 프로파일 설정 UI 제공 |  | 완료 | set_mock_response 연동 |
| Nav2 순찰 | PatrolWaypoints 액션 | `javis_interfaces` 확장, DDC 연동 |  | 완료 | `NavWaypointDesign.md` |
|  | DriveInterface 확장 | `start_patrol/cancel_patrol` API 추가, Mock 구현 포함 |  | 완료 |  |
|  | DMC 로직 | AUTONOMY 진입 시 순찰 시작/중단, 상태 업데이트 |  | 완료 |  |
| Task Executor | 실행자 로직 점검 | 중복 로직 정리, 결과/피드백 일관성 확보 |  | 대기 |  |
| 문서 동기화 | DevelopmentPlan/TestGuiDesign/StateIntrospection | 구현 진행에 따라 문서 문구 갱신 |  | 대기 |  |

> 체크 방법: 작업 착수 시 `상태`를 `진행중`으로 변경하고, 완료 후 `완료`로 전환하며 필요시 `비고`에 커밋 SHA 또는 참고 사항을 기록한다.
