# JAVIS DMC 리그레션 테스트 체크리스트 (v2025-03)

각 항목은 ✅/❌ 체크 후 `노트`에 결과 및 이슈 링크를 남긴다. UI/의존 모듈은 Test 모드와 실제 모드를 모두 확인한다.

## 1. 서비스/토픽 노출
| 항목 | 명령 | 기대 결과 | 상태 | 노트 |
| :--- | :--- | :--- | :--- | :--- |
| describe_state_machine 서비스 | `ros2 service call /debug/describe_state_machine std_srvs/srv/Trigger {}` | success=true, JSON payload 반환 |  |  |
| state_transitions 토픽 | `ros2 topic echo /debug/state_transitions` | 상태 변경 시 이벤트 출력 |  |  |
| set_mock_response 서비스 | `ros2 service call /test/set_mock_response javis_dmc_test_msgs/srv/SetMockResponse "{interface_name: 'drive', scenario: 'move_to_target:ERROR', success_override: false}"` | success=true, Mock 설정 로그 |  |  |

## 2. Test GUI 상태 다이어그램
| 단계 | 절차 | 기대 결과 | 상태 | 노트 |
| :--- | :--- | :--- | :--- | :--- |
| 초기 로드 | Test GUI 실행 후 describe 서비스 호출 성공 | 상태 그래프가 우측 패널에 표시 |  |  |
| 상태 하이라이트 | `ros2 topic pub /status/robot_state`로 상태 변경 | 해당 노드가 하이라이트 표시 |  |  |
| 전이 표시 | `ros2 topic pub /debug/state_transitions`로 전이 이벤트 송출 | 화살표/하이라이트 갱신 |  |  |

## 3. Test/Actual 모드 토글
| 단계 | 절차 | 기대 결과 | 상태 | 노트 |
| :--- | :--- | :--- | :--- | :--- |
| Mock → Real 전환 | GUI 라디오 버튼(Mock→Real) 클릭 | `use_mock_interfaces` 파라미터 false, 로그에 전환 메시지 |  |  |
| Real → Mock 전환 | Real→Mock | 파라미터 true, 상태 메시지 |  |  |

## 4. 시나리오 Runner
| 항목 | 절차 | 기대 결과 | 상태 | 노트 |
| :--- | :--- | :--- | :--- | :--- |
| 성공 케이스 | `guiding_smoke` 실행 | 모든 Step SUCCESS 로그 |  |  |
| Fail + 간주 | Mock 실패 설정 후 실행 → 팝업에서 “예” 선택 | “성공 간주” 로그, 시나리오 진행 계속 |  |  |
| Fail + 중단 | 동일 상황에서 “아니오” 선택 | ERROR 로그, 시나리오 종료 |  |  |
| 취소 | “취소” 선택 | 시나리오 중단, 사용자 취소 메시지 |  |  |

## 5. LISTENING/목적지 세션 타임아웃
| 항목 | 절차 | 기대 결과 | 상태 | 노트 |
| :--- | :--- | :--- | :--- | :--- |
| LISTENING 타임아웃 | `set_listening_mode` 후 입력 없음 | 20초 후 자동 종료, 상태 전이 이벤트 |  |  |
| 목적지 타임아웃 | GUI 목적지 미선택 시나리오 실행 | 60초 후 aborted 로그, 상태 전이 |  |  |

## 6. Mock 인터페이스
| 항목 | 절차 | 기대 결과 | 상태 | 노트 |
| :--- | :--- | :--- | :--- | :--- |
| Mock Drive 응답 | `set_next_result(success=False)` 적용 후 시나리오 실행 | 실패 팝업 등장, 로그 기록 |  |  |
| Mock Arm 응답 | 동일 | 실패 검출 및 처리 |  |  |

## 7. Nav2 순찰
| 항목 | 절차 | 기대 결과 | 상태 | 노트 |
| :--- | :--- | :--- | :--- | :--- |
| 순찰 시작 | AUTONOMY 모드 전환 → `ros2 service call /debug/describe_state_machine` | `runtime.patrol_active: true`, 순찰 시작 로그 |  |  |
| 순찰 중단 | Standby 모드 전환 혹은 작업 시작 | `runtime.patrol_active: false`, cancel 로그 |  |  |
| Mock 순찰 | Mock 모드에서 `test/set_mock_response`로 `start_patrol` 실패 설정 후 자동 전환 | 실패 로그 및 상태 유지 확인 |  |  |

## 8. 단위 테스트 & 컴파일
| 항목 | 명령 | 기대 결과 | 상태 | 노트 |
| :--- | :--- | :--- | :--- | :--- |
| 컴파일 확인 | `python -m compileall javis_dmc` | 오류 없음 | ✅ | 2025-03-15 확인 |
| 단위 테스트 | `colcon test` 또는 개별 pytest | 통과 |  |  |

---

이 체크리스트는 매 빌드마다 업데이트되며, 신규 기능 추가 시 섹션을 확장한다.
