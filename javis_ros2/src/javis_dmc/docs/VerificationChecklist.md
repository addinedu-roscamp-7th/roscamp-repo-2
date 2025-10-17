# JAVIS DMC 검증 체크리스트

## 빌드 및 환경
- [ ] `source /opt/ros/jazzy/setup.bash` 후 `colcon build --symlink-install`
- [ ] `colcon test` 및 `colcon test-result` 확인

## 노드 기동 확인
- [ ] `ros2 launch javis_dmc dmc_single.launch.py` 실행
- [ ] `ros2 action list | grep main/`으로 액션 서버 5종 확인
- [ ] `ros2 topic echo`로 `status/robot_state`, `status/battery_status` 갱신 확인

## 기능 시나리오
- [ ] 각 작업 액션 Goal 전송 후 상태 전이(메인/서브) 로그 확인
- [ ] 배터리 충전/소모 타이머가 임계값에 따라 상태를 전환하는지 확인
- [ ] 취소 요청 시 `cancel_current_task`가 정상 수행되는지 테스트

## Mock 및 테스트 도구
- [ ] `javis_dmc_test_msgs/SetBattery`, `SetMockResponse` 서비스 호출 검증
- [ ] Test GUI에서 배터리/모의 응답 제어가 가능한지 확인

## 문서 및 정합성
- [ ] `docs/DevelopmentPlan.md`와 코드 구조 비교
- [ ] 상태 정의(`states/state_enums.py`)와 메시지(`DobbyState.msg`) 값 일치 검토
- [ ] 테스트 케이스 및 린트 스크립트 최신화
