# javis_dmc_test

JAVIS DMC 테스트 도구 패키지

## 개요

DMC(Dobby Main Controller)의 Mock 테스트 환경을 제공하는 패키지입니다. DMC 노드와 완전히 분리된 Mock Bridge 아키텍처를 통해 실시간으로 개별 메서드의 Mock 응답을 제어할 수 있습니다.

## 포함 내용

- **Mock Bridge Node**: DMC의 서비스/액션 호출을 가로채서 Mock 응답 반환 (✅ 완료)
- **Mock RCS Node**: 테스트용 작업 Goal 전송 (실제 RCS 역할) (✅ 완료)
- **Test GUI (Simple)**: Mock 제어 및 작업 전송 GUI (✅ 완료)

## 주요 기능

### Mock Bridge
- ✅ 28개 메서드별 개별 Mock 제어
- ✅ 실시간 설정 변경 (재시작 불필요)
- ✅ 지연 시간 시뮬레이션
- ✅ 성공/실패 응답 제어

### Mock RCS
- ✅ 도서 픽업 작업 전송
- ✅ 길 안내 작업 전송
- ✅ 반납 정리 작업 전송
- ✅ 좌석 청소 작업 전송

## 빌드

```bash
cd ~/dev_ws/roscamp-repo-2/javis_ros2

# 메시지 패키지 빌드
colcon build --packages-select javis_dmc_test_msgs --symlink-install

# 테스트 패키지 빌드
colcon build --packages-select javis_dmc_test --symlink-install

source install/setup.bash
```

## 실행 방법

### Launch 파일 실행 (권장)

```bash
# 전체 테스트 환경 실행 (DMC + Mock Bridge + Mock RCS)
ros2 launch javis_dmc_test test_full.launch.py

# 네임스페이스 지정
ros2 launch javis_dmc_test test_full.launch.py robot_namespace:=dobby2

# Mock 비활성화 (실제 하위 컨트롤러 사용)
ros2 launch javis_dmc_test test_full.launch.py use_mock:=false
```

### 개별 노드 실행

```bash
# Mock Bridge 노드
ros2 run javis_dmc_test mock_bridge_node

# Mock RCS 노드
ros2 run javis_dmc_test mock_rcs_node

# Test GUI (Simple)
ros2 run javis_dmc_test test_gui_simple
```

## 사용 예시

### Mock 설정 변경

```bash
# move_to_target Mock 활성화 (성공 응답, 1초 지연)
ros2 service call /test/mock_bridge/set_method javis_dmc_test_msgs/srv/SetMockMethod \
  "{interface: 'drive', method: 'move_to_target', enabled: true, success: true, delay: 1.0}"

# pick_book Mock 활성화 (실패 응답, 3초 지연)
ros2 service call /test/mock_bridge/set_method javis_dmc_test_msgs/srv/SetMockMethod \
  "{interface: 'arm', method: 'pick_book', enabled: true, success: false, delay: 3.0}"
```

### 작업 전송

```bash
# 도서 픽업 작업 전송
ros2 service call /test/mock_rcs/send_task javis_dmc_test_msgs/srv/SendTask \
  "{task_type: 'pickup'}"

# 길 안내 작업 전송
ros2 service call /test/mock_rcs/send_task javis_dmc_test_msgs/srv/SendTask \
  "{task_type: 'guiding'}"
```

## 문서

- **구현 상세**: `IMPLEMENTATION.md` - 구현 완료 내용 및 사용 방법
- **설계 문서**: `/javis_dmc/docs/DevelopmentPlan/TEST_GUI_REFACTOR_V2.md` - 전체 아키텍처 설계
- **문제점 분석**: `/javis_dmc/docs/DevelopmentPlan/TEST_GUI_IMPROVEMENTS.md` - 기존 문제점

## 아키텍처

```
RCS (Mock) ──[Action Goal]──> DMC Node ──[Service/Action]──> Real Controllers
                                                                      ↑
                                                       Mock Bridge (가로채기)
                                                                      ↓
                                            TEST_GUI ──[실시간 제어]──> Mock Bridge
```

## 다음 단계

- [ ] TEST_GUI Mock 제어 패널 구현
- [ ] 나머지 인터페이스 메서드 Mock 구현
- [ ] DMC 노드에서 Mock 코드 제거

## 상태

- **Phase 1**: ✅ 완료 (Mock Bridge + Mock RCS 노드)
- **Phase 2**: ✅ 완료 (Simple TEST_GUI 구현)
- **Phase 3**: 🔜 예정 (전체 인터페이스 Mock 확장)
- **Phase 4**: 🔜 예정 (DMC Mock 코드 제거)
