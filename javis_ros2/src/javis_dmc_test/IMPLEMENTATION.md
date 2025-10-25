# javis_dmc_test 패키지 구현 완료

**작성일**: 2025-10-25
**상태**: Phase 1 완료 (Mock Bridge + Mock RCS 노드)

---

## 구현된 컴포넌트

### 1. Mock Bridge Node (`mock_bridge_node.py`)

DMC의 서비스/액션 호출을 가로채서 Mock 응답을 반환하는 브릿지 노드.

**주요 기능**:
- **개별 메서드별 Mock 제어**: 28개 메서드를 개별적으로 활성화/비활성화
- **실시간 설정 변경**: 노드 재시작 없이 Mock 응답 변경 가능
- **지연 시간 시뮬레이션**: 각 메서드별로 delay 설정 가능
- **성공/실패 제어**: 각 메서드별로 success/failure 응답 제어

**구현된 Mock 메서드**:

Drive Interface (4개):
- `move_to_target` (Action)
- `stop` (Service)
- `resume` (Service)
- `start_patrol` (Service)

Arm Interface (3개):
- `pick_book` (Action)
- `place_book` (Action)
- `change_pose` (Service)

AI Interface (2개):
- `detect_book` (Service)
- `change_tracking_mode` (Service)

**제어 인터페이스**:
- Service: `/test/mock_bridge/set_method`
- 타입: `SetMockMethod.srv`
- 파라미터:
  - `interface`: 'drive', 'arm', 'ai'
  - `method`: 메서드 이름
  - `enabled`: Mock 활성화 여부
  - `success`: 성공/실패 응답
  - `delay`: 지연 시간(초)

### 2. Mock RCS Node (`mock_rcs_node.py`)

TEST_GUI에서 DMC로 작업 Goal을 전송하는 Mock RCS 노드.

**주요 기능**:
- 실제 RCS를 대체하여 테스트 작업 전송
- DMC의 Action Server로 Goal 전송
- Goal 수락/거부 및 작업 결과 로깅

**구현된 작업**:
- `send_pickup_task()`: 도서 픽업 작업
- `send_guiding_task()`: 길 안내 작업
- `send_reshelving_task()`: 반납 정리 작업
- `send_clean_seat_task()`: 좌석 청소 작업

**제어 인터페이스**:
- Service: `/test/mock_rcs/send_task`
- 타입: `SendTask.srv`
- 파라미터:
  - `task_type`: 'pickup', 'guiding', 'reshelving', 'clean_seat'

### 3. Service Definitions (`javis_dmc_test_msgs`)

새로 추가된 서비스 정의:

**SetMockMethod.srv**:
```
string interface
string method
bool enabled
bool success
float64 delay
---
bool success
string message
```

**SendTask.srv**:
```
string task_type
---
bool success
string message
```

### 4. Launch 파일 (`test_full.launch.py`)

전체 테스트 환경을 실행하는 Launch 파일.

**실행 노드**:
- DMC Node (javis_dmc)
- Mock Bridge Node (javis_dmc_test)
- Mock RCS Node (javis_dmc_test)

**Launch Arguments**:
- `robot_namespace`: 로봇 네임스페이스 (기본값: 'dobby1')
- `use_mock`: Mock Bridge 사용 여부 (기본값: 'true')

### 5. 설정 파일 (`mock_responses.yaml`)

Mock Bridge의 기본 응답 설정 파일.

**구조**:
```yaml
drive:
  move_to_target:
    enabled: false
    success: true
    delay: 0.0
  ...

arm:
  pick_book:
    enabled: false
    success: true
    delay: 3.0
  ...

ai:
  detect_book:
    enabled: false
    success: true
    delay: 0.5
  ...
```

---

## 패키지 구조

```
javis_dmc_test/
├── javis_dmc_test/
│   ├── __init__.py
│   ├── mock_bridge_node.py      # Mock Bridge 메인 노드
│   └── mock_rcs_node.py          # Mock RCS 노드
├── launch/
│   └── test_full.launch.py       # 전체 테스트 환경 Launch
├── config/
│   └── mock_responses.yaml       # 기본 Mock 응답 설정
├── resource/
│   └── javis_dmc_test
├── package.xml
├── setup.py
├── README.md
└── IMPLEMENTATION.md             # 이 파일
```

```
javis_dmc_test_msgs/
├── srv/
│   ├── SetBattery.srv            # 기존
│   ├── SetMockResponse.srv       # 기존
│   ├── SetMockMethod.srv         # 신규
│   └── SendTask.srv              # 신규
├── CMakeLists.txt
└── package.xml
```

---

## 빌드 및 실행

### 빌드

```bash
cd ~/dev_ws/roscamp-repo-2/javis_ros2

# 메시지 패키지 빌드
colcon build --packages-select javis_dmc_test_msgs --symlink-install

# 테스트 패키지 빌드
colcon build --packages-select javis_dmc_test --symlink-install

# 전체 빌드
colcon build --symlink-install

source install/setup.bash
```

### 실행

**개별 노드 실행**:
```bash
# Mock Bridge 노드
ros2 run javis_dmc_test mock_bridge_node

# Mock RCS 노드
ros2 run javis_dmc_test mock_rcs_node
```

**Launch 파일 실행**:
```bash
ros2 launch javis_dmc_test test_full.launch.py

# 네임스페이스 지정
ros2 launch javis_dmc_test test_full.launch.py robot_namespace:=dobby2

# Mock 비활성화
ros2 launch javis_dmc_test test_full.launch.py use_mock:=false
```

---

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

---

## 테스트 시나리오

### 시나리오 1: 도서 픽업 성공

1. Mock 설정:
   ```bash
   ros2 service call /test/mock_bridge/set_method javis_dmc_test_msgs/srv/SetMockMethod \
     "{interface: 'drive', method: 'move_to_target', enabled: true, success: true, delay: 0.5}"

   ros2 service call /test/mock_bridge/set_method javis_dmc_test_msgs/srv/SetMockMethod \
     "{interface: 'arm', method: 'pick_book', enabled: true, success: true, delay: 3.0}"
   ```

2. 작업 전송:
   ```bash
   ros2 service call /test/mock_rcs/send_task javis_dmc_test_msgs/srv/SendTask \
     "{task_type: 'pickup'}"
   ```

3. 예상 결과:
   - DMC가 도서 픽업 작업 시작
   - move_to_target이 0.5초 후 성공 응답
   - pick_book이 3초 후 성공 응답
   - 작업 완료

### 시나리오 2: 도서 집기 실패

1. Mock 설정:
   ```bash
   ros2 service call /test/mock_bridge/set_method javis_dmc_test_msgs/srv/SetMockMethod \
     "{interface: 'drive', method: 'move_to_target', enabled: true, success: true, delay: 0.5}"

   ros2 service call /test/mock_bridge/set_method javis_dmc_test_msgs/srv/SetMockMethod \
     "{interface: 'arm', method: 'pick_book', enabled: true, success: false, delay: 3.0}"
   ```

2. 작업 전송:
   ```bash
   ros2 service call /test/mock_rcs/send_task javis_dmc_test_msgs/srv/SendTask \
     "{task_type: 'pickup'}"
   ```

3. 예상 결과:
   - DMC가 도서 픽업 작업 시작
   - move_to_target이 0.5초 후 성공 응답
   - pick_book이 3초 후 **실패** 응답
   - DMC가 실패 처리 로직 실행 (재시도 또는 중단)

---

## 아키텍처 개선 효과

### 이전 (DMC 내장 Mock)
- DMC와 Mock이 강하게 결합
- Mock/Real 전환 시 노드 재시작 필수
- 인터페이스 단위 제어만 가능 (4개)
- 디버깅 어려움

### 개선 후 (Mock Bridge)
- DMC와 Mock 완전 분리
- **실시간 Mock 전환** (재시작 불필요)
- **메서드별 세밀한 제어** (28개)
- GUI를 통한 쉬운 디버깅

---

## 다음 단계

### Phase 2: TEST_GUI 개선 (예정)
- [ ] Mock 제어 패널 UI 구현
- [ ] 작업 전송 패널 구현
- [ ] 기존 "수동 상태 설정" 제거
- [ ] Mock Bridge와 통합

### Phase 3: 전체 인터페이스 Mock 구현 (예정)
- [ ] 나머지 Drive 메서드 (6개)
- [ ] 나머지 Arm 메서드 (3개)
- [ ] 나머지 AI 메서드 (5개)
- [ ] GUI/VRC Mock

### Phase 4: DMC Mock 코드 제거 (예정)
- [ ] dmc_node.py에서 Mock import 제거
- [ ] use_mock_interfaces 파라미터 제거
- [ ] Mock 인터페이스 클래스 제거

---

## 참고 문서

- `/javis_dmc/docs/DevelopmentPlan/TEST_GUI_REFACTOR_V2.md`: 전체 설계 문서
- `/javis_dmc/docs/DevelopmentPlan/TEST_GUI_IMPROVEMENTS.md`: 기존 문제점 분석
- `/javis_dmc/README.md`: DMC 패키지 설명

---

**작성자**: Claude Code
**검토**: 필요
