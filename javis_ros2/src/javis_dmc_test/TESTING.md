# javis_dmc_test 테스트 가이드

**작성일**: 2025-10-25
**목적**: Mock Bridge 및 Mock RCS 노드 테스트 절차

---

## 사전 준비

### 1. 빌드

```bash
cd ~/dev_ws/roscamp-repo-2/javis_ros2

# 메시지 패키지 빌드
colcon build --packages-select javis_dmc_test_msgs --symlink-install

# 테스트 패키지 빌드
colcon build --packages-select javis_dmc_test --symlink-install

# 환경 변수 로드
source install/setup.bash
```

### 2. 빌드 확인

```bash
# 패키지 확인
ros2 pkg list | grep javis_dmc_test

# 실행 파일 확인
ros2 pkg executables javis_dmc_test

# 서비스 타입 확인
ros2 interface list | grep javis_dmc_test_msgs
```

**예상 출력**:
```
javis_dmc_test
javis_dmc_test_msgs

javis_dmc_test mock_bridge_node
javis_dmc_test mock_rcs_node

javis_dmc_test_msgs/srv/SendTask
javis_dmc_test_msgs/srv/SetBattery
javis_dmc_test_msgs/srv/SetMockMethod
javis_dmc_test_msgs/srv/SetMockResponse
```

---

## 테스트 시나리오

### 시나리오 1: Mock Bridge 노드 단독 테스트

**목적**: Mock Bridge가 정상적으로 실행되고 서비스를 제공하는지 확인

#### Step 1: Mock Bridge 노드 실행

**터미널 1**:
```bash
cd ~/dev_ws/roscamp-repo-2/javis_ros2
source install/setup.bash

ros2 run javis_dmc_test mock_bridge_node
```

**예상 출력**:
```
[INFO] [mock_bridge_node]: Mock Action Servers 생성 완료
[INFO] [mock_bridge_node]: Mock Service Servers 생성 완료
[INFO] [mock_bridge_node]: Mock Bridge 노드 초기화 완료 (namespace: dobby1)
```

#### Step 2: 노드 및 서비스 확인

**터미널 2**:
```bash
source install/setup.bash

# 노드 확인
ros2 node list

# 서비스 확인
ros2 service list | grep test

# Action 확인
ros2 action list | grep dobby1
```

**예상 출력**:
```
/mock_bridge_node

/test/mock_bridge/set_method

/dobby1/arm/pick_book
/dobby1/arm/place_book
/dobby1/drive/move_to_target
```

#### Step 3: Mock 설정 변경 테스트

**터미널 2**:
```bash
# move_to_target Mock 활성화
ros2 service call /test/mock_bridge/set_method javis_dmc_test_msgs/srv/SetMockMethod \
  "{interface: 'drive', method: 'move_to_target', enabled: true, success: true, delay: 1.0}"
```

**예상 출력**:
```
waiting for service to become available...
requester: making request: javis_dmc_test_msgs.srv.SetMockMethod_Request(
  interface='drive',
  method='move_to_target',
  enabled=True,
  success=True,
  delay=1.0)

response:
javis_dmc_test_msgs.srv.SetMockMethod_Response(
  success=True,
  message='Mock drive.move_to_target updated: enabled=True, success=True, delay=1.0s')
```

**터미널 1 로그**:
```
[INFO] [mock_bridge_node]: Mock drive.move_to_target updated: enabled=True, success=True, delay=1.0s
```

#### Step 4: Mock Action 호출 테스트

**터미널 2**:
```bash
# move_to_target Action Goal 전송
ros2 action send_goal /dobby1/drive/move_to_target javis_interfaces/action/MoveToTarget \
  "{location_name: 'test_location'}"
```

**예상 출력**:
```
Waiting for an action server to become available...
Sending goal:
     location_name: test_location

Goal accepted with ID: ...

Result:
    success: True
    message: Mock response (success=True)
```

**터미널 1 로그**:
```
[INFO] [mock_bridge_node]: [Mock] move_to_target: test_location
```

#### Step 5: Mock 실패 응답 테스트

**터미널 2**:
```bash
# pick_book Mock 활성화 (실패 응답)
ros2 service call /test/mock_bridge/set_method javis_dmc_test_msgs/srv/SetMockMethod \
  "{interface: 'arm', method: 'pick_book', enabled: true, success: false, delay: 2.0}"

# pick_book Action Goal 전송
ros2 action send_goal /dobby1/arm/pick_book javis_interfaces/action/PickBook \
  "{book_id: 'TEST_BOOK_001'}"
```

**예상 출력**:
```
Result:
    success: False
    book_id: TEST_BOOK_001
    message: Mock response (success=False)

Goal aborted
```

**터미널 1 로그**:
```
[INFO] [mock_bridge_node]: Mock arm.pick_book updated: enabled=True, success=False, delay=2.0s
[INFO] [mock_bridge_node]: [Mock] pick_book: TEST_BOOK_001
```

---

### 시나리오 2: Mock RCS 노드 단독 테스트

**목적**: Mock RCS가 정상적으로 작업 Goal을 전송하는지 확인

#### Step 1: DMC 노드 실행 (필수)

**터미널 1**:
```bash
source install/setup.bash

# DMC 노드 실행 (Mock RCS가 Goal을 전송할 대상)
ros2 run javis_dmc dmc_node --ros-args -p robot_namespace:=dobby1
```

**예상 출력**:
```
[INFO] [dmc_node]: DMC 노드 초기화 완료
[INFO] [dmc_node]: Action Servers 초기화 완료
```

#### Step 2: Mock RCS 노드 실행

**터미널 2**:
```bash
source install/setup.bash

ros2 run javis_dmc_test mock_rcs_node
```

**예상 출력**:
```
[INFO] [mock_rcs_node]: Mock RCS 노드 초기화 완료 (namespace: dobby1)
```

#### Step 3: 작업 전송 테스트

**터미널 3**:
```bash
source install/setup.bash

# 도서 픽업 작업 전송
ros2 service call /test/mock_rcs/send_task javis_dmc_test_msgs/srv/SendTask \
  "{task_type: 'pickup'}"
```

**예상 출력 (터미널 3)**:
```
response:
javis_dmc_test_msgs.srv.SendTask_Response(
  success=True,
  message='pickup task sent')
```

**예상 로그 (터미널 2 - Mock RCS)**:
```
[INFO] [mock_rcs_node]: 작업 전송 요청: pickup
[INFO] [mock_rcs_node]: 도서 픽업 작업 전송: book_id=TEST_BOOK_001, storage_id=1
[INFO] [mock_rcs_node]: 작업 Goal이 수락되었습니다
```

**예상 로그 (터미널 1 - DMC)**:
```
[INFO] [dmc_node]: 도서 픽업 작업 수신: TEST_BOOK_001
[INFO] [dmc_node]: 메인 상태 변경: IDLE -> PICKING_UP_BOOK
```

#### Step 4: 다른 작업 전송 테스트

**터미널 3**:
```bash
# 길 안내 작업
ros2 service call /test/mock_rcs/send_task javis_dmc_test_msgs/srv/SendTask \
  "{task_type: 'guiding'}"

# 반납 정리 작업
ros2 service call /test/mock_rcs/send_task javis_dmc_test_msgs/srv/SendTask \
  "{task_type: 'reshelving'}"

# 좌석 청소 작업
ros2 service call /test/mock_rcs/send_task javis_dmc_test_msgs/srv/SendTask \
  "{task_type: 'clean_seat'}"
```

---

### 시나리오 3: 통합 테스트 (Launch 파일 사용)

**목적**: DMC + Mock Bridge + Mock RCS를 함께 실행하여 전체 흐름 테스트

#### Step 1: Launch 파일 실행

**터미널 1**:
```bash
source install/setup.bash

ros2 launch javis_dmc_test test_full.launch.py
```

**예상 출력**:
```
[INFO] [dmc_node]: DMC 노드 초기화 완료
[INFO] [mock_bridge_node]: Mock Bridge 노드 초기화 완료 (namespace: dobby1)
[INFO] [mock_rcs_node]: Mock RCS 노드 초기화 완료 (namespace: dobby1)
```

#### Step 2: Mock 설정 및 작업 전송

**터미널 2**:
```bash
source install/setup.bash

# Step 2-1: Mock 설정
ros2 service call /test/mock_bridge/set_method javis_dmc_test_msgs/srv/SetMockMethod \
  "{interface: 'drive', method: 'move_to_target', enabled: true, success: true, delay: 1.0}"

ros2 service call /test/mock_bridge/set_method javis_dmc_test_msgs/srv/SetMockMethod \
  "{interface: 'arm', method: 'pick_book', enabled: true, success: true, delay: 3.0}"

ros2 service call /test/mock_bridge/set_method javis_dmc_test_msgs/srv/SetMockMethod \
  "{interface: 'arm', method: 'place_book', enabled: true, success: true, delay: 2.0}"

# Step 2-2: 도서 픽업 작업 전송
ros2 service call /test/mock_rcs/send_task javis_dmc_test_msgs/srv/SendTask \
  "{task_type: 'pickup'}"
```

#### Step 3: 로그 확인

**터미널 1 예상 로그**:
```
[INFO] [mock_bridge_node]: Mock drive.move_to_target updated: enabled=True, success=True, delay=1.0s
[INFO] [mock_bridge_node]: Mock arm.pick_book updated: enabled=True, success=True, delay=3.0s
[INFO] [mock_bridge_node]: Mock arm.place_book updated: enabled=True, success=True, delay=2.0s

[INFO] [mock_rcs_node]: 작업 전송 요청: pickup
[INFO] [mock_rcs_node]: 도서 픽업 작업 전송: book_id=TEST_BOOK_001, storage_id=1
[INFO] [mock_rcs_node]: 작업 Goal이 수락되었습니다

[INFO] [dmc_node]: 도서 픽업 작업 수신
[INFO] [dmc_node]: 메인 상태: IDLE -> PICKING_UP_BOOK
[INFO] [dmc_node]: 서브 상태: NONE -> MOVE_TO_SHELF

[INFO] [mock_bridge_node]: [Mock] move_to_target: pickup_shelf
[INFO] [dmc_node]: 서가 도착 완료

[INFO] [dmc_node]: 서브 상태: MOVE_TO_SHELF -> PICKUP_BOOK
[INFO] [mock_bridge_node]: [Mock] pick_book: TEST_BOOK_001
[INFO] [dmc_node]: 도서 집기 완료

[INFO] [dmc_node]: 서브 상태: PICKUP_BOOK -> MOVE_TO_STORAGE
[INFO] [mock_bridge_node]: [Mock] move_to_target: storage_1
[INFO] [dmc_node]: 보관함 도착 완료

[INFO] [dmc_node]: 서브 상태: MOVE_TO_STORAGE -> PLACE_BOOK
[INFO] [mock_bridge_node]: [Mock] place_book: TEST_BOOK_001
[INFO] [dmc_node]: 도서 놓기 완료

[INFO] [dmc_node]: 도서 픽업 작업 완료
[INFO] [mock_rcs_node]: 작업 결과: success=True
```

---

### 시나리오 4: 실패 시나리오 테스트

**목적**: Mock에서 실패 응답을 반환했을 때 DMC의 처리 확인

#### Step 1: Launch 실행

**터미널 1**:
```bash
source install/setup.bash
ros2 launch javis_dmc_test test_full.launch.py
```

#### Step 2: 실패 Mock 설정

**터미널 2**:
```bash
source install/setup.bash

# move_to_target은 성공
ros2 service call /test/mock_bridge/set_method javis_dmc_test_msgs/srv/SetMockMethod \
  "{interface: 'drive', method: 'move_to_target', enabled: true, success: true, delay: 0.5}"

# pick_book은 실패 (책을 못 집음 시뮬레이션)
ros2 service call /test/mock_bridge/set_method javis_dmc_test_msgs/srv/SetMockMethod \
  "{interface: 'arm', method: 'pick_book', enabled: true, success: false, delay: 2.0}"

# 작업 전송
ros2 service call /test/mock_rcs/send_task javis_dmc_test_msgs/srv/SendTask \
  "{task_type: 'pickup'}"
```

#### Step 3: 로그 확인

**예상 로그**:
```
[INFO] [dmc_node]: 도서 픽업 작업 수신
[INFO] [dmc_node]: 서브 상태: NONE -> MOVE_TO_SHELF
[INFO] [mock_bridge_node]: [Mock] move_to_target: pickup_shelf (성공)
[INFO] [dmc_node]: 서가 도착 완료

[INFO] [dmc_node]: 서브 상태: MOVE_TO_SHELF -> PICKUP_BOOK
[INFO] [mock_bridge_node]: [Mock] pick_book: TEST_BOOK_001 (실패)
[ERROR] [dmc_node]: 도서 집기 실패
[INFO] [dmc_node]: 재시도 또는 작업 중단 처리

[INFO] [mock_rcs_node]: 작업 결과: success=False, message=도서 집기 실패
```

---

## 디버깅 팁

### 1. 로그 레벨 조정

```bash
ros2 run javis_dmc_test mock_bridge_node --ros-args --log-level debug
```

### 2. Topic Echo로 상태 모니터링

```bash
# DMC 상태 확인
ros2 topic echo /dobby1/dmc/state

# 피드백 확인 (Action 실행 중)
ros2 topic echo /dobby1/main/pickup_book/_action/feedback
```

### 3. Action 상태 확인

```bash
# Action 서버 목록
ros2 action list

# Action 정보
ros2 action info /dobby1/drive/move_to_target

# Action 인터페이스
ros2 interface show javis_interfaces/action/MoveToTarget
```

### 4. Service 테스트

```bash
# Service 목록
ros2 service list

# Service 타입 확인
ros2 service type /test/mock_bridge/set_method

# Service 인터페이스 확인
ros2 interface show javis_dmc_test_msgs/srv/SetMockMethod
```

---

## 자주 발생하는 문제

### 문제 1: "Action server not available"

**증상**:
```
Waiting for an action server to become available...
(계속 대기)
```

**원인**: Mock Bridge 또는 DMC 노드가 실행되지 않음

**해결**:
```bash
# 노드 확인
ros2 node list

# Action 서버 확인
ros2 action list

# 노드 재실행
ros2 run javis_dmc_test mock_bridge_node
```

### 문제 2: "Service not available"

**증상**:
```
waiting for service to become available...
(계속 대기)
```

**원인**: Mock Bridge 또는 Mock RCS 노드가 실행되지 않음

**해결**:
```bash
# 서비스 확인
ros2 service list | grep test

# 노드 재실행
ros2 run javis_dmc_test mock_bridge_node
ros2 run javis_dmc_test mock_rcs_node
```

### 문제 3: Mock이 동작하지 않음

**증상**: Action을 호출했는데 Mock 응답이 없음

**원인**: Mock이 비활성화되어 있음

**해결**:
```bash
# Mock 활성화 확인
ros2 service call /test/mock_bridge/set_method javis_dmc_test_msgs/srv/SetMockMethod \
  "{interface: 'drive', method: 'move_to_target', enabled: true, success: true, delay: 0.0}"
```

### 문제 4: 빌드 오류

**증상**:
```
CMake Error: Could not find package javis_dmc_test_msgs
```

**원인**: 메시지 패키지를 먼저 빌드하지 않음

**해결**:
```bash
# 순서대로 빌드
colcon build --packages-select javis_dmc_test_msgs
source install/setup.bash
colcon build --packages-select javis_dmc_test
source install/setup.bash
```

---

## 테스트 체크리스트

### 기본 기능 테스트
- [ ] Mock Bridge 노드 실행 확인
- [ ] Mock RCS 노드 실행 확인
- [ ] `/test/mock_bridge/set_method` 서비스 호출 성공
- [ ] `/test/mock_rcs/send_task` 서비스 호출 성공

### Mock Bridge 테스트
- [ ] `drive.move_to_target` Mock 활성화 및 호출
- [ ] `arm.pick_book` Mock 활성화 및 호출
- [ ] `arm.place_book` Mock 활성화 및 호출
- [ ] delay 시간 설정 동작 확인
- [ ] success=true 응답 확인
- [ ] success=false 응답 확인

### Mock RCS 테스트
- [ ] `pickup` 작업 전송
- [ ] `guiding` 작업 전송
- [ ] `reshelving` 작업 전송
- [ ] `clean_seat` 작업 전송

### 통합 테스트
- [ ] Launch 파일 실행
- [ ] Mock 설정 후 작업 전송
- [ ] DMC 상태 변화 확인
- [ ] 작업 완료 확인

### 실패 시나리오 테스트
- [ ] Mock 실패 응답 설정
- [ ] DMC 실패 처리 확인

---

**테스트 완료 후**:
다음 단계(TEST_GUI 개선 또는 DMC Mock 제거)를 진행할 수 있습니다.
