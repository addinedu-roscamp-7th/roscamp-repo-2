# JAVIS DMC Test - 빠른 시작 가이드

**작성일**: 2025-10-25
**목적**: TEST GUI를 사용한 Mock 테스트 빠른 시작

---

## 🚀 5분 안에 시작하기

### 1. 빌드 (처음 한 번만)

```bash
cd ~/dev_ws/roscamp-repo-2/javis_ros2

# 빌드
colcon build --packages-select javis_dmc_test_msgs javis_dmc_test --symlink-install

# 환경 변수 로드
source install/setup.bash
```

### 2. 전체 실행 (Launch 파일 사용 - 권장)

```bash
# 터미널 1: 전체 환경 실행
ros2 launch javis_dmc_test test_full.launch.py
```

**실행되는 노드**:
- ✅ DMC 노드
- ✅ Mock Bridge 노드
- ✅ Mock RCS 노드
- ✅ **Test GUI** (자동으로 창이 열립니다)

---

## 🎮 TEST GUI 사용 방법

### GUI 화면 구성

```
┌────────────────────────────────────────────────┐
│     JAVIS DMC Simple Test GUI                  │
├────────────────────────────────────────────────┤
│ [작업 전송 (Mock RCS)]                         │
│   [도서 픽업] [길 안내] [반납 정리] [좌석 청소] │
├────────────────────────────────────────────────┤
│ [Mock 제어 (개별 메서드)]                      │
│                                                │
│  DRIVE Interface (4개)                         │
│    Method         |Enabled|Success|Delay| [적용] │
│    move_to_target |  ☑    |  ☑    | 1.0 | [적용] │
│    stop           |  ☐    |  ☑    | 0.0 | [적용] │
│    resume         |  ☐    |  ☑    | 0.0 | [적용] │
│    start_patrol   |  ☐    |  ☑    | 1.0 | [적용] │
│                                                │
│  ARM Interface (3개)                           │
│    Method         |Enabled|Success|Delay| [적용] │
│    pick_book      |  ☑    |  ☑    | 3.0 | [적용] │
│    place_book     |  ☑    |  ☑    | 2.0 | [적용] │
│    change_pose    |  ☐    |  ☑    | 0.5 | [적용] │
│                                                │
│  AI Interface (2개)                            │
│    Method         |Enabled|Success|Delay| [적용] │
│    detect_book    |  ☐    |  ☑    | 0.5 | [적용] │
│    change_tracking|  ☐    |  ☑    | 0.0 | [적용] │
├────────────────────────────────────────────────┤
│ [이벤트 로그]                                   │
│ [15:30:45] [요청] drive.move_to_target 적용    │
│ [15:30:46] [Mock] drive.move_to_target: 설정 완료│
│ [15:30:47] [요청] pickup 작업 전송 요청        │
│ [15:30:48] [Task] pickup: pickup task sent    │
└────────────────────────────────────────────────┘
```

---

## 📝 시나리오 예제

### 시나리오 1: 도서 픽업 성공 테스트

#### Step 1: Mock 설정

GUI에서 다음 체크박스를 체크하고 **[적용]** 버튼 클릭:

**DRIVE Interface**:
- `move_to_target`: Enabled ☑, Success ☑, Delay: `1.0`

**ARM Interface**:
- `pick_book`: Enabled ☑, Success ☑, Delay: `3.0`
- `place_book`: Enabled ☑, Success ☑, Delay: `2.0`

#### Step 2: 작업 전송

GUI 상단의 **[도서 픽업]** 버튼 클릭

#### Step 3: 결과 확인

**예상 로그 (GUI 하단)**:
```
[15:30:00] [요청] pickup 작업 전송 요청
[15:30:01] [Task] pickup: pickup task sent
[15:30:02] [Mock] drive.move_to_target 실행 (1초 지연)
[15:30:03] [DMC] 서가 도착 완료
[15:30:03] [Mock] arm.pick_book 실행 (3초 지연)
[15:30:06] [DMC] 도서 집기 완료
[15:30:06] [Mock] drive.move_to_target 실행
[15:30:07] [DMC] 보관함 도착
[15:30:07] [Mock] arm.place_book 실행 (2초 지연)
[15:30:09] [DMC] 작업 완료
```

---

### 시나리오 2: 도서 집기 실패 테스트

#### Step 1: Mock 설정 (실패)

**DRIVE Interface**:
- `move_to_target`: Enabled ☑, Success ☑, Delay: `0.5`

**ARM Interface**:
- `pick_book`: Enabled ☑, Success ☐ **(체크 해제!)**, Delay: `2.0`

#### Step 2: 작업 전송

**[도서 픽업]** 버튼 클릭

#### Step 3: 결과 확인

**예상 로그**:
```
[15:31:00] [요청] pickup 작업 전송 요청
[15:31:01] [Mock] drive.move_to_target 실행 (성공)
[15:31:02] [Mock] arm.pick_book 실행 (실패!)
[15:31:04] [DMC] 도서 집기 실패 - 작업 중단
```

---

## 🎯 GUI 사용 팁

### Mock 설정

1. **Enabled**: Mock을 활성화할지 선택
   - ☑ 체크: Mock 응답 반환
   - ☐ 해제: 실제 하드웨어로 전달 (없으면 실패)

2. **Success**: Mock 응답의 성공/실패
   - ☑ 체크: 성공 응답
   - ☐ 해제: 실패 응답

3. **Delay**: Mock 응답 지연 시간 (초)
   - 실제 하드웨어 동작 시간 시뮬레이션
   - 예: `3.0` = 3초 후 응답

4. **[적용]** 버튼: 설정을 Mock Bridge에 전송

### 작업 전송 버튼

- **도서 픽업**: 책 서가 → 집기 → 보관함 배치
- **길 안내**: 사용자를 목적지까지 안내
- **반납 정리**: 반납대의 책들을 서가로 재배치
- **좌석 청소**: 좌석의 쓰레기 수거

---

## 🔧 문제 해결

### GUI가 열리지 않음

**원인**: X11 디스플레이 문제

**해결**:
```bash
# WSL인 경우 X server 설치 필요 (VcXsrv, X410 등)
export DISPLAY=:0
```

### "Service not available" 오류

**원인**: Mock Bridge 또는 Mock RCS 노드가 실행 중이지 않음

**해결**:
```bash
# 노드 확인
ros2 node list

# Mock Bridge가 없으면
ros2 run javis_dmc_test mock_bridge_node
```

### Mock이 동작하지 않음

**원인**: Enabled 체크 안 됨 또는 적용 안 됨

**해결**:
1. Enabled 체크박스 확인
2. **[적용]** 버튼 클릭
3. 로그에서 `[Mock] ... 설정 완료` 확인

---

## 🎓 다음 단계

### 터미널로 고급 테스트

```bash
# Mock 설정
ros2 service call /test/mock_bridge/set_method javis_dmc_test_msgs/srv/SetMockMethod \
  "{interface: 'drive', method: 'move_to_target', enabled: true, success: true, delay: 1.0}"

# 작업 전송
ros2 service call /test/mock_rcs/send_task javis_dmc_test_msgs/srv/SendTask \
  "{task_type: 'pickup'}"
```

### 개별 노드 실행

```bash
# 터미널 1: Mock Bridge
ros2 run javis_dmc_test mock_bridge_node

# 터미널 2: Mock RCS
ros2 run javis_dmc_test mock_rcs_node

# 터미널 3: Test GUI
ros2 run javis_dmc_test test_gui_simple

# 터미널 4: DMC (테스트 대상)
ros2 run javis_dmc dmc_node
```

---

## 📚 참고 문서

- **상세 테스트**: `TESTING.md` - 전체 테스트 시나리오
- **구현 내용**: `IMPLEMENTATION.md` - 구현 상세
- **설계 문서**: `/javis_dmc/docs/DevelopmentPlan/TEST_GUI_REFACTOR_V2.md`

---

**즐거운 테스트 되세요!** 🚀
