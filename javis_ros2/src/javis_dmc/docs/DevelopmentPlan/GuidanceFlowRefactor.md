# DMC 길안내 플로우 개선 구현 계획

## 📋 요약
기존의 DestinationSession (GUI 목적지 선택 대기) 방식을 제거하고,
사전에 확정된 목적지 좌표로 바로 길안내를 시작하는 방식으로 변경합니다.

---

## 🎯 사용자 결정사항 (v4.0 업데이트)
1. **RequestGuidance 서비스를 DMC가 제공** → GUI/VRC가 직접 DMC 호출 가능
2. **목적지는 사용자가 입력한 것 사용** → GUI/VRC에서 확정된 좌표만 사용
3. **사용자 주도 작업은 RCS를 거침** → DMC가 CreateUserTask 호출, RCS가 GuidePerson Action 호출
4. **QueryLocationInfo는 GUI만 사용** → VRC는 LLM Service에서 좌표 직접 획득
5. **QueryLocationInfo가 WAITING_DEST_INPUT 진입점** → 목적지 입력 의사 표현 = 상태 전환

---

## 🔄 새로운 플로우 (v4.0)

### Scenario 1: GUI 터치
```
1. GUI: 초기 화면 표시 ("길안내 시 터치해주세요")
2. 사용자가 화면 터치
3. GUI → DMC: QueryLocationInfo("") - 목적지 입력 의사 표현 + 목록 요청
4. DMC: State IDLE/ROAMING → WAITING_DEST_INPUT (60초 타이머)
5. DMC → GUI: {목적지 목록: [화장실, 카페, 출입구, ...]}
6. GUI: 지도 화면 + 목적지 버튼들 표시
7. 사용자가 "화장실" 버튼 터치
8. GUI → DMC: RequestGuidance(dest_name="화장실", dest_pose={...}, source="gui")
9. DMC: 배터리 체크 (≥40%)
10. DMC: 60초 타이머 취소
11. DMC → RCS: CreateUserTask(user_initiated=True)
12. RCS → DMC: GuidePerson action goal (user_initiated=True)
13. DMC: State WAITING_DEST_INPUT → GUIDING
14. DMC: 피안내자 스캔 → 추종 이동
```

### Scenario 2: VRC 음성
```
1. VRC: "도비야" → wakeWord 감지
2. DMC: State IDLE/ROAMING → LISTENING (20초 타이머)
3. VRC → Voice API Service: 음성 스트리밍 ("화장실 가고 싶어")
4. Voice API Service (LLM): 
   - STT 변환
   - Intent 분석: navigation, target="화장실"
   - 목적지 좌표 획득: {x: 10.5, y: -5.0, theta: 1.57}
5. VRC → DMC: RequestGuidance(dest_name="화장실", dest_pose={...}, source="voice")
6. DMC: 배터리 체크 (≥40%)
7. DMC: 20초 타이머 취소
8. DMC → RCS: CreateUserTask(user_initiated=True)
9. RCS → DMC: GuidePerson action goal (user_initiated=True)
10. DMC: State LISTENING → GUIDING
11. DMC: 피안내자 스캔 → 추종 이동
```

### Scenario 3: RCS 작업 할당 (기존 방식 유지)
```
1. RCS → DMC: GuidePerson action goal (dest_location 포함)
2. DMC: State IDLE/ROAMING → GUIDING
3. DMC: _run_guiding_sequence 실행
4. DMC: 피안내자 스캔 → 추종 이동
```

---

## 🛠️ DMC 구현 수정사항

### 1. library_locations.yaml 로딩
```python
# dmc_node.py __init__

def _load_library_locations(self) -> Dict[str, Dict]:
    """도서관 위치 정보 YAML 로드"""
    try:
        package_share = get_package_share_directory('javis_dmc')
        yaml_path = os.path.join(package_share, 'config', 'library_locations.yaml')
        
        with open(yaml_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        
        locations = data.get('locations', [])
        # location_id → location 매핑
        location_map = {}
        for loc in locations:
            loc_id = loc.get('id', '')
            if loc_id:
                location_map[loc_id] = loc
        
        return location_map
    except Exception as e:
        self.get_logger().error(f'library_locations.yaml 로드 실패: {e}')
        return {}

# __init__에 추가
self._library_locations = self._load_library_locations()
```

### 2. QueryLocationInfo 서비스 (v4.0: GUI 전용, WAITING_DEST_INPUT 진입점)
```python
# dmc_node.py

from javis_interfaces.srv import QueryLocationInfo

# __init__에 추가
self._query_location_srv = self.create_service(
    QueryLocationInfo,
    f'{self.robot_namespace}/admin/query_location_info',
    self._handle_query_location
)

def _handle_query_location(self, request, response):
    """위치 정보 조회 서비스 핸들러 (GUI 전용)
    
    QueryLocationInfo 호출 = 사용자의 목적지 입력 의사 표현
    → WAITING_DEST_INPUT 상태로 전환 (60초 타이머)
    """
    
    # 1. 상태 전환: IDLE/ROAMING → WAITING_DEST_INPUT
    if self.main_state in [MainState.IDLE, MainState.ROAMING]:
        self._set_main_state(MainState.WAITING_DEST_INPUT)
        self._start_dest_input_timer(60.0)  # 60초 타이머 시작
        self.get_logger().info('QueryLocationInfo 호출: WAITING_DEST_INPUT 진입 (60초)')
    
    # 2. 목록 요청 처리 (빈 문자열 = 전체 목록)
    if not request.location_name or request.location_name.strip() == "":
        response.found = True
        response.locations = []
        for loc_id, loc in self._library_locations.items():
            location_info = {
                'name': loc.get('name', ''),
                'pose': {
                    'x': loc['pose']['x'],
                    'y': loc['pose']['y'],
                    'theta': loc['pose']['theta']
                },
                'description': loc.get('description', ''),
                'aliases': loc.get('aliases', [])
            }
            response.locations.append(location_info)
        response.message = f'{len(response.locations)}개의 목적지를 찾았습니다'
        return response
    
    # 3. 특정 위치 조회
    location_name = request.location_name.strip().lower()
    
    for loc_id, loc in self._library_locations.items():
        # 이름 매칭
        if loc.get('name', '').lower() == location_name:
            response.found = True
            response.location_name = loc.get('name', '')
            response.location_id = loc_id
            response.pose.x = loc['pose']['x']
            response.pose.y = loc['pose']['y']
            response.pose.theta = loc['pose']['theta']
            response.description = loc.get('description', '')
            response.aliases = loc.get('aliases', [])
            response.message = '위치를 찾았습니다'
            return response
        
        # 별칭 매칭
        aliases = loc.get('aliases', [])
        if any(alias.lower() == location_name for alias in aliases):
            response.found = True
            response.location_name = loc.get('name', '')
            response.location_id = loc_id
            response.pose.x = loc['pose']['x']
            response.pose.y = loc['pose']['y']
            response.pose.theta = loc['pose']['theta']
            response.description = loc.get('description', '')
            response.aliases = aliases
            response.message = '위치를 찾았습니다'
            return response
    
    # 찾지 못함
    response.found = False
    response.message = f'"{request.location_name}" 위치를 찾을 수 없습니다'
    return response

def _start_dest_input_timer(self, timeout_sec: float):
    """목적지 입력 대기 타이머 시작"""
    if hasattr(self, '_dest_input_timer') and self._dest_input_timer:
        self._dest_input_timer.cancel()
    
    self._dest_input_timer = self.create_timer(
        timeout_sec,
        self._on_dest_input_timeout
    )

def _on_dest_input_timeout(self):
    """목적지 입력 타임아웃 핸들러"""
    if self.main_state == MainState.WAITING_DEST_INPUT:
        self.get_logger().warn('목적지 입력 대기 시간 초과 (60초)')
        self._set_main_state(self._previous_state)  # 이전 상태로 복귀
    
    if hasattr(self, '_dest_input_timer') and self._dest_input_timer:
        self._dest_input_timer.cancel()
        self._dest_input_timer = None
```

### 3. RequestGuidance 서비스 (v4.0: WAITING_DEST_INPUT 상태에서 호출됨)
```python
# dmc_node.py

from javis_interfaces.srv import RequestGuidance, CreateUserTask

# __init__에 추가
self._request_guidance_srv = self.create_service(
    RequestGuidance,
    f'{self.robot_namespace}/admin/request_guidance',
    self._handle_request_guidance
)

# CreateUserTask 클라이언트 추가 (DMC → RCS)
self._create_user_task_client = self.create_client(
    CreateUserTask,
    '/rcs/create_user_task'
)

def _handle_request_guidance(self, request, response):
    """길안내 요청 서비스 핸들러 (v4.0)
    
    이미 WAITING_DEST_INPUT 상태에서 호출됨 (QueryLocationInfo가 진입점)
    """
    
    # 1. 상태 확인 (WAITING_DEST_INPUT 또는 LISTENING이어야 함)
    if self.main_state not in [MainState.WAITING_DEST_INPUT, MainState.LISTENING]:
        response.success = False
        response.message = f'현재 상태({self.main_state})에서는 길안내를 시작할 수 없습니다'
        return response
    
    # 2. 배터리 확인
    if self.battery_percentage < 40.0:
        response.success = False
        response.message = '배터리가 부족합니다'
        return response
    
    # 3. 타이머 취소
    if self.main_state == MainState.WAITING_DEST_INPUT:
        self._cancel_dest_input_timer()
    elif self.main_state == MainState.LISTENING:
        self._cancel_listening_timer()
    
    # 4. RCS에 사용자 작업 생성 요청
    try:
        if not self._create_user_task_client.wait_for_service(timeout_sec=2.0):
            response.success = False
            response.message = 'RCS 서비스를 찾을 수 없습니다'
            return response
        
        # CreateUserTask 호출
        task_request = CreateUserTask.Request()
        task_request.task_type = 'guide_person'
        task_request.destination_name = request.destination_name
        task_request.dest_pose = request.dest_pose
        task_request.user_initiated = True
        
        task_future = self._create_user_task_client.call_async(task_request)
        rclpy.spin_until_future_complete(self, task_future, timeout_sec=5.0)
        
        task_response = task_future.result()
        
        if not task_response.success:
            response.success = False
            response.message = f'작업 생성 실패: {task_response.message}'
            return response
        
        # 5. RCS가 GuidePerson Action 호출 → DMC가 Action Goal 수신
        # → _execute_guide_person()에서 GUIDING으로 전환됨
        
        response.success = True
        response.task_id = task_response.task_id
        response.message = f'{request.destination_name}로 길안내를 시작합니다'
        
        self.get_logger().info(
            f'길안내 요청 성공: {request.destination_name} '
            f'(source: {request.request_source}, task_id: {response.task_id})'
        )
        
    except Exception as e:
        response.success = False
        response.message = f'길안내 작업 시작 실패: {e}'
        self.get_logger().error(f'RequestGuidance 처리 중 오류: {e}')
    
    return response

def _cancel_dest_input_timer(self):
    """목적지 입력 타이머 취소"""
    if hasattr(self, '_dest_input_timer') and self._dest_input_timer:
        self._dest_input_timer.cancel()
        self._dest_input_timer = None
        self.get_logger().info('목적지 입력 타이머 취소')
```

### 4. _run_guiding_sequence 수정
```python
# dmc_node.py Line 994-1074

def _run_guiding_sequence(
    self,
    goal: Optional[GuidePerson.Goal],
    set_sub_state: Callable[[SubState], None],
    publish_feedback: Callable[[float], None],
) -> GuidingOutcome:
    '''길 안내 시나리오를 세부 구현한다.'''
    start_time = time.monotonic()

    if goal is None or not hasattr(goal, 'dest_location'):
        return GuidingOutcome(False, '목적지 정보가 전달되지 않았습니다.')

    dest_location = goal.dest_location
    requested_name = str(getattr(goal, 'destination_name', '')).strip() or 'custom_destination'

    # ===== 수정: destination_session 제거 =====
    # 이미 GUI/VRC에서 목적지를 확정했으므로 선택 단계 스킵
    # set_sub_state(SubState.SELECT_DEST)  # ← 제거
    # self.destination_session.begin_selection()  # ← 제거
    # ... destination_session 관련 코드 모두 제거
    
    # 바로 피안내자 스캔으로 진행
    dest_pose = self._pose2d_to_pose(dest_location)
    total_distance = self._estimate_distance(dest_location)

    self.get_logger().info(f'길안내 시작: {requested_name} ({dest_location.x}, {dest_location.y})')
    
    set_sub_state(SubState.SCAN_USER)  # ← 바로 스캔으로 시작
    publish_feedback(0.2)

    # 피안내자 등록 로직 (기존 유지)
    person_detected = False
    if not self.ai.is_initialized():
        self.get_logger().warn('AI 인터페이스가 초기화되지 않아 피안내자 등록 절차를 생략합니다.')
        person_detected = True
    else:
        try:
            self._reset_tracking_detection()
            if not self.ai.change_tracking_mode('registration'):
                self.get_logger().warn('등록 모드 전환에 실패했습니다.')
            detection_success, tracking_snapshot = self._wait_for_person_detection()
            if detection_success:
                person_detected = True
                if not self.ai.change_tracking_mode('tracking'):
                    self.get_logger().warn('추적 모드 전환에 실패했습니다.')
            else:
                person_detected = False
                self.ai.change_tracking_mode('idle')
        except Exception as exc:
            self.get_logger().error(f'피안내자 등록 중 예외가 발생했습니다: {exc}')
            person_detected = False

    publish_feedback(0.7 if person_detected else 0.45)
    if not person_detected:
        duration = time.monotonic() - start_time
        if self.ai.is_initialized():
            self.ai.change_tracking_mode('idle')
        return GuidingOutcome(False, '피안내자를 찾지 못했습니다.', total_distance, duration, False)

    # 목적지로 이동 (기존 유지)
    set_sub_state(SubState.GUIDING_TO_DEST)
    navigation_success = self._navigate_with_follow_mode(dest_pose)
    publish_feedback(1.0 if navigation_success else 0.85)

    duration = time.monotonic() - start_time
    message = '길 안내를 완료했습니다.' if navigation_success else '길 안내 주행에 실패했습니다.'

    if self.ai.is_initialized():
        self.ai.change_timing_mode('idle')

    return GuidingOutcome(navigation_success, message, total_distance, duration, person_detected)
```

### 5. _wait_for_destination_selection 제거
```python
# dmc_node.py Line 1180-1194

# 이 메서드는 더 이상 필요 없음 → 제거 또는 deprecated 표시
```

---

## 📄 문서 업데이트

### DevelopmentPlan.md 수정
- **4.5.2 목적지 선택 타이머 시퀀스** 섹션 수정/제거
- **5.4 Sub State 정의** - SELECT_DEST 제거 또는 deprecated 표시
- **5.5 Main State Diagram** - GUIDING 내부 플로우 수정

### GuidingScenario.md 수정
- **섹션 3** (목적지 입력 및 피안내자 인식 준비) 전체 수정
  - 60초 타이머 제거
  - GUI 지도 승인 요청 제거
  - 바로 피안내자 스캔으로 진행

---

## 🔍 VRC 연동 방식 결정 (v4.0)

### 방식: VRC는 LLM Service에서 좌표 직접 획득
```python
# VRC 코드 (v4.0)
# 1. 음성 스트리밍 → Voice API Service
# 2. LLM이 Intent 분석 + 좌표 획득
# 3. VRC가 좌표와 함께 RequestGuidance 호출

# VRC 코드 예시
def handle_voice_command(self, destination_name, dest_pose):
    """LLM으로부터 받은 목적지 정보로 길안내 요청"""
    
    guidance_request = RequestGuidance.Request()
    guidance_request.destination_name = destination_name
    guidance_request.dest_pose = dest_pose
    guidance_request.request_source = "voice"
    
    # DMC에 길안내 요청
    guidance_response = self.request_guidance_client.call(guidance_request)
    
    if guidance_response.success:
        self.get_logger().info(f'길안내 시작: {guidance_response.task_id}')
    else:
        self.get_logger().error(f'길안내 실패: {guidance_response.message}')
```

**핵심 차이점:**
- **GUI**: QueryLocationInfo로 목록 조회 → 선택 → RequestGuidance
- **VRC**: LLM Service에서 좌표 직접 획득 → RequestGuidance (QueryLocationInfo 사용 안 함)

**장점:**
- VRC는 QueryLocationInfo 불필요
- LLM이 자연어로 목적지 좌표 직접 제공
- GUI와 VRC 경로 명확히 분리

---

## ✅ 구현 체크리스트

- [ ] `_load_library_locations()` 구현
- [ ] `QueryLocationInfo` 서비스 추가
- [ ] `RequestGuidance` 서비스 추가
- [ ] `_run_guiding_sequence` 수정 (destination_session 제거)
- [ ] `_wait_for_destination_selection` 제거
- [ ] VRC 연동 구현
- [ ] DevelopmentPlan.md 업데이트
- [ ] GuidingScenario.md 업데이트
- [ ] 테스트 시나리오 작성 (Test GUI 탭 3 활용)

---

## 🧪 테스트 계획

### 단위 테스트
1. `QueryLocationInfo` 서비스
   - 존재하는 위치: "화장실" → found=true
   - 별칭: "restroom" → found=true
   - 없는 위치: "xxx" → found=false

2. `RequestGuidance` 서비스
   - 정상 케이스: IDLE 상태 + 배터리 충분
   - 거부 케이스: 다른 작업 진행 중
   - 거부 케이스: 배터리 부족

### 통합 테스트
1. GUI 터치 → 길안내 완료
2. VRC 음성 → 길안내 완료
3. RCS 작업 할당 → 길안내 완료
