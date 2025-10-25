# TEST GUI 개선 계획서

**작성일**: 2025-03-15
**대상**: `test_gui/test_gui_node.py`, `test_gui/test_gui_widget.py`
**목적**: 인수인계 전 TEST GUI의 문제점 파악 및 개선 방안 제시

---

## 1. 현재 문제점 분석

### 1.1 로봇 상태 표시 부분 레이아웃 문제

**문제**:
- `test_gui_widget.py:309` - 로봇 상태 표시 라벨의 `width=18`로 고정되어 있어 긴 텍스트가 잘림
- 특히 서브 상태 이름이 길 때 (예: `MOVE_TO_STORAGE`) UI에서 잘려서 표시됨

**원인**:
```python
# test_gui_widget.py:305-312
value_label = tk.Label(
    frame,
    textvariable=var,
    bg=self._LABEL_BG,
    fg=self._TEXT_FG,
    font=self._VALUE_FONT,
    anchor='w',
    padx=12,
    pady=4,
    width=18,  # ← 고정 폭으로 인해 텍스트 잘림
)
```

**해결 방안**:
- `width` 파라미터를 제거하고 `wraplength` 사용
- 또는 동적 폭 조정을 위해 Grid의 `weight` 설정 개선

---

### 1.2 "현재 작업 성공 처리" 버튼 미작동

**문제**:
- "현재 작업 성공 처리" 버튼이 클릭해도 아무 반응이 없음
- `test_gui_widget.py:354` 버튼 정의는 되어 있으나, 실제 호출되는 서비스가 제대로 동작하지 않음

**원인**:
```python
# test_gui_widget.py:354
('현재 작업 성공 처리', lambda: self.node.request_force_task_result(True)),
```

`request_force_task_result()` 메서드는 호출되지만:
1. **DMC 노드에 활성 작업이 없을 때**: `_active_goal_handle is None`이면 서비스가 거부됨
2. **결과 메시지 생성 실패**: `_build_result()` 메서드가 필수 kwargs를 받지 못하면 ValueError 발생

**DMC 코드 분석** (`dmc_node.py:468-508`):
```python
def _on_force_task_result(self, request, response):
    if self._active_goal_handle is None or self._active_main_state is None:
        response.applied = False
        response.detail = '활성 작업이 없어 처리할 수 없습니다.'
        return response

    # 문제: _build_result에 필수 kwargs가 없으면 실패
    result_msg = self._build_result(self._active_main_state, success, message)
```

**해결 방안**:
1. **버튼 기능 재정의**: "작업 건너뛰기/강제 완료" 용도로 명확화
2. **DMC 서비스 개선**: `force_task_result` 서비스가 kwargs 없이도 작동하도록 기본값 제공
3. **GUI 피드백 개선**: 서비스 호출 결과를 사용자에게 명확히 표시

---

### 1.3 "현재 작업 실패 처리" 버튼 필요성

**문제**:
- 사용자가 "실패 처리" 기능의 용도를 모름
- "성공 처리"만으로도 작업 건너뛰기가 가능

**해결 방안**:
- **옵션 A**: 삭제하고 "성공 처리" 버튼만 "작업 건너뛰기"로 변경
- **옵션 B**: 두 버튼 모두 유지하되 명확한 라벨링
  - "작업 성공으로 완료" / "작업 실패로 중단"

---

### 1.4 수동 상태 설정 로직 문제

**핵심 문제**:
수동 상태 설정(`set_manual_state`)은 **상태만 변경**하고, **해당 상태에 필요한 통신/인터페이스 초기화를 수행하지 않음**

**사용자 기대 동작**:
```
IDLE → PICKING_UP_BOOK로 수동 전환 시
→ RCS에서 작업을 할당받은 것처럼 Executor 초기화, 서브 상태 전환, 인터페이스 활성화
```

**실제 동작**:
```python
# dmc_node.py:510-552
def _on_set_manual_state(self, request, response):
    # 단순히 상태만 변경
    self.state_machine.set_main_state(target_main)
    self.state_machine.set_sub_state(target_sub)

    self._publish_state_immediately()
    # ← Executor 시작, 인터페이스 통신, 하위 컨트롤러 명령 전송 등이 없음!
```

**정상적인 작업 시작 흐름** (`_execute_task` 메서드):
```python
# dmc_node.py:1099-1118
def _execute_task(self, goal_handle, main_state: MainState):
    # 1. 상태 머신 전환
    self.state_machine.start_task(main_state)

    # 2. Goal handle 등록
    self._active_goal_handle = goal_handle
    self._active_main_state = main_state

    # 3. Executor 시작 ← 핵심!
    self.start_task(main_state, goal)

    # 4. Executor가 하위 인터페이스 호출
    # - drive.move_to_target()
    # - arm.pick_book()
    # - ai.change_tracking_mode()
```

**해결 방안**:

#### 옵션 A: 수동 상태 설정 기능 제거
- 디버깅용 기능이므로 제거하고, 대신 **Mock RCS 액션 클라이언트** 구현
- Mock RCS가 실제 작업 Goal을 보내면 정상 흐름으로 처리됨

#### 옵션 B: 수동 상태 설정 시 Executor도 강제 시작
```python
def _on_set_manual_state(self, request, response):
    # 기존: 상태만 변경
    self.state_machine.set_main_state(target_main)

    # 추가: TASK 상태이면 Executor 시작
    if target_main in self.executors:
        # 더미 Goal 생성
        dummy_goal = self._create_dummy_goal(target_main)
        self.start_task(target_main, dummy_goal)
```

**문제점**:
- 더미 Goal 생성이 복잡 (각 작업 타입별로 필수 파라미터가 다름)
- RCS 작업 할당 메커니즘을 우회하므로 추적/로깅 불가

#### 옵션 C (권장): TEST GUI에서 Mock Action Client 제공
```python
# test_gui_node.py에 추가
def send_mock_task(self, task_type: str):
    '''Mock RCS처럼 DMC에 작업 Goal을 전송'''
    if task_type == 'pickup_book':
        client = self.create_client(PickupBook, self._ns('main/pickup_book'))
        goal = PickupBook.Goal()
        goal.book_id = 'TEST_BOOK_001'
        goal.storage_id = 1
        # ... 필수 파라미터 설정
        client.send_goal_async(goal)
```

---

### 1.5 인터페이스 Mock/Real 분기 문제

**문제**:
- `use_mock_interfaces` 파라미터 변경 시 **노드 재시작 필요**
- GUI의 Real/Mock 라디오 버튼이 즉시 적용되지 않아 혼란 유발

**현재 구현** (`test_gui_widget.py:389-405`):
```python
ttk.Radiobutton(
    mode_frame,
    text='Real',
    value='Real',
    variable=self.interface_mode_var,
    command=lambda: self._apply_interface_mode('Real'),
).grid(row=0, column=2, padx=4, pady=4, sticky='w')
```

**_apply_interface_mode 분석** (`test_gui_widget.py:708-718`):
```python
def _apply_interface_mode(self, mode_label: str):
    target_label = 'Mock' if mode_label.lower() == 'mock' else 'Real'
    if self.interface_mode_var.get() == target_label:
        changed = self.node.set_interface_mode(target_label.lower() == 'mock')
    else:
        self.interface_mode_var.set(target_label)
        changed = self.node.set_interface_mode(target_label.lower() == 'mock')
```

**DMC 파라미터 콜백** (`dmc_node.py:1680-1704`):
```python
def _on_parameters(self, params):
    for param in params:
        if param.name == 'use_mock_interfaces':
            self.use_mock_interfaces = bool(param.value)
            # 메시지: "현재 세션에서는 즉시 적용되지 않으므로 노드 재시작이 필요합니다."
```

**해결 방안**:

#### 옵션 A: 라디오 버튼 제거, 인터페이스별 Mock 모드만 사용
- `use_mock_interfaces` 전역 스위치 제거
- 각 인터페이스(drive, arm, ai, gui)별로 개별 제어
- 재시작 후 적용 안내 문구 추가

#### 옵션 B (권장): "설정 후 재시작" 버튼 추가
```python
# GUI 레이아웃 변경
[Real 라디오] [Mock 라디오] [설정 저장] [노드 재시작]
```

- "설정 저장": 파라미터만 변경
- "노드 재시작": DMC 노드를 종료하고 새 파라미터로 재실행
  - ROS2에서는 노드 재시작이 복잡하므로, 사용자에게 수동 재시작 안내

#### 옵션 C: 런타임 인터페이스 교체 (복잡, 비권장)
- DMC 노드 내부에서 `self.drive`, `self.arm` 등을 런타임에 교체
- 진행 중인 작업이 있으면 실패할 수 있어 위험

---

### 1.6 "시나리오 실행" 기능 미구현

**문제**:
- 시나리오 YAML 파일은 로드되지만 (`config/test_gui_scenarios.yaml`)
- 실제 시나리오 단계 실행 로직이 불완전

**현재 구현** (`test_gui_widget.py:720-758`):
```python
def _run_selected_scenario(self) -> None:
    # 시나리오 로드 및 준비는 정상
    for idx, step in enumerate(steps, start=1):
        success, detail = self._execute_scenario_step(step, interactive)
        # ...
```

**_execute_scenario_step 분석** (`test_gui_widget.py:760-814`):
- `call_service`: 정상 구현
- `wait_for_state`: 정상 구현
- `sleep`: 정상 구현
- `publish_mock`: **미구현** (WARN 로그만 출력)
- `log`: 정상 구현

**해결 방안**:
1. `publish_mock` 액션 구현
   - Mock 인터페이스에 특정 응답 설정
   - 또는 Mock RCS에서 작업 Goal 전송
2. 시나리오 YAML에 실제 테스트 케이스 추가
3. 진행 현황 패널 연동 확인

---

### 1.7 "Mock 응답 제어" 기능 미작동

**문제**:
- Mock 응답 설정 UI는 있지만 실제 적용 확인 어려움
- 인터페이스가 Mock 모드가 아니면 서비스 호출 실패

**현재 구현** (`test_gui_widget.py:848-861`):
```python
def _apply_mock_response(self, success_flag: bool):
    interface = self.mock_interface_var.get().strip()
    method = self.mock_method_var.get().strip()
    error_code = self.mock_error_var.get().strip()
    scenario = method if not error_code else f'{method}:{error_code}'
    result = self.node.set_mock_response(interface, scenario, success_flag)
```

**DMC 서비스** (`dmc_node.py:581-609`):
```python
def _on_set_mock_response(self, request, response):
    interface_key = request.interface_name.strip().lower()
    if interface_key not in self._mock_interfaces:
        response.success = False
        response.message = f'{request.interface_name} 인터페이스는 현재 Mock 모드가 아닙니다.'
        return response
```

**해결 방안**:
1. Mock 응답 설정 전 인터페이스 Mock 모드 확인
2. 실패 시 사용자에게 "해당 인터페이스를 Mock 모드로 전환하세요" 안내
3. Mock 메서드 목록을 `config/mock_method_profiles.yaml`에서 정확히 로드

---

### 1.8 "시나리오 진행 현황" 패널 반응 없음

**문제**:
- 시나리오 선택 및 실행 시 Treeview가 업데이트되지만, 시각적 피드백 부족
- `_prepare_scenario_tracking()` 호출은 정상이나, 실제 작업 진행과 동기화 안 됨

**원인**:
- 시나리오 단계와 DMC 상태 전환이 연동되지 않음
- 시나리오는 서비스 호출 중심이지만, 실제 작업 진행은 Action 기반

**해결 방안**:
1. 시나리오에 Action Goal 전송 단계 추가
2. Action Feedback을 구독하여 Treeview 업데이트
3. 또는 시나리오를 "서비스 테스트"와 "작업 워크플로우 테스트"로 분리

---

### 1.9 상태 다이어그램 초기 로드 실패

**문제**:
- GUI 시작 시 "상태 정보 요청 실패" 로그 출력
- 이후 상태 다이어그램이 빈 화면으로 표시됨

**원인** (`test_gui_widget.py:215-222`):
```python
self.state_graph_data = self.node.describe_state_machine() or {}
# describe_state_machine() 호출 시 DMC 노드가 아직 준비 안 됨
```

**해결 방안**:
1. GUI 초기화 시 재시도 로직 추가 (1초 대기 후 재시도)
2. 또는 "상태 다이어그램 새로고침" 버튼 제공
3. DMC 노드 준비 완료 확인 후 GUI 초기화

---

### 1.10 상태 표시 반응 느림

**문제**:
- 상태 업데이트 메시지는 수신되지만 GUI 반영이 지연됨

**원인**:
- 이벤트 큐 폴링 주기가 150ms (`test_gui_widget.py:573`)
- ROS subscriber 콜백과 GUI 업데이트 사이 동기화 지연

**해결 방안**:
1. 폴링 주기를 50ms로 단축
2. 또는 ThreadPoolExecutor로 이벤트 처리 병렬화
3. 상태 변경 시 즉시 GUI 업데이트 (polling 우회)

---

## 2. 우선순위별 개선 계획

### P0 (Critical - 인수인계 전 필수)
1. ✅ **로봇 상태 표시 레이아웃 수정** → `width` 제거, wraplength 적용
2. ✅ **"현재 작업 성공 처리" 버튼 기능 명확화** → "작업 건너뛰기"로 변경
3. ✅ **수동 상태 설정 로직 개선** → Mock Action Client 추가 (권장)
4. ✅ **인터페이스 Mock/Real 분기 안내 개선** → "재시작 필요" 메시지 강조

### P1 (High - 1주일 내)
5. 상태 다이어그램 초기 로드 재시도 로직
6. 시나리오 실행 기능 완성 (publish_mock 구현)
7. Mock 응답 제어 가이드 개선

### P2 (Medium - 추후)
8. 상태 표시 반응 속도 개선
9. 시나리오 진행 현황과 Action Feedback 연동

---

## 3. 리팩토링 제안

### 3.1 TEST GUI 코드 구조 개선
현재 `TestGuiApp` 클래스가 1000줄 가까이 되어 유지보수 어려움

**제안**:
```
test_gui/
├── test_gui_node.py         # ROS 노드 (변경 없음)
├── test_gui_widget.py        # 메인 GUI (간소화)
├── panels/
│   ├── status_panel.py       # 로봇 상태 표시 패널
│   ├── control_panel.py      # 제어 버튼 패널
│   ├── scenario_panel.py     # 시나리오 실행 패널
│   ├── mock_control_panel.py # Mock 응답 제어
│   └── state_diagram.py      # 상태 다이어그램 (기존 StateGraphCanvas)
└── utils/
    ├── colors.py             # 색상 상수
    └── formatting.py         # 텍스트 포맷 유틸
```

### 3.2 DMC 노드 디버깅 인터페이스 추가
```python
# javis_dmc/interfaces/debug_interface.py
class DebugInterface:
    '''테스트용 인터페이스 - Mock RCS 작업 전송'''

    def send_pickup_task(self, book_id: str, storage_id: int):
        '''도서 픽업 작업 Goal 전송'''
        pass

    def send_guiding_task(self, destination: str):
        '''길 안내 작업 Goal 전송'''
        pass
```

---

## 4. 다음 단계

1. ✅ 이 문서를 기반으로 개선 작업 우선순위 확정
2. P0 항목부터 순차적으로 코드 수정
3. 수정 완료 후 통합 테스트
4. 인수인계 문서에 TEST GUI 사용법 추가

---

## 부록 A: 수동 상태 설정 vs Mock Action Client 비교

| 항목 | 수동 상태 설정 | Mock Action Client |
|------|----------------|-------------------|
| 구현 복잡도 | 낮음 | 중간 |
| 실제 동작 재현 | 불가 (상태만 변경) | 가능 (정상 흐름 실행) |
| 하위 인터페이스 호출 | 없음 | 있음 |
| RCS 로깅 | 불가 | 가능 |
| 디버깅 유용성 | 낮음 | 높음 |
| **권장 여부** | ❌ | ✅ |

---

## 부록 B: 긴급 패치 체크리스트

인수인계 전 최소한의 수정으로 GUI를 사용 가능하게 만들기:

- [ ] `test_gui_widget.py:309` - `width=18` → `width=22` (임시 조치)
- [ ] `test_gui_widget.py:354` - 버튼 라벨 "작업 건너뛰기 (성공 처리)"로 변경
- [ ] `test_gui_widget.py:355` - 실패 처리 버튼 삭제 또는 비활성화
- [ ] `test_gui_widget.py:389-405` - Real/Mock 라디오 버튼 비활성화, 안내 문구 추가
- [ ] `test_gui_widget.py:215` - `describe_state_machine()` 재시도 로직 추가
- [ ] DMC 노드 주석에 "수동 상태 설정은 테스트용이며, 실제 작업 흐름을 재현하지 않음" 명시

---

**작성자**: Claude Code
**검토 필요**: 팀원 코드 리뷰 및 인수인계 담당자 확인
