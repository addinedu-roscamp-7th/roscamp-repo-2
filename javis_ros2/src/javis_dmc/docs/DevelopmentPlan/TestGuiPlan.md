# DMC Test GUI 설계 v1.0

작성일: 2025-02-18  
작성자: AI 도우미  
연계 문서: `DevelopmentPlan/ImplementationChecklist.md`, `docs/Architecture/SoftwareArchitecture.md`

---

## 1. 목적

1. `javis_dmc` 노드의 상태, 모드, 세션 정보를 직관적으로 모니터링한다.  
2. 관리자 서비스(`set_robot_mode`, `emergency_stop`, `resume_navigation`, `set_listening_mode`)를 GUI에서 바로 호출한다.  
3. 작업/서브 상태가 외부 입력 대기 때문에 병목되는 상황을 완화하기 위해, 작업별 상세 진행 상황을 GUI에서 기록하고 병렬 진행 여부를 표시할 수 있도록 한다.

---

## 2. 요구 기능

| 구분 | 세부 기능 | 비고 |
| :--- | :--- | :--- |
| 상태 모니터링 | `status/robot_state`, `admin/mode_feedback`, `status/battery_status` 구독 | 메인/서브 상태 이름 매핑 |
| 세션 확인 | LISTENING 활성 여부, 목적지 선택 세션 타이머 표시 | 향후 `diagnostics` 토픽 연동 예정 |
| 관리자 제어 | Standby/Autonomy 전환, 긴급 정지/해제, LISTENING on/off | 서비스 성공/실패 결과 메시지 표시 |
| 작업 강제 처리 | `admin/force_task_result` 호출로 현재 작업을 성공/실패 처리 | RCS 액션 종결 후 다음 테스트 단계 진입 가능 |
| 상태 수동 설정 | `admin/set_manual_state`로 모드/메인/서브 상태 강제 전환 | Test GUI 입력값 기반 |
| 병목 해소 보조 | 작업 목록 등록/수정, 상태(`대기`, `진행중`, `병렬`, `완료`, `보류`) 직접 기입 | GUI 내부 기록, CSV/JSON 내보내기 추후 검토 |
| 이벤트 로그 | GUI 내부 로그 창에 주요 이벤트 출력 | ROS → GUI 큐 기반 |

---

## 3. 아키텍처 개요

```
┌──────────────────────────────┐
│ TestGuiRosNode (rclpy.Node)  │
│  - ROS 구독/서비스 클라이언트 │
│  - 상태 변화 → Queue enqueue │
└──────────────┬───────────────┘
               │ thread-safe Queue
┌──────────────▼───────────────┐
│ TestGuiApp (Tkinter)          │
│  - after()로 Queue polling    │
│  - 버튼 이벤트 → ROS 호출     │
│  - 작업 상태 테이블 관리      │
└──────────────────────────────┘
```

* ROS 스핀은 별도 스레드(`MultiThreadedExecutor`)로 실행한다.  
* GUI 스레드는 Tkinter 메인 루프를 유지하며, Queue 수신 데이터를 기반으로 위젯을 업데이트한다.  
* 서비스 호출 결과는 비동기 Future callback에서 Queue로 전송한다.

---

## 4. 작업 상태 병목 대응 전략

1. **작업 카드 등록**  
   - 작업 ID/설명 입력 후 추가.  
   - 기본 상태는 `대기`.

2. **상태 전환 버튼**  
   - `진행중`, `병렬`, `완료`, `보류`, `삭제` 버튼 제공.  
   - 병렬 진행은 내부적으로 `parallel` 플래그 설정 후 강조 표시.

3. **병렬 진행 가이드**  
   - 병렬 전환 시 GUI가 안내 문구(예: "병렬 진행 중, 자원 배분 확인")를 로그에 출력.  
   - 향후 RCS 연동 시 이 정보를 토픽으로 발행할 여지를 문서화(현재는 GUI 내부 기록만 관리).

4. **로그 & 내역**  
   - 상태 전환, 서비스 호출, 병목 해소 등 이벤트를 로그 창에 남긴다.  
   - `force_task_result`로 작업을 종료한 뒤 `set_manual_state`로 원하는 상태에서 테스트 재개 가능.  
   - 추후 Export 기능 추가가 필요하면 CSV/JSON 파일 저장 API를 연결한다.

---

## 5. 구현 계획

1. **코드 구성**
   - `test_gui/test_gui_node.py`: ROS 노드 및 Tkinter 구동 엔트리 포인트.
   - `test_gui/test_gui_widget.py`: Tkinter 위젯 정의 및 이벤트 처리.
   - `setup.py`: `console_scripts`에 `dmc_test_gui` 추가.
   - `docs/DevelopmentPlan/ImplementationChecklist.md`: CL-10 진행 메모 업데이트.

2. **최초 기능 목록**
   - 상태 라벨(모드/메인/서브) + 배터리 표시.
   - 관리자 제어 버튼 세트.
   - LISTENING on/off 버튼.
   - 작업 강제 성공/실패 버튼.
   - 모드/메인/서브 상태 수동 입력 후 적용 버튼.
   - 작업 상태 테이블 + 상태 전환 버튼.
   - 로그 Text 위젯.

3. **향후 확장**
   - 목적지 선택 세션, Vision 이벤트를 별도 영역에 표시.
   - ROS Topic/Service 호출 결과를 파일로 저장.
   - 병렬 진행 상태를 RCS와 공유할 수 있는 커스텀 메시지 정의.

---

## 6. 테스트 전략

1. `ros2 launch javis_dmc dmc_test.launch.py use_sim_time:=false`로 mock DMC 실행.  
2. `ros2 run javis_dmc dmc_test_gui --ros-args -p robot_namespace:=dobby_test`로 GUI 실행.  
3. 서비스 호출 버튼이 정상 동작하는지 확인: `/dobby_test/admin/set_robot_mode`, `/dobby_test/admin/emergency_stop`, `/dobby_test/set_listening_mode`, `/dobby_test/admin/force_task_result`, `/dobby_test/admin/set_manual_state`.  
4. 작업 상태 테이블에서 병렬/완료 상태 전환 시 로그 기록 확인.  
5. 긴급 정지 → 해제, LISTENING on/off, 작업 강제 처리/수동 상태 적용 후 상태 라벨과 로그가 갱신되는지 검증.

---

## 7. Checklist 반영

- CL-07, CL-09, CL-10과 직접 연계.  
- GUI 및 병목 대응 기능 구현 완료 시 `ImplementationChecklist.md` 상태를 갱신한다.  
- 추가 문서/인터페이스 변경 시 `docs/DevelopmentPlan/changelog/`에 업데이트 내역 기록.
