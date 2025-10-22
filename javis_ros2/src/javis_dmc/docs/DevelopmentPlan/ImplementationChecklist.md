# JAVIS DMC 구현 체크리스트 v1.0

본 체크리스트는 `docs/DevelopmentPlan/DevelopmentPlan.md`의 구현 전략(§8)과 연계 문서를 기반으로 작업 진행 상태를 추적하기 위한 문서입니다. 각 항목은 기본 상태를 `대기`로 두며, 진행 시 `진행중`, 완료 시 `완료`로 변경합니다.


## 진행 현황 요약

- 전체 항목 수: 10
- 완료: 1 / 진행중: 4 / 대기: 5
- 마지막 갱신: 2025-02-18 (AI 업데이트)


## 세부 체크리스트

| ID | 기능 영역 | 작업 내용 | 연계 문서 | 담당 | 상태 | 메모 |
| :-- | :-- | :-- | :-- | :-- | :-- | :-- |
| CL-01 | 음성 파이프라인 | `VoiceRecognitionInterface` 리팩터링 및 `set_listening_mode` 서비스 연동 | DevelopmentPlan §2.3, §4.5.1<br>`docs/Architecture/SoftwareArchitecture.md` |  | 진행중 | 인터페이스 골격 및 이벤트 콜백 반영 |
| CL-02 | 음성 파이프라인 | `VoiceApiInterface` REST 클라이언트 구현 (`/voice/v1/stream`, `/voice/v1/greet`) | DevelopmentPlan §4.5.1<br>`docs/Architecture/SoftwareArchitecture.md` |  | 대기 |  |
| CL-03 | LISTENING 세션 | `ListeningSession` 매니저 구현 (타이머, 음성 스트림 제어, 응답 큐) | DevelopmentPlan §4.5.1<br>`docs/SequenceDiagram/GuidingScenario.md` |  | 진행중 | DMC 노드에 LISTENING 서비스/타이머 연동 |
| CL-04 | GUIDING 세션 | `DestinationSession` 타이머 및 GUI/비전 연동 로직 구현 (60초 타임아웃) | DevelopmentPlan §4.5.2<br>`docs/SequenceDiagram/GuidingScenario.md` |  | 대기 |  |
| CL-05 | 작업 실행기 | `GuidingExecutor` 업데이트 (서브 상태, Vision 이벤트, abort 로직) | DevelopmentPlan §4.4<br>`docs/SequenceDiagram/GuidingScenario.md` |  | 대기 |  |
| CL-06 | 상태 머신 | `DmcStateMachine`/Enum 확장 및 ROAMING·EMERGENCYC_STOP 전이 구현 | DevelopmentPlan §5.3~§5.7<br>`docs/StateDefinition/StateDefinition.md` |  | 완료 | 모드 전환/긴급 정지/리스닝 처리 로직 반영 |
| CL-07 | 관리자 연동 | `set_robot_mode`·`emergency_stop`·`resume_navigation` 등 관리자 서비스 처리 | DevelopmentPlan §5.2<br>`docs/Architecture/SoftwareArchitecture.md` |  | 진행중 | 모드/긴급/수동 완료/수동 상태 서비스 구현 |
| CL-08 | 파라미터/타이머 | `config/action_timeouts.yaml` 및 Voice API 파라미터 로딩 | DevelopmentPlan §4.5, §7 |  | 진행중 | action_timeouts.yaml 로딩 함수 적용 |
| CL-09 | 로깅/모니터링 | LISTENING/모드 전환/긴급 정지 로그 및 Admin GUI 피드백 | DevelopmentPlan §5.2, §7 |  | 진행중 | Test GUI 로그/피드백 표시 초안 구현 |
| CL-10 | 테스트 | Test GUI & 단위 테스트 작성/보강 (세션, 모드, 인터페이스) | DevelopmentPlan §8.1<br>`test/` 디렉터리 |  | 진행중 | `dmc_test_gui` 초안 구현, 후속 자동화 필요 |


## 사용 가이드

1. 항목 착수 시 `상태`를 `진행중`으로 변경하고 필요 시 `메모`에 참고 사항을 기록합니다.
2. 작업 완료 후 관련 문서(DevelopmentPlan, StateDefinition, SequenceDiagram 등)를 검토하여 정합성을 유지하고 `상태`를 `완료`로 갱신합니다.
3. 신규 작업이 발생하면 ID 체계를 유지하며 표에 행을 추가하고, 전체 항목 수/진행 현황을 상단 요약에 반영합니다.
4. 변경 이력은 `docs/DevelopmentPlan/changelog/` 디렉터리에 날짜별로 기록합니다.
