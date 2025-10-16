? Root (Fallback: 최상위 결정)
|
├── -> 1. 비상/긴급 상황 처리 (가장 먼저 체크!)
|   |
|   ├── ? IsBatteryCritical(level=20%)  // 배터리가 20% 이하인가? (Condition)
|   └── -> ForceGoToCharger()           // 그렇다면, 강제 충전소 이동 (Action)
|
├── -> 2. 작업 수행 (비상 상황이 아닐 때)
|   |
|   ├── ? IsTaskAssigned()              // 할당된 작업이 있는가? (Condition)
|   └── ? TaskSelector (Fallback: 어떤 작업인지 선택)
|       |
|       ├── -> Execute_PickingUpBook    // "도서 픽업" 서브트리 실행
|       ├── -> Execute_ReshelvingBook   // "반납 정리" 서브트리 실행
|       ├── -> Execute_Guiding          // "길 안내" 서브트리 실행
|       ├── -> Execute_CleaningDesk     // "좌석 정리" 서브트리 실행
|       └── -> Execute_SortingShelves   // "서가 정리" 서브트리 실행
|
└── -> 3. 기본 상태 (아무 일도 없을 때)
    |
    └── ? DefaultBehavior (Fallback: 기본 행동 결정)
        |
        ├── -> GoToChargerIfNeeded      // 충전이 필요하면 충전소로 이동
        |   |
        |   ├── ? IsAtChargerStation()      // 이미 충전소에 있는가? (Inverter + Condition)
        |   ├── ? ShouldCharge(level=40%) // 40% 미만이라 충전이 필요한가? (Condition)
        |   └── -> MoveTo(Charger)        // 충전소로 이동 (Action)
        |
        └── -> Idle