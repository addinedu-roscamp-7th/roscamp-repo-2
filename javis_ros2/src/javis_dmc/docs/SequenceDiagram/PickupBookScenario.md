## 조회   시나리오

```
sequenceDiagram
    actor User as 사용자
    participant GUI as Information Desk GUI
    participant AppService as Application Service
    User->>GUI: 도서조회 클릭 [TCP]
    activate GUI
    GUI-->>GUI: 도서 검색 화면 업데이트
    deactivate GUI
    User->>GUI: 도서 검색어 입력 
    activate GUI
    GUI->>AppService: 도서 조회 요청{도서명} [TCP]
    activate AppService
    AppService-->>AppService: DB에 도서 조회
    AppService-->>GUI: 도서 정보 리스트 응답{도서명, 저자, 출판사, 대여상태} [TCP]
    deactivate AppService
    GUI-->>GUI: 도서 정보 리스트 화면 업데이트 
    deactivate GUI
    User->>GUI: 대출 버튼 클릭
    activate GUI
    GUI-->>GUI: 회원증 스캔 요청 화면 업데이트
    deactivate GUI
```
### 회원 인증 후 자동 도서 픽업 

```
sequenceDiagram
    actor User
    participant Auth as Authentication Controller
    participant GUI as Information Desk GUI
    participant App as Application Service
    participant RCS as Robot Control Service
    participant DMC as Dobby Main Controller
    User->>Auth: 회원증 스캔
    activate Auth
    Auth->>GUI: 회원증 ID 전송 [Serial]
    deactivate Auth
    activate GUI
    GUI->>App: 회원 인증 요청{회원ID} [TCP]
    activate App
    App->>App: DB에 회원 ID조회
    App->>RCS: 도서픽업 작업생성 요청<br>{작업명, 도서명, 위치, 보관함 정보}[TCP]
    activate RCS
    RCS->>RCS: 작업생성
    RCS->>DMC: 도서픽업 명령<br>{도서 정보, 보관함 정보} [ROS2]
    activate DMC
    DMC->>DMC: 상태 변경<br>상위[도서픽업중]<br>하위[픽업위치 이동]
    DMC-->>RCS: 피드백 정보 [ROS2]
    deactivate DMC
    RCS-->>App: 작업생성 완료 [TCP]
    deactivate RCS
    App->>App: DB에 도서, 보관함 정보<br>업데이트
    App-->>GUI: 픽업 정보 전송<br>{보관함 위치, 예상시간} [TCP]
    deactivate App
    GUI->>GUI: 픽업도서 안내화면<br>업데이트
    deactivate GUI
```

### 픽업도서 픽업

```
sequenceDiagram
    participant DMC as Dobby Main Controller
    participant DDC as Dobby Drive Controller
    participant DAC as Dobby Arm Controller
    participant AI as AI Image Service
    participant GUI as Dobby GUI
    activate DMC
    DMC->>DDC: 픽업도서 위치 이동 명령
    activate DDC
    DDC->>DDC: 목적지 이동
    DDC-->>DMC: 이동완료
    deactivate DDC
    DMC->>DMC: 상태변경<br>상위[도서픽업중]<br>하위[도서픽업]
    DMC->>GUI: 로봇상태
    activate GUI
    GUI->>GUI: 로봇상태 디스플레이 업데이트
    deactivate GUI
    DMC->>DMC: 운반함 공간 배정
    DMC->>DAC: 도서 꺼내기 동작 요청<br>{도서 id, 도서 위치, 운반함 위치}
    activate DAC
    DAC->>DAC: 관측 자세
    DAC->>AI: 도서 감지 요청{도서 ID}
    activate AI
    AI->>AI: 이미지에서 도서 감지
    AI-->>DAC: 도서 감지 {도서ID}
    deactivate AI
    DAC->>DAC: 도서 집고 해당 운반함에<br>보관 후 관측 자세
    DAC-->>DMC: 동작완료
    deactivate DAC
    DMC->>DMC: 상태변경<br>상위[도서픽업중]<br>하위[보관위치이동]
    DMC->>GUI: 로봇상태
    activate GUI
    GUI->>GUI: 로봇상태 디스플레이 업데이트
    deactivate GUI
    deactivate DMC
```

###  픽업도서 보관함 보관  

```
sequenceDiagram
    participant DMC as Dobby Main<br>Controller
    participant DDC as Dobby Drive<br>Controller
    participant DAC as Dobby Arm<br>Controller
    participant AIS as AI Image<br>Service
    participant RCS as Robot Control<br>Service
    activate DMC
    DMC ->> DAC: 초기자세 요청
    activate DAC
    DAC ->> DAC: 초기자세 이동
    DAC -->> DMC: 이동 완료
    deactivate DAC
    DMC ->> DDC: 보관위치 이동{목적지}
    activate DDC
    DDC ->> DDC: 목적지 이동
    DDC -->> DMC: 이동완료
    deactivate DDC
    DMC ->> DMC: 상태 변경<br>상위[도서픽업중]<br>하위[도서보관]
    DMC ->> DAC: 도서 보관 동작 요청 {도서 id, 운반함 위치, 보관함 위치}
    activate DAC
    DAC ->> DAC: 관측 자세
    DAC ->> AIS: 보관함 상태 확인{보관함 ID}
    activate AIS
    AIS ->> AIS: 이미지에서 위치 감지
    AIS -->> DAC: 감지 정보 {보관함 ID, 보관함 위치}
    deactivate AIS
    DAC ->> DAC: 운반함에서 도서 픽킹 후<br>보관함에 도서 보관
    DAC -->> DMC: 동작완료
    deactivate DAC
    DMC ->> DMC: 상태 변경<br>상위[대기장소 이동중]<br>하위[]
    DMC -->> RCS: 작업 완료 {이상유무}
    activate RCS
    RCS ->> RCS: 작업 종료
    deactivate RCS
    deactivate DMC
```

### 복귀
       

```
sequenceDiagram
    participant DMC as Dobby Main<br>Controller
    participant DDC as Dobby Drive<br>Controller
    participant DAC as Dobby Arm<br>Controller
    participant RCS as Robot Control<br>Service
    participant GUI as Dobby GUI
    activate DMC
    DMC ->> DAC: 초기자세 요청
    activate DAC
    DAC ->> DAC: 초기자세 이동
    DAC -->> DMC: 이동 완료
    deactivate DAC
    DMC ->> DDC: 충전장소 이동{목적지}
    activate DDC
    DDC ->> DDC: 목적지 이동
    DDC -->> DMC: 이동완료
    deactivate DDC
    DMC ->> DMC: 상태 변경<br>상위[작업대기중]<br>하위[]
    DMC ->> RCS: 로봇상태
    DMC ->> GUI: 로봇상태
    activate GUI
    GUI ->> GUI: 로봇상태 화면<br>업데이트
    deactivate GUI
    deactivate DMC
```

### 보관함 잠금     

```
sequenceDiagram
    participant RCS as Robot Control<br>Service
    participant AppS as Application<br>service
    participant IDG as Information Desk<br>GUI
    participant AuthC as Authentication<br>Controller
    RCS ->> AppS: 정보 업데이트<br>{도서정보, 보관함 정보, 회원 정보} [TCP]
    activate AppS
    AppS ->> AppS: DB 정보 업데이트
    AppS -->> RCS: 업데이트 완료 [TCP]
    AppS ->> IDG: 보관함 잠금 요청<br>{보관함 ID} [TCP]
    activate IDG
    IDG ->> AuthC: 보관함 잠금 요청 [Serial]
    activate AuthC
    AuthC ->> AuthC: 보관함 문 닫기 
    AuthC -->> IDG: 잠금 완료 {보관함 ID} [Serial]
    deactivate AuthC
    IDG -->> AppS: 잠금 완료 {보관함 ID} [TCP]
    deactivate IDG
    AppS ->> AppS: DB 에 보관함 상태<br>업데이트
    deactivate AppS
```
### 회원이 도서 픽업

```
sequenceDiagram
    participant M as 회원
    participant GUI as Information Desk<br>GUI
    participant App as Application<br>Service
    participant Auth as Authentication<br>Controller
    activate M
    M ->> GUI: 가지가기 클릭
    activate GUI
    GUI ->> GUI: 회원인증 페이지<br>업데이트
    M ->> GUI: 회원증 태그
    GUI ->> App: 회원 인증 요청(회원 ID)
    activate App
    App ->> App: DB 회원 ID 조회
    App ->> App: 보관함, 도서 정보<br>업데이트
    App -->> GUI: 인증 성공(보관함 ID)
    deactivate App
    GUI ->> Auth: 보관함 오픈 명령(보관함 ID)
    activate Auth
    Auth ->> Auth: 보관함 오픈
    Auth -->> GUI: 오픈 완료
    deactivate Auth
    GUI ->> GUI: 결과 업데이트<br>(보관함 정보)
    deactivate GUI
    deactivate M
```