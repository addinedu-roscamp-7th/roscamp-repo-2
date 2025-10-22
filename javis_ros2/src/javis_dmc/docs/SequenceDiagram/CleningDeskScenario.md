좌석 예약 


sequenceDiagram
    actor 도서관_회원
    participant Authentication_Controller
    participant Information_Desk_GUI
    participant Application_Service
    도서관_회원->>Information_Desk_GUI: 좌석예약 버튼 클릭 [TCP]
    Information_Desk_GUI->>Information_Desk_GUI: 회원증 인증 페이지 업데이트 
    도서관_회원->>Authentication_Controller: 회원증 태깅 
    Authentication_Controller->>Information_Desk_GUI: 회원 정보 전달[Serial]
    Information_Desk_GUI->>Application_Service: 회원 인증 요청 {회원 ID}[TCP]
    Application_Service->>Application_Service: DB에 회원 ID 조회
    Application_Service->>Application_Service: 회원 인증
    Application_Service-->>Information_Desk_GUI: 인증 결과 응답 {회원ID, 좌석예약 상태}[TCP]
    Information_Desk_GUI->>Information_Desk_GUI: 좌석 정보 페이지 업데이트
    도서관_회원->>Information_Desk_GUI: 해당 좌석 선택 후 예약 버튼 클릭
    Information_Desk_GUI->>Application_Service: 좌석 예약 전달 {좌석 ID, 시간, 예약여부}[TCP]
    Application_Service->>Application_Service: 좌석예약정보 업데이트
    Application_Service-->>Information_Desk_GUI: 좌석예약신청 성공 전송[TCP]
    Information_Desk_GUI->>Information_Desk_GUI: 예약완료 페이지 업데이트 {안내받기, 초기화면 버튼시현}
예약좌석 정리이동


sequenceDiagram
    participant AppService as Application Service
    participant RobotService as Robot Controll Service
    participant MainController as Dobby Main controller
    participant DriveController as Dobby Drive controller
    AppService->>RobotService: 좌석정리 요청 {좌석 위치}[TCP]
    RobotService->>RobotService: 좌석 정리 작업 생성
    RobotService-->>AppService: 생성완료 [TCP]
    loop 작업 가능 로봇 확인
        RobotService->>RobotService: 작업 가능 로봇 확인
    end
    RobotService->>MainController: 좌석 정리 작업할당 {작업명, 좌석 위치} [ROS2]
    MainController->>MainController: 상태 전환<br>상위[좌석정리중]<br>하위[좌석위치이동] 
    MainController->>RobotService: 로봇 상태 [ROS2]
    MainController->>DriveController: 좌석 위치이동 [ROS2]
    DriveController->>DriveController: 목적지 이동 
    DriveController-->>MainController: 이동완료 [ROS2]
    MainController->>MainController: 상태 전환<br>상위[좌석정리중]<br>하위[좌석 탐색중]
    MainController->>RobotService: 로봇 상태 [ROS2]
좌석정리 


sequenceDiagram
    participant DMC as Dobby Main Controller
    participant DAC as Dobby Arm Controller
    participant AIS as AI Image Service
    participant RCS as Robot Control Service
    activate DMC
    DMC->>DAC: 좌석 탐지자세 요청[ROS2]
    activate DAC
    DAC->>DAC: 좌석 탐지 자세 이동
    DAC-->>DMC: 이동완료[ROS2]
    deactivate DAC
    DMC->>AIS: 쓰레기 탐지 요청[ROS2]
    activate AIS
    AIS->>AIS: 좌석 쓰레기 탐지
    AIS-->>DMC: 쓰레기 정보전송 {종류, 위치}[ROS2]
    deactivate AIS
    DMC->>DMC: 상태변경<br>상위[좌석정리중]<br>하위[쓰레기 수거]
    DMC->>RCS: 로봇 상태[ROS2]
    loop 감지 쓰레기 > 0
        DMC->>DAC: 쓰레기 수거 명령{종류, 위치}[ROS2]
        activate DAC
        DAC->>DAC: 쓰레기수거 동작
        DAC-->>DMC: 쓰레기 수거 완료[ROS2]
        deactivate DAC
        DMC->>AIS: 쓰레기 탐지 요청[ROS2]
        activate AIS
        AIS->>AIS: 쓰레기탐지
        AIS-->>DMC: 쓰레기 정보{종류, 위치}[ROS2]
        deactivate AIS
        DMC->>DMC: 상태변경<br>상위[좌석정리중]<br>하위[ 쓰레기통 이동]
        DMC->>RCS: 로봇 상태[ROS2]
    end
    deactivate DMC
 

쓰레기 배출
.



sequenceDiagram
    participant DMC as Dobby Main Controller
    participant DDC as Dobby Drive Controller
    participant DAC as Dobby Arm Controller
    participant RCS as Robot Control Service
    activate DMC
    DMC->>DDC: 쓰레기통 이동 {위치}[ROS2]
    activate DDC
    DDC->>DDC: 쓰레기통 이동
    deactivate DDC
    DDC-->>DMC: 쓰레기통 이동 완료 [ROS2]
    DMC->>DMC: 상태변경<br>상위[좌석정리중]<br>하위[쓰레기배출]
    DMC->>RCS: 로봇 상태 [ROS2]
    DMC->>DAC: 쓰레기 배출 명령 [ROS2]
    activate DAC
    DAC->>DAC: 쓰레기통 배출 동작
    activate RCS 
    deactivate DAC
    DAC-->>DMC: 배출완료 [ROS2]
    RCS-->>DMC: 작업완료 [ROS2]
    DMC->>DMC: 상태변경<br>상위[대기장소이동중]<br>하위[]
    RCS->>RCS: 작업종료 
    deactivate RCS
    DMC->>RCS: 로봇 상태 [ROS2]
    deactivate DMC