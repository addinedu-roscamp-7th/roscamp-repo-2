반납도서 확인


sequenceDiagram
    autonumber
    participant RCS as Robot Control Service
    participant Main as Dobby Main Controller
    participant Drive as Dobby Drive Controller
    rect rgb(245,245,245)
      loop once an hour
        RCS->>RCS: 스케줄링 기반<br/>도서 반납 업무 생성
      end
    end
    RCS->>Main: 도서 반납 업무 배정{작업명, 위치}  [ROS2]
    Note over Main: 상태 변경<br/>상위[반납도서정리중]<br/>하위[반납대 이동]
    activate Main
    Main->>Drive: 반납대 이동 명령(위치) [ROS2]
    activate Drive
    Drive->>Drive: 반납대 이동 명령(위치)  [ROS2]
    Main-->>RCS: 반납대 이동 상태 전달{상태: 이동 중}  [ROS2]
    Drive-->>Main: 도착완료  [ROS2]
    deactivate Main
    deactivate Drive
    Note over Main: 상태 변경<br/>상위[반납도서정리중]<br/>하위[반납도서 회수]
    Main-->>RCS: 반납대 도착{상태}  [ROS2]
 도서 수거


sequenceDiagram
    participant DMC as Dobby<br/>Main Controller
    participant DAC as Dobby Arm<br/>Controller
    participant AIS as AI Image<br/>Service
    participant RCS as Robot Control<br/>Service
    participant AS as Application<br/>Service
    DMC->>DAC: 반납대 관측 자세 이동
    activate DAC
    DAC->>DAC: 반납대 관측 자세<br/>이동
    DAC-->>DMC: 이동완료
    deactivate DAC
    DMC->>AIS: 반납도서 스캔  [ROS2]
    activate AIS
    AIS->>AIS: 반납도서 감지  [ROS2]
    AIS-->>DMC: 반납도서 발견{도서 ID,도서 위치, 반납대 위치} [ROS2]
    deactivate AIS
    DMC->>DMC: 반납도서<br/>운반함 위치 배정
    DMC->>DAC: 반납도서 회수<br/>{도서ID,도서위치, 운반함 위치} [ROS2]
    activate DAC
    rect rgb(220, 240, 255)
        Note over DMC,DAC: loop [반납 도서 > 1]
        DAC->>DAC: 도서 피킹 후<br/>운반함넣기
    end
    DAC-->>DMC: 회수 완료 [ROS2]
    deactivate DAC
    DMC->>RCS: 반납도서 정보 요청{도서 ID} [ROS2]
    activate RCS
    RCS->>AS: 도서정보 요청 [TCP]
    activate AS
    AS->>AS: DB 도서조회 
    AS-->>RCS: 도서 정보 전달 [TCP]
    deactivate AS
    RCS-->>DMC: 도서 정보 전달{도서ID, 도서 위치} [ROS2]
    deactivate RCS
도서책장으로 옮김


sequenceDiagram
    participant DMS as Dobby<br/>Main Service
    participant DDC as Dobby Drive<br/>Controller
    participant DAC as Dobby Arm<br/>Controller
    participant AIS as AI Image<br/>Service
    participant RCS as Robot Control<br/>Service
    participant AS as Application<br/>Service
    rect rgb(245, 245, 245)
        Note over DMS,AS: loop [반납 도서 > 1]
        DMS->>DMS: 상태변경<br/>상위[반납도서정리중]<br/>하위[책장 이동]
        DMS->>DDC: 책장 이동 명령{책장위치} [ROS2]
        activate DDC
        DDC->>DDC: 책장 이동
        DDC-->>DMS: 도착완료 [ROS2]
        deactivate DDC
        DMS->>DMS: 상태변경<br/>상위[반납도서 정리중]<br/>하위[도서 배치]
        rect rgb(235, 245, 255)
            Note over DMS,AIS: loop [같은 책장 반납 도서 > 1]
            DMS->>DAC: 반납 도서 재배치 명령 {도서 ID, 책장위치, 운반함위치} [ROS2]
            activate DAC
            DAC->>DAC: 도서 운반함 피킹,<br/>책장에 도서 배치,<br/>관측 자세
            DAC-->>DMS: 배치 완료 [ROS2]
            deactivate DAC
            DMS->>AIS: 도서 정위치 식별 요청[ROS2]
            activate AIS
            AIS->>AIS: 도서 정위치 식별 
            AIS-->>DMS: 식별 완료 전달 [ROS2]
            deactivate AIS
        end
        DMS->>DMS: 반납도서 정보 업데이트
    end
    DMS->>RCS: 작업 완료{도서 정보} [ROS2]
    activate RCS
    RCS->>AS: 도서 정보 업데이트 [TCP]
    activate AS
    AS->>AS: DB 정보<br/>업데이트
    AS-->>RCS: 업데이트 완료 [TCP]
    deactivate AS
    deactivate RCS
    DMS->>DMS: 상태 변경<br/>상위[대기장소 이동중]<br/>하위[]
 

반납대 ->도비 내부 보관함


sequenceDiagram
    title: 도비 내부 보관함 -> 책장
    participant DMC as Dobby Main Controller
    participant DAC as Dobby Arm Controller
    participant AIS as AI Image Service
    DMC->>DAC: 책장으로 도서 정리 명령 전달 [ROS2]
    activate DAC
    DAC-->>DMC: 명령 수신 [ROS2]
    DAC->>DAC: 책장 이미지 촬영 모드로 변경 및 촬영
	  DAC->>DMC: 책장 식별 및 좌표 요청 [ROS2]
    DMC->>AIS: 책장 식별 및 좌표 요청 [ROS2]
    activate AIS
    AIS-->>DMC: 책장 정보 및 좌표 전달 [ROS2]
    deactivate AIS
    DMC-->>DAC: 책장 정보 및 좌표 전달 [ROS2]
    loop 내부 보관함 안에 해당 책장 책 수 > 0
        DAC->>DAC: 책을 넣을 책장 좌표로 이동
        DAC->>DAC: 책장 촬영
        DAC->>DMC: 위치 보정 좌표 요청 [ROS2]
        DMC->>AIS: 위치 보정 좌표 요청 [ROS2]
        activate AIS
        AIS-->>DMC: 보정 좌표 전달 [ROS2]
        deactivate AIS
        DMC-->>DAC: 보정 좌표 전달 [ROS2]
        DAC->>DAC: 내부 보관함에서 책 픽킹
        DAC->>DAC: 책장에 책 위치
    end
    deactivate DAC