State Definition

Dobby 프로젝트의 로봇 상태(State) 정의 문서입니다.

목차

State Definition

목차

Dobby State Definition

공통 상태 (Common States)

도서 관리 모드 (Library Management Mode)

길 안내 모드 (Guidance Mode)

환경 미화 모드 (Cleaning Mode)

State Transition Rules

공통 전환 규칙

도서 관리 모드 전환 규칙

길 안내 모드 전환 규칙

환경 미화 모드 전환 규칙

버전 히스토리

Dobby State Definition

공통 상태 (Common States)

로봇의 기본적인 동작, 충전 및 대기 상태를 정의합니다.

State ID

한글명

영문명

설명

DB_S00

초기화 중

INITIALIZING

시스템 시작 및 초기화 진행 상태

DB_S01

충전 중

CHARGING

충전소에서 배터리 충전 중인 상태

DB_S02

작업 대기

IDLE

충전 완료 (배터리 40% 이상) 후 작업 지시를 대기하는 상태

DB_S03

충전소 이동 중

MOVING_TO_CHARGER

모든 작업 완료 후 충전소로 복귀하는 상태

DB_S04

강제 충전소 이동

FORCE_MOVE_TO_CHARGER

배터리 20% 이하일 때 모든 작업을 중단하고 충전소로 이동하는 상태

도서 관리 모드 (Library Management Mode)

도서관의 책장 정리, 반납 도서 정리, 도서 픽업 작업을 위한 상태입니다.

State ID

한글명

영문명

설명

(책장 정리)







DB_S10

책장으로 이동

MOVE_TO_SHELF

정리할 책장으로 이동하는 상태

DB_S11

도서 스캔

SCAN_BOOK

책장의 도서를 스캔하여 잘못 배치된 책을 찾는 상태

DB_S12

도서 정리

SORT_BOOK

잘못 배치된 도서를 올바른 위치로 정리하는 상태

(반납 도서 정리)







DB_S20

반납대로 이동

MOVE_TO_RETURN_DESK

반납 도서를 회수하기 위해 반납대로 이동하는 상태

DB_S21

반납 도서 회수

COLLECT_RETURN_BOOKS

반납된 도서를 수거하는 상태

DB_S22

배치할 책장으로 이동

MOVE_TO_PLACE_SHELF

회수한 도서를 배치할 책장으로 이동하는 상태

DB_S23

도서 배치

PLACE_RETURN_BOOK

회수한 도서를 책장에 배치하는 상태

(도서 픽업)







DB_S30

픽업 위치로 이동

MOVE_TO_PICKUP

지정된 도서를 픽업하기 위해 이동하는 상태

DB_S31

도서 픽업

PICKING_BOOK

도서를 그리퍼로 집어 드는 상태

DB_S32

보관 위치로 이동

MOVE_TO_STORAGE

픽업한 도서를 보관 위치(카트 등)로 이동하는 상태

DB_S33

도서 보관

STOWING_BOOK

픽업한 도서를 보관 위치에 내려놓는 상태

길 안내 모드 (Guidance Mode)

사용자를 지정된 목적지까지 안내하는 작업을 위한 상태입니다.

State ID

한글명

영문명

설명

DB_S40

목적지 선택

SELECT_DEST

사용자가 안내받을 목적지를 선택(입력)하는 상태

DB_S41

사용자 스캔

SCAN_USER

안내할 사용자를 인식하고 등록하는 상태

DB_S42

목적지로 안내

GUIDING_TO_DEST

사용자를 따라가며 목적지까지 안내하는 상태

DB_S43

사용자 탐색

FIND_USER

안내 도중 사용자를 놓쳤을 경우 30초간 다시 탐색하는 상태

환경 미화 모드 (Cleaning Mode)

좌석 주변의 쓰레기를 정리하는 작업을 위한 상태입니다.

State ID

한글명

영문명

설명

DB_S50

좌석으로 이동

MOVE_TO_DESK

정리할 좌석으로 이동하는 상태

DB_S51

좌석 스캔

SCAN_DESK

좌석 위 쓰레기 유무를 탐색하는 상태

DB_S52

쓰레기 수거

COLLECT_TRASH

발견된 쓰레기를 수거하는 상태

DB_S53

쓰레기통으로 이동

MOVE_TO_BIN

수거한 쓰레기를 버리기 위해 쓰레기통으로 이동하는 상태

DB_S54

쓰레기 배출

DUMP_TRASH

쓰레기를 쓰레기통에 배출하는 상태

State Transition Rules

공통 전환 규칙

START → DB_S00: 시스템 시작

DB_S00 → DB_S01: 초기화 완료

DB_S01 → DB_S02: 배터리 40% 이상 충전 완료

DB_S02 → DB_S04: 배터리 20% 이하 감지

DB_S02 → [Task Start State]: 각 작업 요청 시 해당 작업의 시작 상태로 전환

[Task Complete State] → DB_S02: 개별 작업 완료 시 대기 상태로 복귀

[All Tasks Complete] → DB_S03: 모든 작업 목록 완료 시 충전소로 이동

DB_S03 → DB_S01: 충전소 도착

DB_S04 → DB_S01: 충전소 도착

도서 관리 모드 전환 규칙

책장 정리 (Sorting)

DB_S02 → DB_S10: 책장 정리 작업 배정

DB_S10 → DB_S11: 책장 도착

DB_S11 → DB_S12: 정리 필요 도서 발견

DB_S12 → DB_S11: 도서 정리 완료

DB_S11 → DB_S10: 다음 책장으로 이동 필요

DB_S11 → DB_S02: 모든 책장 정리 완료

반납 도서 정리 (Reshelving)

DB_S02 → DB_S20: 반납 도서 정리 작업 배정

DB_S20 → DB_S21: 반납대 도착

DB_S21 → DB_S22: 도서 회수 완료

DB_S22 → DB_S23: 배치할 책장 도착

DB_S23 → DB_S22: 도서 배치 완료 (다음 책 이동)

DB_S23 → DB_S02: 모든 반납 도서 배치 완료

도서 픽업 (Pickup)

DB_S02 → DB_S30: 도서 픽업 작업 배정

DB_S30 → DB_S31: 픽업 위치 도착

DB_S31 → DB_S32: 도서 픽업 완료

DB_S32 → DB_S33: 보관 위치 도착

DB_S33 → DB_S02: 도서 보관 완료

길 안내 모드 전환 규칙

DB_S02 → DB_S40: 길 안내 작업 요청

DB_S40 → DB_S41: 목적지 입력 완료

DB_S41 → DB_S42: 사용자 인식 성공

DB_S41 → DB_S41: 사용자 인식 실패 (재시도)

DB_S42 → DB_S02: 목적지 도착, 안내 완료

DB_S42 → DB_S43: 사용자 놓침 (User Lost)

DB_S43 → DB_S42: 사용자 재인식 성공 (User Re-acquired)

DB_S43 → DB_S02: 30초 탐색 시간 초과, 안내 실패

환경 미화 모드 전환 규칙

DB_S02 → DB_S50: 좌석 정리 작업 배정

DB_S50 → DB_S51: 좌석 도착

DB_S51 → DB_S52: 쓰레기 감지

DB_S51 → DB_S50: 쓰레기 없음, 다음 좌석으로 이동

DB_S52 → DB_S53: 쓰레기 수거 완료

DB_S53 → DB_S54: 쓰레기통 도착

DB_S54 → DB_S02: 쓰레기 배출 완료

버전 히스토리

v1.0 (2025-10-17): 초기 문서 생성

Dobby State 정의 (공통/도서관리/길안내/환경미화 모드)

State 전환 규칙 추가