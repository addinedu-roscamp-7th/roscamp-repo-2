항상 한국어로 답변해주세요.

이 프로젝트는 JAVIS 도서관 관리 로봇(DOBBY), 카페 음료제조(KREACHER)  관리 시스템입니다.

모든 코드는 /docs 안의 설계문서와 정합적으로 작성되어야 합니다:

Architecture/       : 하드웨어/소프트웨어 아키텍처 설계
DevelopmentPlan/    : 각 서비스별 개발 계획
ERDiagram/          : 데이터베이스 설계
InterfaceSpecification/ : 서비스 간 인터페이스 명세
Requirements/       : 시스템/사용자 요구사항
SequenceDiagram/    : 시나리오별 시퀀스 다이어그램
StateDiagram/       : 로봇 상태 다이어그램

ROS2 표준:
Package Names: snake_case
Node/Topic/Service/Action/Parameter Names: snake_case
Type Names: PascalCase
Type Field Names: snake_case
Type Constants Names: SCREAMING_SNAKE_CASE
Python 표준:
Package 및 module 이름: snake_case
Class 및 exception 이름: PascalCase
Function, method, parameter, local/instance/global 변수 이름: snake_case
Global/Class constants: SCREAMING_SNAKE_CASE
C++ 표준:
File Names: snake_case
Type Names: PascalCase
Function Names: PascalCase (접근자는 snake_case)
Variable Names: snake_case (클래스 멤버는 _suffix)
Constant Names: k + PascalCase
Macro Names: SCREAMING_SNAKE_CASE
Namespace Names: snake_case
공통 규칙:
주석은 한국어로 작성 (C++: //, Python: #)
세로 간격: 함수와 함수 사이 1줄, 블록 사이 1줄, 헤더와 본문 사이 2줄
Import문은 한줄에 하나씩
제어문은 반드시 중괄호 사용
문자열은 작은따옴표 사용
들여쓰기: C++ 2칸, Python 4칸(tab)