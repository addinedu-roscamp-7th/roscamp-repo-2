
# J A V I S

DOBBY 도서관 관리 로봇과 KREACHER 카페 음료제조 시스템을 하나의 ROS2 기반 통합 플랫폼으로 묶어, 도서 픽업·반납과 카페 주문·제조를 자동화하는 JAVIS 서비스

<img src="https://github.com/user-attachments/assets/d0d0bd77-5cb5-4fc8-9e47-9b23468030de"/>

---
## HW Architecture
<img width="3549" height="1861" alt="Image" src="https://github.com/user-attachments/assets/be30b1fb-4c7c-4fd7-9094-680e46b2be9a" />

## SW Architecture
<img width="4528" height="3074" alt="Image" src="https://github.com/user-attachments/assets/a13095d2-5a04-489e-a958-878c01f90e5b" />

## State Diagram
<img width="1094" height="1215" alt="Image" src="https://github.com/user-attachments/assets/65fd344f-0872-4402-9ffe-656dbe3356b2" />

## 기술 스택
<img width="925" height="540" alt="Image" src="https://github.com/user-attachments/assets/4b88d816-67a9-4c5c-9bf7-b82f33336d55" />

---
## KREACHER

### 시연영상
- 주문 GUI
<img src="https://github.com/user-attachments/assets/f9503bb8-1abf-4ee0-8743-f0408b6c0c5b"/>

- 핫아메리카노
<img src="https://github.com/user-attachments/assets/8bcf9c0b-4ba2-4a1a-a077-6e5894005fad"/>

- 아이스아메리카노
<img src="https://github.com/user-attachments/assets/75065919-f3c2-4977-9960-1645d0b4173d"/>

### 기술
<img width="878" height="473" alt="Image" src="https://github.com/user-attachments/assets/ec2428b0-6eea-4fc3-9981-fe0c6ef18268" />

<img width="882" height="480" alt="Image" src="https://github.com/user-attachments/assets/8a80b000-44e5-4e06-b9fa-30efe051999e" />

---
## Dobby
- Pick Up 요청 후 작업을 할당받은 Dobby가 책장 앞으로 이동 후 책 pickup
  
<img src="https://github.com/user-attachments/assets/d0d0bd77-5cb5-4fc8-9e47-9b23468030de"/>

---
### Dobby Drive

<img src="https://github.com/user-attachments/assets/ce7e6dba-8987-49cb-96f2-9ed3a3f76779"/>

---
### Dobby Arm

- 도서 **Pick/Place Action Server**, **시각 정렬(IBVS)**

---

#### 🔧 주요 기능

##### 🎯 시각 정렬 과정 (Vision Alignment Demo)

아래는 **정렬** 단계에서의 실제 동작 화면입니다.

<img src="https://github.com/user-attachments/assets/98f0ba58-a166-4d6d-aa90-8597faae208d" width="360"/>
<img src="https://github.com/user-attachments/assets/f2320436-40e7-4760-aaff-9e08b264e563" width="360"/>
<img src="https://github.com/user-attachments/assets/4f3ea43a-aa41-4787-a343-41dfa6dc1f46" width="360"/>
<img src="https://github.com/user-attachments/assets/0bdb0a96-a74a-4a5c-a4b8-2559fd77dd9f" width="360"/>

---

##### 🤖 Pick / Place 개요

###### Pick (SHELF → DOBBY)
- 슬롯 여유 확인  
- 기준 포즈 이동  
- 마커 스캔(다방향)  
- 중심 정렬 + yaw 보정  
- `transfer_book("SHELF_TO_DOBBY")` 실행  
- 슬롯 상태 갱신  

###### Place (DOBBY → SHELF)
- 책장/슬롯 매핑 조회  
- 동일한 정렬 루틴 적용  
- `transfer_book("DOBBY_TO_SHELF")` 실행  
- 슬롯 상태 갱신  

---

#### 🎥 Pick & Place 데모
- PICK
<img src="https://github.com/user-attachments/assets/2a4ea6d3-34e8-48d0-a5c4-39040e745f51" width="360"/>
<img src="https://github.com/user-attachments/assets/13966e82-1f9a-44e4-a072-a277a14cbfeb" width="360"/>

- PLACE
<img src="https://github.com/user-attachments/assets/d300cb99-2d5c-4c08-8797-2930bbcb9836" width="360"/>
<img src="https://github.com/user-attachments/assets/23c9d5ea-fff6-471d-aee2-40bb81ba5762" width="360"/>

















--- 

###  프로젝트 규칙
1. dev에서 개발하기
2. javis 루트 폴더에서 colcon build
3. 동영상 이미지 (readme 에서 사용하는 asset은 제외) 지양
4. 폴더 구조대로 개발

---

### *** git clone 이후 아래 과정을 먼저 진행해 주세요 ***
```
cd roscamp-repo-2/
cp gitmessage ~/.gitmessage
git config --global commit.template ~/.gitmessage
```
git commit 시 message 통일을 위해 template을 적용했습니다.

위 명령어 진행하신 뒤 git commit/push 부탁드립니다.


### 파이선 버전 : 3.12( 추가적으로 다른 버전 사용시 패키지 옆에 명시)


###  폴더 구조

    gui 
    └── src/
        ├── cafe_order/
        ├── information_desk/
        └── admin/
    service
    └── src/
        ├── llm_service/
        ├── app_service/
    javis_ros2/    # ROS2 패키지 colcon build는 여기서
    └── src/
        ├── javis_rcs/
        ├── javis_dmc/
        ├── javis_dac/
        ├── javis_ddc/
        ├── javis_dis/
        ├── javis_kc/
        ├── javis_kis/
        └── javis_interfaces/
        ├── msg/
        ├── srv/
        └── action/
    firware
    └── src/
        ├── authentication_controller/
        └── payment_controller/
    readme.md
    .gitignore
    doc/
    ├── develop.md/
    └── .../
    asset/
    ├── image/

---

### commit 규칙


#### 1. 기본 형식

       
```
 [Type/Scope]: <Subject>
    예시: [FEAT/DMC]: 도서 픽업 기능 추가
```

#### 2. Type (커밋 종류)

```
    Type 설명
        FEAT 새 기능
        FIX 버그 수정
        REFACTOR 코드 개선 (기능 변화 없음)
        DOCS 문서 수정
        TEST 테스트 추가/수정
        ENVIR 빌드, 설정 변경
        STYLE 코드 포맷팅
```
commit 시 template에서 확인할 수 있도록 적용했습니다.

프로젝트 규칙 아래 과정 진행 부탁드립니다.

#### 3. Scope (영향 범위)


```
    GUI
    gui/cafe - 카페 주문 GUI
    gui/info-desk - 안내 데스크 GUI
    gui/admin - 관리자 GUI

    Service
    service/llm - LLM 서비스
    service/app - 애플리케이션 서비스

    ROS2
    rcs - Robot Control Service
    dmc - Dobby Main Controller
    dac - Dobby Arm Controller
    ddc - Dobby Drive Controller
    dis - Dobby Image Service
    kc - Kreacher Controller
    kis - Kreacher Image Service
    interfaces - 메시지/액션 정의

    Firmware
    fw/auth - 인증 컨트롤러
    fw/payment - 결제 컨트롤러

    기타
    docs - 문서
    asset - 이미지/비디오

    ~ - 전체 프로젝트

```

### AI 툴 표준 사용 파일
Claude cli  : CLAUDE.md

Gemini cli  : GEMINI.md

Codex cli   : AGENTS.md


### 코드 규칙

## 1. 설계 문서 및 코드 정합성

모든 코드는 `/docs` 디렉토리 내의 설계 문서와 **정합적**으로 작성되어야 합니다.

| 경로 | 내용 |
| :--- | :--- |
| `Architecture/` | 하드웨어/소프트웨어 아키텍처 설계 |
| `DevelopmentPlan/` | 각 서비스별 개발 계획 |
| `ERDiagram/` | 데이터베이스 설계 |
| `InterfaceSpecification/` | 서비스 간 인터페이스 명세 |
| `Requirements/` | 시스템/사용자 요구사항 |
| `SequenceDiagram/` | 시나리오별 시퀀스 다이어그램 |
| `StateDiagram/` | 로봇 상태 다이어그램 |

---

## 2. ROS2 표준 (General)

ROS2에서 사용하는 모든 요소는 아래 명명 규칙을 따릅니다.

| 요소 | 표기법 | 예시 |
| :--- | :--- | :--- |
| **Package / Node / Topic / Service / Action / Parameter Names** | **`snake_case`** | `dobby_manager`, `robot_status` |
| **Type Names (msg, srv, action)** | **`PascalCase`** | `RobotStatus`, `MakeDrink` |
| **Type Field Names** | **`snake_case`** | `battery_level`, `beverage_type` |
| **Type Constants Names** | **`SCREAMING_SNAKE_CASE`** | `STATUS_IDLE`, `MAX_SHELF_CAPACITY` |

---

## 3. 언어별 명명 및 스타일 규칙

### 🅰️ Python 표준

| 요소 | 표기법 | 예시 |
| :--- | :--- | :--- |
| Package/Module 이름 | **`snake_case`** | `dobby_module.py`, `utils.py` |
| Class/Exception 이름 | **`PascalCase`** | `DobbyRobot`, `ItemNotFoundError` |
| Function/Method/Variable 이름 | **`snake_case`** | `process_book_scan`, `current_time` |
| Global/Class Constants | **`SCREAMING_SNAKE_CASE`** | `DEFAULT_TIMEOUT`, `PI_VALUE` |
| **들여쓰기** | **4칸 (space)** | |
| **문자열** | **작은따옴표** 사용 | `'Hello, World!'` |

### 🅱️ C++ 표준

| 요소 | 표기법 | 예시 |
| :--- | :--- | :--- |
| File Names | **`snake_case`** | `robot_manager.cpp`, `sensor_data.hpp` |
| Type Names (Class, Struct, Enum) | **`PascalCase`** | `DobbyManager`, `TaskStatus` |
| Function Names | **`PascalCase`** | `ProcessRequest`, `CalculatePath` |
| Accessor (Getter/Setter) Names | **`snake_case`** | `get_battery_level()` |
| Variable Names | **`snake_case`** | `current_state`, `target_position` |
| Class Member Variables | **`snake_case`** + **`_suffix`** | `current_state_`, `target_position_` |
| Constant Names | **`k`** + **`PascalCase`** | `kMaxSpeed`, `kDefaultTimeout` |
| Macro Names | **`SCREAMING_SNAKE_CASE`** | `DOBBY_VERSION` |
| Namespace Names | **`snake_case`** | `dobby_system`, `kreacher_utils` |
| **들여쓰기** | **2칸 (space)** | |

---

## 4. 공통 스타일 및 주석 규칙

1.  **주석 언어:** 주석은 **한국어**로 작성합니다.
    * C++: `//` 사용
    * Python: `#` 사용
2.  **세로 간격:**
    * 함수와 함수 사이는 **1줄** 비웁니다.
    * 블록(if, for, while, class 등) 사이는 **1줄** 비웁니다.
    * 헤더(import, include)와 본문 사이는 **2줄** 비웁니다.
3.  **Import문:** `import`/`#include` 문은 **한 줄에 하나**씩 작성합니다.
4.  **제어문:** C++ 제어문(`if`, `for`, `while`)은 반드시 **중괄호 `{}`** 를 사용합니다.

---

## 5. 💡 추가 권장 사항 (코드 품질 향상)

프로젝트의 유지보수성과 품질을 높이기 위해 다음 규칙을 추가로 준수하는 것을 권장합니다.

### 5.1. 문서화 및 추적

| 항목 | 권장 사항 |
| :--- | :--- |
| **함수/메소드 문서화** | **Python Docstring** 또는 **C++ Doxygen** 스타일을 사용하여 **입력, 출력, 예외, 상세 설명**을 명시합니다. |
| **TODO/FIXME 주석** | `// TODO(작성자 이름): 해결할 문제 내용`과 같이 **책임자를 명시**하여 작성합니다. |
| **헤더 가드 (C++)** | 모든 헤더 파일은 `#pragma once`를 사용하거나, `PROJECT_FILENAME_HPP_` 형태의 매크로를 사용하여 중복 포함을 방지합니다. |

### 5.2. 안정성 및 명확성

| 항목 | 권장 사항 |
| :--- | :--- |
| **로깅 사용** | 디버깅 및 운영 시 시스템 상태를 확인하기 위해 `rclpy.logging` 또는 `RCLCPP_INFO/ERROR` 등을 적극적으로 활용하고, **레벨(INFO, WARN, ERROR)**을 정확히 구분하여 사용합니다. |
| **매직 넘버** | 코드 내에서 의미가 불분명한 숫자(예: `if x > 1024:`)는 사용하지 않고, 반드시 **명명된 상수(Named Constant)**로 대체합니다. |
| **예외 처리** | 오류 발생 가능성이 있는 부분은 `try...except` (Python) 또는 `try...catch` (C++)를 사용하여 명확하게 예외를 처리하고, 사용자에게 의미 있는 오류 메시지를 전달합니다. |
