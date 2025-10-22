# JAVIS 프로젝트 코딩 컨벤션 🤖☕

이 문서는 **JAVIS 도서관 관리 로봇(DOBBY)** 및 **카페 음료제조 관리 시스템(KREACHER)** 프로젝트를 위한 코딩 컨벤션을 정의합니다. 모든 팀원은 아래 규칙을 엄격히 준수하여 코드의 일관성과 유지보수성을 높여야 합니다.

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
