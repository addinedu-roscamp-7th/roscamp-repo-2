# JAVIS DMC (Dobby Main Controller)

This document provides a comprehensive overview of the JAVIS DMC project, its architecture, and how to build, run, and contribute to it.

## Project Overview

항상 한국어로 답변해주세요.

이 프로젝트는 javis 도서관 비서 로봇 시스템입니다.

모든 코드는 /docs 안의 설계문서와 정합적으로 작성되어야 합니다:

The JAVIS DMC is the main controller for the Dobby robot, a library assistant robot. It is responsible for managing the robot's state, executing tasks, and interfacing with other parts of the JAVIS system, such as the drive system, arm, AI, and GUI.

The DMC is a ROS 2 Python package that uses a state machine to manage the robot's behavior. It exposes a set of action servers for performing tasks such as picking up books, reshelving books, and guiding users. It also provides a set of services for administrative tasks, such as setting the robot's mode and performing an emergency stop.

### Key Technologies

- **ROS 2:** The project is built on the Robot Operating System 2 (ROS 2) framework.
- **Python:** The main programming language used in the project.
- **YAML:** Used for configuration files.

### Architecture

The DMC is a central component in the JAVIS robot control system. It communicates with other ROS 2 nodes and hardware through a set of interfaces. The main components of the DMC are:

- **`dmc_node`:** The main ROS 2 node that orchestrates the robot's behavior.
- **`DmcStateMachine`:** A state machine that manages the robot's states.
- **Task Executors:** A set of classes that implement the logic for each of the robot's tasks.
- **Interfaces:** A set of classes that provide an abstraction layer for communicating with other ROS 2 nodes and hardware.
- **Services:** A set of ROS 2 services for administrative tasks.
- **Action Servers:** A set of ROS 2 action servers for performing tasks.

## Building and Running

### Building

To build the project, use the `colcon` build tool:

```bash
colcon build --packages-select javis_dmc
```

### Running

The project can be run using the provided launch files. The `dmc_test.launch.py` file is a good starting point for running the DMC in a test environment.

```bash
ros2 launch javis_dmc dmc_test.launch.py
```

This will launch the `javis_dmc_node` and the `test_gui_node`. The `test_gui_node` provides a graphical interface for testing the DMC.

By default, the `dmc_test.launch.py` file sets the `use_mock_interfaces` parameter to `true`. This means that the DMC will use mock interfaces for the drive system, arm, AI, and GUI. This is useful for testing the DMC without needing to have the actual hardware connected.

## Development Conventions

The project follows a strict set of coding conventions, which are documented in the `AGENTS.md` file. All contributors are expected to follow these conventions to ensure code consistency and maintainability.

### Key Conventions

- **ROS 2 Naming Conventions:** All ROS 2 elements (packages, nodes, topics, etc.) should follow the `snake_case` naming convention.
- **Python Style Guide:** The project follows the PEP 8 style guide for Python code.
- **Commenting:** All comments should be written in Korean.
- **Documentation:** All code should be well-documented using Python docstrings.

## State Machine

The DMC uses a state machine to manage the robot's behavior. The state machine is defined in the `javis_dmc/states/main_states.py` file. The main states of the robot are:

- `INITIALIZING`: The robot is initializing.
- `CHARGING`: The robot is charging.
- `IDLE`: The robot is idle and waiting for a task.
- `MOVING_TO_CHARGER`: The robot is moving to the charger.
- `PICKING_UP_BOOK`: The robot is picking up a book.
- `RESHELVING_BOOK`: The robot is reshelving a book.
- `GUIDING`: The robot is guiding a user.
- `CLEANING_DESK`: The robot is cleaning a desk.
- `SORTING_SHELVES`: The robot is sorting shelves.
- `FORCE_MOVE_TO_CHARGER`: The robot is moving to the charger due to a low battery.
- `LISTENING`: The robot is listening for a voice command.
- `ROAMING`: The robot is roaming around the library.
- `EMERGENCY_STOP`: The robot is in an emergency stop state.
- `MAIN_ERROR`: The robot is in an error state.

The state machine transitions between these states based on events such as task requests, battery status, and user input.

## Key Packages and Nodes

- **`javis_dmc`:** The main package for the DMC.
  - **`dmc_node`:** The main ROS 2 node for the DMC.
  - **`test_gui_node`:** A graphical interface for testing the DMC.
- **`javis_interfaces`:** A package that contains the message and service definitions used by the JAVIS project.
- **`javis_dmc_test_msgs`:** A package that contains message and service definitions used for testing the DMC.

---
