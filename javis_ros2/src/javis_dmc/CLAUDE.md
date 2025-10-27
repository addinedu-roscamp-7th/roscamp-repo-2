# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.
항상 한국어로 답변해줘. 
## Project Overview

**JAVIS DMC (Dobby Main Controller)** is a ROS 2 Python package that serves as the main controller for the Dobby library assistant robot. This is part of the larger JAVIS (Library Automation System) project, which includes multiple Dobby robots and a Kreacher cafe robot for library services.

The DMC orchestrates robot behavior through a state machine architecture, managing tasks like book pickup, reshelving, user guidance, desk cleaning, and shelf sorting.

## Development Commands

### Building

Build the javis_dmc package:
```bash
colcon build --packages-select javis_dmc
```

Build with specific options:
```bash
colcon build --packages-select javis_dmc --symlink-install
```

### Running

Launch in test mode with mock interfaces:
```bash
ros2 launch javis_dmc dmc_test.launch.py
```

Launch for single robot:
```bash
ros2 launch javis_dmc dmc_single.launch.py robot_namespace:=dobby1
```

Launch for multiple robots:
```bash
ros2 launch javis_dmc dmc_multi.launch.py
```

### Testing

Run all tests:
```bash
colcon test --packages-select javis_dmc
```

Run pytest with coverage:
```bash
pytest --cov=javis_dmc --cov-report=term-missing test/
```

Run linting checks:
```bash
# PEP8 style check
pytest test/test_flake8.py

# Docstring style check
pytest test/test_pep257.py

# Copyright header check
pytest test/test_copyright.py
```

Run a specific test:
```bash
pytest test/test_battery_manager.py -v
```

## Code Architecture

### State Machine Design

The DMC uses a hierarchical state machine with:

**Main States** (`MainState` enum in `javis_dmc/states/state_enums.py`):
- System states: `INITIALIZING`, `IDLE`, `CHARGING`, `MOVING_TO_CHARGER`, `FORCE_MOVE_TO_CHARGER`, `ROAMING`, `LISTENING`, `EMERGENCY_STOP`, `MAIN_ERROR`
- Task states: `PICKING_UP_BOOK`, `RESHELVING_BOOK`, `GUIDING`, `CLEANING_DESK`, `SORTING_SHELVES`

**Sub States** (`SubState` enum): Fine-grained states within each task (e.g., `MOVE_TO_PICKUP`, `PICKUP_BOOK`, `MOVE_TO_STORAGE`, `STOWING_BOOK` for book pickup tasks)

**Robot Modes** (`RobotMode` enum):
- `STANDBY`: Waits at charger, accepts manual task assignments
- `AUTONOMY`: Autonomously patrols waypoints, accepts task assignments

The state machine is implemented in `javis_dmc/states/main_states.py` (`DmcStateMachine` class).

### Task Execution Pattern

Tasks are executed through an **Executor pattern**:
- Base class: `BaseExecutor` in `javis_dmc/task_executors/base_executor.py`
- Specific executors: `PickupExecutor`, `ReshelvingExecutor`, `GuidingExecutor`, `CleaningExecutor`, `SortingExecutor`
- Each executor manages sub-state transitions and provides progress feedback
- Executors use callbacks to update DMC state and publish feedback to action clients

### Interface Abstraction Layer

The DMC communicates with subsystems through interface classes in `javis_dmc/interfaces/`:
- `RosDriveInterface`: Navigation and movement control (DDC)
- `RosArmInterface`: Manipulator and gripper control (DAC)
- `RosAIInterface`: Vision and tracking services (DVS)
- `RosGUIInterface`: User interface communication
- `RosVoiceRecognitionInterface`: Voice recognition and TTS

**Mock Interfaces** are available for testing without hardware (see `javis_dmc/mock/`).

### Session Management

Sessions track time-limited interactions:
- `ListeningSession`: Manages voice interaction timeouts
- `DestinationSession`: Manages destination selection for guidance tasks

Sessions are defined in `javis_dmc/sessions/`.

### Action Servers

The DMC exposes ROS 2 action servers for each task:
- `/dobby1/main/pickup_book` → `PickupBook.action`
- `/dobby1/main/reshelving_book` → `ReshelvingBook.action`
- `/dobby1/main/guide_person` → `GuidePerson.action`
- `/dobby1/main/clean_seat` → `CleanSeat.action`
- `/dobby1/main/sorting_shelves` → `RearrangeBook.action`

Actions are defined in the `javis_interfaces` package.

### Administrative Services

The DMC provides ROS 2 services for robot control:
- `admin/set_robot_mode`: Switch between STANDBY and AUTONOMY modes
- `admin/emergency_stop`: Immediately halt all operations
- `admin/resume_navigation`: Resume from emergency stop
- `admin/force_task_result`: Manually complete/abort active tasks
- `admin/set_manual_state`: Directly set state machine values (debug)
- `set_listening_mode`: Activate/deactivate voice recognition mode

## Coding Conventions

**IMPORTANT**: All code must conform to conventions defined in `AGENTS.md`. Key points:

### Naming Conventions

**Python**:
- Modules/packages: `snake_case`
- Classes/Exceptions: `PascalCase`
- Functions/variables: `snake_case`
- Constants: `SCREAMING_SNAKE_CASE`
- Indentation: 4 spaces
- Strings: Single quotes preferred

**ROS 2 Elements**:
- Package/node/topic/service/action names: `snake_case`
- Message/service/action type names: `PascalCase`
- Message field names: `snake_case`
- Message constants: `SCREAMING_SNAKE_CASE`

### Documentation

- Comments: Written in Korean (한국어)
- Python docstrings: Use for all public functions/classes
- Format: Brief summary followed by details if needed

### Code Organization

- Imports: One per line, grouped by standard library → third-party → local
- Spacing: 1 blank line between functions/blocks, 2 blank lines after imports
- Control statements: Always use braces in C++, explicit blocks in Python

### Design Alignment

All code must align with design documents in `/docs`:
- `Architecture/`: Hardware/software architecture
- `DevelopmentPlan/`: Service development plans
- `InterfaceSpecification/`: Inter-service interfaces
- `SequenceDiagram/`: Scenario sequence diagrams
- `StateDefinition/`: State definitions and transitions

## Configuration Files

Configuration is managed through YAML files in `config/`:
- `dmc_params.yaml`: General DMC parameters
- `battery_config.yaml`: Battery thresholds and behavior
- `action_timeouts.yaml`: Timeout values for sessions (listening, destination selection)
- `patrol_routes.yaml`: Waypoint definitions for autonomous patrol
- `test_gui_scenarios.yaml`: Test scenarios for GUI testing
- `mock_method_profiles.yaml`: Mock interface response configurations

## Testing Strategy

The DMC supports **mock interfaces** to enable testing without hardware:
- Set `use_mock_interfaces:=true` when launching
- Individual interfaces can be mocked: `mock_mode_drive`, `mock_mode_arm`, `mock_mode_ai`, `mock_mode_gui`
- Mock values: `-1` (auto), `0` (real), `1` (mock)
- Mock responses can be configured via `/test/set_mock_response` service

Use the Test GUI (`test_gui_node`) to simulate user interactions and task requests during development.

## Battery Management

The `BatteryManager` class (`javis_dmc/battery_manager.py`) simulates battery behavior:
- Drains during tasks/movement (`MOVING_TO_CHARGER`, task states)
- Charges when at charger (`CHARGING` state)
- Idle consumption in other states
- Thresholds trigger automatic charging behavior (warning → `MOVING_TO_CHARGER`, critical → `FORCE_MOVE_TO_CHARGER`)

Battery thresholds are configurable in `config/battery_config.yaml`.

## Key Workflows

### Task Execution Flow
1. RCS (Robot Control Service) sends task goal via action server
2. DMC `_goal_callback` validates state and battery level
3. If accepted, DMC transitions to task state and instantiates executor
4. Executor runs task sequence, updating sub-states and publishing feedback
5. On completion, executor returns outcome
6. DMC publishes result, cleans up, and transitions to post-task state

### Voice Interaction Flow
1. Wake word detected → Voice Recognition Controller calls `set_listening_mode(True)`
2. DMC enters `LISTENING` state, stops movement
3. User voice processed by Voice API Service
4. Intent/task request sent to DMC via dialog callbacks
5. DMC processes intent, optionally starts task
6. Session timeout or completion → DMC exits `LISTENING`, resumes previous state

### Emergency Stop Flow
1. Admin GUI calls `admin/emergency_stop` service
2. DMC cancels active tasks, stops all movement
3. DMC enters `EMERGENCY_STOP` state
4. Resume via `admin/resume_navigation` → returns to `IDLE` or `ROAMING` based on mode

## Integration Points

The DMC integrates with:
- **Robot Control Service (RCS)**: Task assignment and status reporting
- **Voice API Service**: STT/TTS and dialog management
- **Application Service**: Book/user/seat data queries (future)
- **Dobby Drive Controller (DDC)**: Navigation commands
- **Dobby Arm Controller (DAC)**: Manipulation commands
- **Dobby Vision Service (DVS)**: Object detection and tracking
- **GUI Nodes**: User interaction and admin monitoring

Communication protocols are detailed in `docs/Architecture/SoftwareArchitecture.md`.

## State Introspection

For debugging, use the `debug/describe_state_machine` service to get JSON output of:
- Available modes, main states, sub states
- State transition rules
- Current runtime configuration (mock modes, patrol status)

Example:
```bash
ros2 service call /dobby1/debug/describe_state_machine std_srvs/srv/Trigger
```

## Commit Message Format

Follow the template in `.gitmessage`:

```
[TYPE/SCOPE]: Brief description

TYPE: FEAT, FIX, REFACTOR, DOCS, TEST, ENVIR, STYLE
SCOPE: Your feature/module name (or omit if global)
```

Examples:
- `[FEAT/DMC]: Add book pickup feature`
- `[FIX/battery]: Correct charging threshold calculation`
- `[REFACTOR]: Improve message structure`
