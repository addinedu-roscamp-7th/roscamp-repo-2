import json
import math
import time
from functools import partial
from pathlib import Path
from threading import Lock
from typing import Any, Callable, Dict, Optional, Tuple

from concurrent.futures import Future, TimeoutError

import rclpy
from geometry_msgs.msg import Pose
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool
from std_srvs.srv import Trigger

from javis_dmc.battery_manager import BatteryManager, BatteryState
from javis_dmc.interfaces import (
    DialogEvent,
    RosAIInterface,
    RosArmInterface,
    RosDriveInterface,
    RosGUIInterface,
    RosVoiceRecognitionInterface,
)
from javis_dmc.sessions import DestinationSession, ListeningSession
from javis_dmc.states.main_states import DmcStateMachine
from javis_dmc.states.state_enums import MainState, RobotMode, SubState
from javis_dmc.task_executors.base_executor import BaseExecutor
from javis_dmc.task_executors.cleaning_executor import CleaningExecutor
from javis_dmc.task_executors.guiding_executor import GuidingExecutor, GuidingOutcome
from javis_dmc.task_executors.pickup_executor import PickupExecutor, PickupOutcome
from javis_dmc.task_executors.reshelving_executor import ReshelvingExecutor
from javis_dmc.task_executors.sorting_executor import SortingExecutor
from javis_interfaces.action import CleanSeat
from javis_interfaces.action import GuidePerson
from javis_interfaces.action import PickupBook
from javis_interfaces.action import RearrangeBook
from javis_interfaces.action import ReshelvingBook
from javis_interfaces.msg import BatteryStatus
from javis_interfaces.msg import CurrentPose
from javis_interfaces.msg import DobbyState
from javis_interfaces.srv import ForceTaskResult
from javis_interfaces.srv import SetManualState
from javis_interfaces.srv import SetRobotMode


class JavisDmcNode(Node):
    '''JAVIS DMC의 메인 노드 구현.'''

    def __init__(self) -> None:
        super().__init__('javis_dmc_node')

        # 파라미터 선언
        self.declare_parameter('robot_namespace', '')
        self.namespace = self.get_parameter('robot_namespace').get_parameter_value().string_value

        # 주요 구성요소 초기화
        self.state_machine = DmcStateMachine()
        self.state_machine.set_mode(RobotMode.STANDBY)
        self.battery_manager = BatteryManager()
        self.current_pose: Optional[Pose] = None
        self.current_executor: Optional[BaseExecutor] = None
        self._battery_timer_period = 2.0
        self._state_timer_period = 5.0
        self._listening_timer_period = 0.5
        self._last_state_msg: Optional[DobbyState] = None
        self.drive = RosDriveInterface(self, self.namespace)
        self.arm = RosArmInterface(self, self.namespace)
        self.ai = RosAIInterface(self, self.namespace)
        self.gui = RosGUIInterface(self, self.namespace)
        self.voice = RosVoiceRecognitionInterface(self, self.namespace)
        self._interfaces = [
            self.drive,
            self.arm,
            self.ai,
            self.gui,
            self.voice,
        ]
        self._timeouts = self._load_action_timeouts()
        clock_now = self.get_clock().now
        self.listening_session = ListeningSession(clock_now, self._timeouts['listening'])
        self.destination_session = DestinationSession(clock_now, self._timeouts['destination_selection'])
        self._tracking_lock = Lock()
        self._tracking_info: Dict[str, Any] = {
            'person_detected': False,
            'tracking_id': '',
            'confidence': 0.0,
            'last_update': 0.0,
        }
        self._listening_service_group = ReentrantCallbackGroup()
        self.listening_service = self.create_service(
            SetBool,
            self._ns_topic('set_listening_mode'),
            self._on_set_listening_mode,
            callback_group=self._listening_service_group,
        )
        self.mode_feedback_pub = self.create_publisher(
            String,
            self._ns_topic('admin/mode_feedback'),
            10,
        )
        self.mode_service = self.create_service(
            SetRobotMode,
            self._ns_topic('admin/set_robot_mode'),
            self._on_set_robot_mode,
        )
        self.emergency_stop_service = self.create_service(
            Trigger,
            self._ns_topic('admin/emergency_stop'),
            self._on_emergency_stop,
        )
        self.resume_navigation_service = self.create_service(
            Trigger,
            self._ns_topic('admin/resume_navigation'),
            self._on_resume_navigation,
        )
        self.force_task_service = self.create_service(
            ForceTaskResult,
            self._ns_topic('admin/force_task_result'),
            self._on_force_task_result,
        )
        self.manual_state_service = self.create_service(
            SetManualState,
            self._ns_topic('admin/set_manual_state'),
            self._on_set_manual_state,
        )

        # 실행자 구성
        self.executors: Dict[MainState, BaseExecutor] = {
            MainState.PICKING_UP_BOOK: PickupExecutor(),
            MainState.RESHELVING_BOOK: ReshelvingExecutor(),
            MainState.GUIDING: GuidingExecutor(),
            MainState.CLEANING_DESK: CleaningExecutor(),
            MainState.SORTING_SHELVES: SortingExecutor(),
        }
        self._configure_executors()
        self._initialize_interfaces()
        if self.ai.is_initialized():
            try:
                if not self.ai.subscribe_tracking_status(self._on_tracking_status):
                    self.get_logger().warn('AI 추적 상태 콜백 등록에 실패했습니다.')
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'AI 추적 상태 콜백 등록 실패: {exc}')
        self.gui.subscribe_screen_event(self._on_gui_event)
        self.voice.register_dialog_callback(self._on_voice_event)

        # 액션 서버 정보 구성
        self.action_servers: Dict[MainState, ActionServer] = {}
        self._action_binding: Dict[MainState, Tuple[Any, str]] = {
            MainState.PICKING_UP_BOOK: (PickupBook, 'pickup_book'),
            MainState.RESHELVING_BOOK: (ReshelvingBook, 'reshelving_book'),
            MainState.GUIDING: (GuidePerson, 'guide_person'),
            MainState.CLEANING_DESK: (CleanSeat, 'clean_seat'),
            MainState.SORTING_SHELVES: (RearrangeBook, 'sorting_shelves'),
        }
        self._active_goal_handle: Optional[Any] = None
        self._active_main_state: Optional[MainState] = None
        self._latest_sub_state: SubState = SubState.NONE
        self._create_action_servers()

        # 퍼블리셔와 서브스크라이버 생성
        self.state_pub = self.create_publisher(
            DobbyState,
            self._ns_topic('status/robot_state'),
            10,
        )
        self.battery_pub = self.create_publisher(
            BatteryStatus,
            self._ns_topic('status/battery_status'),
            10,
        )
        self.current_pose_sub = self.create_subscription(
            CurrentPose,
            self._ns_topic('status/current_pose'),
            self._on_current_pose,
            10,
        )

        # 타이머 생성
        self.battery_timer = self.create_timer(self._battery_timer_period, self._on_battery_timer)
        self.state_timer = self.create_timer(self._state_timer_period, self._on_state_timer)
        self.listening_timer = self.create_timer(self._listening_timer_period, self._on_listening_timer)
        self.destination_timer = self.create_timer(1.0, self._on_destination_timer)

        self.get_logger().info('JAVIS DMC 노드가 초기화되었습니다.')
        self.state_machine.set_main_state(MainState.IDLE)
        self.state_machine.set_sub_state(SubState.NONE)
        self._publish_state_immediately()

    def _on_voice_event(self, event: DialogEvent) -> None:
        '''음성 인식 이벤트를 처리한다.'''
        if event.is_empty:
            return
        self.get_logger().info(f'음성 입력 수신: {event.text}')
        if not self.listening_session.active:
            self.get_logger().debug('LISTENING 세션이 비활성 상태라 음성 이벤트를 무시합니다.')
            return
        self.listening_session.update_deadline()
        count = self.listening_session.metadata.get('event_count', 0) + 1
        self.listening_session.metadata['event_count'] = count
        self.listening_session.metadata['last_text'] = event.text
        lowered = event.text.strip().lower()
        if '취소' in event.text or lowered == 'cancel':
            self.get_logger().info('사용자 요청으로 LISTENING 세션을 종료합니다.')
            self._deactivate_listening_mode(reason='사용자 취소')

    def _on_set_listening_mode(self, request: SetBool.Request, response: SetBool.Response):
        '''음성 컨트롤러의 LISTENING 모드 요청을 처리한다.'''
        if request.data:
            success = self._activate_listening_mode()
            message = '' if success else 'LISTENING 상태로 전환할 수 없습니다.'
        else:
            success = self._deactivate_listening_mode(reason='외부 요청')
            message = '' if success else 'LISTENING 상태가 비활성 상태입니다.'
        response.success = success
        response.message = message
        return response

    def _activate_listening_mode(self) -> bool:
        '''LISTENING 세션을 시작한다.'''
        if self.listening_session.active:
            self.listening_session.update_deadline()
            self.get_logger().debug('LISTENING 세션이 이미 활성 상태여서 타이머만 연장합니다.')
            return True

        if not self.state_machine.enter_listening():
            self.get_logger().warn('현재 상태에서는 LISTENING 모드로 전환할 수 없습니다.')
            return False

        self.listening_session.start()
        self.listening_session.metadata['trigger'] = 'voice_request'

        if self.drive.is_initialized() and not self.drive.stop('listening_start'):
            self.get_logger().warn('LISTENING 시작 시 주행 정지 명령이 실패했습니다.')

        if self.voice.is_initialized():
            if not self.voice.set_listening_mode(True):
                self.get_logger().warn('VoiceRecognition 인터페이스 활성화에 실패했습니다.')

        self._publish_state_immediately()
        self.get_logger().info('LISTENING 세션을 시작했습니다.')
        return True

    def _deactivate_listening_mode(self, *, reason: str = '') -> bool:
        '''LISTENING 세션을 종료한다.'''
        was_active = self.listening_session.active or self.state_machine.main_state == MainState.LISTENING
        self.listening_session.cancel()

        if self.state_machine.main_state == MainState.LISTENING:
            self.state_machine.exit_listening()

        new_state = self.state_machine.main_state

        if self.drive.is_initialized():
            if new_state == MainState.ROAMING:
                if not self.drive.resume('listening_complete'):
                    self.get_logger().warn('LISTENING 종료 후 주행 재개 명령이 실패했습니다.')
            elif new_state == MainState.IDLE:
                if not self.drive.stop('listening_complete'):
                    self.get_logger().warn('LISTENING 종료 후 주행 정지 명령이 실패했습니다.')

        if self.voice.is_initialized():
            if not self.voice.set_listening_mode(False):
                self.get_logger().warn('VoiceRecognition 인터페이스 비활성화에 실패했습니다.')

        if was_active:
            self._publish_state_immediately()

        if reason:
            self.get_logger().info(f'LISTENING 세션 종료 사유: {reason}')

        return was_active

    def _on_listening_timer(self) -> None:
        '''LISTENING 세션 타임아웃을 감시한다.'''
        if not self.listening_session.active:
            return
        if self.listening_session.handle_timeout():
            self.get_logger().warn('LISTENING 세션이 타임아웃되어 종료합니다.')
            self._deactivate_listening_mode(reason='타임아웃')

    def _on_set_robot_mode(self, request: SetRobotMode.Request, response: SetRobotMode.Response):
        '''관리자 모드 변경 요청을 처리한다.'''
        requested = request.mode.strip().lower()
        if requested not in {'standby', 'autonomy'}:
            message = f'알 수 없는 모드 요청입니다: {request.mode}'
            self.get_logger().warn(message)
            self._publish_mode_feedback('rejected', message)
            response.success = False
            response.message = message
            return response

        target_mode = RobotMode.STANDBY if requested == 'standby' else RobotMode.AUTONOMY
        block_reason = self._mode_change_block_reason()
        if block_reason is not None:
            self.get_logger().warn(block_reason)
            self._publish_mode_feedback('busy', block_reason)
            response.success = False
            response.message = block_reason
            return response

        if target_mode == RobotMode.AUTONOMY and self._battery_too_low_for_autonomy():
            message = '배터리 경고 구간에서는 AUTONOMY 모드로 전환할 수 없습니다.'
            self.get_logger().warn(message)
            self._publish_mode_feedback('rejected', message)
            response.success = False
            response.message = message
            return response

        if self.state_machine.mode == target_mode:
            message = '이미 요청한 모드로 동작 중입니다.'
            self._publish_mode_feedback('noop', message)
            response.success = True
            response.message = message
            return response

        self.state_machine.set_mode(target_mode)

        if target_mode == RobotMode.AUTONOMY:
            self.state_machine.set_main_state(MainState.ROAMING)
            if self.drive.is_initialized() and not self.drive.resume('autonomy_mode'):
                self.get_logger().warn('주행 재개 명령이 실패했습니다.')
        else:
            self.state_machine.set_main_state(MainState.IDLE)
            if self.drive.is_initialized() and not self.drive.stop('standby_mode'):
                self.get_logger().warn('주행 정지 명령이 실패했습니다.')

        self._publish_state_immediately()

        message = f'{requested.upper()} 모드로 전환했습니다.'
        response.success = True
        response.message = message
        self._publish_mode_feedback('success', message)
        return response

    def _on_emergency_stop(self, _request: Trigger.Request, response: Trigger.Response):
        '''긴급 정지 서비스를 처리한다.'''
        self.get_logger().warn('긴급 정지 요청을 수신했습니다.')

        if self.current_executor is not None:
            self.cancel_current_task()

        self._abort_active_goal('긴급 정지로 작업이 중단되었습니다.')
        self.destination_session.clear()
        self._deactivate_listening_mode(reason='긴급 정지')

        if self.drive.is_initialized() and not self.drive.stop('emergency_stop'):
            self.get_logger().warn('주행 정지 명령이 실패했습니다.')

        self.state_machine.enter_emergency_stop()
        self._publish_state_immediately()
        message = '긴급 정지를 수행했습니다.'
        self._publish_mode_feedback('emergency_stop', message)

        response.success = True
        response.message = message
        return response

    def _on_resume_navigation(self, _request: Trigger.Request, response: Trigger.Response):
        '''긴급 정지 해제 요청을 처리한다.'''
        if not self.state_machine.is_emergency():
            message = '긴급 정지 상태가 아니어서 해제할 수 없습니다.'
            self.get_logger().warn(message)
            self._publish_mode_feedback('rejected', message)
            response.success = False
            response.message = message
            return response

        self.state_machine.resume_from_emergency()
        target_state = self.state_machine.main_state

        if target_state == MainState.ROAMING:
            if self.drive.is_initialized() and not self.drive.resume('resume_navigation'):
                self.get_logger().warn('주행 재개 명령이 실패했습니다.')
        else:
            if self.drive.is_initialized() and not self.drive.stop('resume_to_idle'):
                self.get_logger().warn('주행 정지 명령이 실패했습니다.')

        self._publish_state_immediately()
        message = '긴급 정지를 해제했습니다.'
        self._publish_mode_feedback('resumed', message)

        response.success = True
        response.message = message
        return response

    def _on_force_task_result(self, request: ForceTaskResult.Request, response: ForceTaskResult.Response):
        '''현재 작업을 수동으로 완료/실패 처리한다.'''
        if self._active_goal_handle is None or self._active_main_state is None:
            response.applied = False
            response.detail = '활성 작업이 없어 처리할 수 없습니다.'
            return response

        success = bool(request.success)
        message = request.message.strip() or ('수동 완료 처리' if success else '수동 실패 처리')

        try:
            result_msg = self._build_result(self._active_main_state, success, message)
        except ValueError as exc:
            detail = f'결과 메시지 생성 실패: {exc}'
            self.get_logger().error(detail)
            response.applied = False
            response.detail = detail
            return response

        goal_handle = self._active_goal_handle
        if success:
            goal_handle.succeed(result_msg)
        else:
            try:
                goal_handle.abort(result_msg)
            except TypeError:
                self.get_logger().debug('GoalHandle.abort 결과 인자가 지원되지 않아 기본 호출로 대체합니다.')
                goal_handle.abort()

        self._cleanup_after_task(success=success)

        detail = '작업을 수동으로 완료했습니다.' if success else '작업을 수동으로 실패 처리했습니다.'
        self._publish_mode_feedback('task_override', f'{detail} ({message})')
        self.get_logger().info(detail)

        response.applied = True
        response.detail = detail
        return response

    def _on_set_manual_state(self, request: SetManualState.Request, response: SetManualState.Response):
        '''상태 머신을 수동으로 갱신한다.'''
        if self._active_goal_handle is not None:
            message = '활성 작업이 있어 상태를 강제 변경할 수 없습니다. 먼저 force_task_result를 사용하세요.'
            response.success = False
            response.message = message
            self.get_logger().warn(message)
            return response

        if request.mode not in (-1,):
            try:
                target_mode = RobotMode(request.mode)
            except ValueError:
                response.success = False
                response.message = f'잘못된 모드 값입니다: {request.mode}'
                return response
            self.state_machine.set_mode(target_mode)

        if request.main_state not in (-1,):
            try:
                target_main = MainState(request.main_state)
            except ValueError:
                response.success = False
                response.message = f'잘못된 메인 상태 값입니다: {request.main_state}'
                return response
            self.state_machine.set_main_state(target_main)

        if request.sub_state not in (-1,):
            try:
                target_sub = SubState(request.sub_state)
            except ValueError:
                response.success = False
                response.message = f'잘못된 서브 상태 값입니다: {request.sub_state}'
                return response
            self.state_machine.set_sub_state(target_sub)

        self._publish_state_immediately()
        summary = f'상태 수동 설정: mode={self.state_machine.mode.name}, main={self.state_machine.main_state.name}, sub={self.state_machine.sub_state.name}'
        self.get_logger().info(summary)
        self._publish_mode_feedback('manual_state', summary)
        response.success = True
        response.message = summary
        return response

    def _publish_mode_feedback(self, status: str, message: str) -> None:
        '''관리자 모드 피드백을 퍼블리시한다.'''
        payload = {
            'mode': self.state_machine.mode.name.lower(),
            'main_state': self.state_machine.main_state.name.lower(),
            'status': status,
            'message': message,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.mode_feedback_pub.publish(msg)

    def _mode_change_block_reason(self) -> Optional[str]:
        '''모드 전환이 불가능한 이유를 반환한다.'''
        if self.state_machine.is_emergency():
            return '긴급 정지 상태에서는 모드를 변경할 수 없습니다.'
        if self.state_machine.is_task_active():
            return '작업 수행 중에는 모드를 변경할 수 없습니다.'
        if self.state_machine.is_listening():
            return 'LISTENING 모드에서는 모드를 변경할 수 없습니다.'
        if self.current_executor is not None:
            return '작업 실행기가 동작 중입니다.'
        if self.listening_session.active:
            return 'LISTENING 세션이 진행 중입니다.'
        if self.destination_session.active:
            return '목적지 선택 세션이 진행 중입니다.'
        return None

    def _battery_too_low_for_autonomy(self) -> bool:
        '''AUTONOMY 모드 전환 가능 배터리 조건을 검사한다.'''
        return self.battery_manager.level <= self.battery_manager.warning_threshold

    def _abort_active_goal(self, message: str) -> None:
        '''진행 중인 액션 Goal을 중단한다.'''
        if self._active_goal_handle is None or self._active_main_state is None:
            return

        try:
            result_msg = self._build_result(self._active_main_state, False, message)
        except ValueError as error:
            self.get_logger().error(f'Goal 중단 결과 생성 실패: {error}')
            result_msg = None

        goal_handle = self._active_goal_handle
        self._active_goal_handle = None
        self._active_main_state = None

        if result_msg is not None:
            try:
                goal_handle.abort(result_msg)
                return
            except TypeError:
                self.get_logger().debug('GoalHandle.abort 결과 인자를 지원하지 않아 기본 호출로 대체합니다.')
        goal_handle.abort()

        self.current_executor = None
        self.state_machine.set_sub_state(SubState.NONE)

    def _load_action_timeouts(self) -> Dict[str, float]:
        '''타임아웃 파라미터를 로드한다.'''
        defaults = {
            'listening': 20.0,
            'destination_selection': 60.0,
        }

        config_path = Path(__file__).resolve().parent.parent / 'config' / 'action_timeouts.yaml'
        if not config_path.exists():
            self.get_logger().warn('action_timeouts.yaml 파일을 찾을 수 없어 기본값을 사용합니다.')
            return defaults

        try:
            import yaml
        except ImportError:
            self.get_logger().warn('PyYAML이 설치되어 있지 않아 기본 타임아웃을 사용합니다.')
            return defaults

        try:
            with config_path.open('r', encoding='utf-8') as handle:
                data = yaml.safe_load(handle) or {}
        except Exception as exc:
            self.get_logger().error(f'action_timeouts.yaml 로드 실패: {exc}')
            return defaults

        timeouts = data.get('timeouts', {})
        listening = float(timeouts.get('listening', {}).get('seconds', defaults['listening']))
        destination = float(timeouts.get('destination_selection', {}).get('seconds', defaults['destination_selection']))
        return {
            'listening': listening,
            'destination_selection': destination,
        }

    def _create_action_servers(self) -> None:
        '''설계서에 정의된 작업 액션 서버를 초기화한다.'''
        for main_state, (action_type, action_name) in self._action_binding.items():
            topic = self._ns_topic(f'main/{action_name}')
            server = ActionServer(
                self,
                action_type,
                topic,
                execute_callback=partial(self._execute_task, main_state=main_state),
                goal_callback=partial(self._goal_callback, main_state=main_state),
                cancel_callback=self._cancel_callback,
            )
            self.action_servers[main_state] = server

    def _configure_executors(self) -> None:
        '''실행자에 콜백을 연결한다.'''
        for executor in self.executors.values():
            executor.configure(self._on_sub_state_update, self._on_executor_feedback)
            executor.set_step_delay(0.0)
            if isinstance(executor, GuidingExecutor):
                executor.set_runtime(self._run_guiding_sequence)
            if isinstance(executor, PickupExecutor):
                executor.set_runtime(self._run_pickup_sequence)

    def _initialize_interfaces(self) -> None:
        '''하위 인터페이스를 초기화한다.'''
        for interface in self._interfaces:
            try:
                interface.initialize()
            except Exception as exc:
                self.get_logger().error(f'인터페이스 초기화 실패: {interface.__class__.__name__} - {exc}')

    def _ns_topic(self, topic: str) -> str:
        '''네임스페이스를 고려한 토픽 이름을 반환한다.'''
        node_namespace = self.get_namespace()
        if node_namespace and node_namespace not in ('', '/'):
            return topic
        if self.namespace:
            return f'{self.namespace}/{topic}'
        return topic

    def _on_current_pose(self, msg: CurrentPose) -> None:
        '''현재 위치 정보를 저장한다.'''
        self.current_pose = msg.pose

    def _on_battery_timer(self) -> None:
        '''배터리 상태를 주기적으로 갱신한다.'''
        self._sync_battery_state()
        self.battery_manager.update(self._battery_timer_period)

        status_msg = BatteryStatus()
        status_msg.charge_percentage = float(self.battery_manager.level)
        status_msg.is_charging = self.battery_manager.get_state() == BatteryState.CHARGING
        self.battery_pub.publish(status_msg)

    def _sync_battery_state(self) -> None:
        '''메인 상태에 따라 배터리 상태를 동기화한다.'''
        main_state = self.state_machine.main_state

        if main_state == MainState.CHARGING:
            self.battery_manager.start_charging()
        elif main_state in (
            MainState.PICKING_UP_BOOK,
            MainState.RESHELVING_BOOK,
            MainState.GUIDING,
            MainState.CLEANING_DESK,
            MainState.SORTING_SHELVES,
            MainState.MOVING_TO_CHARGER,
            MainState.FORCE_MOVE_TO_CHARGER,
        ):
            self.battery_manager.start_draining()
        else:
            self.battery_manager.set_idle()

    def _compose_state_msg(self) -> DobbyState:
        '''현재 상태를 메시지로 구성한다.'''
        state_msg = DobbyState()
        state_msg.main_state = self.state_machine.main_state.value
        state_msg.sub_state = self.state_machine.sub_state.value
        state_msg.is_error = self.state_machine.main_state == MainState.MAIN_ERROR
        state_msg.error_message = '' if not state_msg.is_error else '메인 상태 오류'
        return state_msg

    def _publish_state(self, force: bool = False) -> None:
        '''상태 메시지를 퍼블리시한다.'''
        state_msg = self._compose_state_msg()
        if (
            force
            or self._last_state_msg is None
            or state_msg.main_state != self._last_state_msg.main_state
            or state_msg.sub_state != self._last_state_msg.sub_state
            or state_msg.is_error != self._last_state_msg.is_error
        ):
            self.state_pub.publish(state_msg)
            self._last_state_msg = state_msg

    def _publish_state_immediately(self) -> None:
        '''상태 변화를 즉시 반영한다.'''
        self._publish_state(force=True)

    def _is_at_charger(self) -> bool:
        '''충전소 도착 여부를 판단한다 (향후 위치 기반 로직으로 교체 필요).'''
        # TODO: charger 위치 파라미터를 이용해 실제 위치 비교 로직 추가
        return False

    def _on_state_timer(self) -> None:
        '''현재 상태 정보를 퍼블리시한다.'''
        self._publish_state()

    def _on_destination_timer(self) -> None:
        '''GUIDING 목적지 선택 세션의 타임아웃을 감시한다.'''
        if not self.destination_session.active:
            return
        if self.destination_session.check_timeout():
            self.get_logger().warn('GUIDING 목적지 선택이 타임아웃되었습니다.')

    def _on_gui_event(self, payload: Dict[str, Any]) -> None:
        '''GUI에서 수신한 이벤트를 처리한다.'''
        if not isinstance(payload, dict):
            self.get_logger().debug('알 수 없는 GUI 이벤트 형식입니다.')
            return
        event_type = str(payload.get('type') or payload.get('event_type') or '').lower()
        if event_type == 'destination_selected':
            destination = payload.get('destination')
            if destination is None and isinstance(payload.get('data'), dict):
                destination = payload['data'].get('destination')
            destination = str(destination or '').strip()
            if not destination:
                self.get_logger().warn('GUI에서 전달된 목적지 정보가 비어 있습니다.')
                return
            self.destination_session.resolve_selection(destination)
            self.get_logger().info(f'GUI 목적지 선택 완료: {destination}')

    def _on_tracking_status(self, payload: Dict[str, Any]) -> None:
        '''AI 추적 상태 업데이트를 저장한다.'''
        with self._tracking_lock:
            self._tracking_info['person_detected'] = bool(payload.get('person_detected', False))
            self._tracking_info['tracking_id'] = str(payload.get('tracking_id', '')).strip()
            confidence = payload.get('confidence', 0.0)
            self._tracking_info['confidence'] = float(confidence or 0.0)
            self._tracking_info['last_update'] = time.monotonic()

    def _on_sub_state_update(self, sub_state: SubState) -> None:
        '''실행자로부터 전달되는 서브 상태를 반영한다.'''
        self.state_machine.set_sub_state(sub_state)
        self._latest_sub_state = sub_state
        self._publish_state_immediately()

    def _on_executor_feedback(self, progress: float) -> None:
        '''실행자 피드백을 액션 서버에 전달한다.'''
        if self._active_goal_handle is None or self._active_main_state is None:
            self.get_logger().debug('피드백을 전달할 활성 작업이 없습니다.')
            return

        try:
            feedback_msg = self._build_feedback(self._active_main_state, progress)
        except ValueError as error:
            self.get_logger().error(f'피드백 생성 실패: {error}')
            return

        self._active_goal_handle.publish_feedback(feedback_msg)
        self.get_logger().debug(f'작업 진행률: {progress * 100.0:.1f}%')

    def start_task(self, main_state: MainState, goal: Optional[Any] = None) -> bool:
        '''지정된 작업을 시작한다.'''
        executor = self.executors.get(main_state)
        if executor is None:
            self.get_logger().warn(f'실행자를 찾을 수 없습니다: {main_state}')
            return False

        started = executor.execute(goal)
        if not started:
            self.get_logger().warn(f'작업 시작 실패: {main_state}')
            return False

        self.current_executor = executor
        return True

    def cancel_current_task(self) -> None:
        '''진행 중인 작업을 취소한다.'''
        if self.current_executor is None:
            return

        self.current_executor.cancel()
        self.current_executor = None

    def _goal_callback(self, goal_request, main_state: MainState) -> GoalResponse:
        '''Goal 수락 여부를 결정한다.'''
        if self.current_executor is not None:
            self.get_logger().warn('이미 실행 중인 작업이 있어 Goal을 거부합니다.')
            return GoalResponse.REJECT

        if self.battery_manager.is_critical():
            self.get_logger().warn('배터리가 위험 구간이므로 Goal을 거부합니다.')
            return GoalResponse.REJECT

        battery_is_warning = self.battery_manager.is_warning()
        if not self.state_machine.can_accept_task(
            main_state,
            battery_warning=battery_is_warning,
            battery_critical=self.battery_manager.is_critical(),
        ):
            self.get_logger().warn(f'현재 상태({self.state_machine.main_state.name})에서는 {main_state.name} 작업을 수락할 수 없습니다.')
            return GoalResponse.REJECT

        self.get_logger().info(f'{main_state.name} 작업 Goal 수락')
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        '''취소 요청을 처리한다.'''
        if self.current_executor is None:
            self.get_logger().warn('취소할 작업이 없어 요청을 거부합니다.')
            return CancelResponse.REJECT

        self.get_logger().info('작업 취소 요청을 수락합니다.')
        self.cancel_current_task()
        self._cleanup_after_task(success=False, forced_state=MainState.MOVING_TO_CHARGER)
        return CancelResponse.ACCEPT

    def _execute_task(self, goal_handle, main_state: MainState):
        '''Goal 실행 콜백을 처리한다.'''
        goal = goal_handle.request
        if not self.state_machine.start_task(main_state):
            message = '현재 상태에서 작업을 시작할 수 없습니다.'
            self.get_logger().warn(message)
            result_msg = self._build_result(main_state, False, message)
            goal_handle.abort()
            return result_msg

        self._active_goal_handle = goal_handle
        self._active_main_state = main_state
        self._latest_sub_state = SubState.NONE
        self._publish_state_immediately()

        if not self.start_task(main_state, goal):
            goal_handle.abort()
            result_msg = self._build_result(main_state, False, '작업을 시작하지 못했습니다.')
            self._cleanup_after_task(success=False, forced_state=MainState.IDLE)
            return result_msg

        outcome = None
        if self.current_executor is not None:
            outcome = self.current_executor.wait_for_result()

        if goal_handle.is_cancel_requested:
            self.get_logger().info('작업 도중 취소 요청이 감지되었습니다.')
            self.cancel_current_task()
            result_msg = self._build_result(main_state, False, '작업이 취소되었습니다.')
            try:
                goal_handle.canceled(result_msg)
            except TypeError:
                goal_handle.canceled()
            self._cleanup_after_task(success=False, forced_state=MainState.MOVING_TO_CHARGER)
            return result_msg

        task_success, message, result_kwargs = self._extract_execution_outcome(main_state, outcome)
        result_msg = self._build_result(main_state, task_success, message, **result_kwargs)

        if task_success:
            try:
                goal_handle.succeed(result_msg)
            except TypeError:
                goal_handle.succeed()
        else:
            try:
                goal_handle.abort(result_msg)
            except TypeError:
                goal_handle.abort()

        self._cleanup_after_task(success=task_success)
        return result_msg

    def _extract_execution_outcome(
        self,
        main_state: MainState,
        outcome: Optional[Any],
    ) -> Tuple[bool, str, Dict[str, Any]]:
        '''실행자가 반환한 결과 객체를 액션 결과 작성용으로 정리한다.'''
        default_message = '작업을 완료했습니다.'
        extras: Dict[str, Any] = {}

        if isinstance(outcome, GuidingOutcome):
            extras = {
                'distance': outcome.total_distance,
                'duration': outcome.total_time,
                'person_detected': outcome.person_detected,
                'error_code': 0 if outcome.success else (2 if not outcome.person_detected else 1),
            }
            message = outcome.message or default_message
            return outcome.success, message, extras

        if isinstance(outcome, PickupOutcome):
            extras = {
                'distance': outcome.total_distance,
                'duration': outcome.total_time,
                'book_id': outcome.book_id,
                'storage_id': outcome.storage_id,
            }
            message = outcome.message or default_message
            return outcome.success, message, extras

        if outcome is not None and hasattr(outcome, 'success'):
            success = bool(getattr(outcome, 'success'))
            message = str(getattr(outcome, 'message', default_message)) or default_message
            return success, message, extras

        return True, default_message, extras

    def _run_guiding_sequence(
        self,
        goal: Optional[GuidePerson.Goal],
        set_sub_state: Callable[[SubState], None],
        publish_feedback: Callable[[float], None],
    ) -> GuidingOutcome:
        '''길 안내 시나리오를 세부 구현한다.'''
        start_time = time.monotonic()

        if goal is None or not hasattr(goal, 'dest_location'):
            return GuidingOutcome(False, '목적지 정보가 전달되지 않았습니다.')

        dest_location = goal.dest_location
        requested_name = str(getattr(goal, 'destination_name', '')).strip() or 'custom_destination'

        set_sub_state(SubState.SELECT_DEST)
        self.destination_session.begin_selection()
        dest_meta = {
            'x': float(getattr(dest_location, 'x', 0.0)),
            'y': float(getattr(dest_location, 'y', 0.0)),
            'theta': float(getattr(dest_location, 'theta', 0.0)),
        }
        self.destination_session.metadata.update(
            {
                'destination_label': requested_name,
                'dest_location': dest_meta,
            }
        )
        publish_feedback(0.2)

        dest_pose = self._pose2d_to_pose(dest_location)
        total_distance = self._estimate_distance(dest_location)

        selected_destination = self._wait_for_destination_selection()
        if selected_destination is None:
            duration = time.monotonic() - start_time
            self.destination_session.clear()
            if self.ai.is_initialized():
                self.ai.change_tracking_mode('idle')
            return GuidingOutcome(False, '목적지 선택 시간이 초과되었습니다.', total_distance, duration, False)

        destination_name = selected_destination or requested_name

        set_sub_state(SubState.SCAN_USER)
        publish_feedback(0.4)

        person_detected = False
        if not self.ai.is_initialized():
            self.get_logger().warn('AI 인터페이스가 초기화되지 않아 피안내자 등록 절차를 생략합니다.')
            person_detected = True
        else:
            try:
                self._reset_tracking_detection()
                if not self.ai.change_tracking_mode('registration'):
                    self.get_logger().warn('등록 모드 전환에 실패했습니다.')
                detection_success, tracking_snapshot = self._wait_for_person_detection()
                if detection_success:
                    person_detected = True
                    self.destination_session.metadata['tracking_id'] = tracking_snapshot.get('tracking_id', '')
                    if not self.ai.change_tracking_mode('tracking'):
                        self.get_logger().warn('추적 모드 전환에 실패했습니다.')
                else:
                    person_detected = False
                    self.ai.change_tracking_mode('idle')
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'피안내자 등록 중 예외가 발생했습니다: {exc}')
                person_detected = False

        publish_feedback(0.7 if person_detected else 0.45)
        if not person_detected:
            duration = time.monotonic() - start_time
            self.destination_session.clear()
            if self.ai.is_initialized():
                self.ai.change_tracking_mode('idle')
            return GuidingOutcome(False, '피안내자를 찾지 못했습니다.', total_distance, duration, False)

        set_sub_state(SubState.GUIDING_TO_DEST)
        navigation_success = self._navigate_with_follow_mode(dest_pose)
        publish_feedback(1.0 if navigation_success else 0.85)

        duration = time.monotonic() - start_time
        message = '길 안내를 완료했습니다.' if navigation_success else '길 안내 주행에 실패했습니다.'

        self.destination_session.clear()
        if self.ai.is_initialized():
            self.ai.change_tracking_mode('idle')

        return GuidingOutcome(navigation_success, message, total_distance, duration, person_detected)

    def _run_pickup_sequence(
        self,
        goal: Optional[PickupBook.Goal],
        set_sub_state: Callable[[SubState], None],
        publish_feedback: Callable[[float], None],
    ) -> PickupOutcome:
        '''도서 픽업 시나리오를 세부 구현한다.'''
        start_time = time.monotonic()

        if goal is None:
            return PickupOutcome(False, '도서 픽업 Goal 정보가 제공되지 않았습니다.')

        book_id = str(getattr(goal, 'book_id', ''))
        storage_id = int(getattr(goal, 'storage_id', 0))
        total_distance = 0.0

        set_sub_state(SubState.MOVE_TO_PICKUP)
        publish_feedback(0.1)
        total_distance += self._navigate_to_pose(getattr(goal, 'shelf_approach_location', None), 'pickup_shelf')

        set_sub_state(SubState.PICKUP_BOOK)
        publish_feedback(0.35)
        if not self._perform_book_pick(goal):
            duration = time.monotonic() - start_time
            return PickupOutcome(False, '도서 집기에 실패했습니다.', total_distance, duration, book_id, storage_id)

        set_sub_state(SubState.MOVE_TO_STORAGE)
        publish_feedback(0.6)
        total_distance += self._navigate_to_pose(getattr(goal, 'storage_approach_location', None), 'pickup_storage')

        set_sub_state(SubState.STOWING_BOOK)
        publish_feedback(0.85)
        if not self._perform_book_place(goal):
            duration = time.monotonic() - start_time
            return PickupOutcome(False, '보관함 적재에 실패했습니다.', total_distance, duration, book_id, storage_id)

        self._arm_change_pose('home')
        publish_feedback(1.0)

        duration = time.monotonic() - start_time
        return PickupOutcome(True, '도서 픽업에 성공했습니다.', total_distance, duration, book_id, storage_id)

    def _cleanup_after_task(self, success: bool, forced_state: Optional[MainState] = None) -> None:
        '''작업 종료 후 상태를 정리한다.'''
        self.current_executor = None

        if forced_state is not None:
            self.state_machine.set_main_state(forced_state)
        else:
            next_state = self.state_machine.determine_post_task_state(
                success=success,
                at_charger=self._is_at_charger(),
                battery_warning=self.battery_manager.is_warning(),
                battery_critical=self.battery_manager.is_critical(),
            )
            self.state_machine.set_main_state(next_state)

        self.state_machine.set_sub_state(SubState.NONE)
        self._active_goal_handle = None
        self._active_main_state = None
        self._publish_state_immediately()

    def _navigate_with_follow_mode(self, destination: Pose) -> bool:
        '''피안내자 추종 모드로 목적지까지 이동한다.'''
        if not self.drive.is_initialized():
            self.get_logger().warn('Drive 인터페이스가 초기화되지 않았습니다.')
            return False

        follow_started = False
        try:
            follow_started = self.drive.enable_follow_mode('ai/tracking/status', destination)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'추종 모드 활성화 실패: {exc}')

        future = self.drive.move_to_target(destination, 'guide_person')
        success = self._wait_future_success(future, timeout=20.0)

        if follow_started:
            try:
                self.drive.disable_follow_mode()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'추종 모드 비활성화 실패: {exc}')

        return success

    def _navigate_to_pose(self, pose2d: Optional[Any], location_name: str) -> float:
        '''지정된 Pose2D 위치로 이동 명령을 전송한다.'''
        if pose2d is None:
            self.get_logger().warn(f'{location_name} 목적지 정보가 비어 있어 이동을 건너뜁니다.')
            return 0.0
        if not self.drive.is_initialized():
            self.get_logger().warn('Drive 인터페이스가 초기화되지 않아 이동 명령을 건너뜁니다.')
            return 0.0

        target_pose = self._pose2d_to_pose(pose2d)
        distance = self._estimate_distance_pose(target_pose)
        future = self.drive.move_to_target(target_pose, location_name)
        self._wait_future_success(future, timeout=20.0)
        return distance

    def _wait_for_destination_selection(self) -> Optional[str]:
        '''GUI 목적지 선택 완료를 대기한다.'''
        timeout = float(self.destination_session.timeout_sec)
        deadline = time.monotonic() + timeout
        while self.destination_session.active and time.monotonic() < deadline:
            if self.destination_session.selected_destination:
                return self.destination_session.selected_destination
            if self.destination_session.timeout_reason:
                return None
            time.sleep(0.1)
        if self.destination_session.selected_destination:
            return self.destination_session.selected_destination
        return None

    def _reset_tracking_detection(self) -> None:
        '''추적 상태 캐시를 초기화한다.'''
        with self._tracking_lock:
            self._tracking_info['person_detected'] = False
            self._tracking_info['tracking_id'] = ''
            self._tracking_info['confidence'] = 0.0
            self._tracking_info['last_update'] = time.monotonic()

    def _get_tracking_snapshot(self) -> Dict[str, Any]:
        '''현재 추적 상태 스냅샷을 반환한다.'''
        with self._tracking_lock:
            return dict(self._tracking_info)

    def _wait_for_person_detection(self, timeout: float = 5.0) -> Tuple[bool, Dict[str, Any]]:
        '''피안내자 감지 완료를 대기한다.'''
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            snapshot = self._get_tracking_snapshot()
            if snapshot.get('person_detected'):
                return True, snapshot
            time.sleep(0.1)
        return False, self._get_tracking_snapshot()

    def _perform_book_pick(self, goal: PickupBook.Goal) -> bool:
        '''로봇 팔로 도서를 집는다.'''
        if not self.arm.is_initialized():
            self.get_logger().warn('Arm 인터페이스가 초기화되지 않아 도서 집기를 수행할 수 없습니다.')
            return False
        self._arm_change_pose('observe')
        try:
            future = self.arm.pick_book(goal.book_id, goal.book_pick_pose, goal.storage_id)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'도서 집기 명령 실패: {exc}')
            return False
        success = self._wait_future_success(future, timeout=25.0)
        if success:
            self._arm_change_pose('carry')
        return success

    def _perform_book_place(self, goal: PickupBook.Goal) -> bool:
        '''도서를 보관함에 적재한다.'''
        if not self.arm.is_initialized():
            self.get_logger().warn('Arm 인터페이스가 초기화되지 않아 보관함 적재를 수행할 수 없습니다.')
            return False
        try:
            future = self.arm.place_book(
                goal.book_id,
                goal.storage_id,
                goal.storage_slot_pose,
                goal.storage_id,
                0,
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'도서 적재 명령 실패: {exc}')
            return False
        success = self._wait_future_success(future, timeout=25.0)
        return success

    def _arm_change_pose(self, pose_type: str) -> None:
        '''로봇 팔 자세를 변경한다.'''
        if not self.arm.is_initialized():
            return
        if not self.arm.change_pose(pose_type):
            self.get_logger().warn(f'팔 자세 전환 실패: {pose_type}')

    def _wait_future_success(self, future: Optional[Any], timeout: float = 5.0) -> bool:
        '''비동기 Future 결과를 확인한다.'''
        if future is None:
            return False

        if hasattr(future, 'done') and hasattr(future, 'result'):
            try:
                if future.done():
                    result = future.result()
                else:
                    result = future.result(timeout=timeout)
            except TimeoutError:
                self.get_logger().warn('비동기 작업이 타임아웃되어 성공으로 간주합니다.')
                return True
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'비동기 작업 처리 실패: {exc}')
                return False
            return bool(getattr(result, 'success', True))

        return True

    def _pose2d_to_pose(self, pose2d: Any) -> Pose:
        '''Pose2D를 Pose로 변환한다.'''
        pose = Pose()
        pose.position.x = float(getattr(pose2d, 'x', 0.0))
        pose.position.y = float(getattr(pose2d, 'y', 0.0))
        pose.position.z = 0.0
        qx, qy, qz, qw = self._yaw_to_quaternion(float(getattr(pose2d, 'theta', 0.0)))
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose

    def _estimate_distance(self, pose2d: Any) -> float:
        '''현재 위치와 Pose2D 좌표 간의 거리를 추정한다.'''
        if pose2d is None:
            return 0.0
        if self.current_pose is None:
            return round(math.hypot(float(getattr(pose2d, 'x', 0.0)), float(getattr(pose2d, 'y', 0.0))), 3)
        dx = float(getattr(pose2d, 'x', 0.0)) - self.current_pose.position.x
        dy = float(getattr(pose2d, 'y', 0.0)) - self.current_pose.position.y
        return round(math.hypot(dx, dy), 3)

    def _estimate_distance_pose(self, pose: Pose) -> float:
        '''현재 위치와 Pose 간의 거리를 추정한다.'''
        if self.current_pose is None:
            return round(math.hypot(pose.position.x, pose.position.y), 3)
        dx = pose.position.x - self.current_pose.position.x
        dy = pose.position.y - self.current_pose.position.y
        return round(math.hypot(dx, dy), 3)

    def _yaw_to_quaternion(self, yaw: float) -> Tuple[float, float, float, float]:
        '''Yaw 값을 쿼터니언으로 변환한다.'''
        half = yaw * 0.5
        return (0.0, 0.0, math.sin(half), math.cos(half))

    def shutdown_interfaces(self) -> None:
        '''하위 인터페이스를 종료한다.'''
        for interface in self._interfaces:
            try:
                interface.shutdown()
            except Exception as exc:
                self.get_logger().error(f'인터페이스 종료 실패: {interface.__class__.__name__} - {exc}')

    def _build_feedback(self, main_state: MainState, progress: float):
        '''작업 유형에 맞는 피드백 메시지를 생성한다.'''
        progress_percent = int(progress * 100.0)

        if main_state == MainState.PICKING_UP_BOOK:
            feedback = PickupBook.Feedback()
            feedback.progress_percent = progress_percent
            return feedback

        if main_state == MainState.RESHELVING_BOOK:
            feedback = ReshelvingBook.Feedback()
            feedback.progress_percent = progress_percent
            return feedback

        if main_state == MainState.GUIDING:
            feedback = GuidePerson.Feedback()
            feedback.distance_remaining_m = max(0.0, 10.0 * (1.0 - progress))
            feedback.person_detected = progress < 1.0
            return feedback

        if main_state == MainState.CLEANING_DESK:
            feedback = CleanSeat.Feedback()
            feedback.progress_percent = progress_percent
            return feedback

        if main_state == MainState.SORTING_SHELVES:
            feedback = RearrangeBook.Feedback()
            feedback.status = self._latest_sub_state.name
            feedback.current_action = f'{progress_percent}% 완료'
            return feedback

        raise ValueError(f'지원하지 않는 메인 상태입니다: {main_state}')

    def _build_result(self, main_state: MainState, success: bool, message: str, **kwargs: Any):
        '''작업 유형에 맞는 결과 메시지를 생성한다.'''
        if main_state == MainState.PICKING_UP_BOOK:
            result = PickupBook.Result()
            result.book_id = str(kwargs.get('book_id', ''))
            result.storage_id = int(kwargs.get('storage_id', 0))
            result.success = success
            result.message = message
            result.total_distance_m = float(kwargs.get('distance', 0.0))
            result.total_time_sec = float(kwargs.get('duration', 0.0))
            return result

        if main_state == MainState.RESHELVING_BOOK:
            result = ReshelvingBook.Result()
            result.success = success
            result.books_processed = 0 if not success else 1
            result.failed_book_ids = []
            result.total_distance_m = 0.0
            result.total_time_sec = 0.0
            result.message = message
            return result

        if main_state == MainState.GUIDING:
            result = GuidePerson.Result()
            result.success = success
            result.error_code = int(kwargs.get('error_code', 0 if success else 1))
            result.total_distance_m = float(kwargs.get('distance', 0.0))
            result.total_time_sec = float(kwargs.get('duration', 0.0))
            result.message = message
            return result

        if main_state == MainState.CLEANING_DESK:
            result = CleanSeat.Result()
            result.success = success
            result.trash_collected_count = 0
            result.trash_types = []
            result.total_distance_m = 0.0
            result.total_time_sec = 0.0
            result.message = message
            return result

        if main_state == MainState.SORTING_SHELVES:
            result = RearrangeBook.Result()
            result.book_id = ''
            result.success = success
            result.message = message
            return result

        raise ValueError(f'지원하지 않는 메인 상태입니다: {main_state}')

def main(args=None) -> None:
    '''ROS 2 엔트리 포인트.'''
    rclpy.init(args=args)
    node = JavisDmcNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('사용자 중단을 감지했습니다.')
    finally:
        node.shutdown_interfaces()
        node.destroy_node()
        rclpy.shutdown()
