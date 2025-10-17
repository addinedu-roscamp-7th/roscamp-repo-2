from functools import partial
from typing import Any, Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import Pose
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node

from javis_dmc.battery_manager import BatteryManager, BatteryState
from javis_dmc.interfaces import (
    RosAIInterface,
    RosArmInterface,
    RosDriveInterface,
    RosGUIInterface,
    RosLLMInterface,
    RosSTTInterface,
    RosTTSInterface,
)
from javis_dmc.states.main_states import DmcStateMachine
from javis_dmc.states.state_enums import MainState, SubState
from javis_dmc.task_executors.base_executor import BaseExecutor
from javis_dmc.task_executors.cleaning_executor import CleaningExecutor
from javis_dmc.task_executors.guiding_executor import GuidingExecutor
from javis_dmc.task_executors.pickup_executor import PickupExecutor
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


class JavisDmcNode(Node):
    '''JAVIS DMC의 메인 노드 구현.'''

    def __init__(self) -> None:
        super().__init__('javis_dmc_node')

        # 파라미터 선언
        self.declare_parameter('robot_namespace', '')
        self.namespace = self.get_parameter('robot_namespace').get_parameter_value().string_value

        # 주요 구성요소 초기화
        self.state_machine = DmcStateMachine()
        self.battery_manager = BatteryManager()
        self.current_pose: Optional[Pose] = None
        self.current_executor: Optional[BaseExecutor] = None
        self._battery_timer_period = 2.0
        self._state_timer_period = 5.0
        self._last_state_msg: Optional[DobbyState] = None
        self.drive = RosDriveInterface(self, self.namespace)
        self.arm = RosArmInterface(self, self.namespace)
        self.ai = RosAIInterface(self, self.namespace)
        self.gui = RosGUIInterface(self, self.namespace)
        self.llm = RosLLMInterface(self, self.namespace)
        self.stt = RosSTTInterface(self, self.namespace)
        self.tts = RosTTSInterface(self, self.namespace)
        self._interfaces = [
            self.drive,
            self.arm,
            self.ai,
            self.gui,
            self.llm,
            self.stt,
            self.tts,
        ]

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

        self.get_logger().info('JAVIS DMC 노드가 초기화되었습니다.')
        self.state_machine.set_main_state(MainState.IDLE)
        self.state_machine.set_sub_state(SubState.NONE)
        self._publish_state_immediately()

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
            executor.set_step_delay(1.0)

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

        if not self.state_machine.can_accept_task(main_state):
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
        self._cleanup_after_task(MainState.MOVING_TO_CHARGER)
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

        if goal_handle.is_cancel_requested:
            self.get_logger().info('작업 도중 취소 요청이 감지되었습니다.')
            self.cancel_current_task()
            goal_handle.canceled()
            result_msg = self._build_result(main_state, False, '작업이 취소되었습니다.')
            self._cleanup_after_task(success=False, forced_state=MainState.MOVING_TO_CHARGER)
            return result_msg

        result_msg = self._build_result(main_state, True, '작업을 완료했습니다.')
        goal_handle.succeed()
        self._cleanup_after_task(success=True)
        return result_msg

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

    def _build_result(self, main_state: MainState, success: bool, message: str):
        '''작업 유형에 맞는 결과 메시지를 생성한다.'''
        if main_state == MainState.PICKING_UP_BOOK:
            result = PickupBook.Result()
            result.success = success
            result.message = message
            result.total_distance_m = 0.0
            result.total_time_sec = 0.0
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
            result.error_code = 0 if success else 1
            result.total_distance_m = 0.0
            result.total_time_sec = 0.0
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
