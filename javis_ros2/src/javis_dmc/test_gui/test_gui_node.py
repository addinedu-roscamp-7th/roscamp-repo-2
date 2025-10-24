'''DMC Test GUI ROS 노드 및 실행 엔트리 포인트.'''

import json
import threading
import time
from pathlib import Path
from queue import Queue
from threading import Lock
from typing import Dict, List, Optional

import tkinter as tk

import rclpy
from rcl_interfaces.msg import Parameter, ParameterValue
from rcl_interfaces.msg import ParameterType  # type: ignore[attr-defined]
from rcl_interfaces.srv import SetParameters
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool
from std_srvs.srv import Trigger

import yaml

from javis_dmc.states.state_enums import MainState, RobotMode, SubState
from javis_interfaces.msg import BatteryStatus
from javis_interfaces.msg import DobbyState
from javis_interfaces.srv import ForceTaskResult
from javis_interfaces.srv import SetManualState
from javis_interfaces.srv import SetRobotMode
from javis_dmc_test_msgs.srv import SetMockResponse

from .test_gui_widget import TestGuiApp


class TestGuiRosNode(Node):
    '''DMC 테스트 GUI용 ROS 노드.'''

    def __init__(self, event_queue: Queue) -> None:
        super().__init__('test_gui_node_node')
        self.event_queue = event_queue
        self._state_lock = Lock()
        self._current_main_state: str = MainState.INITIALIZING.name
        self._current_sub_state: str = SubState.NONE.name

        self.declare_parameter('robot_namespace', 'dobby1')  # 기본 네임스페이스 설정
        self.robot_namespace = self.get_parameter('robot_namespace').get_parameter_value().string_value

        self._mode_client = self.create_client(SetRobotMode, self._ns('admin/set_robot_mode'))
        self._emergency_client = self.create_client(Trigger, self._ns('admin/emergency_stop'))
        self._resume_client = self.create_client(Trigger, self._ns('admin/resume_navigation'))
        self._listening_client = self.create_client(SetBool, self._ns('set_listening_mode'))
        self._force_task_client = self.create_client(ForceTaskResult, self._ns('admin/force_task_result'))
        self._manual_state_client = self.create_client(SetManualState, self._ns('admin/set_manual_state'))
        self._describe_client = self.create_client(Trigger, self._ns('debug/describe_state_machine'))
        self._state_transition_sub = self.create_subscription(
            String,
            self._ns('debug/state_transitions'),
            self._on_state_transition,
            10,
        )
        self._set_param_client = self.create_client(SetParameters, f'{self._dmc_node_fqn()}/set_parameters')
        self._mock_response_client = self.create_client(SetMockResponse, self._ns('test/set_mock_response'))

        self.create_subscription(DobbyState, self._ns('status/robot_state'), self._on_state, 10)
        self.create_subscription(BatteryStatus, self._ns('status/battery_status'), self._on_battery, 10)
        self.create_subscription(String, self._ns('admin/mode_feedback'), self._on_mode_feedback, 10)

        self._current_mode = RobotMode.STANDBY.name.lower()
        self._use_mock_interfaces = False
        self._patrol_active = False
        self._scenarios = self._load_scenarios()
        self._mock_methods = self._load_mock_methods()
        self._mock_modes: Dict[str, int] = {
            'drive': -1,
            'arm': -1,
            'ai': -1,
            'gui': -1,
        }

    def _ns(self, topic: str) -> str:
        if not self.robot_namespace:
            return topic
        return f'{self.robot_namespace}/{topic}'

    def _dmc_node_fqn(self) -> str:
        node_name = self._ns('javis_dmc_node')
        if not node_name.startswith('/'):
            node_name = f'/{node_name}'
        return node_name

    def _put_event(self, event: dict, level: str = 'INFO') -> None:
        payload = dict(event)
        payload.setdefault('level', level)
        try:
            self.event_queue.put_nowait(payload)
        except Exception as exc:
            self.get_logger().error(f'GUI 이벤트 큐 적재 실패: {exc}')

    # ===== Subscriber callbacks =====

    def _on_state(self, msg: DobbyState) -> None:
        main_name = self._enum_name(MainState, msg.main_state)
        sub_name = self._enum_name(SubState, msg.sub_state)
        with self._state_lock:
            self._current_main_state = main_name
            self._current_sub_state = sub_name
        log_msg = f'상태 업데이트: {main_name} / {sub_name}'
        self._put_event({
            'type': 'state',
            'main': main_name,
            'sub': sub_name,
            'mode': self._current_mode.upper(),
            'log': log_msg,
        }, level='STATUS')

    def _on_battery(self, msg: BatteryStatus) -> None:
        status = '충전중' if msg.is_charging else '대기'
        text = f'{msg.charge_percentage:.1f}% ({status})'
        self._put_event({
            'type': 'battery',
            'text': text,
        }, level='STATUS')

    def _on_mode_feedback(self, msg: String) -> None:
        message = msg.data
        parsed: Optional[dict] = None
        try:
            parsed = json.loads(message)
        except json.JSONDecodeError:
            parsed = None
        if parsed:
            self._current_mode = parsed.get('mode', self._current_mode)
            log_msg = parsed.get('message', '모드 피드백 수신')
            status_key = str(parsed.get('status', '')).lower()
            level = self._status_to_level(status_key)
            self._put_event({
                'type': 'mode_feedback',
                'mode': self._current_mode,
                'message': log_msg,
                'log': f"모드 피드백: {log_msg}",
                'status': status_key,
            }, level=level)
        else:
            self._put_event({
                'type': 'notify',
                'message': f'모드 피드백(raw): {message}',
            }, level='WARN')

    # ===== Utility / public API =====

    def describe_state_machine(self) -> Optional[Dict[str, object]]:
        if not self._describe_client.wait_for_service(timeout_sec=1.0):
            self._put_event({'type': 'notify', 'message': '⚠️ describe_state_machine 서비스 미준비'}, level='WARN')
            return None

        future = self._describe_client.call_async(Trigger.Request())
        success, response = self._await_future(future)
        if not success or response is None:
            self._put_event({'type': 'notify', 'message': '⚠️ 상태 정보 요청 실패'}, level='WARN')
            return None
        if not response.success:
            self._put_event({'type': 'notify', 'message': f'⚠️ 상태 정보 실패: {response.message}'}, level='WARN')
            return None
        try:
            data = json.loads(response.message or '{}')
            if isinstance(data, dict):
                runtime = data.get('runtime', {})
                if isinstance(runtime, dict):
                    if 'use_mock_interfaces' in runtime:
                        self._use_mock_interfaces = bool(runtime['use_mock_interfaces'])
                    if 'patrol_active' in runtime:
                        self._patrol_active = bool(runtime['patrol_active'])
                    if 'mock_modes' in runtime and isinstance(runtime['mock_modes'], dict):
                        for key, value in runtime['mock_modes'].items():
                            if isinstance(value, int):
                                self._mock_modes[key] = value
            return data
        except json.JSONDecodeError as exc:
            self._put_event({'type': 'notify', 'message': f'⚠️ 상태 정보 파싱 실패: {exc}'}, level='ERROR')
            return None

    def set_interface_mode(self, use_mock: bool) -> bool:
        if not self._set_param_client.wait_for_service(timeout_sec=1.0):
            self._put_event({'type': 'notify', 'message': '⚠️ set_parameters 서비스 미준비'}, level='WARN')
            return False

        request = SetParameters.Request()
        parameter = Parameter()
        parameter.name = 'use_mock_interfaces'
        value = ParameterValue()
        value.type = ParameterType.PARAMETER_BOOL
        value.bool_value = bool(use_mock)
        parameter.value = value
        request.parameters.append(parameter)

        future = self._set_param_client.call_async(request)
        success, response = self._await_future(future)
        if not success or response is None:
            self._put_event({'type': 'notify', 'message': '⚠️ 인터페이스 모드 변경 실패'}, level='ERROR')
            return False

        result = all(result.successful for result in response.results)
        if result:
            self._use_mock_interfaces = use_mock
            mode = 'Mock' if use_mock else 'Real'
            self._put_event({'type': 'interface_mode', 'mode': mode}, level='INFO')
        else:
            self._put_event({'type': 'notify', 'message': '⚠️ 인터페이스 파라미터 적용 실패'}, level='WARN')
        return result

    def wait_for_state(self, main: Optional[str], sub: Optional[str], timeout: float = 10.0) -> bool:
        deadline = time.time() + timeout
        main = (main or '').upper()
        sub = (sub or '').upper()
        while time.time() < deadline:
            with self._state_lock:
                current_main = self._current_main_state
                current_sub = self._current_sub_state
            if (not main or current_main == main) and (not sub or current_sub == sub):
                return True
            time.sleep(0.1)
        return False

    def call_service_step(self, target: str, **kwargs) -> bool:
        mapping = {
            'set_listening_mode': self._call_set_listening,
            'set_robot_mode': self._call_set_mode,
            'emergency_stop': lambda: self._call_trigger(self._emergency_client),
            'resume_navigation': lambda: self._call_trigger(self._resume_client),
            'force_task_success': lambda: self._call_force_task(True),
            'force_task_failure': lambda: self._call_force_task(False),
        }
        handler = mapping.get(target)
        if handler is None:
            self._put_event({'type': 'notify', 'message': f'⚠️ 지원하지 않는 시나리오 서비스: {target}'}, level='WARN')
            return False
        try:
            return handler(**kwargs) if kwargs else handler()
        except TypeError:
            return handler()

    def get_scenarios(self) -> Dict[str, Dict[str, object]]:
        return dict(self._scenarios)

    def get_mock_methods(self) -> Dict[str, List[str]]:
        return {key: list(values) for key, values in self._mock_methods.items()}

    def get_mock_modes(self) -> Dict[str, int]:
        return dict(self._mock_modes)

    def set_mock_mode(self, interface: str, mode: str) -> bool:
        interface_key = interface.lower()
        if interface_key not in self._mock_modes:
            self._put_event({'type': 'notify', 'message': f'⚠️ 지원하지 않는 인터페이스: {interface}'}, level='WARN')
            return False

        mapping = {
            'auto': -1,
            'real': 0,
            'mock': 1,
        }
        if mode not in mapping:
            self._put_event({'type': 'notify', 'message': f'⚠️ 알 수 없는 모드 값입니다: {mode}'}, level='WARN')
            return False

        target_value = mapping[mode]

        if not self._set_param_client.wait_for_service(timeout_sec=1.0):
            self._put_event({'type': 'notify', 'message': '⚠️ set_parameters 서비스 미준비'}, level='WARN')
            return False

        request = SetParameters.Request()
        parameter = Parameter()
        parameter.name = f'mock_mode_{interface_key}'
        value = ParameterValue()
        value.type = ParameterType.PARAMETER_INTEGER
        value.integer_value = target_value
        parameter.value = value
        request.parameters.append(parameter)

        future = self._set_param_client.call_async(request)
        success, response = self._await_future(future)
        if not success or response is None:
            self._put_event({'type': 'notify', 'message': '⚠️ Mock 모드 설정 실패'}, level='ERROR')
            return False

        result = all(result.successful for result in response.results)
        if result:
            self._mock_modes[interface_key] = target_value
            label = { -1: '자동', 0: '실장비', 1: 'Mock' }.get(target_value, '자동')
            self._put_event({'type': 'notify', 'message': f'{interface_key} 인터페이스 모드를 {label}로 설정했습니다. 재시작 후 적용됩니다.'}, level='INFO')
            return True

        self._put_event({'type': 'notify', 'message': '⚠️ Mock 모드 파라미터 적용 실패'}, level='WARN')
        return False

    def get_mock_methods(self) -> Dict[str, List[str]]:
        return {key: list(values) for key, values in self._mock_methods.items()}

    def set_mock_response(self, interface: str, scenario: str, success_override: bool) -> bool:
        if not self._mock_response_client.wait_for_service(timeout_sec=0.5):
            self._put_event({'type': 'notify', 'message': '⚠️ set_mock_response 서비스 미준비'}, level='WARN')
            return False
        request = SetMockResponse.Request()
        request.interface_name = interface
        request.scenario = scenario
        request.success_override = bool(success_override)
        future = self._mock_response_client.call_async(request)
        success, result = self._await_future(future)
        if not success or result is None:
            self._put_event({'type': 'notify', 'message': '⚠️ Mock 응답 설정 실패'}, level='ERROR')
            return False
        self._put_event({'type': 'notify', 'message': result.message}, level=('SUCCESS' if result.success else 'WARN'))
        return bool(result.success)

    # ===== 내부 헬퍼 =====

    def _await_future(self, future, timeout: float = 5.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if future.done():
                try:
                    return True, future.result()
                except Exception as exc:  # noqa: BLE001
                    return False, exc
            time.sleep(0.05)
        return False, None

    def _call_set_listening(self, enabled: bool = True) -> bool:
        if not self._listening_client.wait_for_service(timeout_sec=0.5):
            self._put_event({'type': 'notify', 'message': '⚠️ set_listening_mode 서비스 미준비'}, level='WARN')
            return False
        request = SetBool.Request()
        request.data = bool(enabled)
        future = self._listening_client.call_async(request)
        success, response = self._await_future(future)
        if not success or response is None:
            return False
        return bool(response.success)

    def _call_set_mode(self, mode: str) -> bool:
        if not self._mode_client.wait_for_service(timeout_sec=0.5):
            self._put_event({'type': 'notify', 'message': '⚠️ set_robot_mode 서비스 미준비'}, level='WARN')
            return False
        request = SetRobotMode.Request()
        request.mode = mode
        future = self._mode_client.call_async(request)
        success, response = self._await_future(future)
        if not success or response is None:
            return False
        return bool(response.success)

    def _call_force_task(self, success_flag: bool) -> bool:
        if not self._force_task_client.wait_for_service(timeout_sec=0.5):
            self._put_event({'type': 'notify', 'message': '⚠️ force_task_result 미준비'}, level='WARN')
            return False
        request = ForceTaskResult.Request()
        request.success = bool(success_flag)
        request.message = 'Scenario override'
        future = self._force_task_client.call_async(request)
        success, response = self._await_future(future)
        if not success or response is None:
            return False
        return bool(response.applied)

    def _call_trigger(self, client) -> bool:
        if not client.wait_for_service(timeout_sec=0.5):
            return False
        future = client.call_async(Trigger.Request())
        success, response = self._await_future(future)
        if not success or response is None:
            return False
        return bool(response.success)

    def _load_scenarios(self) -> Dict[str, Dict[str, object]]:
        scenarios_path = Path(__file__).resolve().parent.parent / 'config' / 'test_gui_scenarios.yaml'
        if not scenarios_path.exists():
            return {}
        try:
            with scenarios_path.open('r', encoding='utf-8') as handle:
                data = yaml.safe_load(handle) or {}
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'시나리오 파일 로드 실패: {exc}')
            return {}
        scenarios = data.get('scenarios', {})
        return scenarios if isinstance(scenarios, dict) else {}

    def _load_mock_methods(self) -> Dict[str, List[str]]:
        methods_path = Path(__file__).resolve().parent.parent / 'config' / 'mock_method_profiles.yaml'
        if not methods_path.exists():
            return {}
        try:
            with methods_path.open('r', encoding='utf-8') as handle:
                data = yaml.safe_load(handle) or {}
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Mock 메서드 파일 로드 실패: {exc}')
            return {}
        methods_section = data.get('mock_methods', {})
        if not isinstance(methods_section, dict):
            return {}
        result: Dict[str, List[str]] = {}
        for key, value in methods_section.items():
            if isinstance(value, list):
                result[str(key)] = [str(item) for item in value]
        return result

    # ===== Service helpers =====

    def request_set_mode(self, mode: str) -> None:
        '''모드 변경 요청을 전송한다.'''
        if not self._mode_client.wait_for_service(timeout_sec=0.5):
            self._put_event({'type': 'notify', 'message': '⚠️ set_robot_mode 서비스가 준비되지 않았습니다.'}, level='WARN')
            return

        request = SetRobotMode.Request()
        request.mode = mode
        future = self._mode_client.call_async(request)
        future.add_done_callback(lambda fut: self._handle_mode_response(fut, mode))
        self._put_event({'type': 'notify', 'message': f'모드 전환 요청: {mode}'}, level='INFO')

    def _handle_mode_response(self, future, requested_mode: str) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self._put_event({'type': 'notify', 'message': f'⚠️ 모드 전환 실패: {exc}'}, level='ERROR')
            return

        status = '성공' if response.success else '실패'
        level = 'INFO' if response.success else 'WARN'
        self._put_event({
            'type': 'notify',
            'message': f'모드 전환 {status}: {requested_mode} - {response.message}',
        }, level=level)

    def request_emergency_stop(self) -> None:
        '''긴급 정지 요청을 보낸다.'''
        if not self._emergency_client.wait_for_service(timeout_sec=0.5):
            self._put_event({'type': 'notify', 'message': '⚠️ emergency_stop 서비스가 준비되지 않았습니다.'}, level='WARN')
            return
        future = self._emergency_client.call_async(Trigger.Request())
        future.add_done_callback(self._handle_simple_trigger('긴급 정지'))
        self._put_event({'type': 'notify', 'message': '긴급 정지 요청을 전송했습니다.'}, level='INFO')

    def request_resume_navigation(self) -> None:
        '''긴급 해제 요청을 보낸다.'''
        if not self._resume_client.wait_for_service(timeout_sec=0.5):
            self._put_event({'type': 'notify', 'message': '⚠️ resume_navigation 서비스가 준비되지 않았습니다.'}, level='WARN')
            return
        future = self._resume_client.call_async(Trigger.Request())
        future.add_done_callback(self._handle_simple_trigger('긴급 해제'))
        self._put_event({'type': 'notify', 'message': '긴급 해제 요청을 전송했습니다.'}, level='INFO')

    def request_listening(self, enabled: bool) -> None:
        '''LISTENING 모드를 토글한다.'''
        if not self._listening_client.wait_for_service(timeout_sec=0.5):
            self._put_event({'type': 'notify', 'message': '⚠️ set_listening_mode 서비스가 준비되지 않았습니다.'}, level='WARN')
            return

        request = SetBool.Request()
        request.data = enabled
        future = self._listening_client.call_async(request)
        future.add_done_callback(lambda fut: self._handle_listening_response(fut, enabled))
        action = '시작' if enabled else '종료'
        self._put_event({'type': 'notify', 'message': f'LISTENING {action} 요청을 전송했습니다.'}, level='INFO')

    def _handle_simple_trigger(self, label: str):
        def _callback(future):
            try:
                response = future.result()
            except Exception as exc:
                self._put_event({'type': 'notify', 'message': f'⚠️ {label} 실패: {exc}'}, level='ERROR')
                return
            status = '성공' if response.success else '실패'
            level = 'INFO' if response.success else 'WARN'
            self._put_event({'type': 'notify', 'message': f'{label} {status}: {response.message}'}, level=level)
            if label == '긴급 정지' and response.success:
                self._put_event({'type': 'mode_feedback', 'mode': self._current_mode, 'message': response.message, 'status': 'emergency_stop'}, level='WARN')

        return _callback

    def _handle_listening_response(self, future, enabled: bool) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self._put_event({'type': 'notify', 'message': f'⚠️ LISTENING 토글 실패: {exc}'}, level='ERROR')
            return
        status = '성공' if response.success else '실패'
        action = '시작' if enabled else '종료'
        message = response.message or ''
        level = 'INFO' if response.success else 'WARN'
        self._put_event({'type': 'notify', 'message': f'LISTENING {action} {status} {message}'}, level=level)
        listening_text = '활성' if enabled and response.success else '비활성'
        self._put_event({'type': 'listening', 'text': listening_text}, level='STATUS')

    def request_force_task_result(self, success: bool) -> None:
        '''현재 작업을 강제로 성공/실패 처리한다.'''
        if not self._force_task_client.wait_for_service(timeout_sec=0.5):
            self._put_event({'type': 'notify', 'message': '⚠️ force_task_result 서비스가 준비되지 않았습니다.'}, level='WARN')
            return
        req = ForceTaskResult.Request()
        req.success = bool(success)
        req.message = 'Test GUI override'
        future = self._force_task_client.call_async(req)
        future.add_done_callback(self._handle_force_task_response(success))
        label = '성공' if success else '실패'
        self._put_event({'type': 'notify', 'message': f'현재 작업 {label} 처리 요청을 전송했습니다.'}, level='INFO')

    def _handle_force_task_response(self, success: bool):
        def _callback(future):
            try:
                response = future.result()
            except Exception as exc:
                self._put_event({'type': 'notify', 'message': f'⚠️ 작업 수동 처리 오류: {exc}'}, level='ERROR')
                return
            label = '성공' if success else '실패'
            status = '적용' if response.applied else '거부'
            detail = response.detail or ''
            level = 'INFO' if response.applied else 'WARN'
            self._put_event({'type': 'notify', 'message': f'작업 {label} {status}: {detail}'}, level=level)
        return _callback

    def request_manual_state(self, mode: Optional[int], main: Optional[int], sub: Optional[int]) -> None:
        '''상태 머신을 수동으로 설정한다.'''
        if not self._manual_state_client.wait_for_service(timeout_sec=0.5):
            self._put_event({'type': 'notify', 'message': '⚠️ set_manual_state 서비스가 준비되지 않았습니다.'}, level='WARN')
            return

        req = SetManualState.Request()
        req.mode = mode if mode is not None else -1
        req.main_state = main if main is not None else -1
        req.sub_state = sub if sub is not None else -1

        future = self._manual_state_client.call_async(req)
        future.add_done_callback(self._handle_manual_state_response)
        self._put_event({'type': 'notify', 'message': '상태 수동 설정 요청을 전송했습니다.'}, level='INFO')

    def _handle_manual_state_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self._put_event({'type': 'notify', 'message': f'⚠️ 수동 상태 설정 실패: {exc}'}, level='ERROR')
            return
        status = '성공' if response.success else '실패'
        level = 'INFO' if response.success else 'WARN'
        self._put_event({'type': 'notify', 'message': f'수동 상태 설정 {status}: {response.message}'}, level=level)

    def _on_state_transition(self, msg: String) -> None:
        payload: Optional[Dict[str, str]] = None
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            payload = None
        if payload:
            if isinstance(payload, dict) and 'patrol_active' in payload:
                self._patrol_active = bool(payload.get('patrol_active'))
                self._put_event({'type': 'patrol_status', 'active': self._patrol_active}, level='STATUS')
            self._put_event({'type': 'state_transition', 'payload': payload}, level='STATUS')

    @staticmethod
    def _status_to_level(status: str) -> str:
        mapping = {
            'success': 'SUCCESS',
            'resumed': 'SUCCESS',
            'manual_state': 'INFO',
            'noop': 'STATUS',
            'task_override': 'INFO',
            'busy': 'WARN',
            'rejected': 'WARN',
            'emergency_stop': 'ERROR',
            'error': 'ERROR',
        }
        return mapping.get(status, 'INFO')

    @staticmethod
    def _enum_name(enum_cls, value: int) -> str:
        try:
            return enum_cls(value).name
        except ValueError:
            return f'UNKNOWN({value})'


def main() -> None:
    '''Test GUI 실행 엔트리 포인트.'''
    rclpy.init()
    event_queue: Queue = Queue()
    node = TestGuiRosNode(event_queue)

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    stop_event = threading.Event()

    def _spin() -> None:
        while not stop_event.is_set():
            executor.spin_once(timeout_sec=0.1)

    spin_thread = threading.Thread(target=_spin, daemon=True)
    spin_thread.start()

    root = tk.Tk()
    app = TestGuiApp(root, node, event_queue)

    def _on_close() -> None:
        stop_event.set()
        root.destroy()

    root.protocol('WM_DELETE_WINDOW', _on_close)
    try:
        root.mainloop()
    finally:
        stop_event.set()
        spin_thread.join(timeout=1.0)
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
