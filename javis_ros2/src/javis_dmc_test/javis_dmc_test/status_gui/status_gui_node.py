'''DMC Status GUI ROS 노드 및 실행 엔트리 포인트.'''

import json
import threading
import time
from queue import Queue
from threading import Lock
from typing import Dict, List, Optional

import tkinter as tk

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.msg import Log
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from javis_dmc.states.state_enums import MainState, RobotMode, SubState
from javis_interfaces.msg import BatteryStatus, DobbyState
from javis_interfaces.srv import SetRobotMode

from .status_gui_widget import StatusGuiApp


def _rosout_level_to_tag(level: int) -> str:
    """rosout 로그 레벨 정수값을 GUI 태그 문자열로 변환한다."""
    # rcl_interfaces/msg/Log levels: DEBUG=10, INFO=20, WARN=30, ERROR=40, FATAL=50
    if level >= 50:
        return 'ERROR'
    if level >= 40:
        return 'ERROR'
    if level >= 30:
        return 'WARN'
    if level >= 20:
        return 'INFO'
    return 'STATUS'


class StatusGuiRosNode(Node):
    '''DMC 상태 모니터링 GUI용 ROS 노드.'''

    def __init__(self, event_queue: Queue) -> None:
        super().__init__('status_gui_node')
        self.event_queue = event_queue
        self._state_lock = Lock()
        self._current_main_state: str = MainState.INITIALIZING.name
        self._current_sub_state: str = SubState.NONE.name

        self.declare_parameter('robot_namespace', 'dobby1')
        self.robot_namespace = self.get_parameter('robot_namespace').get_parameter_value().string_value

        # Clients
        self._mode_client = self.create_client(SetRobotMode, self._ns('admin/set_robot_mode'))
        self._emergency_client = self.create_client(Trigger, self._ns('admin/emergency_stop'))
        self._resume_client = self.create_client(Trigger, self._ns('admin/resume_navigation'))
        self._listening_client = self.create_client(SetBool, self._ns('set_listening_mode'))
        self._describe_client = self.create_client(Trigger, self._ns('debug/describe_state_machine'))
        self._set_param_client = self.create_client(SetParameters, f'{self._dmc_node_fqn()}/set_parameters')

        # Subscriptions
        self.create_subscription(DobbyState, self._ns('status/robot_state'), self._on_state, 10)
        self.create_subscription(BatteryStatus, self._ns('status/battery_status'), self._on_battery, 10)
        self.create_subscription(String, self._ns('admin/mode_feedback'), self._on_mode_feedback, 10)
        self.create_subscription(String, self._ns('debug/state_transitions'), self._on_state_transition, 10)
        # ROSOUT 구독으로 로그 패널 구성
        self.create_subscription(Log, '/rosout', self._on_rosout, 50)

        self._current_mode = RobotMode.STANDBY.name
        self._patrol_active = False

        # rclpy logger verbosity
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

    # ------------------------------ utils
    def _ns(self, topic: str) -> str:
        """로봇 네임스페이스를 토픽 이름에 접두사로 추가한다."""
        if not self.robot_namespace:
            return topic
        return f'{self.robot_namespace}/{topic}'

    def _dmc_node_fqn(self) -> str:
        """DMC 노드의 fully qualified name을 반환한다."""
        node_name = self._ns('javis_dmc_node')
        if not node_name.startswith('/'):
            node_name = f'/{node_name}'
        return node_name

    def _put_event(self, event: dict, level: str = 'INFO') -> None:
        """이벤트를 GUI 큐에 추가한다."""
        payload = dict(event)
        payload.setdefault('level', level)
        payload.setdefault('timestamp', time.time())
        try:
            self.event_queue.put_nowait(payload)
        except Exception as exc:
            self.get_logger().error(f'GUI 이벤트 큐 적재 실패: {exc}')

    # ------------------------------ Callbacks
    def _on_state(self, msg: DobbyState) -> None:
        """로봇 상태 메시지를 수신하여 GUI 이벤트로 변환한다."""
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
            'mode': self._current_mode,
            'log': log_msg,
        }, level='STATUS')

    def _on_battery(self, msg: BatteryStatus) -> None:
        """배터리 상태 메시지를 수신하여 GUI 이벤트로 변환한다."""
        status = '충전중' if msg.is_charging else '대기'
        text = f'{msg.charge_percentage:.1f}% ({status})'
        self._put_event({'type': 'battery', 'text': text}, level='STATUS')

    def _on_mode_feedback(self, msg: String) -> None:
        """모드 피드백 메시지를 수신하여 GUI 이벤트로 변환한다."""
        message = msg.data
        parsed: Optional[dict]
        try:
            parsed = json.loads(message)
        except json.JSONDecodeError:
            parsed = None

        if parsed:
            old_mode = self._current_mode
            self._current_mode = parsed.get('mode', self._current_mode)
            log_msg = parsed.get('message', '모드 피드백 수신')
            status_key = str(parsed.get('status', '')).lower()
            level = self._status_to_level(status_key)
            if old_mode != self._current_mode:
                self.get_logger().info(f'모드 변경: {old_mode} → {self._current_mode}')
            self._put_event({
                'type': 'mode_feedback',
                'mode': self._current_mode,
                'message': log_msg,
                'log': f'모드 피드백: {log_msg}',
                'status': status_key,
                'raw_json': message,
            }, level=level)
            if status_key == 'emergency_stop':
                self._put_event({
                    'type': 'state',
                    'main': 'EMERGENCY_STOP',
                    'sub': 'NONE',
                    'mode': self._current_mode,
                    'log': '긴급 정지 상태로 전환(피드백 기반)'
                }, level='STATUS')
        else:
            self._put_event({'type': 'notify', 'message': msg}, level='ERROR')

    def _handle_simple_trigger(self, future, action_name: str) -> None:
        """단순 트리거 서비스(Trigger) 응답을 처리하여 GUI 이벤트로 전달한다."""
        try:
            resp = future.result()
            if resp.success:
                self._put_event({'type': 'notify', 'message': f'{action_name} 성공!'}, level='INFO')
            else:
                msg = resp.message if resp.message else '알 수 없는 오류'
                self._put_event({'type': 'notify', 'message': f'{action_name} 실패: {msg}'}, level='WARN')
        except Exception as e:
            msg = f'{action_name} 실패: {e}'
            self._put_event({'type': 'notify', 'message': msg}, level='ERROR')

    def _handle_listening_response(self, future, action: str) -> None:
        """LISTENING 모드 서비스 응답을 처리하여 GUI 이벤트로 전달한다."""
        try:
            resp = future.result()
            if resp.success:
                self._put_event({'type': 'notify', 'message': f'LISTENING {action} 성공!'}, level='INFO')
            else:
                self._put_event({'type': 'notify', 'message': f'LISTENING {action} 실패: {resp.message}'}, level='WARN')
        except Exception as e:
            self._put_event({'type': 'notify', 'message': f'LISTENING {action} 호출 오류: {e}'}, level='ERROR')

    def _on_state_transition(self, msg: String) -> None:
        """상태 전이 메시지를 수신하여 GUI 이벤트로 변환한다."""
        payload: Optional[Dict[str, str]]
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            payload = None
        if payload:
            if isinstance(payload, dict) and 'patrol_active' in payload:
                self._patrol_active = bool(payload.get('patrol_active'))
                self._put_event({'type': 'patrol_status', 'active': self._patrol_active}, level='STATUS')
            self._put_event({'type': 'state_transition', 'payload': payload, 'raw_json': msg.data}, level='STATUS')

    def _on_rosout(self, msg: Log) -> None:
        """/rosout 토픽의 로그 메시지를 수신하여 GUI 이벤트로 변환한다."""
        level_tag = _rosout_level_to_tag(int(getattr(msg, 'level', 20)))
        name = getattr(msg, 'name', '') or ''
        text = getattr(msg, 'msg', '') or ''
        stamp = getattr(msg, 'stamp', None)
        ts = time.time()
        try:
            if stamp and hasattr(stamp, 'sec'):
                ts = float(stamp.sec) + float(getattr(stamp, 'nanosec', 0)) / 1e9
        except Exception:
            pass
        self._put_event({
            'type': 'rosout',
            'timestamp': ts,
            'node': name,
            'message': text,
            'level': level_tag,
        }, level=level_tag)

    # ------------------------------ helpers
    def _await_future(self, future, timeout: float = 5.0):
        """Future 객체의 완료를 대기하고 (성공여부, 결과/예외) 튜플을 반환한다."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if future.done():
                try:
                    return True, future.result()
                except Exception as exc:
                    return False, exc
            time.sleep(0.05)
        return False, None

    def describe_state_machine(self) -> Optional[Dict[str, object]]:
        """DMC 노드로부터 상태 머신 정보(상태/전이/런타임)를 조회한다."""
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
                if isinstance(runtime, dict) and 'patrol_active' in runtime:
                    self._patrol_active = bool(runtime['patrol_active'])
            return data
        except json.JSONDecodeError as exc:
            self._put_event({'type': 'notify', 'message': f'⚠️ 상태 정보 파싱 실패: {exc}'}, level='ERROR')
            return None

    def wait_for_state(self, main: Optional[str], sub: Optional[str], timeout: float = 10.0) -> bool:
        """지정된 메인/서브 상태에 도달할 때까지 대기한다."""
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

    def request_set_mode(self, mode: str) -> None:
        """로봇 모드 변경 서비스를 호출한다."""
        if not self._mode_client.wait_for_service(timeout_sec=0.5):
            self._put_event({'type': 'notify', 'message': '⚠️ set_robot_mode 서비스가 준비되지 않았습니다.'}, level='WARN')
            return
        request = SetRobotMode.Request()
        request.mode = mode
        future = self._mode_client.call_async(request)
        future.add_done_callback(lambda fut: self._handle_mode_response(fut, mode))
        self._put_event({'type': 'notify', 'message': f'모드 전환 요청: {mode}'}, level='INFO')

    def request_emergency_stop(self) -> None:
        """긴급 정지 서비스를 호출한다."""
        if not self._emergency_client.wait_for_service(timeout_sec=0.5):
            self._put_event({'type': 'notify', 'message': '⚠️ emergency_stop 서비스가 준비되지 않았습니다.'}, level='WARN')
            return
        future = self._emergency_client.call_async(Trigger.Request())
        future.add_done_callback(self._handle_simple_trigger('긴급 정지'))
        self._put_event({'type': 'notify', 'message': '긴급 정지 요청을 전송했습니다.'}, level='INFO')

    def request_resume_navigation(self) -> None:
        """긴급 정지 해제 서비스를 호출한다."""
        if not self._resume_client.wait_for_service(timeout_sec=0.5):
            self._put_event({'type': 'notify', 'message': '⚠️ resume_navigation 서비스가 준비되지 않았습니다.'}, level='WARN')
            return
        future = self._resume_client.call_async(Trigger.Request())
        future.add_done_callback(self._handle_simple_trigger('긴급 해제'))
        self._put_event({'type': 'notify', 'message': '긴급 해제 요청을 전송했습니다.'}, level='INFO')

    def request_listening(self, enabled: bool) -> None:
        """LISTENING 모드 활성화/비활성화 서비스를 호출한다."""
        if not self._listening_client.wait_for_service(timeout_sec=0.5):
            self._put_event({'type': 'notify', 'message': '⚠️ set_listening_mode 서비스가 준비되지 않았습니다.'}, level='WARN')
            return
        request = SetBool.Request()
        request.data = enabled
        future = self._listening_client.call_async(request)
        future.add_done_callback(lambda fut: self._handle_listening_response(fut, enabled))
        action = '시작' if enabled else '종료'
        self._put_event({'type': 'notify', 'message': f'LISTENING {action} 요청을 전송했습니다.'}, level='INFO')

    # -------------- helpers (callbacks)
    def _handle_mode_response(self, future, requested_mode: str) -> None:
        """모드 변경 서비스 응답을 처리하여 GUI 이벤트로 전달한다."""
        try:
            response = future.result()
        except Exception as exc:
            self._put_event({'type': 'notify', 'message': f'⚠️ 모드 전환 실패: {exc}'}, level='ERROR')
            return
        status = '성공' if response.success else '실패'
        level = 'INFO' if response.success else 'WARN'
        self._put_event({'type': 'notify', 'message': f'모드 전환 {status}: {requested_mode} - {response.message}'}, level=level)

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
    """Status GUI 애플리케이션의 메인 엔트리 포인트(ROS 노드 초기화 및 Tkinter 실행)."""
    rclpy.init()
    event_queue: Queue = Queue()
    node = StatusGuiRosNode(event_queue)

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    stop_event = threading.Event()

    def _spin() -> None:
        while not stop_event.is_set():
            executor.spin_once(timeout_sec=0.1)

    spin_thread = threading.Thread(target=_spin, daemon=True)
    spin_thread.start()

    root = tk.Tk()
    app = StatusGuiApp(root, node, event_queue)

    def _on_close() -> None:
        stop_event.set()
        root.destroy()

    root.protocol('WM_DELETE_WINDOW', _on_close)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        if spin_thread.is_alive():
            spin_thread.join(timeout=1.0)
        try:
            executor.remove_node(node)
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
