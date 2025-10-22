'''DMC Test GUI ROS 노드 및 실행 엔트리 포인트.'''

import json
import threading
from queue import Queue
from typing import Optional

import tkinter as tk

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool
from std_srvs.srv import Trigger

from javis_dmc.states.state_enums import MainState, RobotMode, SubState
from javis_interfaces.msg import BatteryStatus
from javis_interfaces.msg import DobbyState
from javis_interfaces.srv import ForceTaskResult
from javis_interfaces.srv import SetManualState
from javis_interfaces.srv import SetRobotMode

from .test_gui_widget import TestGuiApp


class TestGuiRosNode(Node):
    '''DMC 테스트 GUI용 ROS 노드.'''

    def __init__(self, event_queue: Queue) -> None:
        super().__init__('dmc_test_gui_node')
        self.event_queue = event_queue

        self.declare_parameter('robot_namespace', '')
        self.robot_namespace = self.get_parameter('robot_namespace').get_parameter_value().string_value

        self._mode_client = self.create_client(SetRobotMode, self._ns('admin/set_robot_mode'))
        self._emergency_client = self.create_client(Trigger, self._ns('admin/emergency_stop'))
        self._resume_client = self.create_client(Trigger, self._ns('admin/resume_navigation'))
        self._listening_client = self.create_client(SetBool, self._ns('set_listening_mode'))
        self._force_task_client = self.create_client(ForceTaskResult, self._ns('admin/force_task_result'))
        self._manual_state_client = self.create_client(SetManualState, self._ns('admin/set_manual_state'))

        self.create_subscription(DobbyState, self._ns('status/robot_state'), self._on_state, 10)
        self.create_subscription(BatteryStatus, self._ns('status/battery_status'), self._on_battery, 10)
        self.create_subscription(String, self._ns('admin/mode_feedback'), self._on_mode_feedback, 10)

        self._current_mode = RobotMode.STANDBY.name.lower()

    def _ns(self, topic: str) -> str:
        if not self.robot_namespace:
            return topic
        return f'{self.robot_namespace}/{topic}'

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
