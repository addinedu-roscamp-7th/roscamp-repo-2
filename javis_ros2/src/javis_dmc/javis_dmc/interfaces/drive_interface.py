'''JAVIS DMC의 주행 제어 인터페이스.'''

from abc import abstractmethod
from concurrent.futures import Future
from threading import Lock
from types import SimpleNamespace
from typing import Optional

from geometry_msgs.msg import Pose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from javis_interfaces.action import GuideNavigation, MoveToTarget
from javis_interfaces.msg import CurrentPose
from javis_interfaces.srv import DriveControlCommand

from .base_interface import BaseInterface


class DriveInterface(BaseInterface):
    '''로봇 주행 제어(DDC 통신)를 위한 추상 인터페이스.'''

    @abstractmethod
    def move_to_target(self, pose: Pose, location_name: str = '') -> Future:
        '''지정된 위치로 이동을 수행한다.'''

    @abstractmethod
    def enable_follow_mode(
        self,
        tracking_topic: str,
        destination: Pose,
        follow_distance: float = 1.5,
    ) -> bool:
        '''사람 추종 모드를 활성화한다.'''

    @abstractmethod
    def disable_follow_mode(self) -> bool:
        '''사람 추종 모드를 비활성화한다.'''

    @abstractmethod
    def stop(self) -> bool:
        '''비상 정지를 수행한다.'''

    @abstractmethod
    def rotate_in_place(self, angle: float) -> bool:
        '''제자리 회전을 수행한다.'''

    @abstractmethod
    def get_current_pose(self) -> Optional[Pose]:
        '''현재 로봇 위치를 반환한다.'''


class RosDriveInterface(DriveInterface):
    '''ROS 2 액션/서비스 기반 주행 인터페이스 구현.'''

    def __init__(self, node, namespace: str = ''):
        super().__init__(node, namespace)
        self._cb_group = ReentrantCallbackGroup()
        self._move_client: Optional[ActionClient] = None
        self._guide_client: Optional[ActionClient] = None
        self._drive_command_client = None
        self._current_pose: Optional[Pose] = None
        self._current_pose_lock = Lock()
        self._current_pose_sub = None
        self._guide_goal_handle = None

    def initialize(self) -> bool:
        '''ROS 2 액션/서비스 클라이언트를 초기화한다.'''
        self._move_client = ActionClient(
            self.node,
            MoveToTarget,
            self._create_topic_name('drive/move_to_target'),
            callback_group=self._cb_group,
        )
        self._guide_client = ActionClient(
            self.node,
            GuideNavigation,
            self._create_topic_name('drive/guide_navigation'),
            callback_group=self._cb_group,
        )
        self._drive_command_client = self.node.create_client(
            DriveControlCommand,
            self._create_topic_name('drive/control_command'),
            callback_group=self._cb_group,
        )
        self._current_pose_sub = self.node.create_subscription(
            CurrentPose,
            self._create_topic_name('status/current_pose'),
            self._on_pose,
            10,
            callback_group=self._cb_group,
        )

        self._wait_for_action(self._move_client, 'move_to_target')
        self._wait_for_action(self._guide_client, 'guide_navigation')
        self._wait_for_service(self._drive_command_client, 'drive/control_command')

        self._set_initialized(True)
        return True

    def move_to_target(self, pose: Pose, location_name: str = '') -> Future:
        '''지정된 위치로 이동을 수행한다.'''
        goal = MoveToTarget.Goal()
        goal.target_pose = pose
        goal.location_name = location_name
        return self._send_goal(self._move_client, goal)

    def enable_follow_mode(
        self,
        tracking_topic: str,
        destination: Pose,
        follow_distance: float = 1.5,
    ) -> bool:
        '''사람 추종 모드를 활성화한다.'''
        del tracking_topic  # 추후 AIS 연동 시 활용
        goal = GuideNavigation.Goal()
        goal.destination = destination
        goal.max_speed = 0.8
        goal.person_follow_distance = follow_distance

        future = self._send_goal(self._guide_client, goal, store_handle=True)
        future.add_done_callback(lambda _: None)
        return True

    def disable_follow_mode(self) -> bool:
        '''사람 추종 모드를 비활성화한다.'''
        if self._guide_goal_handle is None:
            return False

        cancel_future = self._guide_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(lambda _: None)
        self._guide_goal_handle = None
        return True

    def stop(self) -> bool:
        '''비상 정지 명령을 발행한다.'''
        if self._drive_command_client is None:
            return False
        request = DriveControlCommand.Request()
        request.command = DriveControlCommand.Request.STOP
        request.reason = 'emergency_stop'
        self._drive_command_client.call_async(request)
        return True

    def rotate_in_place(self, angle: float) -> bool:
        '''제자리 회전을 위한 추상 명령을 발행한다.'''
        request = DriveControlCommand.Request()
        request.command = DriveControlCommand.Request.RESUME
        request.reason = f'rotate:{angle:.1f}'
        self._drive_command_client.call_async(request)
        return True

    def get_current_pose(self) -> Optional[Pose]:
        '''현재 로봇 위치를 반환한다.'''
        with self._current_pose_lock:
            return self._current_pose

    def shutdown(self) -> None:
        '''생성된 자원을 정리한다.'''
        if self._move_client is not None:
            self._move_client.destroy()
            self._move_client = None
        if self._guide_client is not None:
            self._guide_client.destroy()
            self._guide_client = None
        if self._drive_command_client is not None:
            self._drive_command_client.destroy()
            self._drive_command_client = None
        if self._current_pose_sub is not None:
            self.node.destroy_subscription(self._current_pose_sub)
            self._current_pose_sub = None
        super().shutdown()

    def _send_goal(self, client, goal, store_handle: bool = False) -> Future:
        '''액션 Goal을 전송하고 결과를 Future로 반환한다.'''
        result_future: Future = Future()

        if client is None:
            result_future.set_result(SimpleNamespace(success=False))
            return result_future

        if not client.wait_for_server(timeout_sec=0.5):
            message = '액션 서버가 준비되지 않았습니다.'
            self.logger.error(message)
            result_future.set_result(SimpleNamespace(success=False, message=message))
            return result_future

        goal_future = client.send_goal_async(goal, feedback_callback=self._log_feedback)

        def _goal_response(fut):
            goal_handle = fut.result()
            if not goal_handle.accepted:
                message = '액션 Goal이 거절되었습니다.'
                self.logger.warn(message)
                result_future.set_result(SimpleNamespace(success=False, message=message))
                return

            if store_handle:
                self._guide_goal_handle = goal_handle

            result = goal_handle.get_result_async()

            def _result_ready(res_future):
                try:
                    result_msg = res_future.result().result
                except Exception as exc:
                    result_future.set_exception(exc)
                    return
                result_future.set_result(result_msg)
                if store_handle and self._guide_goal_handle is goal_handle:
                    self._guide_goal_handle = None

            result.add_done_callback(_result_ready)

        goal_future.add_done_callback(_goal_response)
        return result_future

    def _wait_for_action(self, client, name: str) -> None:
        '''액션 서버 준비 상태를 확인한다.'''
        if client is None:
            return
        if not client.wait_for_server(timeout_sec=1.0):
            self.logger.warn(f'{name} 액션 서버가 아직 준비되지 않았습니다.')

    def _wait_for_service(self, client, name: str) -> None:
        '''서비스 서버 준비 상태를 확인한다.'''
        if client is None:
            return
        if not client.wait_for_service(timeout_sec=1.0):
            self.logger.warn(f'{name} 서비스가 아직 준비되지 않았습니다.')

    def _on_pose(self, msg: CurrentPose) -> None:
        '''현재 위치를 캐시한다.'''
        with self._current_pose_lock:
            self._current_pose = msg.pose

    def _log_feedback(self, feedback) -> None:
        '''액션 피드백을 디버그 로그로 출력한다.'''
        self.logger.debug(f'주행 피드백 수신: {feedback}')
