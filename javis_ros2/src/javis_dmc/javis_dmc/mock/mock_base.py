'''모든 Mock 인터페이스를 위한 공통 기반 클래스. 테스트 및 시뮬레이션 편의 기능을 제공한다.'''

import threading
import time
from typing import Dict, Any, Optional
from concurrent.futures import Future
from rclpy.node import Node



class MockResponse:
    '''Mock 동작 응답을 구성하는 데이터.'''

    def __init__(
        self,
        success: bool = True,
        delay: float = 0.0,
        error_code: str = '',
        data: Dict[str, Any] = None,
    ):
        self.success = success
        self.delay = delay
        self.error_code = error_code
        self.data = data or {}


class MockFuture:
    '''Mock 비동기 작업을 표현하는 Future 객체.'''

    def __init__(self, response: MockResponse):
        self._response = response
        self._result = None
        self._done = False
        self._callbacks = []

        # 비동기 응답을 모사한다.
        threading.Thread(target=self._execute, daemon=True).start()

    def _execute(self) -> None:
        '''지연 시간을 반영해 Mock 작업을 실행한다.'''
        if self._response.delay > 0:
            time.sleep(self._response.delay)

        # 사전에 정의된 응답 데이터를 생성한다.
        self._result = self._create_result()
        self._done = True

        # 완료 콜백을 순차적으로 호출한다.
        for callback in self._callbacks:
            try:
                callback(self)
            except Exception as exc:  # pragma: no cover - logging only
                print(f'Mock callback error: {exc}')

    def _create_result(self):
        '''Mock 응답 객체를 생성한다.'''
        result = type('MockResult', (), {})()
        result.success = self._response.success
        result.error_code = self._response.error_code

        # 추가 데이터는 속성으로 부여한다.
        for key, value in self._response.data.items():
            setattr(result, key, value)

        return result

    def done(self) -> bool:
        '''작업 완료 여부를 반환한다.'''
        return self._done

    def result(self, timeout: Optional[float] = None):
        '''완료된 Mock 결과를 반환한다.'''
        start_time = time.time()
        while not self._done:
            if timeout is not None and (time.time() - start_time) > timeout:
                raise TimeoutError('Mock operation timed out')
            time.sleep(0.01)
        return self._result

    def add_done_callback(self, callback) -> None:
        '''작업 완료 시 실행할 콜백을 등록한다.'''
        if self._done:
            callback(self)
        else:
            self._callbacks.append(callback)


class MockBase:
    '''
    모든 Mock 인터페이스에서 공통으로 사용하는 유틸리티를 제공한다.

    - Test GUI를 통한 응답 프로파일 설정
    - 지연 및 실패 상황 시뮬레이션
    - Mock 비동기 작업 흐름
    '''

    def __init__(self, node: Node, namespace: str = ''):
        self.node = node
        self.namespace = namespace
        self.logger = node.get_logger()

        # 응답 프로파일
        self._responses: Dict[str, MockResponse] = {}
        self._default_response = MockResponse()

        # 연속 데이터 퍼블리시 제어
        self._publishing = False
        self._publish_threads = []

        self.logger.info(f'Created {self.__class__.__name__} mock interface')

    def set_mock_response(self, method_name: str, response: MockResponse) -> None:
        '''
        특정 메서드용 Mock 응답을 등록한다.

        Args:
            method_name: 응답을 설정할 메서드 이름
            response: Mock 응답 구성
        '''
        self._responses[method_name] = response
        self.logger.info(
            f'Set mock response for {method_name}: success={response.success}, delay={response.delay}s',
        )

    def get_mock_response(self, method_name: str) -> MockResponse:
        '''
        지정된 메서드의 Mock 응답을 반환한다.

        Args:
            method_name: 조회할 메서드 이름
        '''
        return self._responses.get(method_name, self._default_response)

    def create_mock_future(self, method_name: str, additional_data: Dict[str, Any] = None) -> MockFuture:
        '''
        Mock 비동기 작업 Future를 생성한다.

        Args:
            method_name: 응답을 사용할 메서드 이름
            additional_data: 결과에 포함할 추가 데이터
        '''
        response = self.get_mock_response(method_name)
        if additional_data:
            response.data.update(additional_data)

        return MockFuture(response)

    def start_topic_publishing(self, topic_name: str, rate_hz: float = 10.0) -> None:
        '''
        Mock 토픽 데이터를 주기적으로 퍼블리시한다.

        Args:
            topic_name: 퍼블리시할 토픽 이름
            rate_hz: 퍼블리시 주기(Hz)
        '''
        if self._publishing:
            return

        self._publishing = True
        thread = threading.Thread(
            target=self._publish_loop,
            args=(topic_name, rate_hz),
            daemon=True,
        )
        thread.start()
        self._publish_threads.append(thread)

    def stop_topic_publishing(self) -> None:
        '''토픽 퍼블리시를 중단한다.'''
        self._publishing = False

    def _publish_loop(self, topic_name: str, rate_hz: float) -> None:
        '''토픽 퍼블리시 루프를 실행한다.'''
        rate = 1.0 / rate_hz
        while self._publishing:
            try:
                # 하위 클래스에서 실제 데이터를 채운다.
                self._publish_topic_data(topic_name)
                time.sleep(rate)
            except Exception as exc:  # pragma: no cover - logging only
                self.logger.error(f'Error in mock topic publishing: {exc}')
                break

    def _publish_topic_data(self, topic_name: str) -> None:  # pragma: no cover - override 대상
        '''
        하위 클래스에서 특정 토픽 데이터를 퍼블리시하도록 오버라이드한다.

        Args:
            topic_name: 퍼블리시 대상 토픽
        '''

    def cleanup(self) -> None:
        '''Mock 인터페이스 자원을 정리한다.'''
        self.stop_topic_publishing()
        self.logger.info(f"Cleaned up {self.__class__.__name__} mock interface")
