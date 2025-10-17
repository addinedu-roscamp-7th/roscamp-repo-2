'''JAVIS DMC 인터페이스가 공통으로 상속하는 기반 클래스.'''

from abc import ABC, abstractmethod
from typing import Optional
import rclpy
from rclpy.node import Node


class BaseInterface(ABC):
    '''JAVIS DMC 인터페이스의 공통 동작을 정의하는 추상 클래스.'''
    
    def __init__(self, node: Node, namespace: str = ''):
        '''기본 인터페이스를 초기화한다.'''
        self.node = node
        self.namespace = namespace
        self.logger = node.get_logger()
        self._initialized = False
        
        # 인터페이스 생성 로그 출력
        self.logger.info(f'Creating {self.__class__.__name__} with namespace: {namespace}')
    
    @abstractmethod
    def initialize(self) -> bool:
        '''인터페이스 초기화를 수행한다.'''
        pass
    
    def shutdown(self) -> None:
        '''인터페이스를 종료하고 자원을 정리한다.'''
        if self._initialized:
            self.logger.info(f'Shutting down {self.__class__.__name__}')
            self._initialized = False
        else:
            self.logger.debug(f'{self.__class__.__name__} was not initialized, skipping shutdown')
    
    def _create_topic_name(self, topic: str) -> str:
        '''네임스페이스 정보를 반영한 토픽 이름을 생성한다.'''
        node_namespace = self.node.get_namespace().strip('/')
        interface_namespace = self.namespace.strip('/')

        if not interface_namespace:
            return topic

        if node_namespace == interface_namespace:
            return topic

        return f'{interface_namespace}/{topic}'
        return topic
    
    def is_initialized(self) -> bool:
        '''초기화 여부를 반환한다.'''
        return self._initialized
    
    def _set_initialized(self, status: bool) -> None:
        '''초기화 상태를 지정한다.'''
        self._initialized = status
        if status:
            self.logger.info(f'{self.__class__.__name__} initialized successfully')
        else:
            self.logger.warn(f'{self.__class__.__name__} initialization failed')
    
    def get_namespace(self) -> str:
        '''현재 네임스페이스를 반환한다.'''
        return self.namespace
    
    def get_logger(self):
        '''로거 인스턴스를 반환한다.'''
        return self.logger
