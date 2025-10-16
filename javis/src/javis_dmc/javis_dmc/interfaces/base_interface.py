"""
Base Interface for all JAVIS DMC interfaces.
All interfaces must inherit from this abstract base class.
"""

from abc import ABC, abstractmethod
from typing import Optional
import rclpy
from rclpy.node import Node


class BaseInterface(ABC):
    """
    Abstract base class for all JAVIS DMC interfaces.
    
    This class provides common functionality for all interfaces including:
    - ROS2 node integration
    - Namespace handling
    - Logging capabilities
    - Common initialization and shutdown patterns
    """
    
    def __init__(self, node: Node, namespace: str = ""):
        """
        Initialize the base interface.
        
        Args:
            node: ROS2 node instance
            namespace: Robot namespace (e.g., "dobby1")
        """
        self.node = node
        self.namespace = namespace
        self.logger = node.get_logger()
        self._initialized = False
        
        # Log interface creation
        self.logger.info(f"Creating {self.__class__.__name__} with namespace: {namespace}")
    
    @abstractmethod
    def initialize(self) -> bool:
        """
        Initialize the interface.
        
        This method must be implemented by all concrete interfaces.
        It should set up all necessary ROS2 communications (topics, services, actions).
        
        Returns:
            bool: True if initialization successful, False otherwise
        """
        pass
    
    def shutdown(self) -> None:
        """
        Shutdown the interface and clean up resources.
        
        This method can be overridden by concrete interfaces if they need
        custom shutdown behavior.
        """
        if self._initialized:
            self.logger.info(f"Shutting down {self.__class__.__name__}")
            self._initialized = False
        else:
            self.logger.debug(f"{self.__class__.__name__} was not initialized, skipping shutdown")
    
    def _create_topic_name(self, topic: str) -> str:
        """
        Create a properly namespaced topic name.
        
        Args:
            topic: Base topic name (e.g., "status/battery")
            
        Returns:
            str: Fully qualified topic name (e.g., "dobby1/status/battery")
        """
        if self.namespace:
            return f"{self.namespace}/{topic}"
        return topic
    
    def is_initialized(self) -> bool:
        """
        Check if the interface has been initialized.
        
        Returns:
            bool: True if initialized, False otherwise
        """
        return self._initialized
    
    def _set_initialized(self, status: bool) -> None:
        """
        Set the initialization status.
        
        This is a protected method to be called by concrete implementations
        after successful initialization.
        
        Args:
            status: Initialization status to set
        """
        self._initialized = status
        if status:
            self.logger.info(f"{self.__class__.__name__} initialized successfully")
        else:
            self.logger.warn(f"{self.__class__.__name__} initialization failed")
    
    def get_namespace(self) -> str:
        """
        Get the current namespace.
        
        Returns:
            str: Current namespace
        """
        return self.namespace
    
    def get_logger(self):
        """
        Get the logger instance.
        
        Returns:
            ROS2 logger instance
        """
        return self.logger
