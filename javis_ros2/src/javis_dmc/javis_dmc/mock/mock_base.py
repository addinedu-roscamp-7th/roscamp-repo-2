"""
Base class for all Mock interfaces.
Provides common functionality for testing and simulation.
"""

import asyncio
import random
import threading
import time
from typing import Dict, Any, Optional
from concurrent.futures import Future
from queue import Queue
from rclpy.node import Node


class MockResponse:
    """Mock response configuration."""
    def __init__(self, success: bool = True, delay: float = 0.0, 
                 error_code: str = "", data: Dict[str, Any] = None):
        self.success = success
        self.delay = delay
        self.error_code = error_code
        self.data = data or {}


class MockFuture:
    """Mock future object for async operations."""
    def __init__(self, response: MockResponse):
        self._response = response
        self._result = None
        self._done = False
        self._callbacks = []
        
        # Start the mock operation
        threading.Thread(target=self._execute, daemon=True).start()
    
    def _execute(self):
        """Execute the mock operation with delay."""
        if self._response.delay > 0:
            time.sleep(self._response.delay)
        
        # Set result based on response configuration
        self._result = self._create_result()
        self._done = True
        
        # Call any registered callbacks
        for callback in self._callbacks:
            try:
                callback(self)
            except Exception as e:
                print(f"Mock callback error: {e}")
    
    def _create_result(self):
        """Create result object based on response."""
        result = type('MockResult', (), {})()
        result.success = self._response.success
        result.error_code = self._response.error_code
        
        # Add any additional data
        for key, value in self._response.data.items():
            setattr(result, key, value)
            
        return result
    
    def done(self) -> bool:
        """Check if the operation is complete."""
        return self._done
    
    def result(self, timeout: Optional[float] = None):
        """Get the result of the operation."""
        start_time = time.time()
        while not self._done:
            if timeout is not None and (time.time() - start_time) > timeout:
                raise TimeoutError("Mock operation timed out")
            time.sleep(0.01)
        return self._result
    
    def add_done_callback(self, callback):
        """Add callback to be called when operation completes."""
        if self._done:
            callback(self)
        else:
            self._callbacks.append(callback)


class MockBase:
    """
    Base class for all Mock interfaces.
    
    Provides common functionality for:
    - Response configuration from Test GUI
    - Simulated delays and failures
    - Mock async operations
    """
    
    def __init__(self, node: Node, namespace: str = ""):
        self.node = node
        self.namespace = namespace
        self.logger = node.get_logger()
        
        # Response configuration
        self._responses: Dict[str, MockResponse] = {}
        self._default_response = MockResponse()
        
        # Topic publishing for continuous data
        self._publishing = False
        self._publish_threads = []
        
        self.logger.info(f"Created {self.__class__.__name__} mock interface")
    
    def set_mock_response(self, method_name: str, response: MockResponse):
        """
        Set mock response for a specific method.
        
        Args:
            method_name: Name of the method to mock
            response: Mock response configuration
        """
        self._responses[method_name] = response
        self.logger.info(f"Set mock response for {method_name}: "
                        f"success={response.success}, delay={response.delay}s")
    
    def get_mock_response(self, method_name: str) -> MockResponse:
        """
        Get mock response for a method.
        
        Args:
            method_name: Name of the method
            
        Returns:
            MockResponse: Response configuration
        """
        return self._responses.get(method_name, self._default_response)
    
    def create_mock_future(self, method_name: str, 
                          additional_data: Dict[str, Any] = None) -> MockFuture:
        """
        Create a mock future for async operations.
        
        Args:
            method_name: Name of the method being mocked
            additional_data: Additional data to include in result
            
        Returns:
            MockFuture: Mock future object
        """
        response = self.get_mock_response(method_name)
        if additional_data:
            response.data.update(additional_data)
        
        return MockFuture(response)
    
    def start_topic_publishing(self, topic_name: str, rate_hz: float = 10.0):
        """
        Start publishing mock topic data.
        
        Args:
            topic_name: Name of topic to publish
            rate_hz: Publishing rate in Hz
        """
        if self._publishing:
            return
            
        self._publishing = True
        thread = threading.Thread(
            target=self._publish_loop, 
            args=(topic_name, rate_hz), 
            daemon=True
        )
        thread.start()
        self._publish_threads.append(thread)
    
    def stop_topic_publishing(self):
        """Stop all topic publishing."""
        self._publishing = False
    
    def _publish_loop(self, topic_name: str, rate_hz: float):
        """Topic publishing loop."""
        rate = 1.0 / rate_hz
        while self._publishing:
            try:
                # Override in subclasses to publish actual data
                self._publish_topic_data(topic_name)
                time.sleep(rate)
            except Exception as e:
                self.logger.error(f"Error in mock topic publishing: {e}")
                break
    
    def _publish_topic_data(self, topic_name: str):
        """
        Override in subclasses to publish specific topic data.
        
        Args:
            topic_name: Name of topic to publish data for
        """
        pass
    
    def cleanup(self):
        """Clean up mock interface resources."""
        self.stop_topic_publishing()
        self.logger.info(f"Cleaned up {self.__class__.__name__} mock interface")